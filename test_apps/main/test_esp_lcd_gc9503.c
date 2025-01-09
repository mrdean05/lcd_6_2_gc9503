/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_io_expander_tca9554.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"

#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_lcd_gc9503.h"


#define LV_TICK_PERIOD_MS                       2
#define TEST_LCD_H_RES              (480)
#define TEST_LCD_V_RES              (960)
#define TEST_LCD_BIT_PER_PIXEL      (18)
#define TEST_RGB_BIT_PER_PIXEL      (16)
#define TEST_LCD_DATA_WIDTH         (16)

#define TEST_LCD_IO_RGB_DISP                    (GPIO_NUM_NC)
#define TEST_LCD_IO_RGB_VSYNC                   (GPIO_NUM_42)
#define TEST_LCD_IO_RGB_HSYNC                   (GPIO_NUM_41)
#define TEST_LCD_IO_RGB_DE                      (GPIO_NUM_2)
#define TEST_LCD_IO_RGB_PCLK                    (GPIO_NUM_1)
#define TEST_LCD_IO_RGB_DATA0                   (GPIO_NUM_11)     //B1      GPIO_NUM_11
#define TEST_LCD_IO_RGB_DATA1                   (GPIO_NUM_10)     //B2      GPIO_NUM_10
#define TEST_LCD_IO_RGB_DATA2                   (GPIO_NUM_9)     // B3     GPIO_NUM_9
#define TEST_LCD_IO_RGB_DATA3                   (GPIO_NUM_46)      // B4      GPIO_NUM_46
#define TEST_LCD_IO_RGB_DATA4                   (GPIO_NUM_3)      //B5      GPIO_NUM_3
#define TEST_LCD_IO_RGB_DATA5                   (GPIO_NUM_48)    
#define TEST_LCD_IO_RGB_DATA6                   (GPIO_NUM_47)
#define TEST_LCD_IO_RGB_DATA7                   (GPIO_NUM_21)
#define TEST_LCD_IO_RGB_DATA8                   (GPIO_NUM_14)
#define TEST_LCD_IO_RGB_DATA9                   (GPIO_NUM_13)
#define TEST_LCD_IO_RGB_DATA10                  (GPIO_NUM_12)
#define TEST_LCD_IO_RGB_DATA11                  (GPIO_NUM_40)     //R1        GPIO_NUM_40
#define TEST_LCD_IO_RGB_DATA12                  (GPIO_NUM_39)     // R2       PIO_NUM_39
#define TEST_LCD_IO_RGB_DATA13                  (GPIO_NUM_38)      // R3      GPIO_NUM_38
#define TEST_LCD_IO_RGB_DATA14                  (GPIO_NUM_0)     // R4       GPIO_NUM_0
#define TEST_LCD_IO_RGB_DATA15                  (GPIO_NUM_45)      // R5       GPIO_NUM_45
#define TEST_LCD_IO_SPI_CS_GPIO                 (GPIO_NUM_0)
#define TEST_LCD_IO_SPI_CS_EXPANDER             (IO_EXPANDER_PIN_NUM_1)
#define TEST_LCD_IO_SPI_SCK_WITHOUT_MULTIPLEX   (GPIO_NUM_15)
#define TEST_LCD_IO_SPI_SCK_WITH_MULTIPLEX      (TEST_LCD_IO_RGB_DATA14)
#define TEST_LCD_IO_SPI_SDA_WITHOUT_MULTIPLEX   (GPIO_NUM_16)
#define TEST_LCD_IO_SPI_SDA_WITH_MULTIPLEX      (TEST_LCD_IO_RGB_DATA15)
#define TEST_LCD_IO_RST                         (GPIO_NUM_NC)


#define TEST_EXPANDER_I2C_HOST      (0)
#define TEST_EXPANDER_I2C_ADDR      (0x3F)
#define TEST_EXPANDER_IO_I2C_SCL    (GPIO_NUM_18)
#define TEST_EXPANDER_IO_I2C_SDA    (GPIO_NUM_8)

static char *TAG = "gc9503_test";

#define CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM 0

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif


esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;
esp_io_expander_handle_t expander_handle = NULL;

static void test_draw_bitmap(esp_lcd_panel_handle_t panel_handle)
{
    uint16_t row_line = TEST_LCD_V_RES / TEST_RGB_BIT_PER_PIXEL;
    uint8_t byte_per_pixel = TEST_RGB_BIT_PER_PIXEL / 8;
    uint8_t *color = (uint8_t *)heap_caps_calloc(1, row_line * TEST_LCD_H_RES * byte_per_pixel, MALLOC_CAP_SPIRAM);
    

    for (int j = 0; j < TEST_RGB_BIT_PER_PIXEL; j++) {
        for (int i = 0; i < row_line * TEST_LCD_H_RES; i++) {
            for (int k = 0; k < byte_per_pixel; k++) {
                color[i * byte_per_pixel + k] = (BIT(j) >> (k * 8)) & 0xff;
            }
        }
        esp_lcd_panel_draw_bitmap(panel_handle, 0, j * row_line, TEST_LCD_H_RES, (j + 1) * row_line, color);
    }
    free(color);
}


static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

void test_screen_ioexpander ( void )
{
    ESP_LOGI(TAG, "Install I2C");
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TEST_EXPANDER_IO_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = TEST_EXPANDER_IO_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 400 * 1000
    };
    i2c_param_config(TEST_EXPANDER_I2C_HOST, &i2c_conf);
    i2c_driver_install(TEST_EXPANDER_I2C_HOST, i2c_conf.mode, 0, 0, 0);

    ESP_LOGI(TAG, "Create TCA9554 IO expander");
    
    esp_io_expander_new_i2c_tca9554(TEST_EXPANDER_I2C_HOST, TEST_EXPANDER_I2C_ADDR, &expander_handle);

    ESP_LOGI(TAG, "Install 3-wire SPI panel IO");
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_EXPANDER,
        .cs_gpio_num = IO_EXPANDER_PIN_NUM_1,
        .scl_io_type = IO_TYPE_EXPANDER,
        .scl_gpio_num = IO_EXPANDER_PIN_NUM_0,
        .sda_io_type = IO_TYPE_EXPANDER,
        .sda_gpio_num = IO_EXPANDER_PIN_NUM_7,
        .io_expander = expander_handle,
    };
    esp_lcd_panel_io_3wire_spi_config_t io_config = GC9503_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle);

    ESP_LOGI(TAG, "Install GC9503 panel driver");
    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .psram_trans_align = 64,
        .num_fbs = 1,
        //.bounce_buffer_size_px = 10 * TEST_LCD_H_RES,
        .data_width = TEST_LCD_DATA_WIDTH,
        .bits_per_pixel = TEST_RGB_BIT_PER_PIXEL,
        .de_gpio_num = TEST_LCD_IO_RGB_DE,
        .pclk_gpio_num = TEST_LCD_IO_RGB_PCLK,
        .vsync_gpio_num = TEST_LCD_IO_RGB_VSYNC,
        .hsync_gpio_num = TEST_LCD_IO_RGB_HSYNC,
        .disp_gpio_num = TEST_LCD_IO_RGB_DISP,
        .data_gpio_nums = {
            TEST_LCD_IO_RGB_DATA0,
            TEST_LCD_IO_RGB_DATA1,
            TEST_LCD_IO_RGB_DATA2,
            TEST_LCD_IO_RGB_DATA3,
            TEST_LCD_IO_RGB_DATA4,
            TEST_LCD_IO_RGB_DATA5,
            TEST_LCD_IO_RGB_DATA6,
            TEST_LCD_IO_RGB_DATA7,
            TEST_LCD_IO_RGB_DATA8,
            TEST_LCD_IO_RGB_DATA9,
            TEST_LCD_IO_RGB_DATA10,
            TEST_LCD_IO_RGB_DATA11,
            TEST_LCD_IO_RGB_DATA12,
            TEST_LCD_IO_RGB_DATA13,
            TEST_LCD_IO_RGB_DATA14,
            TEST_LCD_IO_RGB_DATA15,
        },
        .timings = GC9503_480_960_PANEL_60HZ_RGB_TIMING(),
        .flags.fb_in_psram = 1,
    };
    gc9503_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        .flags = {
            .mirror_by_cmd = 1,
            .auto_del_panel_io = 0,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TEST_LCD_IO_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = TEST_LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    
    esp_lcd_new_panel_gc9503(io_handle, &panel_config, &panel_handle);

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL));

    esp_lcd_panel_reset(panel_handle);

    esp_lcd_panel_init(panel_handle);

    // Mirror the panel by hardware
    esp_lcd_panel_mirror(panel_handle, true, true);
    
}


void app_main(void)
{
    /**
     *    ___   ___  ___  ____   ___ _____
     *   / _ \ / __\/ _ \| ___| / _ \___ /
     *  / /_\// /  | (_) |___ \| | | ||_ \
     * / /_\\/ /___ \__, |___) | |_| |__) |
     * \____/\____/   /_/|____/ \___/____/
    */

   #if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM    
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
    #endif

    test_screen_ioexpander();

    test_draw_bitmap(panel_handle);

}