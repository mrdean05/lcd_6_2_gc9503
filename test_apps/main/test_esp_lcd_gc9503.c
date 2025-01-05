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
#include "lvgl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "demos/lv_demos.h"
#include "lv_conf.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_lcd_gc9503.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          6      // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (50)    // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

#define LV_TICK_PERIOD_MS                       2
#define TEST_LCD_H_RES              (480)
#define TEST_LCD_V_RES              (960)
#define TEST_LCD_BIT_PER_PIXEL      (18)
#define TEST_RGB_BIT_PER_PIXEL      (16)
#define TEST_LCD_DATA_WIDTH         (16)

/*
#define TEST_LCD_IO_RGB_DISP    GPIO_NUM_NC                          // -1
#define TEST_LCD_IO_RGB_VSYNC (GPIO_NUM_39)             // EXAMPLE_PIN_NUM_VSYNC
#define TEST_LCD_IO_RGB_HSYNC (GPIO_NUM_38)             // EXAMPLE_PIN_NUM_HSYNC
#define TEST_LCD_IO_RGB_DE (GPIO_NUM_40)                // EXAMPLE_PIN_NUM_DE
#define TEST_LCD_IO_RGB_PCLK (GPIO_NUM_41)              // EXAMPLE_PIN_NUM_PCLK

// Data pins
#define TEST_LCD_IO_RGB_DATA0 (GPIO_NUM_5)              // EXAMPLE_PIN_NUM_DATA0
#define TEST_LCD_IO_RGB_DATA1 (GPIO_NUM_45)             // EXAMPLE_PIN_NUM_DATA1
#define TEST_LCD_IO_RGB_DATA2 (GPIO_NUM_48)             // EXAMPLE_PIN_NUM_DATA2
#define TEST_LCD_IO_RGB_DATA3 (GPIO_NUM_47)             // EXAMPLE_PIN_NUM_DATA3
#define TEST_LCD_IO_RGB_DATA4 (GPIO_NUM_21)             // EXAMPLE_PIN_NUM_DATA4
#define TEST_LCD_IO_RGB_DATA5 (GPIO_NUM_14)             // EXAMPLE_PIN_NUM_DATA5
#define TEST_LCD_IO_RGB_DATA6 (GPIO_NUM_13)             // EXAMPLE_PIN_NUM_DATA6
#define TEST_LCD_IO_RGB_DATA7 (GPIO_NUM_12)             // EXAMPLE_PIN_NUM_DATA7
#define TEST_LCD_IO_RGB_DATA8 (GPIO_NUM_11)             // EXAMPLE_PIN_NUM_DATA8
#define TEST_LCD_IO_RGB_DATA9 (GPIO_NUM_10)             // EXAMPLE_PIN_NUM_DATA9
#define TEST_LCD_IO_RGB_DATA10 (GPIO_NUM_9)             // EXAMPLE_PIN_NUM_DATA10
#define TEST_LCD_IO_RGB_DATA11 (GPIO_NUM_46)            // EXAMPLE_PIN_NUM_DATA11
#define TEST_LCD_IO_RGB_DATA12 (GPIO_NUM_3)             // EXAMPLE_PIN_NUM_DATA12
#define TEST_LCD_IO_RGB_DATA13 (GPIO_NUM_8)             // EXAMPLE_PIN_NUM_DATA13
#define TEST_LCD_IO_RGB_DATA14 (GPIO_NUM_18)            // EXAMPLE_PIN_NUM_DATA14
#define TEST_LCD_IO_RGB_DATA15 (GPIO_NUM_17)            // EXAMPLE_PIN_NUM_DATA15
#define TEST_LCD_IO_SPI_CS_GPIO                 (GPIO_NUM_42)
#define TEST_LCD_IO_SPI_CS_EXPANDER             (IO_EXPANDER_PIN_NUM_1)
#define TEST_LCD_IO_SPI_SCK_WITHOUT_MULTIPLEX   (GPIO_NUM_2)
#define TEST_LCD_IO_SPI_SCK_WITH_MULTIPLEX      (TEST_LCD_IO_RGB_DATA14)
#define TEST_LCD_IO_SPI_SDA_WITHOUT_MULTIPLEX   (GPIO_NUM_1)
#define TEST_LCD_IO_SPI_SDA_WITH_MULTIPLEX      (TEST_LCD_IO_RGB_DATA15)
#define TEST_LCD_IO_RST                         (GPIO_NUM_NC)
*/


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

#define TEST_DELAY_TIME_MS          (3000)

static char *TAG = "gc9503_test";

#define GPIO_LCD 17
#define CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM 0

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif


esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;
esp_io_expander_handle_t expander_handle = NULL;
bool notLcdDisplay = true;

static void lv_example_get_started_1(void *pvParameters);

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

static void draw_all_blue(esp_lcd_panel_handle_t panel_handle) {
    // Define the number of rows to draw at once (adjust based on available memory)
    uint16_t row_line = 16; // Example: Process 16 rows at a time
    uint8_t byte_per_pixel = TEST_RGB_BIT_PER_PIXEL / 8;

    // Allocate memory for one chunk of rows
    uint8_t *color = (uint8_t *)heap_caps_calloc(1, row_line * TEST_LCD_H_RES * byte_per_pixel, MALLOC_CAP_DMA);
    if (!color) {
        printf("Failed to allocate memory for color buffer\n");
        return;
    }

    // Fill the buffer with blue color
    for (int i = 0; i < row_line * TEST_LCD_H_RES; i++) {
        if (byte_per_pixel == 2) {  // RGB565
            color[i * 2] = 0x1F;     // Blue component 0001 1111
            color[i * 2 + 1] = 0x00; // Upper byte 0000 0000
        }
    }

    // Draw blue lines to fill the screen
    for (int j = 0; j < TEST_LCD_V_RES; j += row_line) {
        esp_lcd_panel_draw_bitmap(
            panel_handle,
            0,
            j,
            TEST_LCD_H_RES,
            j + row_line > TEST_LCD_V_RES ? TEST_LCD_V_RES : j + row_line, // Avoid overshooting
            color
        );
    }

    free(color);
}


static void draw_all_yellow(esp_lcd_panel_handle_t panel_handle) {
    // Define the number of rows to draw at once (adjust based on available memory)
    uint16_t row_line = 16; // Example: Process 16 rows at a time
    uint8_t byte_per_pixel = TEST_RGB_BIT_PER_PIXEL / 8;

    // Allocate memory for one chunk of rows
    uint8_t *color = (uint8_t *)heap_caps_calloc(1, row_line * TEST_LCD_H_RES * byte_per_pixel, MALLOC_CAP_DMA);
    if (!color) {
        printf("Failed to allocate memory for color buffer\n");
        return;
    }

    // Fill the buffer with yellow color (RGB565: 0xFFE0)
    for (int i = 0; i < row_line * TEST_LCD_H_RES; i++) {
        if (byte_per_pixel == 2) {  // RGB565
            color[i * 2] = 0xE0;     // Lower byte (1110 0000)
            color[i * 2 + 1] = 0xFF; // Upper byte (1111 1111)
        }
    }

    // Draw yellow lines to fill the screen
    for (int j = 0; j < TEST_LCD_V_RES; j += row_line) {
        esp_lcd_panel_draw_bitmap(
            panel_handle,
            0,
            j,
            TEST_LCD_H_RES,
            j + row_line > TEST_LCD_V_RES ? TEST_LCD_V_RES : j + row_line, // Avoid overshooting
            color
        );
    }

    free(color);
}




static void test_draw_color_bar(esp_lcd_panel_handle_t panel_handle, uint16_t h_res, uint16_t v_res)
{
    uint8_t byte_per_pixel = (TEST_RGB_BIT_PER_PIXEL + 7) / 8;
    uint16_t row_line = v_res / byte_per_pixel / 8;
    uint8_t *color = (uint8_t *)heap_caps_calloc(1, row_line * h_res * byte_per_pixel, MALLOC_CAP_SPIRAM);

    for (int j = 0; j < byte_per_pixel * 8; j++) {
        for (int i = 0; i < row_line * h_res; i++) {
            for (int k = 0; k < byte_per_pixel; k++) {
                color[i * byte_per_pixel + k] = (BIT(j) >> (k * 8)) & 0xff;
            }
        }
        esp_lcd_panel_draw_bitmap(panel_handle, 0, j * row_line, h_res, (j + 1) * row_line, color);
    }

    uint16_t color_line = row_line * byte_per_pixel * 8;
    uint16_t res_line = v_res - color_line;
    if (res_line) {
        for (int i = 0; i < res_line * h_res; i++) {
            for (int k = 0; k < byte_per_pixel; k++) {
                color[i * byte_per_pixel + k] = 0xff;
            }
        }
        esp_lcd_panel_draw_bitmap(panel_handle, 0, color_line, h_res, v_res, color);
    }

    free(color);
}


static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    static uint32_t lasthandshaketime_us;
    uint32_t currtime_us = esp_timer_get_time();
    uint32_t diff = currtime_us - lasthandshaketime_us;
    if (diff < 1000) {
        return; //ignore everything <1ms after an earlier irq
    }
    lasthandshaketime_us = currtime_us;

    esp_lcd_panel_disp_on_off(panel_handle, false);

    notLcdDisplay = false;

    
}

static void setuplcd_disablegpio( void ){

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode =  GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pin_bit_mask = BIT64(GPIO_LCD),
    };

    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_LCD, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(GPIO_LCD, gpio_handshake_isr_handler, NULL);

}


static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
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


static void lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

static void display_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

     // Add the offsets manually
    int col_offset1 = 0;  // As in your Arduino code
    int row_offset1 = 0;   // No row offset in your case
    int col_offset2 = 0;   // Additional column offset
    
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    //printf("The values for the offsets are, offsetx1: %d offsetx2: %d, offsety1: %d, offsetsety2: %d\n\n", offsetx1, offsetx2, offsety1, offsety2);
    #if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
        xSemaphoreGive(sem_gui_ready);
        xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
    #endif

    //printf("The values for the offsets are, offsetx1: %d offsetx2: %d, offsety1: %d, offsetsety2: %d\n\n", offsetx1, offsetx2, offsety1, offsety2);
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void Display_lvglInit(void){
    /* Contains internal graphic buffer called draw buffer */
    static lv_disp_draw_buf_t disp_buf;

    /* Contains the callback functions */
    static lv_disp_drv_t disp_drv;

    /* Initalize the lv_init library */
    lv_init();

    void *buf1 = NULL;
    void *buf2 = NULL;

    buf1 = heap_caps_malloc(TEST_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = heap_caps_malloc(TEST_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);

    /* Initialize LVGL draw buffers */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, TEST_LCD_H_RES * TEST_LCD_V_RES);

    /* Register display driver to LVGL */
    lv_disp_drv_init(&disp_drv);
    disp_drv.sw_rotate    = 1;
    disp_drv.rotated    = LV_DISP_ROT_270;
    disp_drv.hor_res    = TEST_LCD_H_RES;
    disp_drv.ver_res    = TEST_LCD_V_RES; 
    disp_drv.flush_cb   = display_flush_cb;
    disp_drv.draw_buf   = &disp_buf;
    disp_drv.user_data  = panel_handle;
    disp_drv.full_refresh = 0;
    disp_drv.direct_mode = 0;

    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    /* Install LVGL tick timer */
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick,
        .name = "lvgl_tick"
    };

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LV_TICK_PERIOD_MS * 1000));
}


//TEST_CASE("test gc9503 to draw color bar with RGB interface, using GPIO", "[gc9503][rgb][gpio]")
/*
void test_screen ( void ){
    
    
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    
    
    ESP_LOGI(TAG, "Install 3-wire SPI panel IO");
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO,
        .cs_gpio_num = TEST_LCD_IO_SPI_CS_GPIO,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = TEST_LCD_IO_SPI_SCK_WITHOUT_MULTIPLEX,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = TEST_LCD_IO_SPI_SDA_WITHOUT_MULTIPLEX,
        .io_expander = NULL,
    };
    esp_lcd_panel_io_3wire_spi_config_t io_config = GC9503_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle);

    ESP_LOGI(TAG, "Install GC9503 panel driver");
    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .psram_trans_align = 64,
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
            .auto_del_panel_io = 1,
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
    //esp_lcd_panel_disp_on_off(panel_handle, true);
    //vTaskDelay(pdMS_TO_TICKS(1000));


    
    //test_draw_bitmap(panel_handle);
    //draw_all_blue(panel_handle);
    //test_draw_color_bar(panel_handle, TEST_LCD_H_RES, TEST_LCD_V_RES);
    //vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_TIME_MS));

    // Mirror the panel by hardware
    esp_lcd_panel_mirror(panel_handle, true, true);
    //vTaskDelay(pdMS_TO_TICKS(10000));

    //esp_lcd_panel_io_del(io_handle);
    //esp_lcd_panel_del(panel_handle);
}
*/

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
    //esp_lcd_panel_disp_on_off(panel_handle, true);
    //vTaskDelay(pdMS_TO_TICKS(1000));


    //draw_all_yellow(panel_handle);
    //test_draw_bitmap(panel_handle);
    // draw_all_blue(panel_handle);
    //test_draw_color_bar(panel_handle, TEST_LCD_H_RES, TEST_LCD_V_RES);
    //vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_TIME_MS));

    // Mirror the panel by hardware
    esp_lcd_panel_mirror(panel_handle, true, true);
    //vTaskDelay(pdMS_TO_TICKS(10000));

    //esp_lcd_panel_io_del(io_handle);
    //esp_lcd_panel_del(panel_handle);
    //i2c_driver_delete(TEST_EXPANDER_I2C_HOST);
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
    //test_screen();

    Display_lvglInit();

    setuplcd_disablegpio();


    //draw_all_blue()

    xTaskCreate(lv_example_get_started_1, "alternate_display_task", 4096, NULL, 5, NULL);

    /*
    // Set the background color of the active screen
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x5042DD), LV_PART_MAIN);

    // Create a label and set its text
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello world");

    // Set the text color for the label
    lv_obj_set_style_text_color(label, lv_color_hex(0xffffff), LV_PART_MAIN);

    // Align the label to the center of the screen
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    // Make sure the screen refreshes after creating the label
    //lv_scr_load(lv_scr_act());
    */

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(5));
        if (notLcdDisplay) lv_task_handler();

    }


}

static void lv_example_get_started_1(void *pvParameters)
{
    while (1)
    {
        // Display the first message
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
        
        lv_obj_t *label = lv_label_create(lv_scr_act());
        lv_label_set_text(label, "Hello world");
        lv_obj_set_style_text_color(label, lv_color_hex(0x003a57), LV_PART_MAIN);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

        // Wait for 200 milliseconds
        vTaskDelay(pdMS_TO_TICKS(4000));
        
        // Clear the screen
        lv_obj_clean(lv_scr_act());
        
        printf ("The value is %d\n", gpio_get_level(GPIO_LCD));

        // Display the second message
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);

        label = lv_label_create(lv_scr_act());
        lv_label_set_text(label, "It has worked\n");
        lv_obj_set_style_text_color(label, lv_color_hex(0xffffff), LV_PART_MAIN);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

        // Wait for 200 milliseconds again
        vTaskDelay(pdMS_TO_TICKS(4000));

        printf ("The value annex is %d\n", gpio_get_level(GPIO_LCD));

        // Clear the screen before looping
        lv_obj_clean(lv_scr_act());



    }
}