/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_camera.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
#include "esp_lcd_ili9341.h"
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
#include "esp_lcd_gc9a01.h"
#endif
#include <driver/i2c_master.h>

static const char *TAG = "example";

// Using SPI2 in the example
#define LCD_HOST    SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK           GPIO_NUM_3
#define EXAMPLE_PIN_NUM_MOSI           GPIO_NUM_45
#define EXAMPLE_PIN_NUM_MISO           GPIO_NUM_46
#define EXAMPLE_PIN_NUM_LCD_DC         GPIO_NUM_47
#define EXAMPLE_PIN_NUM_LCD_RST        GPIO_NUM_21
#define EXAMPLE_PIN_NUM_LCD_CS         GPIO_NUM_14
#define EXAMPLE_PIN_NUM_BK_LIGHT       GPIO_NUM_0

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              320
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              240
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 10
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (8 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

#define CAM_WIDTH 96
#define CAM_HEIGHT 96
extern void detect_task(void *arg);
static SemaphoreHandle_t lvgl_mux = NULL;

int64_t perf_start;
int64_t perf_end;

uint8_t *cbuf;
uint8_t *snapshot_buf;

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}


static int camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}
#define WIDTH 96
#define HEIGHT 96
void detect_task(void *arg)
{
    ei_impulse_result_t result = { 0 };
    signal_t signal;
    signal.total_length = impulse_609002_0.input_width * impulse_609002_0.input_height;
    signal.get_data = &camera_get_data;
    EI_IMPULSE_ERROR res = EI_IMPULSE_OK;
    int err = 0;
    size_t snapshot_size = WIDTH * HEIGHT * 3;
    snapshot_buf = (uint8_t *)heap_caps_malloc(snapshot_size, MALLOC_CAP_SPIRAM);
    if (snapshot_buf == NULL) {
        ESP_LOGE("detect", "Failed to allocate snapshot buffer");
        vTaskDelete(NULL);
    }

    while (1) {
        fmt2rgb888((const uint8_t *)cbuf, (96*96*2), PIXFORMAT_RGB565, snapshot_buf);
        res = run_classifier(&impulse_handle_609002_0, &signal, &result, false);
        if(res != EI_IMPULSE_OK) {
            ESP_LOGE("detect", "ERR: Failed to run classifier (%d)", res);
            vTaskDelay(1);
            continue;
        }
        ESP_LOGI("detect", "Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        bool bb_found = result.bounding_boxes[0].value > 0;
        for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
            auto bb = result.bounding_boxes[ix];
            if (bb.value == 0) {
                continue;
            }
            ESP_LOGI("detect", "    %s (%f) [ x: %lu, y: %lu, width: %lu, height: %lu ]", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
        }
        vTaskDelay(1);
    }
}
void detect_setup(){

}
extern "C" void app_main(void)
{
    const camera_config_t camera_config = {                                   
        .pin_pwdn = GPIO_NUM_NC,         
        .pin_reset = GPIO_NUM_NC,        
        .pin_xclk = GPIO_NUM_15,     
        .pin_sccb_sda = GPIO_NUM_4,     
        .pin_sccb_scl = GPIO_NUM_5,     
        .pin_d7 = GPIO_NUM_16,         
        .pin_d6 = GPIO_NUM_17,         
        .pin_d5 = GPIO_NUM_18,         
        .pin_d4 = GPIO_NUM_12,         
        .pin_d3 = GPIO_NUM_10,         
        .pin_d2 = GPIO_NUM_8,         
        .pin_d1 = GPIO_NUM_9,         
        .pin_d0 = GPIO_NUM_11,         
        .pin_vsync = GPIO_NUM_6,   
        .pin_href = GPIO_NUM_7,    
        .pin_pclk = GPIO_NUM_13,     
        .xclk_freq_hz = 20000000,        
        .ledc_timer = LEDC_TIMER_0,      
        .ledc_channel = LEDC_CHANNEL_0,  
        .pixel_format = PIXFORMAT_RGB565,
        // .frame_size = FRAMESIZE_128X128, 
        .frame_size = FRAMESIZE_96X96, // FRAMESIZE_128X128
        .jpeg_quality = 12,              
        .fb_count = 1,                   
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = 1,    
    };
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
    ESP_LOGI(TAG, "Camera Init done");

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ*2,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
#if CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = NULL;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    // lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreatePinnedToCore(
        example_lvgl_port_task,
        "LVGL",
        EXAMPLE_LVGL_TASK_STACK_SIZE,
        NULL,
        EXAMPLE_LVGL_TASK_PRIORITY,
        NULL,
        0
    );
    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    uint32_t cam_buff_size = CAM_WIDTH * CAM_HEIGHT * 2;
    cbuf = (uint8_t *)heap_caps_malloc(cam_buff_size, MALLOC_CAP_SPIRAM);
    xTaskCreatePinnedToCore(detect_task,
        "detect_task",
        11111,
        NULL,
        5,
        NULL,
        1
    );
    camera_fb_t *pic;
    if (example_lvgl_lock(-1)) {
        lv_draw_label_dsc_t label_dsc;
        lv_draw_label_dsc_init(&label_dsc);
        label_dsc.color = lv_palette_main(LV_PALETTE_BLUE);
        lv_obj_t * canvas = lv_canvas_create(lv_scr_act());
        lv_canvas_set_buffer(canvas, cbuf, CAM_WIDTH, CAM_HEIGHT, LV_IMG_CF_TRUE_COLOR);
        lv_obj_center(canvas);
        lv_canvas_fill_bg(canvas, lv_palette_lighten(LV_PALETTE_GREY, 3), LV_OPA_COVER);
        lv_canvas_draw_text(canvas, 0, 0, 96, &label_dsc, "Some text on text canvas ssssssssssssssssssssssssssssssss");
        // Release the mutex
        example_lvgl_unlock();
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lv_obj_t * canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(canvas, cbuf, CAM_WIDTH, CAM_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_center(canvas);
    while(1){
        pic = esp_camera_fb_get();
        if (pic) {
            if (example_lvgl_lock(-1)) {
                memcpy(cbuf, pic->buf, cam_buff_size);
                esp_camera_fb_return(pic);
                lv_obj_invalidate(canvas);
                example_lvgl_unlock();
            }
        } else {
            ESP_LOGE(TAG, "Frame buffer could not be acquired");
        }
        vTaskDelay(1);
    }
}
