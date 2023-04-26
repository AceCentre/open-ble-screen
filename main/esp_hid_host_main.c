#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"

// LVGL
#include "lvgl.h"
#include "lvgl_helpers.h"
#define LV_COLOR_GREEN LV_COLOR_MAKE(0x00,0xFF,0x00)
#define LV_TICK_PERIOD_MS 1
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
SemaphoreHandle_t xGuiSemaphore;

void scan_task(void *pvParameters);

#include "keycodes_tables.h"

uint8_t prev_input_buffer[8];
uint8_t input_buffer[8];
int input_i;
bool equal_buffers;
bool new_input = false;

uint8_t last_modifier;
uint8_t last_keycode;


enum status_type {disconnected, scanning, choosing_device, connection_attempt, connected} status;

lv_obj_t * message_area;
lv_style_t text_area_style;

esp_hidh_dev_t *curr_hid_dev = NULL; // connected device

char scan_label_buffer[22];
size_t scan_results_len = 0;
int active_entry_i = 0;
esp_hid_scan_result_t *scan_results_head = NULL;
esp_hid_scan_result_t *scan_result_entry = NULL;
bool new_scan_results = false;

static const char *TAG = "BLE_HID_HOST";

#define SCAN_BUTTON 23
int scan_button_state_prev = 1;
int scan_button_state = 1;

#define NEXT_BUTTON 22
int next_button_state_prev = 1;
int next_button_state = 1;

#define CONFIRM_BUTTON 21
int confirm_button_state_prev = 1;
int confirm_button_state = 1;

// creating list of available fonts
lv_font_t * font_list[] = {&lv_font_montserrat_16, &lv_font_montserrat_24, &lv_font_montserrat_32, &lv_font_montserrat_40};
int active_font_i = 0;
int n_fonts = sizeof(font_list) / sizeof(lv_font_t*);

// creating list of available color palettes
// palette format: {TEXT_COLOR, BACKGROUND_COLOR}
// color also can be defined as LV_COLOR_MAKE(0xFF,0xFF,0xFF)
lv_color_t palette_list[][2] = {
    {LV_COLOR_BLACK, LV_COLOR_WHITE}, 
    {LV_COLOR_YELLOW, LV_COLOR_BLACK},
    {LV_COLOR_BLACK, LV_COLOR_YELLOW},
    {LV_COLOR_GREEN, LV_COLOR_BLACK}
};
int active_palette_i = 0;
int n_palettes = sizeof(palette_list) / sizeof(palette_list[0]);

bool in_previous_buffer(uint8_t keycode) {
    int i = 2;
    while(i < sizeof(prev_input_buffer) && prev_input_buffer[i] != 0) {
        if(keycode == prev_input_buffer[i]) {
            return true;
        }
        i += 1;
    }

    return false;
} 

void handle_keycode(uint8_t modifier, uint8_t keycode) {
    last_modifier = modifier;
    last_keycode = keycode;

    if((keycode > 3 && keycode < 41) || (keycode > 43 && keycode < 57)) {
        // printable keys
        char received_char = 0;
        if(modifier == 0) {
            received_char = printable_keys[keycode];
        } else if(modifier == 2) {
            received_char = printable_shift_keys[keycode];
        }
        
        lv_textarea_add_char(message_area, received_char);
    }else if(keycode == 58) {
        // F1 keycode
        lv_textarea_set_text(message_area, "");
    } else if(keycode == 42) {
        // backspace keycode
        lv_textarea_del_char(message_area);
    } else if(keycode == 76) {
        // delete keycode
        lv_textarea_del_char_forward(message_area);
    } else if(keycode == 82) {
        // up arrow keycode
        lv_textarea_cursor_up(message_area);
    } else if(keycode == 81) {
        // down arrow keycode
        lv_textarea_cursor_down(message_area);
    } else if(keycode == 80) {
        // left arrow keycode
        lv_textarea_cursor_left(message_area);
    } else if(keycode == 79) {
        // right arrow keycode
        lv_textarea_cursor_right(message_area);        
    } else if(keycode == 61) {
        // F2 keycode
        if(active_font_i < (n_fonts-1)) {
            active_font_i++;
        } else {
            active_font_i = 0;
        }                        

        lv_style_set_text_font(&text_area_style, LV_STATE_DEFAULT, font_list[active_font_i]);
        lv_obj_refresh_style(message_area, LV_OBJ_PART_ALL, LV_STYLE_PROP_ALL);
    } else if(keycode == 62) {
        // F3 keycode
        if(active_palette_i < (n_palettes-1)) {
            active_palette_i++;
        } else {
            active_palette_i = 0;
        }

        lv_style_set_text_color(&text_area_style, LV_STATE_DEFAULT, palette_list[active_palette_i][0]);
        lv_style_set_bg_color(&text_area_style, LV_STATE_DEFAULT, palette_list[active_palette_i][1]);                        
        lv_obj_refresh_style(message_area, LV_OBJ_PART_ALL, LV_STYLE_PROP_ALL);
    }
}

static void guiTask(void *pvParameter) {
    // configure buttons
    gpio_set_direction(SCAN_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(SCAN_BUTTON);
    
    gpio_set_direction(NEXT_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(NEXT_BUTTON);

    gpio_set_direction(CONFIRM_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(CONFIRM_BUTTON);

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820         \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A    \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D     \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306

    /* Actual size in pixels, not bytes. */
    size_in_px *= 8;
#endif

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

#if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    disp_drv.rotated = 1;
#endif

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Register an input device when enabled on the menuconfig */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
#endif

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));
    
    // text area style init
    lv_style_init(&text_area_style);
    lv_style_set_text_font(&text_area_style, LV_STATE_DEFAULT, font_list[active_font_i]);
    lv_style_set_text_color(&text_area_style, LV_STATE_DEFAULT, palette_list[active_palette_i][0]);
    lv_style_set_bg_color(&text_area_style, LV_STATE_DEFAULT, palette_list[active_palette_i][1]);
    
    // bigger font style init
    lv_style_t bigger_font_style;
    lv_style_init(&bigger_font_style);
    lv_style_set_text_font(&bigger_font_style, LV_STATE_DEFAULT, &lv_font_montserrat_24);

    // get the current screen 
    lv_obj_t * scr = lv_disp_get_scr_act(NULL);

    // create TabView widget
    lv_obj_t *tabview;
    tabview = lv_tabview_create(scr, NULL);
    lv_tabview_set_btns_pos(tabview, LV_TABVIEW_TAB_POS_NONE);    

    // scan tab section
    lv_obj_t *scan_tab = lv_tabview_add_tab(tabview, "Scan tab");
    lv_page_set_scrlbar_mode(scan_tab, LV_SCRLBAR_MODE_OFF);

    // create scan label
    lv_obj_t * scan_label =  lv_label_create(scan_tab, NULL);
    lv_obj_set_pos(scan_label, 10, 10);
    lv_label_set_text(scan_label, scan_label_buffer);
    lv_obj_add_style(scan_label, LV_STATE_DEFAULT, &bigger_font_style);

    // create devices list
    lv_obj_t * list1 = lv_list_create(scan_tab, NULL);
    lv_obj_align(list1, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_pos(list1, 10, 44);        
    lv_obj_set_size(list1, 300, 186);   
    
    lv_obj_t * active_button = NULL;

    //lv_obj_set_event_cb(list_btn, event_handler);
    
    // message tab section
    lv_obj_t *text_tab = lv_tabview_add_tab(tabview, "Message tab");
    lv_page_set_scrlbar_mode(text_tab, LV_SCRLBAR_MODE_OFF);
    
    // Create text area widget
    message_area = lv_textarea_create(text_tab, NULL);    
    lv_obj_set_pos(message_area, 0, 0);
    lv_obj_set_size(message_area, 320, 240);
    lv_textarea_set_placeholder_text(message_area, "...");
    lv_textarea_set_text(message_area, "");
    lv_obj_add_style(message_area, LV_STATE_DEFAULT, &text_area_style);

    while (1) {
        // read buttons
        scan_button_state = gpio_get_level(SCAN_BUTTON);
        if(scan_button_state == 0 && scan_button_state_prev == 1) {
            printf("Scan button press\n");
            if(status == choosing_device) {
                // start scanning again
                status = disconnected;
                
            }else if(status == connected) {
                // close current connection
                esp_hidh_dev_close(curr_hid_dev);
            }
        }
        scan_button_state_prev = scan_button_state;
        
        next_button_state = gpio_get_level(NEXT_BUTTON);
        if(next_button_state == 0 && next_button_state_prev == 1) {
            printf("Next button press\n");
            if(active_entry_i < (scan_results_len-1)) {
                // choose next
                active_button = lv_list_get_next_btn(list1, active_button);
                scan_result_entry = scan_result_entry->next;
                active_entry_i++;
            } else {
                // if last, choose first
                active_button = lv_list_get_next_btn(list1, NULL);
                scan_result_entry = scan_results_head;
                active_entry_i = 0;
            }
            
            lv_list_focus_btn(list1, active_button);
        }
        next_button_state_prev = next_button_state;

        confirm_button_state = gpio_get_level(CONFIRM_BUTTON);
        if(confirm_button_state == 0 && confirm_button_state_prev == 1) {
            printf("Confirm button press\n");
            printf("Chosen device: %s\n", scan_result_entry->name);

            curr_hid_dev = esp_hidh_dev_open(scan_result_entry->bda, scan_result_entry->transport, scan_result_entry->ble.addr_type);
            if(curr_hid_dev != NULL) {
                lv_tabview_set_tab_act(tabview, 1, LV_ANIM_OFF);

                lv_obj_t * mbox = lv_msgbox_create(text_tab, NULL);
                lv_msgbox_set_text(mbox, "Connection successful");
                lv_obj_set_pos(mbox, 35, 70);
                lv_obj_set_size(mbox, 250, 100);
                lv_msgbox_start_auto_close(mbox, 5000);
                //lv_obj_set_event_cb(mbox, mbox_event_cb);
                printf("Connection successful\n");

                status = connected;
            } else {
                lv_obj_t * mbox = lv_msgbox_create(scan_tab, NULL);
                lv_msgbox_set_text(mbox, "Connection failed");
                lv_obj_set_pos(mbox, 35, 70);
                lv_obj_set_size(mbox, 250, 100);
                lv_msgbox_start_auto_close(mbox, 5000);
                //lv_obj_set_event_cb(mbox, mbox_event_cb);
                printf("Connection failed\n");

                //state = disconnected;
            }
        }
        confirm_button_state_prev = confirm_button_state;

        if(status == disconnected) {
            curr_hid_dev = NULL;
            lv_list_clean(list1);

            lv_tabview_set_tab_act(tabview, 0, LV_ANIM_OFF);

            // update scan label
            snprintf(scan_label_buffer,  sizeof(scan_label_buffer)-1, "Scanning...");
            lv_label_set_text(scan_label, scan_label_buffer);
            
            // start scanning task
            xTaskCreate(&scan_task, "scan_hid_task", 6 * 1024, NULL, 2, NULL);
            status = scanning;
        }

        if(status == scanning) {
            if(new_scan_results) {
                // update scan label
                snprintf(scan_label_buffer,  sizeof(scan_label_buffer)-1, "Found %u devices", scan_results_len);
                lv_label_set_text(scan_label, scan_label_buffer);

                printf("new scan results:\n");
                scan_result_entry = scan_results_head;
                while(scan_result_entry) {
                    printf("%s\n", scan_result_entry->name);
                    lv_list_add_btn(list1, NULL, scan_result_entry->name);
                    
                    scan_result_entry = scan_result_entry->next;
                }

                // set first entry as active
                active_button = lv_list_get_next_btn(list1, NULL);
                lv_list_focus_btn(list1, active_button);
                scan_result_entry = scan_results_head;
                active_entry_i = 0;
                new_scan_results = false;

                status = choosing_device;
            }
        }

        if(status == connected) {
            if(new_input) {
                ESP_LOG_BUFFER_HEX("New input received", input_buffer, sizeof(input_buffer));

                input_i = 2;
                equal_buffers = true;

                // check if buffer isn't empty                
                if(input_buffer[2] != 0) {                    
                    // skip same characters from previous buffer
                    while(input_i < sizeof(prev_input_buffer)) {
                        if(input_buffer[input_i] != prev_input_buffer[input_i]) {
                            equal_buffers = false;
                            break;
                        }
                        if(input_buffer[input_i] == 0) {
                            break;
                        }
                        input_i += 1;
                    }

                    // if buffers fully equal then repeat last action
                    if(equal_buffers && input_buffer[0] == prev_input_buffer[0]) {
                            //printf("Buffer repeat\n");
                            handle_keycode(last_modifier, last_keycode);              
                    } else {
                        // continue checking other keycodes
                        while(input_i < sizeof(input_buffer) && input_buffer[input_i] != 0) {            
                            // check if keycode wasn't on any position of previous buffer
                            if(!in_previous_buffer(input_buffer[input_i])) {
                                handle_keycode(input_buffer[0], input_buffer[input_i]);  // handle keycode                    
                            }
                            input_i += 1;
                        }
                    }
                }
                memcpy(prev_input_buffer, input_buffer, sizeof(input_buffer));
                new_input = false;
            }
        }

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }

        vTaskDelay(10 / portTICK_RATE_MS);
    }

    /* A task should NEVER return */
    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
}

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        //const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        //ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        //ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
                
        // handle keyboard input
        if(param->input.usage == 1 && param->input.length == 8) {
            memcpy(input_buffer, param->input.data, 8);
            new_input = true;        
        }
        
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);

        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        
        status = disconnected;
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

#define SCAN_DURATION_SECONDS 5

void scan_task(void *pvParameters)
{    
    // cleaning of previous data before scanning
    curr_hid_dev = NULL;
    esp_hid_scan_results_free(scan_results_head); // free the results
    scan_results_len = 0;
    active_entry_i = 0;    
    scan_results_head = NULL;
    scan_result_entry = NULL;
    
    while(scan_results_len == 0) {
        ESP_LOGI(TAG, "Start scanning...");
        
        //start scan for BLE HID devices
        esp_hid_scan(SCAN_DURATION_SECONDS, &scan_results_len, &scan_results_head);
        ESP_LOGI(TAG, "SCAN: %u results", scan_results_len);
        /*
        if (scan_results_len) {
            esp_hid_scan_result_t *r = scan_results_head;

            while (r) {
                printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
                printf("NAME: %s ", r->name ? r->name : "");
                printf("RSSI: %d, ", r->rssi);
                printf("USAGE: %s, ", esp_hid_usage_str(r->usage));

                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));

                //curr_hid_dev = esp_hidh_dev_open(r->bda, r->transport, r->ble.addr_type);
                
                printf("\n");
                r = r->next;
            }
        }
        */
    }

    new_scan_results = true;

    vTaskDelete(NULL);
}

void app_main(void)
{
    // start lvgl task
    xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, 0, NULL, 1);

    // init BLE hid host
    esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config) );
}
