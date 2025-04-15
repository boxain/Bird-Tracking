#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"            
#include "freertos/event_groups.h"
#include "freertos/semphr.h" 

#include "esp_chip_info.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

#include <cJSON.h>

#include "esp_camera.h"
#include "esp_http_client.h"


/*
    WIFI SETTING
*/
#define EXAMPLE_ESP_WIFI_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;

/*
    WEBSOCKET SETTING
*/
#define WEBSOCKET_HOST "192.168.1.103"
#define WEBSOCKET_PORT 8000
#define WEBSOCKET_PATH "/api/device/ws/test_mac"
#define NO_DATA_TIMEOUT_SEC 43200 // 30 Days
static esp_websocket_client_handle_t websocket_client;
static EventGroupHandle_t s_wifi_event_group;

// WEBSOCKET RECEIVED MESSAGE QUEUE
static QueueHandle_t  websocket_received_message_queue;
typedef struct {
    uint8_t op_code;
    char* data;
    int data_len;
} websocket_received_message_t;

// WEBSOCKET SENDING MESSAGE QUEUE
static QueueHandle_t  websocket_sending_message_queue;
typedef struct {
    uint8_t op_code;
    char* text_data;
    uint8_t* binary_data;
    int data_len;
    enum {
        ERROR,
        RECEIVED,
        COMPLETED,
    } status;
} websocket_sending_message_t;


/*
    HTTP CLIENT
*/
#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048

/*
    CAMERA
*/
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 30, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE("Camera", "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}
#endif


/*
    SD CARD
*/

#define MAX_READING_CHAR 64
#define MOUNT_POINT "/sdcard"




#define MIN(a,b) ((a) < (b) ? (a) : (b))


static void websocket_sending_task(){
    websocket_sending_message_t msg;
    while(1){
        // Consider add thread block
        if(xQueueReceive(websocket_sending_message_queue, &msg, portMAX_DELAY) == pdTRUE){
            ESP_LOGI("Websocket Sending Task", "Sending message (opcode: %d), (data_len: %d)", msg.op_code, msg.data_len);
            
            if(msg.op_code == 0x01){

                char sending_text[50];
                snprintf(sending_text, 50, "{\"action\":%s,\"status\":%d}", msg.text_data, msg.status);
                int len = strlen(sending_text);
                free(msg.text_data);
                ESP_LOGI("Websocket Sending Task", "sending text: %s, length: %d", sending_text, len);
                esp_websocket_client_send_text(websocket_client, sending_text, len, portMAX_DELAY);

            }else{
                // --- Measure sending time
                int64_t send_start = esp_timer_get_time();
                int result = esp_websocket_client_send_bin(websocket_client, (char *)msg.binary_data, msg.data_len, portMAX_DELAY);
                int64_t send_end = esp_timer_get_time();
                int64_t sending_time = (send_end - send_start)/1000;

                if (result) {
                    if( sending_time >= 150 ) {
                        ESP_LOGW("Timing", "esp_websocket_client_send_bin took %lld ms for %d bytes", sending_time , msg.data_len);
                    }else{
                        ESP_LOGI("Timing", "esp_websocket_client_send_bin took %lld ms for %d bytes", sending_time , msg.data_len);
                    }
                    
                    ESP_LOGI("Websocket Sending Task", "sending binary");
                }else {
                    ESP_LOGW("Websocket Sending Task", "sending binary failed");
                }
                free(msg.binary_data);
            }
        }            
    }
}


static void websocket_process_task(){
    /*
        CJSON: 
        https://blog.csdn.net/Mculover666/article/details/103796256
    */

    websocket_received_message_t msg;
    while(1){
        if(xQueueReceive(websocket_received_message_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI("Websocket", "Received message (opcode: %d), (data_len: %d)", msg.op_code, msg.data_len);

            if(msg.op_code == 0x01) {
               
                cJSON* msg_json = cJSON_Parse(msg.data);
                if(msg_json == NULL) {
                    ESP_LOGE("Websocket", "Fail to sparse message into json format");
                }else{
                    cJSON* action = cJSON_GetObjectItem(msg_json, "action");
                    if(action && cJSON_IsString(action)) {
                        ESP_LOGI("Websocket", "Task: %s", action->valuestring);
                        char* action_printed = cJSON_Print(action);
                        int action_printed_len = strlen(action_printed);

                        websocket_sending_message_t sending_msg = {0};
                        sending_msg.text_data = malloc(action_printed_len+1); 
        
                        if(sending_msg.text_data == NULL){
                            ESP_LOGE("Websocket", "Fail to allocate websocket sending message data");
                            free(sending_msg.text_data);
                        }else{
                            
                            strcpy(sending_msg.text_data, action_printed);
                            sending_msg.op_code = 0x01;
                            sending_msg.data_len = action_printed_len;
                            sending_msg.status = RECEIVED;
                            xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                        
                        }
                        free(action_printed);
                        
                    }else{
                        ESP_LOGE("Websocket", "Message isn't right format");
                    }
                    cJSON_Delete(msg_json);
                }

            }else{
                ESP_LOGI("Websocket", "Received binary data");
            }

            free(msg.data);
        
        }
    }
}


static void camera_init_task(){
    #if ESP_CAMERA_SUPPORTED
        if(ESP_OK != init_camera()){
            return;
        }

        // --- FPS counter ---
        uint32_t frame_count = 0;
        int64_t last_fps_time = esp_timer_get_time();

        while(1) {
            ESP_LOGI("Camera", "Taking picture...");
            // 當 JPEG 的壓縮率太低時，會造成寫入溢出 FB-OVF，而當 FB-OVF 發生卻又沒有清除，造成 NO-EOI，並形成 recursive，最後發生 stack overflow
            // 後續補 merge request
            camera_fb_t* pic = esp_camera_fb_get();
            // 補 Null 判斷
            ESP_LOGI("Camera", "Picture taken! Its size was: %zu bytes, width: %zu, height: %zu.", pic->len, pic->width, pic->height);
            typedef struct {
                uint8_t op_code;
                char* text_data;
                uint8_t* binary_data;
                int data_len;
                enum {
                    ERROR,
                    RECEIVED,
                    COMPLETED,
                } status;
            } websocket_sending_message_t;

            uint8_t* binary_data = malloc(pic->len);
            if (binary_data == NULL){
                ESP_LOGE("Camera", "Failed to allocate heap for camera");
                esp_camera_fb_return(pic);
            }else{
                // if websocket does't connected, and then release resouces and continues
                bool websocket_connected = esp_websocket_client_is_connected(websocket_client);

                if(websocket_connected){

                    memcpy(binary_data, pic->buf, pic->len);
                    websocket_sending_message_t msg = {0};
                    msg.op_code = 0x02;
                    msg.data_len = pic->len;
                    msg.binary_data = binary_data;
                    msg.status = COMPLETED;
                    
                    // --- Measure Queue sending cost time
                    int64_t queue_start_time = esp_timer_get_time();
                    BaseType_t send_result = xQueueSend(websocket_sending_message_queue, &msg, 0);
                    int64_t queue_end_time = esp_timer_get_time();
                    int64_t queue_cost = queue_end_time - queue_start_time;
    
                    if( send_result == pdPASS ){
                        frame_count++;
                        ESP_LOGI("FPS", "Queue sending cost: %lld millseconds", queue_cost / 1000);
                    }else{
                        free(binary_data);
                    }
                    
                }else{
                    ESP_LOGE("Websocket", "Websocket doesn't connected.");
                    free(binary_data);
                }
                
                esp_camera_fb_return(pic);
            }

            // Measure FPS & Sending Queue spaces
            int64_t now  = esp_timer_get_time();
            int64_t elapsed_us = now - last_fps_time;
            if (elapsed_us >= 5000000) {
                float fps = (float) frame_count / (elapsed_us / 1000000.0f);
                ESP_LOGI("FPS", "Average FPS over last %lld ms: %.2f", elapsed_us / 1000, fps);
                frame_count = 0;
                last_fps_time = now;

                UBaseType_t remain_queue = uxQueueSpacesAvailable(websocket_sending_message_queue);
                ESP_LOGI("Sending Queue", "Sending queue spaces remaining: %d", remain_queue);

            }
        }
    #else
        ESP_LOGE("Camera", "Camera support is not available for this chip");
        return;
    #endif
}


static void log_error_if_nonzero(const char *message, int error_code){
    if(error_code != 0) {
        ESP_LOGE("Websocket", "Last error %s: 0x%x", message, error_code);
    }
}

  
static void websocket_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void* event_data){

    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch(event_id) {
        case WEBSOCKET_EVENT_BEGIN:
            ESP_LOGI("Websocket", "WEBSOCKET_EVENT_BEGIN");
            break;
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI("Websocket", "WEBSOCKET_EVENT_CONNECTED");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI("Websocket", "WEBSOCKET_EVENT_DISCONNECTED");
            if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  data->error_handle.esp_transport_sock_errno);
            }
            break;
        case WEBSOCKET_EVENT_DATA:
            ESP_LOGI("Websocket", "WEBSOCKET_EVENT_DATA");
            if(data->op_code == 0x01 || data->op_code == 0x02){
                websocket_received_message_t msg;
                msg.data = (char*) malloc(data->data_len+1);
                if(msg.data == NULL){
                    ESP_LOGE("Websocket", "Fail to allocate websocket message data");
                    free(msg.data);
                }
                msg.data[data->data_len] = '\0';
                memcpy(msg.data, data->data_ptr, data->data_len);
                msg.data_len = data->data_len;
                msg.op_code = data->op_code;

                if(xQueueSend(websocket_received_message_queue, &msg, portMAX_DELAY) != pdTRUE){
                    ESP_LOGE("Websocket", "Fail sending message to queue");
                    free(msg.data);
                }


            }else if(data->op_code == 0x09){
                ESP_LOGW("Websocket","Received PING message");
            }else if(data->op_code == 0x08 && data->data_len == 2) {
                ESP_LOGW("Websocket", "Received closed frame with code=%d", 256 * data->data_ptr[0] + data->data_ptr[1]);
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGI("Websocket", "WEBSOCKET_EVENT_ERROR");
            log_error_if_nonzero("HTTP status code", data->error_handle.esp_ws_handshake_status_code);
            if(data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGI("Websocket", "ERROR REASON: WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT");

            }else if(data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_PONG_TIMEOUT){
                ESP_LOGI("Websocket", "ERROR REASON: WEBSOCKET_ERROR_TYPE_PONG_TIMEOUT");

            }else if(data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_HANDSHAKE){
                ESP_LOGI("Websocket", "ERROR REASON: WEBSOCKET_ERROR_TYPE_HANDSHAKE");
            }
            log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", data->error_handle.esp_transport_sock_errno);
            break;
        case WEBSOCKET_EVENT_FINISH:
            ESP_LOGI("Websocket", "WEBSOCKET_EVENT_FINISHE");
            break;
        
    }
}


static void websocket_init_task(){

    // Setting Websockt config
    esp_websocket_client_config_t websocket_cfg = {};
    websocket_cfg.host = WEBSOCKET_HOST;
    websocket_cfg.port = WEBSOCKET_PORT;
    websocket_cfg.path = WEBSOCKET_PATH;
    websocket_cfg.disable_auto_reconnect = false;

    // Init Websocket client
    websocket_client = esp_websocket_client_init(&websocket_cfg);

    // Can using esp_event_handler_instance_register to replace
    // https://docs.espressif.com/projects/esp-idf/zh_CN/v5.4/esp32/api-reference/system/esp_event.html
    esp_websocket_register_events(websocket_client, WEBSOCKET_EVENT_ANY, websocket_handler, (void*)websocket_client);
    esp_websocket_client_start(websocket_client);

    // Create websocket message queue
    websocket_received_message_queue = xQueueCreate(10, sizeof(websocket_received_message_t));
    if (websocket_received_message_queue == NULL) {
        ESP_LOGE("Websocket", "Failed to create received message queue");
        vTaskDelete(NULL);
    }
    websocket_sending_message_queue = xQueueCreate(20, sizeof(websocket_sending_message_t));
    if (websocket_sending_message_queue == NULL) {
        ESP_LOGE("Websocket", "Failed to create sending message queue");
        vTaskDelete(NULL);
    }

    // Create websocket process task
    xTaskCreatePinnedToCore(websocket_process_task,"Websocket process task", 4096, NULL, 6, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(websocket_sending_task, "Websocket sending task", 4096, NULL, 6, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(camera_init_task, "Camera init", 4096, NULL, 2, NULL, tskNO_AFFINITY);
    
}


static void flash_init(){
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}


static void wifi_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
        esp_wifi_connect();
        // init websocket
    } else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // clear websocket resource
        if(s_retry_num < EXAMPLE_ESP_WIFI_MAXIMUM_RETRY){
            esp_wifi_connect();
            s_retry_num++;
            printf("Wifi station retry to connect to the AP.\n");
        }else{
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        printf("Wifi station connect to the AP fail.\n");
    
    } else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI("Wifi station", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }

}


static void wifi_init_task(){
    // step 1.
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());                     // create netif
    ESP_ERROR_CHECK(esp_event_loop_create_default());      // create event_loop for event_handle
    
    // step 2. 
    esp_netif_create_default_wifi_sta();                   // create abstract esp_netif_t instance
    

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();   // init WiFi driver
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(   // register event_handler
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_handler,
        NULL,
        &instance_any_id
    ));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_handler,
        NULL,
        &instance_got_ip
    ));

    
    printf("WiFi SSID: %s, PASSWORD: %s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));                 // setting WiFi driver
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));   // setting WiFi driver
    ESP_ERROR_CHECK(esp_wifi_start());                                 // start WiFi driver


    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("Wifi station", "connected to ap SSID:%s password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
        websocket_init_task();
    } else if(bits & WIFI_FAIL_BIT) {
        ESP_LOGI("Wifi station", "Failed to connect to SSID:%s, password:%s",
            CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    } else{
        ESP_LOGE("Wifi station", "UNEXPECTED EVENT");
    }

}


static esp_err_t sdmmc_card_write_file(const char* path, char* data){
    ESP_LOGI("SD Card", "Opening file %s", path);

    FILE* f = fopen(path, "w");
    if( f == NULL ){
        ESP_LOGE("SD Card", "Failed to open file for writing");
        return ESP_FAIL;
    }

    fprintf(f, data);
    fclose(f);
    ESP_LOGI("SD Card", "File written");

    return ESP_OK;
}


static esp_err_t sdmmc_card_read_file(const char* path){
    ESP_LOGI("SD Card", "Reading file %s", path);

    FILE* f = fopen(path, "r");
    if( f == NULL ){
        ESP_LOGE("SD Card", "Failed to open file for reading");
        return ESP_FAIL;
    }
    
    char line[MAX_READING_CHAR];
    fgets(line, sizeof(line), f);
    fclose(f);

    char* pos = strchr(line, '\n');
    if(pos){
        *pos = '\0'; // ?
    }
    ESP_LOGI("SD Card", "Read from file: '%s'", line);
    
    return ESP_OK;
}


static void sdmmc_card_init_task(){
    esp_err_t ret;

    // Options for mounting the filesystem.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 3,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI("SD Card", "Initializing SD card");
    ESP_LOGI("SD Card", "Using SDMMC peripheral");


    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 40MHz for SDMMC)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
#if CONFIG_EXAMPLE_SDMMC_SPEED_HS
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR50
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_SDR50;
    host.flags &= ~SDMMC_HOST_FLAG_DDR
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_DDR50
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_DDR50;
#endif


    // For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
    // When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
    // and the internal LDO power supply, we need to initialize the power supply first.
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNEAL_IO
    sd_pwr_ctrl_config_t ldo_config = {
        .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif


    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
#if EXAMPLE_IS_UHS1
    slot_config.flags |= SDMMC_SLOT_FLAG_UHS1;
#endif

#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
    slot_config.width = 4;
#else
    slot_config.width = 1;
#endif


    // On chips where the GPIOs used for SD card can be configured, set them in
    // the slot_config structure:
#ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
    slot_config.clk = CONFIG_EXAMPLE_PIN_CLK;
    slot_config.cmd = CONFIG_EXAMPLE_PIN_CMD;
    slot_config.d0 = CONFIG_EXAMPLE_PIN_D0;
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
    slot_config.d1 = CONFIG_EXAMPLE_PIN_D1;
    slot_config.d2 = CONFIG_EXAMPLE_PIN_D2;
    slot_config.d3 = CONFIG_EXAMPLE_PIN_D3;
#endif
#endif
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;


    ESP_LOGI("SD Card", "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);


    if(ret != ESP_OK){
        if(ret == ESP_FAIL){
            ESP_LOGE("SD Card", "Failed to mount filesystem.");
        }else{
            ESP_LOGE("SD Card", "Failed to initialize the card (%s). Error Number: (%s)", 
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
            check_sd_card_pins(&config, pin_count);
#endif
        }
        return;
    }
    // Card has been initialized, print its properties
    ESP_LOGI("SD Card", "Filesystem mounted");
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files
    // Create a file.
    const char* file_hello = MOUNT_POINT"/hello.txt";
    char data[MAX_READING_CHAR];
    snprintf(data, MAX_READING_CHAR, "%s %s!\n", "Hello", card->cid.name);
    ret = sdmmc_card_write_file(file_hello, data);
    if(ret != ESP_OK){
        return;
    }


    const char* file_foo = MOUNT_POINT"/foo.txt";
    // Check if destination file exist before renaming
    struct stat st;
    if(stat(file_foo, &st) == 0){
        // Delete it if it exists
        unlink(file_foo);
    }

    // Rename original file
    ESP_LOGI("SD Card", "Renaming file %s to %s", file_hello, file_foo);
    if(rename(file_hello, file_foo) != 0){
        ESP_LOGE("SD Card", "Rename failed");
        return;
    }

    ret = sdmmc_card_read_file(file_foo);
    if(ret != ESP_OK){
        return;
    }


    // Format FATFS
#ifdef CONFIG_EXAMPLE_FORMAT_SD_CARD
    ret = esp_vfs_fat_sdcard_format(mount_point, card);
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to format FATFS (%s)", esp_err_to_name(ret));
        return;
    }

    if(stat(file_foo, &st) == 0){
        ESP_LOGI("SD Card", "file still exists");
        return;
    }else{
        ESP_LOGI("SD Card", "file doesn't exist, formatting done");
    }
#endif

    const char* file_nihao = MOUNT_POINT"/nihao.txt";
    memset(data, 0, MAX_READING_CHAR);
    snprintf(data, MAX_READING_CHAR, "%s %s!\n", "Nihao", card->cid.name);
    ret = sdmmc_card_write_file(file_nihao, data);
    if(ret != ESP_OK){
        return;
    }

    ret = sdmmc_card_read_file(file_nihao);
    if(ret != ESP_OK){
        return;
    }

    // All done, unmount partition and disable SDMMC peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI("SD Card", "Card unmounted");

    // Deinitialize the power control driver if it was used
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    ret = sd_pwr_ctrl_del_on_chip_ldo(pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete the on-chip LDO power control driver");
        return;
    }
#endif
}


void app_main(void){
    /**
     * 方案 1: 使用 MJPEG 格式傳輸照片，並使用 Websocket 傳輸推論結果
     * 
     * 方案 2: 使用 Websocket 全程傳輸 ( 優先採納 )
     * 
     */

     /**
      * 待辦事項
      * 
      * 1. 串接 SD 卡
      * 2. 串接 model inference
      * 3. 使用 GPIO 33 通知 WiFi 是否連線
      * 
      */
    flash_init();
    sdmmc_card_init_task();
    wifi_init_task();
}


