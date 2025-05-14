#include <string.h>
#include <stdio.h>

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
#include "img_converters.h"
#include "esp_http_client.h"
#include "dl_tool.hpp"
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>


// WIFI SETTING
#define EXAMPLE_ESP_WIFI_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

// WEBSOCKET SETTING
#define WEBSOCKET_HOST "192.168.1.102"
#define WEBSOCKET_PORT 8000
#define WEBSOCKET_PATH "/api/device/ws/6e837227-93b7-461b-bc73-caa9828b7f26"
#define NO_DATA_TIMEOUT_SEC 43200 // 30 Days
static char MAC[18];
static esp_websocket_client_handle_t websocket_client;

// WEBSOCKET TASK STATUS
enum StatusType {
    STATUS_ERROR,
    STATUS_RECEIVED,
    STATUS_COMPLETED,
};

// WEBSOCKET Action
enum ActionType {
    ACTION_START_INFERENCE,
    ACTION_STOP_INFERENCE,
    ACTION_RETURN_INFERENCE,
    ACTION_MODE_SWITCH,
    ACTION_LOG_INFO
};

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
    uint8_t op_code;     // 換成 opcode       
    ActionType action;
    uint8_t* image_data;
    size_t image_len;
    std::vector<std::vector<int>> bounding_boxes;
    StatusType status;
} websocket_sending_message_t;

// HTTP Client
#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048

// Camera Setting
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
    .frame_size = FRAMESIZE_240X240,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 25, //0-63, for OV series camera sensors, lower number means higher quality
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
        ESP_LOGE("Camera", "Camera Init Failed: %d", err);
        return err;
    }

    return ESP_OK;
}
#endif

// SD Card Setting
#define MAX_READING_CHAR 64
#define MOUNT_POINT "/sdcard"

// Inference
#define TWO_STAGE 0 /*<! 1: detect by two-stage which is more accurate but slower(with keypoints). */
                    /*<! 0: detect by one-stage which is less accurate but faster(without keypoints). */

#define MIN(a,b) ((a) < (b) ? (a) : (b))


/*
    New Setting for switch mode
    // 怎樣算是匿名宣告
    // typedef 的意思
    // static 的意思與儲存位置
*/
enum OperationModeType {
    CONTINUOUS_MODE,
    STAND_BY_MODE,
    DEEP_SLEEP_MODE
};
static OperationModeType operation_mode = STAND_BY_MODE;
static EventGroupHandle_t g_system_event_group;
static SemaphoreHandle_t g_mode_mutex;
#define EVT_GRP__BIT_CONTINUOUS_MODE_ACTIVE BIT0
#define EVT_GRP__BIT_TRIGGER_SINGLE_SHOT BIT1


static std::vector<std::vector<int>> model_inference_task(uint8_t* image, uint8_t width, uint8_t height, uint8_t channel){
    dl::tool::Latency latency;
    // initialize
#if TWO_STAGE
    HumanFaceDetectMSR01 s1(0.01F, 0.1F, 10, 0.2F);
    HumanFaceDetectMNP01 s2(0.01F, 0.1F, 5);
#else // ONE_STAGE
    HumanFaceDetectMSR01 s1(0.01F, 0.1F, 10, 0.2F);
#endif

    // inference
    latency.start();
#if TWO_STAGE
    std::list<dl::detect::result_t> &candidates = s1.infer(image, {height, width, channel});
    std::list<dl::detect::result_t> &results = s2.infer(image, {height, width, channel}, candidates);
#else // ONE_STAGE
    std::list<dl::detect::result_t> &results = s1.infer(image, {height, width, channel});
#endif
    latency.end();
    latency.print("Inference latency");

    // display
    int i = 0;
    std::vector<std::vector<int>> bounding_boxes;
    for (std::list<dl::detect::result_t>::iterator prediction = results.begin() ; prediction != results.end(); prediction++, i++)
    {
        printf("[%d] score: %f, box: [%d, %d, %d, %d]\n", i, prediction->score, prediction->box[0], prediction->box[1], prediction->box[2], prediction->box[3]);
        
        std::vector<int> box_coords;
        box_coords.push_back(prediction->box[0]);
        box_coords.push_back(prediction->box[1]);
        box_coords.push_back(prediction->box[2]);
        box_coords.push_back(prediction->box[3]);
        bounding_boxes.push_back(prediction->box);
    }
    return bounding_boxes;
}



static char* action_enum_to_string(ActionType action){
    switch (action) {
        case ACTION_START_INFERENCE: return "START_INFERENCE";
        case ACTION_STOP_INFERENCE: return"STOP_INFERENCE";
        case ACTION_RETURN_INFERENCE: return "INFERENCE_RESULT";
        case ACTION_MODE_SWITCH: return "MODE_SWITCH";
        default: return "UNKNOWN_ACTION";   
    }
}



static char* status_enum_to_string(StatusType status){
    switch (status) {
        case STATUS_ERROR: return "ERROR";
        case STATUS_RECEIVED: return "RECEIVED";
        case STATUS_COMPLETED: return "COMPLETED";
        default: return "UNKNOWN_STATUS";
    }
}



static char* inference_message2json(ActionType action, StatusType status, std::vector<std::vector<int>> bounding_boxes){
    // Root
    char* action_str = action_enum_to_string(action);
    char* status_str = status_enum_to_string(status);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "action", action_str);
    cJSON_AddStringToObject(root, "status", status_str);

    // Content
    cJSON* content = cJSON_CreateObject();
    cJSON* bounding_boxes_array = cJSON_AddArrayToObject(content, "bounding_boxes");

    for(const auto& box: bounding_boxes){
        if (box.size() == 4){
            cJSON* box_array = cJSON_CreateArray();
            cJSON_AddItemToArray(box_array, cJSON_CreateNumber(box[0]));
            cJSON_AddItemToArray(box_array, cJSON_CreateNumber(box[1]));
            cJSON_AddItemToArray(box_array, cJSON_CreateNumber(box[2]));
            cJSON_AddItemToArray(box_array, cJSON_CreateNumber(box[3]));
            
            cJSON_AddItemToArray(bounding_boxes_array, box_array);
        }
    }

    cJSON_AddItemToObject(root, "content", content);
    char* result = cJSON_Print(root);
    cJSON_Delete(root);
    return result;
    
}



static char* control_message2json(ActionType action, StatusType status){
    char* action_str = action_enum_to_string(action);
    char* status_str = status_enum_to_string(status);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "action", action_str);
    cJSON_AddStringToObject(root, "status", status_str);

    char* result = cJSON_Print(root);
    cJSON_Delete(root);
    return result;
}



static void websocket_sending_task(void* pvParameters){
    websocket_sending_message_t msg;
    while(1){
        // Consider add thread block
        if(xQueueReceive(websocket_sending_message_queue, &msg, portMAX_DELAY) == pdTRUE){
            ESP_LOGI("Websocket Sending Task", "Sending message (type: %d)", msg.op_code);
            
            // If is Control Message
            if(msg.op_code == 0x01){

                char* serialized_message = control_message2json(msg.action, msg.status);
                if(serialized_message != NULL){
                    int len = strlen(serialized_message);
                    esp_websocket_client_send_text(websocket_client, serialized_message, len, portMAX_DELAY);
                    free(serialized_message);
                }
            }else{
            // If is Inference Message

                // --- Measure sending time
                // int64_t send_start = esp_timer_get_time();

                char* serialized_message = inference_message2json(msg.action, msg.status, msg.bounding_boxes);
                if(serialized_message == NULL){
                    free(msg.image_data);
                }else{
                    esp_websocket_client_send_bin(websocket_client,(char*)msg.image_data, msg.image_len, portMAX_DELAY);
                    free(msg.image_data);
                    int len = strlen(serialized_message);
                    esp_websocket_client_send_text(websocket_client, serialized_message, len, portMAX_DELAY);
                    free(serialized_message);
                }
                // int64_t send_end = esp_timer_get_time();
                // int64_t sending_time = (send_end - send_start)/1000;

                // if (result) {
                //     if( sending_time >= 150 ) {
                //         ESP_LOGW("Timing", "esp_websocket_client_send_bin took %lld ms for %d bytes", sending_time , msg.data_len);
                //     }else{
                //         ESP_LOGI("Timing", "esp_websocket_client_send_bin took %lld ms for %d bytes", sending_time , msg.data_len);
                //     }
                    
                //     ESP_LOGI("Websocket Sending Task", "sending binary");
                // }else {
                //     ESP_LOGW("Websocket Sending Task", "sending binary failed");
                // }
            }
        }            
    }
}



static void mode_switch(char* mode){
    if(xSemaphoreTake(g_mode_mutex, portMAX_DELAY) == pdTRUE ){   
        if(strcmp(mode, "CONTINUOUS_MODE") == 0){
            operation_mode = CONTINUOUS_MODE;
            xEventGroupSetBits(g_system_event_group, EVT_GRP__BIT_CONTINUOUS_MODE_ACTIVE);

        }else if(strcmp(mode, "STAND_BY_MODE") == 0){
            operation_mode = STAND_BY_MODE;
            xEventGroupClearBits(g_system_event_group, EVT_GRP__BIT_CONTINUOUS_MODE_ACTIVE);
            xEventGroupClearBits(g_system_event_group, EVT_GRP__BIT_TRIGGER_SINGLE_SHOT);
        }

        xSemaphoreGive(g_mode_mutex);
        ESP_LOGI("MODEL SWITCH", "Switch complete");
    }
}



static void websocket_process_task(void* pvParameters){
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
                        websocket_sending_message_t sending_msg = {0};
                        
                        if(strcmp(action->valuestring, "MODE_SWITCH") == 0){
                            cJSON* mode = cJSON_GetObjectItem(msg_json, "mode");
                            if(mode == NULL){
                                ESP_LOGW("Websocket", "Invalid task name");
                            }else{
                                // Send received notification, if using MQTT protocal does not do it.
                                sending_msg.action = ACTION_MODE_SWITCH;
                                sending_msg.status = STATUS_RECEIVED;
                                sending_msg.op_code = 0x01;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                mode_switch(mode->valuestring);
                            }

                        }else if(strcmp(action->valuestring, "INFERENCE") == 0){
                            xEventGroupSetBits(g_system_event_group, EVT_GRP__BIT_TRIGGER_SINGLE_SHOT);

                        }else{
                            ESP_LOGW("Websocket", "Invalid task name");
                        }
                        
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



static void take_picture(){
    ESP_LOGI("Camera", "Taking picture...");
    // 當 JPEG 的壓縮率太低時，會造成寫入溢出 FB-OVF，而當 FB-OVF 發生卻又沒有清除，造成 NO-EOI，並形成 recursive，最後發生 stack overflow
    camera_fb_t* pic = esp_camera_fb_get();
    if(pic == NULL){
        ESP_LOGE("Camera", "Take picture failed");
        return;
    }
    
    ESP_LOGI("Camera", "Picture taken! Its size was: %zu bytes, width: %zu, height: %zu.", pic->len, pic->width, pic->height);

    // if websocket does't connected, and then release resouces and continues
    bool websocket_connected = esp_websocket_client_is_connected(websocket_client);
    if(!websocket_connected){
        ESP_LOGE("Websocket", "Websocket doesn't connected.");
        esp_camera_fb_return(pic);
        return;
    }

    // allocate heap for RGB565 format, if failed to allocate, released resources and continues
    size_t rgb_len = pic->height*pic->width*2;
    uint8_t* rgb_buffer = (uint8_t*)malloc(rgb_len);
    if(rgb_buffer == NULL){
        ESP_LOGE("Camera", "RGB Buffer allocate failed");
        esp_camera_fb_return(pic);
        return;
    }

    // convert image format from JPEG to RGB565, if failed convert, released resources and continues
    bool conversion_success = jpg2rgb565(pic->buf, pic->len, rgb_buffer, JPG_SCALE_NONE);
    if(!conversion_success){
        ESP_LOGE("Camera", "Image converstion failed.");
        free(rgb_buffer);
        esp_camera_fb_return(pic);
        rgb_buffer=NULL;
        return;
    }

    // model inference, when inference completed, release memory about RGB565
    std::vector<std::vector<int>> bounding_boxes = model_inference_task(rgb_buffer, pic->width, pic->height, 3);
    free(rgb_buffer);
    rgb_buffer=NULL;


    // allocate heap for JPEG buffer, in oder to avoid null pointer reference in websocket_sending_message_queue
    // if failed, released reousrces
    uint8_t* jpeg_buffer = (uint8_t*)malloc(pic->len+1);
    if(jpeg_buffer == NULL){
        ESP_LOGE("Camera", "JPEG Buffer allocate failed");
        esp_camera_fb_return(pic);
        return;
    }

    // memory copy
    memcpy(jpeg_buffer, pic->buf, pic->len);

    // prepare sending message struct
    websocket_sending_message_t msg = {0};
    msg.op_code = 0x02;
    msg.image_len = pic->len;
    msg.image_data = jpeg_buffer;
    msg.bounding_boxes = std::move(bounding_boxes);
    msg.action = ACTION_RETURN_INFERENCE;
    msg.status = STATUS_COMPLETED;
    BaseType_t send_result = xQueueSend(websocket_sending_message_queue, &msg, 0);

    if( send_result != pdPASS ){
        free(jpeg_buffer);
        jpeg_buffer = NULL;
    }

    esp_camera_fb_return(pic);
}



static void camera_control_task(void* pvParameters){
    if(ESP_OK != init_camera()){
        ESP_LOGE("Camera", "INIT failed");
        vTaskDelete(NULL);
    }

    while(1) {
        OperationModeType local_mode = STAND_BY_MODE;
        if(xSemaphoreTake(g_mode_mutex, portMAX_DELAY) == pdTRUE){
            local_mode = operation_mode;
            xSemaphoreGive(g_mode_mutex);
        }

        if(local_mode == CONTINUOUS_MODE){
            ESP_LOGI("Camera", "Continuous mode trigger");
            EventBits_t bits = xEventGroupGetBits(g_system_event_group);
            if(bits & EVT_GRP__BIT_CONTINUOUS_MODE_ACTIVE){
                take_picture();
                vTaskDelay(10000 / portTICK_PERIOD_MS);
            }else{
                vTaskDelay(50);
            }
            
        }else if (local_mode == STAND_BY_MODE){
            ESP_LOGI("Camera", "Stand by mode trigger");
            EventBits_t bits = xEventGroupWaitBits(
                g_system_event_group,
                EVT_GRP__BIT_CONTINUOUS_MODE_ACTIVE | EVT_GRP__BIT_TRIGGER_SINGLE_SHOT,
                pdFALSE, 
                pdFALSE,
                portMAX_DELAY
            );

            if(bits & EVT_GRP__BIT_TRIGGER_SINGLE_SHOT){
                take_picture();
                xEventGroupClearBits(g_system_event_group, EVT_GRP__BIT_TRIGGER_SINGLE_SHOT);
            }
        }

    }
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
    char websocket_path[70];
    snprintf(websocket_path, size_t(websocket_path), "%s/%s", WEBSOCKET_PATH, MAC);
    ESP_LOGI("Websocket", "Path: %s", websocket_path);

    esp_websocket_client_config_t websocket_cfg = {};
    websocket_cfg.host = WEBSOCKET_HOST;
    websocket_cfg.port = WEBSOCKET_PORT;
    websocket_cfg.path = websocket_path;
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

    // Create RTOS event group and semaphore
    g_system_event_group = xEventGroupCreate();
    g_mode_mutex = xSemaphoreCreateBinary();

    if(g_mode_mutex == NULL){
        ESP_LOGE("Semahore", "create failed...");
    }else{
        xSemaphoreGive(g_mode_mutex);
    }

    if(g_system_event_group == NULL){
        ESP_LOGE("Event Group", "create failed...");
    }

    // Create websocket process task
    xTaskCreatePinnedToCore(websocket_process_task,"Websocket process task", 2048, NULL, 6, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(websocket_sending_task, "Websocket sending task", 4096, NULL, 6, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(camera_control_task, "Camera control task", 4096, NULL, 2, NULL, tskNO_AFFINITY);
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



static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if(event_base == WIFI_PROV_EVENT){

        switch(event_id){
            case WIFI_PROV_START:{
                ESP_LOGI("WIFI_PROV Event", "Provisioning started");
                break;

            }case WIFI_PROV_CRED_RECV: {
                wifi_sta_config_t* wifi_sta_config = (wifi_sta_config_t*)event_data;
                ESP_LOGI("WIFI_PROV Event", "Received Wi-Fi credentials"
                    "\n\tSSID     : %s\n\tPassword : %s",
                    (const char *) wifi_sta_config->ssid,
                    (const char *) wifi_sta_config->password);
                break;

            }case WIFI_PROV_CRED_FAIL: {
                wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t*)event_data;
                ESP_LOGE("WIFI_PROV Event", "Provisioning failed!\n\tReason : %s"
                    "\n\tPlease reset to factory and retry provisioning",
                    (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
                    "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
                
                // Retry
                break;
            
            }case WIFI_PROV_CRED_SUCCESS: {
                ESP_LOGI("WIFI_PROV Event", "{rpvisioning successful}");
                break;
            
            }case WIFI_PROV_END: {
                wifi_prov_mgr_deinit();
            }

        }
    
    }else if(event_base == WIFI_EVENT) {

        switch (event_id){
            case WIFI_EVENT_STA_START: {
                esp_wifi_connect();          
                break;
            
            }case WIFI_EVENT_STA_DISCONNECTED: {
                if(s_retry_num < EXAMPLE_ESP_WIFI_MAXIMUM_RETRY){
                    ESP_LOGI("WIFI Event", "Disconnected. Connecting to the AP again... \n");
                    s_retry_num++;
                    esp_wifi_connect();
                }else{
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                ESP_LOGI("WIFI", "Wifi station connect to the AP fail.\n");
            
            }case WIFI_EVENT_AP_STACONNECTED: {
                ESP_LOGI("WIFI Event", "SoftAP transport: Connected!");
                break;
            
            }case WIFI_EVENT_AP_STADISCONNECTED: {
                ESP_LOGI("WIFI Event", "SoftAP transport: Disconnected!");
                break;
            }
        }
    
    }else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP ) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI("IP Event", "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    
    }else if(event_base == PROTOCOMM_SECURITY_SESSION_EVENT) {
        switch(event_id){
            case PROTOCOMM_SECURITY_SESSION_SETUP_OK: {
                ESP_LOGI("Security Event", "Secured session established!");
                break;
            }
            case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS: {
                ESP_LOGI("Security Event", "Received invalid secrity parameters for establishing secure session!");
                break;
            
            }case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH: {
                ESP_LOGI("Security Event", "Received incorrect username and/or Pop for establishing secure session!");
                break;

            }default:
                break;
        }

    }
}



static void get_device_service_name(char* service_name, size_t max){
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X", ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}



esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t* inbuf, ssize_t inlen, uint8_t** outbuf, ssize_t* outlen, void* priv_data){
    if (inbuf){
        ESP_LOGI("WIFI_PROV", "Received data: %.*s", inlen, (char*)inbuf);
    }

    char response[] = "SUCCESS";
    *outbuf = (uint8_t*)strdup(response);
    if(*outbuf == NULL){
        ESP_LOGE("WIFI_PROV", "Response out of memory");
        return ESP_ERR_NO_MEM;
    }
    *outlen = strlen(response)+1;
    return ESP_OK;
}



void wifi_init_task(){
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());                     // create netif to init TCP/IP stack
    ESP_ERROR_CHECK(esp_event_loop_create_default());      // create event_loop for event_handle

    // Register Different type of Event
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    esp_netif_create_default_wifi_sta();                  // create abstract esp_netif_t sta instance
    esp_netif_create_default_wifi_ap();                   // create abstract esp_netif_t softap instance
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_softap
    };

    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));
    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));   // Check if wifi credential has been stored in NVS

    if(!provisioned){
        ESP_LOGI("WIFI_PROV", "Starting provisioning");

        char service_name[12];
        get_device_service_name(service_name, sizeof(service_name));  // Fetch WiFi SSID


        wifi_prov_security_t secuirty = WIFI_PROV_SECURITY_1;        
        const char* pop = CONFIG_ESP_WIFI_POP;
        wifi_prov_security1_params_t* sec_params = pop;
        const char* service_key = CONFIG_ESP_WIFI_PASSWORD;

        /* Start provisioning service */
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(secuirty, (const void*)sec_params, service_name, service_key));
        wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL);         // Setting Custom Data

    }else{
        ESP_LOGI("WIFI_PROV", "Already provisioned, starting Wi-Fi STA");
        wifi_prov_mgr_deinit();
        
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );

    if (bits & WIFI_CONNECTED_BIT) {
        uint8_t mac[6];
        esp_err_t err = esp_efuse_mac_get_default(mac);
        if(err != ESP_OK){
            ESP_LOGE("MAC", "Get Mac address failed\n");
        }else{
            snprintf(MAC, sizeof(MAC), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            ESP_LOGI("MAC", "MAC Address is: %s", MAC);
            websocket_init_task();
        }
        
    }
}



static void nvs_init(){
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}



extern "C" void app_main(void){
    /**
     * 
     * 1. 完成不同 mode 的省電設置
     * 2. 完成 OTA
     * 3. 完成在 ESP32-S3 的 Model Deployment
     * 4. Websocket reconnected
     * 
     */
    nvs_init();
    wifi_init_task();
    
}
