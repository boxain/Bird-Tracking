#include <string.h>
#include <stdio.h>
#include <cJSON.h>

#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"            
#include "freertos/event_groups.h"
#include "freertos/semphr.h" 

#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_pm.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"

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
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>

#include "esp_websocket_client.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"

#include "dl_model_base.hpp"
#include "cat_detect.hpp"


// WIFI SETTING
#define EXAMPLE_ESP_WIFI_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

// SERVER SETTING
#define SERVER_HOST "http://192.168.1.101:8000"

// HTTP SETTING
#define AUTH_HTTP_PATH "api/device"
typedef struct {
    char* data_buffer;
    int total_len;
    int downloaded_len;
} http_response_t;

// WEBSOCKET SETTING
#define WEBSOCKET_HOST "ws://192.168.1.101:8000"
#define WEBSOCKET_PATH "api/device/ws"
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
    ACTION_OTA,
    ACTION_MODEL_DOWNLOAD,
    ACTION_MODEL_SWITCH,
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
#define TASK_ID_MAX_LEN 37
static QueueHandle_t  websocket_sending_message_queue;
typedef struct {
    uint8_t op_code;    
    ActionType action;
    char task_id[TASK_ID_MAX_LEN];
    uint8_t* image_data;
    size_t image_len;
    std::list<dl::detect::result_t>* inference_result;
    StatusType status;
} websocket_sending_message_t;

// HTTP Client
#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048

// Camera Setting
// #define CAM_PIN_PWDN 32
// #define CAM_PIN_RESET -1 //software reset will be performed
// #define CAM_PIN_XCLK 0
// #define CAM_PIN_SIOD 26
// #define CAM_PIN_SIOC 27

// #define CAM_PIN_D7 35
// #define CAM_PIN_D6 34
// #define CAM_PIN_D5 39
// #define CAM_PIN_D4 36
// #define CAM_PIN_D3 21
// #define CAM_PIN_D2 19
// #define CAM_PIN_D1 18
// #define CAM_PIN_D0 5
// #define CAM_PIN_VSYNC 25
// #define CAM_PIN_HREF 23
// #define CAM_PIN_PCLK 22

#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1   //software reset will be performed
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_D0 11
#define CAM_PIN_D1 9
#define CAM_PIN_D2 8
#define CAM_PIN_D3 10
#define CAM_PIN_D4 12
#define CAM_PIN_D5 18
#define CAM_PIN_D6 17
#define CAM_PIN_D7 16

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
    .frame_size = FRAMESIZE_HVGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

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
sdmmc_card_t* card;

// Model Setting
CatDetect* model;
static SemaphoreHandle_t g_model_mutex;
typedef struct {
    FILE* file;
    int total_len;
    int downloaded_len;
    char* download_path;
    char* model_id;
    char task_id[TASK_ID_MAX_LEN];
} model_download_params_t ;


// Mode switch
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
#define EVT_GBP__BIT_OTA_IN_PROGRESS BIT2
#define EVT_GBP__BIT_OTA_COMPLETED BIT3
#define EVT_GBP__BIT_MODEL_DOWNLOAD_IN_PROGRESS BIT4
#define EVT_GBP__BIT_MODEL_DOWNLOAD_IN_COMPLETED BIT5
#define EVT_GBP__BIT_MODEL_SWITCH_IN_PROGRESS BIT6
#define EVT_GBP__BIT_MODEL_SWITCH_COMPLETED BIT7
#define ALL_IN_PROGRESS_BITS (EVT_GBP__BIT_OTA_IN_PROGRESS | EVT_GBP__BIT_MODEL_DOWNLOAD_IN_PROGRESS | EVT_GBP__BIT_MODEL_SWITCH_IN_PROGRESS)
#define ALL_COMPLETED_BITS (EVT_GBP__BIT_OTA_COMPLETED | EVT_GBP__BIT_MODEL_DOWNLOAD_IN_COMPLETED | EVT_GBP__BIT_MODEL_SWITCH_COMPLETED)


// Power saving
esp_pm_config_t pm_config;
esp_pm_lock_handle_t max_cpu_mz;
esp_pm_lock_handle_t max_apb_mz;

//OTA
#define HASH_LEN 32
typedef struct {
    char* download_path;
    char task_id[TASK_ID_MAX_LEN];
} ota_params_t ;

static std::list<dl::detect::result_t> model_inference(uint8_t* image, size_t len){
    dl::image::jpeg_img_t jpeg_img = {.data = (void *)image, .data_len = len};
    auto img = sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    
    auto &detect_results = model->run(img);
    for(const auto &res : detect_results){
        ESP_LOGI("MODEL INFERENCE",
                 "[category: %d, score: %f, x1: %d, y1: %d, x2: %d, y2: %d]",
                 res.category,
                 res.score,
                 res.box[0],
                 res.box[1],
                 res.box[2],
                 res.box[3]);
    }

    std::list<dl::detect::result_t> results_copy = detect_results;

    if(img.data != NULL){
        free(img.data);
        img.data = NULL;
    }
    
    return results_copy;
}



static void model_init(char* model_id, char* task_id){
    // Fetch semaphore first
    if(xSemaphoreTake(g_model_mutex, portMAX_DELAY) == pdTRUE){
        char path[128];
        sprintf(path, "%s/%s/%s/%s.espdl", MOUNT_POINT, "models", model_id, model_id);

        ESP_LOGI("MODEL SWITCH", "Starting model switch, path: %s", path);

        // Release old model 
        if(model){
            delete model;
            ESP_LOGI("MODEL SWITCH", "Release original model resources");
        }

        // Must setting FAT LFN ( Long File Name)
        model = new CatDetect(path);
        websocket_sending_message_t sending_msg = {0};
        if(model){
            ESP_LOGI("MODEL SWITCH", "Success");

            // Sending completed message
            sending_msg.op_code = 0x01;
            sending_msg.action = ACTION_MODEL_SWITCH;
            sending_msg.status = STATUS_COMPLETED;
            strncpy(sending_msg.task_id, task_id, TASK_ID_MAX_LEN-1);
            sending_msg.task_id[TASK_ID_MAX_LEN-1] = '\0';
            xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);

        }else{
            ESP_LOGE("MODEL SWITCH", "Failed");
            // Sending error message
            sending_msg.op_code = 0x01;
            sending_msg.action = ACTION_MODEL_SWITCH;
            sending_msg.status = STATUS_ERROR;
            strncpy(sending_msg.task_id, task_id, TASK_ID_MAX_LEN-1);
            sending_msg.task_id[TASK_ID_MAX_LEN-1] = '\0';
            xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
            // unmount partition and disable SDMMC peripheral
            // esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
            // ESP_LOGI("SD Card", "Card unmounted");
        }

        // Update event group to trigger other task
        xEventGroupClearBits(g_system_event_group, EVT_GBP__BIT_MODEL_SWITCH_IN_PROGRESS);
        xEventGroupSetBits(g_system_event_group, EVT_GBP__BIT_MODEL_SWITCH_COMPLETED);
        xSemaphoreGive(g_mode_mutex);
    }
}



static esp_err_t ensure_dir_exist(char* model_id){
    
    char model_root_path[128];
    sprintf(model_root_path, "%s/%s", MOUNT_POINT, "models");
    struct stat info;

    if(stat(model_root_path, &info) == -1){
        if(mkdir(model_root_path, 0755) != 0) {
            ESP_LOGE("MKDIR", "Failed to create directory %s.", model_root_path);
            return ESP_FAIL;
        }
        ESP_LOGI("MKDIR", "Success to create directory %s.", model_root_path);
    }else{
        if(!S_ISDIR(info.st_mode)){
            ESP_LOGE("MKDIR", "Path %s exists but is not a directory.", model_root_path);
            return ESP_FAIL;
        }
        ESP_LOGE("MKDIR", "Directory %s already exists.", model_root_path);
    }

    char model_sub_path[128];
    sprintf(model_sub_path, "%s/%s/%s", MOUNT_POINT, "models", model_id);
    if(stat(model_sub_path, &info) == -1){
        if(mkdir(model_sub_path, 0755) != 0) {
            ESP_LOGE("MKDIR", "Failed to create directory %s", model_sub_path);
            return ESP_FAIL;
        }
        ESP_LOGI("MKDIR", "Success to create directory %s", model_sub_path);
    }else{
        if(!S_ISDIR(info.st_mode)){
            ESP_LOGE("MKDIR", "Path %s exists but is not a directory.", model_sub_path);
            return ESP_FAIL;
        }
        ESP_LOGE("MKDIR", "Directory %s already exists.", model_sub_path);
    }

    return ESP_OK;

}



static esp_err_t check_file_is_exist(char* model_id){
    char model_full_path[128];
    sprintf(model_full_path, "%s/%s/%s/%s.espdl", MOUNT_POINT, "models", model_id, model_id);
    struct stat info;
    if(stat(model_full_path, &info) == -1){
        ESP_LOGE("FILE", "Model: %s does not exist.", model_full_path);
        return ESP_FAIL;
    }else{
        if(!S_ISREG(info.st_mode)){
            ESP_LOGE("FILE", "Path %s exists but is not a regular file.", model_full_path);
            return ESP_FAIL;
        }
    }
    ESP_LOGI("FILE", "Model file %s exists. Size: %ld bytes", model_full_path, info.st_size);
    return ESP_OK;
}



static char* action_enum_to_string(ActionType action){
    switch (action) {
        case ACTION_START_INFERENCE: return "START_INFERENCE";
        case ACTION_STOP_INFERENCE: return"STOP_INFERENCE";
        case ACTION_RETURN_INFERENCE: return "INFERENCE_RESULT";
        case ACTION_MODE_SWITCH: return "MODE_SWITCH";
        case ACTION_MODEL_DOWNLOAD: return "MODEL_DOWNLOAD";
        case ACTION_MODEL_SWITCH: return "MODEL_SWITCH";
        case ACTION_OTA: return "OTA";
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



static char* inference_message2json(ActionType action, StatusType status, std::list<dl::detect::result_t>* inference_results){
    // Root
    char* action_str = action_enum_to_string(action);
    char* status_str = status_enum_to_string(status);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "action", action_str);
    cJSON_AddStringToObject(root, "status", status_str);

    // Content
    cJSON* content = cJSON_CreateObject();
    cJSON* result_array = cJSON_AddArrayToObject(content, "inference_results");

    if(inference_results != NULL){
        for(const auto& inference_result: *inference_results){

            cJSON* result = cJSON_CreateObject();
            cJSON_AddNumberToObject(result, "category", inference_result.category);
            cJSON_AddNumberToObject(result, "score", inference_result.score);
            cJSON* box = cJSON_AddArrayToObject(result, "box");

            cJSON_AddItemToArray(box, cJSON_CreateNumber(inference_result.box[0]));
            cJSON_AddItemToArray(box, cJSON_CreateNumber(inference_result.box[1]));
            cJSON_AddItemToArray(box, cJSON_CreateNumber(inference_result.box[2]));
            cJSON_AddItemToArray(box, cJSON_CreateNumber(inference_result.box[3]));
                
            cJSON_AddItemToArray(result_array, result);
            
        }
    }

    cJSON_AddItemToObject(root, "content", content);
    char* result = cJSON_Print(root);
    cJSON_Delete(root);
    return result;
    
}



static char* control_message2json(ActionType action, StatusType status, char* task_id){
    char* action_str = action_enum_to_string(action);
    char* status_str = status_enum_to_string(status);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "action", action_str);
    cJSON_AddStringToObject(root, "status", status_str);
    cJSON_AddStringToObject(root, "task_id", task_id);

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

                char* serialized_message = control_message2json(msg.action, msg.status, msg.task_id);
                if(serialized_message != NULL){
                    int len = strlen(serialized_message);
                    esp_websocket_client_send_text(websocket_client, serialized_message, len, portMAX_DELAY);
                    free(serialized_message);
                }
            }else{
            // If is Inference Message

                char* serialized_message = inference_message2json(msg.action, msg.status, msg.inference_result);
                if (msg.inference_result != NULL) {
                    delete msg.inference_result;
                    msg.inference_result = NULL;
                }

                if(serialized_message == NULL){
                    if (msg.image_data != NULL) {
                        free(msg.image_data);
                        msg.image_data = NULL;
                    }
                }else{
                    esp_websocket_client_send_bin(websocket_client,(char*)msg.image_data, msg.image_len, portMAX_DELAY);
                    free(msg.image_data);
                    int len = strlen(serialized_message);
                    esp_websocket_client_send_text(websocket_client, serialized_message, len, portMAX_DELAY);
                    free(serialized_message);
                }
            }
        }            
    }
}



esp_err_t model_download_http_event_handler(esp_http_client_event_t* evt){

    model_download_params_t * params = (model_download_params_t *)evt->user_data;
    const char* tag = "MODEL DOWNLOAD HTTP EVENT";

    switch (evt->event_id){
        case HTTP_EVENT_ERROR:{
            ESP_LOGI(tag, "HTTP_EVENT_ERROR");
            break;
        }case HTTP_EVENT_ON_CONNECTED:{
            ESP_LOGI(tag, "HTTP_EVENT_ON_CONNECTED");
            break;
        }case HTTP_EVENT_HEADER_SENT:{
            ESP_LOGI(tag, "HTTP_EVENT_HEADER_SENT");
            break;
        }case HTTP_EVENT_ON_HEADER:{
            ESP_LOGI(tag, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            if(strcmp(evt->header_key, "content-length") == 0){
                params->total_len = atoi(evt->header_value);
                ESP_LOGI("HTTP_EVENT_ON_HEADER", "File size: %d bytes", params->total_len);
            }
            break;
        }case HTTP_EVENT_ON_DATA:{
            ESP_LOGI(tag, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if(params->file==NULL){
                esp_err_t is_dir_exist = ensure_dir_exist(params->model_id);
                if(is_dir_exist == ESP_FAIL){
                    return ESP_FAIL;
                }                
                char filepath[256];
                snprintf(filepath, sizeof(filepath), "%s/models/%s/%s.espdl", MOUNT_POINT, params->model_id, params->model_id);
                params->file = fopen(filepath, "wb");
                if(!params->file){
                    ESP_LOGE(tag, "Failed to open file %s for writing", filepath);
                }
                params->downloaded_len=0;
            }
            if(params->file){
                int written = fwrite(evt->data, 1, evt->data_len, params->file);
                if(written != evt->data_len){
                    ESP_LOGE(tag, "File wrote wrror, written %d, expected %d", written, evt->data_len);
                    return ESP_FAIL;
                }
                params->downloaded_len += written;
                if(params->total_len == params->downloaded_len){
                    fclose(params->file);
                }else if(params->total_len > 0) {
                    int progress = (params->downloaded_len * 100) / params->total_len;
                    ESP_LOGI(tag, "Downloading: %d/%d bytes (%d%%)", params->downloaded_len, params->total_len, progress);
                } else {
                    ESP_LOGI(tag, "Downloading: %d bytes", params->downloaded_len);
                }
            }
            break;
        }case HTTP_EVENT_ON_FINISH:{
            ESP_LOGD(tag, "HTTP_EVENT_ON_FINISH");
            break;
        }case HTTP_EVENT_DISCONNECTED:{
            ESP_LOGD(tag, "HTTP_EVENT_DISCONNECTED");
            break;
        }case HTTP_EVENT_REDIRECT:{
            ESP_LOGD(tag, "HTTP_EVENT_REDIRECT");
            break;
        }
    }
    return ESP_OK;
}



static void model_download_task(void *pvParameter){
    model_download_params_t *params = (model_download_params_t *)pvParameter;
    ESP_LOGI("MODEL_DOWNLOAD", "Starting model download task");
    ESP_LOGI("MODEL_DOWNLOAD", "Download path: %s with model id:%s", params->download_path, params->model_id);

    
    esp_http_client_config_t config = {
        .url = params->download_path,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 60000,
        .event_handler = model_download_http_event_handler,
        .buffer_size = 4096,
        .user_data = params,
        .skip_cert_common_name_check = true  
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    esp_err_t ret = esp_http_client_perform(client);
    websocket_sending_message_t sending_msg = {0};
    if(ret == ESP_OK){
        ESP_LOGI("MODEL_DOWNLOAD", "Success");
        // Sending completed message
        sending_msg.op_code = 0x01;
        sending_msg.action = ACTION_MODEL_DOWNLOAD;
        sending_msg.status = STATUS_COMPLETED;
        strncpy(sending_msg.task_id, params->task_id, TASK_ID_MAX_LEN-1);
        sending_msg.task_id[TASK_ID_MAX_LEN-1] = '\0';
        xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);

        // Trigger other task
        xEventGroupClearBits(g_system_event_group, EVT_GBP__BIT_MODEL_DOWNLOAD_IN_PROGRESS);
        xEventGroupSetBits(g_system_event_group, EVT_GBP__BIT_MODEL_DOWNLOAD_IN_COMPLETED);
    }else{
        ESP_LOGE("MODEL_DOWNLOAD", "Failed");
        ESP_LOGE("MODEL_DOWNLOAD", "Failed reason: %s", esp_err_to_name(ret));

        // Sending error message
        sending_msg.op_code = 0x01;
        sending_msg.action = ACTION_MODEL_DOWNLOAD;
        sending_msg.status = STATUS_ERROR;
        strncpy(sending_msg.task_id, params->task_id, TASK_ID_MAX_LEN-1);
        sending_msg.task_id[TASK_ID_MAX_LEN-1] = '\0';
        xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
    }

    esp_http_client_cleanup(client);
    free(params->model_id);
    free(params->download_path);


    vTaskDelete(NULL);
}



esp_err_t ota_http_event_handler(esp_http_client_event_t* evt){

    const char* tag = "OTA HTTP EVENT";
    switch (evt->event_id){
        case HTTP_EVENT_ERROR:{
            ESP_LOGI(tag, "HTTP_EVENT_ERROR");
            break;
        }case HTTP_EVENT_ON_CONNECTED:{
            ESP_LOGI(tag, "HTTP_EVENT_ON_CONNECTED");
            break;
        }case HTTP_EVENT_HEADER_SENT:{
            ESP_LOGI(tag, "HTTP_EVENT_HEADER_SENT");
            break;
        }case HTTP_EVENT_ON_HEADER:{
            ESP_LOGI(tag, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        }case HTTP_EVENT_ON_DATA:{
            ESP_LOGI(tag, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        }case HTTP_EVENT_ON_FINISH:{
            ESP_LOGD(tag, "HTTP_EVENT_ON_FINISH");
            break;
        }case HTTP_EVENT_DISCONNECTED:{
            ESP_LOGD(tag, "HTTP_EVENT_DISCONNECTED");
            break;
        }case HTTP_EVENT_REDIRECT:{
            ESP_LOGD(tag, "HTTP_EVENT_REDIRECT");
            break;
        }
    }
    return ESP_OK;
}



void ota_task(void *pvParameter){
    ota_params_t* params = (ota_params_t*) pvParameter;
    ESP_LOGI("OTA", "Starting OTA example task");
    ESP_LOGI("OTA", "Download path: %s", params->download_path);
    esp_http_client_config_t config = {
        .url = params->download_path,
        .event_handler = ota_http_event_handler,
        .skip_cert_common_name_check = true,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config
    };

    ESP_LOGI("OTA", "Attempting to download update from %s", config.url);
    esp_err_t ret = esp_https_ota(&ota_config);
    websocket_sending_message_t sending_msg = {0};
    if(ret == ESP_OK){
        ESP_LOGI("OTA", "OTA Succed, Rebooting");

        // Sending completed message
        sending_msg.op_code = 0x01;
        sending_msg.action = ACTION_OTA;
        sending_msg.status = STATUS_COMPLETED;
        strncpy(sending_msg.task_id, params->task_id, TASK_ID_MAX_LEN-1);
        sending_msg.task_id[TASK_ID_MAX_LEN-1] = '\0';
        xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);

        xEventGroupClearBits(g_system_event_group, EVT_GBP__BIT_OTA_IN_PROGRESS);
        xEventGroupSetBits(g_system_event_group, EVT_GBP__BIT_OTA_COMPLETED);
    }else{
        ESP_LOGI("OTA", "Firmware upgrade failed");
        ESP_LOGW("OTA", "Failed reason: %s", esp_err_to_name(ret));

        // Sending error message
        sending_msg.op_code = 0x01;
        sending_msg.action = ACTION_OTA;
        sending_msg.status = STATUS_ERROR;
        strncpy(sending_msg.task_id, params->task_id, TASK_ID_MAX_LEN-1);
        sending_msg.task_id[TASK_ID_MAX_LEN-1] = '\0';
        xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
    }
    free(pvParameter);
    vTaskDelete(NULL);
}



static void acquire_pm_lock(){
    esp_err_t res = esp_pm_lock_acquire(max_cpu_mz);
    if(res != ESP_OK){
        ESP_LOGW("PM", "Max cpu lock acquire failed.");
    }
    res = esp_pm_lock_acquire(max_apb_mz);
    if(res != ESP_OK){
        ESP_LOGW("PM", "Max apb lock acquire failed.");
    }
    ESP_LOGI("PM", "Fetch all acquire lock");
}



static void release_pm_lock(){
    esp_err_t res = esp_pm_lock_release(max_cpu_mz);
    if(res != ESP_OK){
        ESP_LOGW("PM", "Max cpu lock release failed.");
    }
    res = esp_pm_lock_release(max_apb_mz);
    if(res != ESP_OK){
        ESP_LOGW("PM", "Max apb lock release failed.");
    }
    ESP_LOGI("PM", "Release all acquire lock");
}



static void model_switch(char* model_id, char* task_id){
    xEventGroupSetBits(g_system_event_group, EVT_GBP__BIT_MODEL_SWITCH_IN_PROGRESS);
    model_init(model_id, task_id);
    ESP_LOGI("MODEL SWITCH", "Switch complete");
}



static void model_download(model_download_params_t* params){
    xEventGroupSetBits(g_system_event_group, EVT_GBP__BIT_MODEL_DOWNLOAD_IN_PROGRESS);
    xTaskCreate(&model_download_task, "model_download_task", 4096, params, 5, NULL);
}



static void ota_update(ota_params_t* params){
    xEventGroupSetBits(g_system_event_group, EVT_GBP__BIT_OTA_IN_PROGRESS);
    xTaskCreate(&ota_task, "ota", 4096, params, 5, NULL);
}



static void mode_switch(char* mode, char* task_id){
    if(xSemaphoreTake(g_mode_mutex, portMAX_DELAY) == pdTRUE ){   
        if(strcmp(mode, "CONTINUOUS_MODE") == 0){
            operation_mode = CONTINUOUS_MODE;
            // acquire_pm_lock();
            xEventGroupSetBits(g_system_event_group, EVT_GRP__BIT_CONTINUOUS_MODE_ACTIVE);

        }else if(strcmp(mode, "STAND_BY_MODE") == 0){
            operation_mode = STAND_BY_MODE;
            // release_pm_lock();
            xEventGroupClearBits(g_system_event_group, EVT_GRP__BIT_CONTINUOUS_MODE_ACTIVE);
            xEventGroupClearBits(g_system_event_group, EVT_GRP__BIT_TRIGGER_SINGLE_SHOT);
        }

        xSemaphoreGive(g_mode_mutex);
        ESP_LOGI("MODE SWITCH", "Switch complete");
        websocket_sending_message_t sending_msg = {0};
        sending_msg.op_code = 0x01;
        sending_msg.action = ACTION_MODE_SWITCH;
        sending_msg.status = STATUS_COMPLETED;
        strncpy(sending_msg.task_id, task_id, TASK_ID_MAX_LEN-1);
        sending_msg.task_id[TASK_ID_MAX_LEN-1] = '\0';
        xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
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
                            
                            sending_msg.action = ACTION_MODE_SWITCH;
                            sending_msg.status = STATUS_RECEIVED;
                            sending_msg.op_code = 0x01;
                            cJSON* parsed_mode = cJSON_GetObjectItem(msg_json, "mode");
                            cJSON* parsed_task_id = cJSON_GetObjectItem(msg_json, "task_id");
                            
                            if(parsed_mode == NULL){
                                ESP_LOGW("MODE_SWITCH", "Mode does not exist.");                       
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }

                            if(parsed_task_id == NULL){
                                ESP_LOGW("MODE_SWITCH", "task_id does not exist.");                       
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }

                            // Send received notification, if using MQTT protocal does not do it.
                            sending_msg.action = ACTION_MODE_SWITCH;
                            sending_msg.status = STATUS_RECEIVED;
                            sending_msg.op_code = 0x01;
                            strncpy(sending_msg.task_id, parsed_task_id->valuestring, TASK_ID_MAX_LEN-1);
                            sending_msg.task_id[TASK_ID_MAX_LEN-1]='\0';

                            xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                            mode_switch(parsed_mode->valuestring, parsed_task_id->valuestring);
                            

                        }else if(strcmp(action->valuestring, "INFERENCE") == 0){
                            xEventGroupSetBits(g_system_event_group, EVT_GRP__BIT_TRIGGER_SINGLE_SHOT);

                        }else if(strcmp(action->valuestring, "MODEL_DOWNLOAD") == 0){
                        
                            sending_msg.action = ACTION_MODEL_DOWNLOAD;
                            sending_msg.status = STATUS_RECEIVED;
                            sending_msg.op_code = 0x01;
                            model_download_params_t* params = (model_download_params_t*)malloc(sizeof(model_download_params_t));

                            cJSON* parsed_task_id = cJSON_GetObjectItem(msg_json, "task_id");
                            if(parsed_task_id == NULL){
                                ESP_LOGW("MODE_SWITCH", "task_id does not exist.");                       
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }
                            
                            cJSON* parsed_download_path = cJSON_GetObjectItem(msg_json, "download_path");
                            // If message does not contain download path, return error
                            if(parsed_download_path == NULL){
                                ESP_LOGW("MODEL_DEPLOYMENT","Download path does not exist.");
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                free(params);
                                continue;
                            }

                            char* download_path = (char*)malloc(strlen(parsed_download_path->valuestring)+1);
                            // If heap does not availble, return error
                            if(download_path == NULL){
                                ESP_LOGW("MODEL_DEPLOYMENT","Heap does not availble.");
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                free(params);
                                continue;
                            }

                            // Copy string to pointer
                            download_path[strlen(parsed_download_path->valuestring)] = 0;
                            memcpy(download_path, parsed_download_path->valuestring, strlen(parsed_download_path->valuestring));


                            cJSON* parsed_model_id = cJSON_GetObjectItem(msg_json, "model_id");
                            // if message does not contain model id, return error
                            if(parsed_model_id == NULL){
                                ESP_LOGW("MODEL_DEPLOYMENT","model_id does not exist.");
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                free(params);
                                free(download_path);
                                continue;
                            }
                            

                            char* model_id = (char*)malloc(strlen(parsed_model_id->valuestring)+1);
                            // If heap does not availble, return error
                            if(model_id == NULL){
                                ESP_LOGW("MODEL_DEPLOYMENT","Heap does not availble.");
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                free(params);
                                free(download_path);
                                continue;
                            }

                            model_id[strlen(parsed_model_id->valuestring)] = 0;
                            memcpy(model_id, parsed_model_id->valuestring, strlen(parsed_model_id->valuestring));
                            
                            params->download_path=download_path;
                            params->model_id=model_id;
                            params->file=NULL;
                            params->total_len=0;
                            params->downloaded_len=0;
                            strncpy(params->task_id, parsed_task_id->valuestring, TASK_ID_MAX_LEN-1);
                            params->task_id[TASK_ID_MAX_LEN-1]='\0';

                            // Send received notification, if using MQTT protocal does not do it.
                            strncpy(sending_msg.task_id, parsed_task_id->valuestring, TASK_ID_MAX_LEN-1);
                            sending_msg.task_id[TASK_ID_MAX_LEN-1]='\0';
                            xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);

                            model_download(params);
                        
                        }else if(strcmp(action->valuestring, "MODEL_SWITCH") == 0){

                            sending_msg.action = ACTION_MODEL_SWITCH;
                            sending_msg.status = STATUS_RECEIVED;
                            sending_msg.op_code = 0x01;
                            
                            cJSON* parsed_model_id = cJSON_GetObjectItem(msg_json, "model_id");
                            if(parsed_model_id == NULL){
                                ESP_LOGE("MODEL_SWITCH","model_id does not exist.");
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }

                            cJSON* parsed_task_id = cJSON_GetObjectItem(msg_json, "task_id");
                            if(parsed_task_id == NULL){
                                ESP_LOGW("MODE_SWITCH", "task_id does not exist.");                       
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }

                            esp_err_t is_file_exist = check_file_is_exist(parsed_model_id->valuestring);
                            if(is_file_exist == ESP_FAIL){
                                ESP_LOGE("MODEL_SWITCH","file does not exist.");
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }

                            
                            // Send received notification, if using MQTT protocal does not do it.
                            strncpy(sending_msg.task_id, parsed_task_id->valuestring, TASK_ID_MAX_LEN-1);
                            sending_msg.task_id[TASK_ID_MAX_LEN-1]='\0';
                            xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);

                            model_switch(parsed_model_id->valuestring, parsed_task_id->valuestring);
                        
                        }else if(strcmp(action->valuestring, "OTA") == 0){
                            sending_msg.action = ACTION_OTA;
                            sending_msg.status = STATUS_RECEIVED;
                            sending_msg.op_code = 0x01;
                            
                            cJSON* parsed_download_path = cJSON_GetObjectItem(msg_json, "download_path");
                            // If message does not contain download path, return error
                            if(parsed_download_path == NULL){
                                ESP_LOGW("OTA","Download path does not exist.");
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }

                            cJSON* parsed_task_id = cJSON_GetObjectItem(msg_json, "task_id");
                            if(parsed_task_id == NULL){
                                ESP_LOGW("MODE_SWITCH", "task_id does not exist.");                       
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }

                            // If heap does not availble, return error
                            ota_params_t* params = (ota_params_t*)malloc(sizeof(ota_params_t));
                            if(params == NULL){
                                ESP_LOGW("OTA","Heap does not availble.");
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }

                            char* download_path = (char*)malloc(strlen(parsed_download_path->valuestring)+1);
                            // If heap does not availble, return error
                            if(download_path == NULL){
                                ESP_LOGW("OTA","Heap does not availble.");
                                sending_msg.status = STATUS_ERROR;
                                xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                                continue;
                            }

                            // Send received notification, if using MQTT protocal does not do it.
                            strncpy(sending_msg.task_id, parsed_task_id->valuestring, TASK_ID_MAX_LEN-1);
                            sending_msg.task_id[TASK_ID_MAX_LEN-1]='\0';
                            xQueueSend(websocket_sending_message_queue, &sending_msg, portMAX_DELAY);
                            
                            // Prepare ota params
                            memcpy(download_path, parsed_download_path->valuestring, strlen(parsed_download_path->valuestring));
                            download_path[strlen(parsed_download_path->valuestring)] = 0;
                            params->download_path = download_path;
                            strncpy(params->task_id, parsed_task_id->valuestring, TASK_ID_MAX_LEN-1);
                            params->task_id[TASK_ID_MAX_LEN-1]='\0';
                            ota_update(params);

                        }else if(strcmp(action->valuestring, "RESET") == 0){
                            esp_restart();
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

    //  if websocket does't connected, and then release resouces and continues
    bool websocket_connected = esp_websocket_client_is_connected(websocket_client);
    if(!websocket_connected){
        ESP_LOGE("Websocket", "Websocket doesn't connected.");
        esp_camera_fb_return(pic);
        return;
    }

    // allocate heap for JPEG buffer, in oder to avoid null pointer reference in websocket_sending_message_queue
    // if failed, released reousrces
    uint8_t* jpeg_buffer = (uint8_t*)malloc(pic->len+1);
    if(jpeg_buffer == NULL){
        ESP_LOGE("Camera", "JPEG Buffer allocate failed");
        esp_camera_fb_return(pic);
        return;
    }

    // prepare sending message struct
    websocket_sending_message_t msg = {0};
    msg.op_code = 0x02;
    msg.image_len = pic->len;
    msg.image_data = jpeg_buffer;
    msg.action = ACTION_RETURN_INFERENCE;
    msg.status = STATUS_COMPLETED;
    memcpy(jpeg_buffer, pic->buf, pic->len);


    // model inference memory copy
    if(model){
        msg.inference_result = new std::list<dl::detect::result_t>();

        if(msg.inference_result != NULL){
            *msg.inference_result = model_inference(jpeg_buffer, pic->len);
        }else{
            ESP_LOGE("Camera", "Inference result buffer allocate failed");
            free(jpeg_buffer);
            jpeg_buffer=NULL;
            return;
        }
    }else{
        msg.inference_result = NULL;
    }

    BaseType_t send_result = xQueueSend(websocket_sending_message_queue, &msg, 0);
    if( send_result != pdPASS ){
        free(jpeg_buffer);
        jpeg_buffer = NULL;
        if(msg.inference_result != NULL){
            delete msg.inference_result;
            msg.inference_result=NULL;
        }
    }
    esp_camera_fb_return(pic);
}



static void camera_control_task(void* pvParameters){
    if(ESP_OK != init_camera()){
        ESP_LOGE("Camera", "INIT failed");
        vTaskDelete(NULL);
    }

    while(1) {
        if(xEventGroupGetBits(g_system_event_group) & ALL_IN_PROGRESS_BITS){
            xEventGroupWaitBits(
                g_system_event_group,
                ALL_COMPLETED_BITS,
                pdTRUE,
                pdFALSE,
                portMAX_DELAY
            );
        }

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
                vTaskDelay(10000);
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
    char websocket_path[50];
    snprintf(websocket_path, size_t(websocket_path), "%s/%s", WEBSOCKET_HOST, WEBSOCKET_PATH);
    ESP_LOGI("Websocket", "Path: %s", websocket_path);

    esp_websocket_client_config_t websocket_cfg = {};
    websocket_cfg.uri = websocket_path;
    websocket_cfg.disable_auto_reconnect = false;
    char* auth_header = "Authorization: bearer token\r\n";
    websocket_cfg.headers = auth_header;

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
    g_model_mutex = xSemaphoreCreateBinary();

    if(g_mode_mutex == NULL){
        ESP_LOGE("Semaphore", "mode mutext create failed...");
    }else{
        xSemaphoreGive(g_mode_mutex);
    }

    if(g_model_mutex == NULL){
        ESP_LOGE("Semaphore", "model mutext create failed...");
    }else{
        xSemaphoreGive(g_model_mutex);
    }

    if(g_system_event_group == NULL){
        ESP_LOGE("Event Group", "create failed...");
    }

    // Create websocket process task
    xTaskCreatePinnedToCore(websocket_process_task,"Websocket process task", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(websocket_sending_task, "Websocket sending task", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(camera_control_task, "Camera control task", 4096, NULL, 2, NULL, tskNO_AFFINITY);
}



static void sdmmc_card_init_task(){
    esp_err_t ret;

    // Options for mounting the filesystem.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI("SD Card", "Initializing SD card");
    ESP_LOGI("SD Card", "Using SDMMC peripheral");


    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 40MHz for SDMMC)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
#if CONFIG_SDMMC_SPEED_HS
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
    slot_config.clk = GPIO_NUM_39;
    slot_config.cmd = GPIO_NUM_38;
    slot_config.d0 = GPIO_NUM_40;
    // slot_config.d1 = GPIO_NUM_38;
    // slot_config.d2 = GPIO_NUM_33;
    // slot_config.d3 = GPIO_NUM_34;
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



esp_err_t auth_http_event_handler(esp_http_client_event_t* evt){

    http_response_t* params = (http_response_t*)evt->user_data;

    const char* tag = "Authorization HTTP EVENT";
    switch (evt->event_id){
        case HTTP_EVENT_ERROR:{
            ESP_LOGI(tag, "HTTP_EVENT_ERROR");
            break;
        }case HTTP_EVENT_ON_CONNECTED:{
            ESP_LOGI(tag, "HTTP_EVENT_ON_CONNECTED");
            break;
        }case HTTP_EVENT_HEADER_SENT:{
            ESP_LOGI(tag, "HTTP_EVENT_HEADER_SENT");
            break;
        }case HTTP_EVENT_ON_HEADER:{
            ESP_LOGI(tag, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            if(strcmp(evt->header_key, "content-length") == 0){
                params->total_len = atoi(evt->header_value);
                ESP_LOGI("HTTP_EVENT_ON_HEADER", "Response size: %d bytes", params->total_len);
            }
            break;
        }case HTTP_EVENT_ON_DATA:{
            ESP_LOGI(tag, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if(params->data_buffer == NULL){
                char* data_buffer_pointer = (char*)malloc(params->total_len+1);
                if(data_buffer_pointer == NULL){
                    ESP_LOGE("tag", "Failed to allocate heap memory for data_buffer_pointer.");
                    return ESP_FAIL;
                }else{
                    data_buffer_pointer[params->total_len] = '\0';
                    params->data_buffer = data_buffer_pointer;
                }
            }

            memcpy(params->data_buffer + params->downloaded_len, evt->data, evt->data_len);
            params->downloaded_len += evt->data_len;
            break;
        }case HTTP_EVENT_ON_FINISH:{
            ESP_LOGD(tag, "HTTP_EVENT_ON_FINISH");
            break;
        }case HTTP_EVENT_DISCONNECTED:{
            ESP_LOGD(tag, "HTTP_EVENT_DISCONNECTED");
            break;
        }case HTTP_EVENT_REDIRECT:{
            ESP_LOGD(tag, "HTTP_EVENT_REDIRECT");
            break;
        }
    }
    return ESP_OK;
}



static char* http_message2json(){

    // Get MAC
    uint8_t mac[6];
    esp_err_t err = esp_efuse_mac_get_default(mac);
    if(err != ESP_OK){
        ESP_LOGE("MAC", "Get Mac address failed\n");
        return NULL;
    }

    snprintf(MAC, sizeof(MAC), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI("MAC", "MAC Address is: %s", MAC);
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", "esp32s3");
    cJSON_AddStringToObject(root, "processor", "esp32s3");
    cJSON_AddStringToObject(root, "mac", MAC);

    char* result = cJSON_Print(root);
    cJSON_Delete(root);
    return result;
    
}



static http_response_t* auth_http_request(){
    ESP_LOGI("AUTH HTTP REQUEST", "Starting");
    char http_path[50];
    snprintf(http_path, size_t(http_path), "%s/%s", SERVER_HOST, AUTH_HTTP_PATH);
    ESP_LOGI("AUTH HTTP_REQUEST", "Path: %s", http_path);

    http_response_t* params = (http_response_t*)malloc(sizeof(http_response_t));
    if(params == NULL){
        ESP_LOGI("AUTH HTTP_REQUEST", "Failed to allocate heap memory.");
        return NULL;
    }
    params->data_buffer = NULL;
    params->total_len = 0;
    params->downloaded_len = 0;

    esp_http_client_config_t config = {
        .url = http_path,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
        .event_handler = auth_http_event_handler,
        .buffer_size = 4096,
        .user_data = params,
        .skip_cert_common_name_check = true  
    };

    char* post_data = http_message2json();
    if(post_data == NULL){
        ESP_LOGI("AUTH HTTP_REQUEST", "Failed to generate post data.");
        return NULL;
    }
    ESP_LOGI("AUTH HTTP_REQUEST", "post data: %s", post_data);
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_err_t ret = esp_http_client_perform(client);
    if(ret == ESP_OK){
        ESP_LOGI("AUTH HTTP_REQUEST", "Success");
        ESP_LOGI("AUTH HTTP_REQUEST", "response: %s", params->data_buffer);
        return params;
    }else{
        ESP_LOGE("AUTH HTTP_REQUEST", "Failed");
        ESP_LOGE("AUTH HTTP_REQUEST", "Failed reason: %s", esp_err_to_name(ret));
        if(params->data_buffer != NULL){
            free(params->data_buffer);
        }
        free(params);
        return NULL;
    }
    free(post_data);
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
        // uint8_t mac[6];
        // esp_err_t err = esp_efuse_mac_get_default(mac);
        // if(err != ESP_OK){
        //     ESP_LOGE("MAC", "Get Mac address failed\n");
        // }else{
        //     snprintf(MAC, sizeof(MAC), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        //     ESP_LOGI("MAC", "MAC Address is: %s", MAC);
        //     websocket_init_task();
        // }
        http_response_t* response = auth_http_request();
        if(response){
            free(response->data_buffer);
            free(response);
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



static void power_management_init(){
    pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 40,
        .light_sleep_enable = false
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    esp_err_t res = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "Continuous mode cpu pm lock", &max_cpu_mz);
    if(res != ESP_OK){
        ESP_LOGW("PM", "Max cpu lock create failed.");
        return;
    }

    res = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "Continuous mode apb pm lock", &max_apb_mz);
    if(res != ESP_OK){
        ESP_LOGW("PM", "Max apb lock create failed.");
        return;
    }
}



static void print_sha256(const uint8_t* image_hash, const char* label){
    char hash_print[32 * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for(int i = 0 ; i < HASH_LEN ; ++i) {
        sprintf(&hash_print[i*2], "%02x", image_hash[i]);
    }
    ESP_LOGI("OTA", "%s %s", label, hash_print);
}



static void get_sha256_of_partitions(void){
    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    // get sha256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");
}



extern "C" void app_main(void){
    /**
     * 1. customize model init class
     * 2. nvs
     * 3. log
     * 4. device authentication 
     * 5. split code
     */
    // power_management_init();
    nvs_init();
    sdmmc_card_init_task();
    get_sha256_of_partitions();
    wifi_init_task();
}

/**
 * CONFIG_COMPILER_OPTIMIZATION_PERF --- y
 * CONFIG_SPIRAM_SPEED_80M           --- y
 * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240 --- y
 * CONFIG_FATFS_LFN_STACK --- y
 * CONFIG_SOC_XTAL_SUPPORT_40M(SD card) --- y
 * CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP      --- y
 */