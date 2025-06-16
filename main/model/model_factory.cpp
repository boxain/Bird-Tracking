#include "model_factory.hpp"
#include "esp_log.h"


namespace model_factory {
    ESPDet::ESPDet(char* path){
        m_model = new dl::Model(path, fbs::MODEL_LOCATION_IN_SDCARD);
        m_model->minimize();
    #if CONFIG_IDF_TARGET_ESP32P4
        m_image_preprocessor = new dl::image::ImagePreprocessor(m_model, {0, 0, 0}, {255, 255, 255});
    #else
        m_image_preprocessor =
            new dl::image::ImagePreprocessor(m_model, {0, 0, 0}, {255, 255, 255}, DL_IMAGE_CAP_RGB565_BIG_ENDIAN);
    #endif
        m_postprocessor =
            new dl::detect::ESPDetPostProcessor(m_model, 0.25, 0.7, 10, {{8, 8, 4, 4}, {16, 16, 8, 8}, {32, 32, 16, 16}});
    }

    ESPCls::ESPCls(char* path){
        ESP_LOGI("MODEL_INIT", "Does not implement yet.");
    }

} // namespace model_factory


ObjectDetectModel::ObjectDetectModel(char* path){
    m_model = new model_factory::ESPDet(path);
}

ClassifierModel::ClassifierModel(char* path){
    m_model = new model_factory::ESPCls(path);
}
