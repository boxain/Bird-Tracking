#pragma once
#include "dl_detect_base.hpp"
#include "dl_detect_espdet_postprocessor.hpp"
#include "dl_cls_base.hpp"

namespace model_factory {
    class ESPDet : public dl::detect::DetectImpl {
        public:
            ESPDet(char *path);   
    };
    class ESPCls : public dl::cls::ClsImpl {
        public:
            ESPCls(char *path);   
    };
}

class ObjectDetectModel : public dl::detect::DetectWrapper {
    public:
        ObjectDetectModel(char* path);
};

class ClassifierModel : public dl::cls::ClsWrapper {
    public:
        ClassifierModel(char* path);
};
