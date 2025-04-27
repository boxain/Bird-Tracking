idf_build_get_property(target IDF_TARGET)

idf_component_register(
    SRCS "model.c"
    INCLUDE_DIRS "include" "include/tool" "include/nn" "include/layer" "include/image" "include/typedef" "include/detect" "include/model_zoo"
)

if(${IDF_TARGET} STREQUAL "esp32")
    set(lib_dir "lib/esp32")
elseif(${IDF_TARGET} STREQUAL "esp32s3")
    set(lib_dir "lib/esp32s3")
endif()

target_link_libraries(${COMPONENT_LIB} PRIVATE "-L${CMAKE_CURRENT_SOURCE_DIR}/${lib_dir}")
target_link_libraries(${COMPONENT_LIB} PRIVATE "human_face_detect")
target_link_libraries(${COMPONENT_LIB} PRIVATE "dl")