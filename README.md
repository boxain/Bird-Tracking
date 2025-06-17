# Device (ESP32-S3) Documentation

This document provides instructions for setting up and using the firmware for the ESP32-S3 device component of this AIoT project.

## ✨ Features

* **AI Model Deployment**: Remotely deploy new AI models to the device.
* **AI Model Switching**: Switch between multiple AI models stored on the device without reflashing.
* **Over-the-Air (OTA) Updates**: Update the device's firmware remotely.



## 🛠️ Setup

### 1. ESP-IDF Environment

This project is built using **esp-idf**, the official development framework for Espressif's SoCs. You must have it installed and configured first.

* **Setup Guide**: [ESP-IDF Extension for VSCode](https://github.com/espressif/vscode-esp-idf-extension/blob/master/README.md)

### 2. Web Service

Before running the device, ensure the web service (backend/frontend) is running and you have created a user account. The device will need these credentials to connect.

### 3. Menuconfig Configuration

You must configure the project using `idf.py menuconfig`. Set the following parameters:
```
Component config --->
Partition Table --->
[*] Custom partition table CSV
(partitions.csv) Custom partition CSV file
(partitions.csv) Partition Table filename

Example Configuration --->
(your_username) ESP Server Username
(your_password) ESP Server Password
```


## ⚠️ Limitations

* **Hardware Support**: This project currently only supports the **ESP32-S3** series. The AI inference functionality relies on the `esp-idf/dl` component, and it has not been tested on other series like the ESP32-P4.
* **Model Support**: Only **Object Detection** models are supported at the moment. Image Classification models are not yet implemented.
* **Custom Models**: The `detection postprocessor` parameters are currently hardcoded for the default model. If you wish to use your own custom-trained model, you will need to modify these parameters. You can find them in:
    `main/model/model_factory.cpp`


## 🔌 Hardware

This project was developed and tested using the **Freenove ESP32-S3 CAM** board.

* **Product Link**: [Freenove ESP32-S3 CAM](https://store.freenove.com/products/fnk0085)
* **Product Link**: [Freenove ESP32-S3 CAM for Shopee](https://shopee.tw/%E3%80%90%E6%A8%82%E6%84%8F%E5%89%B5%E5%AE%A2%E5%AE%98%E6%96%B9%E5%BA%97%E3%80%91%E6%A8%82%E9%91%AB%E5%8E%9F%E5%BB%A0-ESP32-S3-WROOM-CAM-%E9%96%8B%E7%99%BC%E6%9D%BF-Nodemcu-%E9%81%A9%E7%94%A8Arduino-IDE-i.139069730.23909529147)

If you intend to use a different ESP32-S3 development board, pay close attention to the **camera and SD card pin configurations**. You will need to modify the corresponding pin definitions in `main.cpp` to match your hardware.