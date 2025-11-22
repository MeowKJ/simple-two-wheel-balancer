#ifndef BOARD_H
#define BOARD_H

#define BOARD_NAME "LCKFB-ESP32S3R8N8"
#define BOARD_VENDOR "LCKFB"
#define BOARD_URL "https://oshwhub.com/li-chuang-kai-fa-ban/li-chuang-esp32s3r8n8-kai-fa-ban"

#define LED_PIN 48

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 3

#define BAT_ADC_PIN 1


#define WS2812B_PIN 14

#define LMOTOR_IN1_PIN 45
#define LMOTOR_IN2_PIN 32
#define LMOTOR_SLEEP_PIN 46
#define LMOTOR_ENCODER_A_PIN 37
#define LMOTOR_ENCODER_B_PIN 36


#define RMOTOR_IN1_PIN 48
#define RMOTOR_IN2_PIN 0
#define RMOTOR_SLEEP_PIN 47
#define RMOTOR_ENCODER_A_PIN 38
#define RMOTOR_ENCODER_B_PIN 39

#define IMU_INT_PIN 7

#endif // BOARD_H