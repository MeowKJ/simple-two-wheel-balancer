#include <Arduino.h>
#include "board.h"
#include "BalancingRobot.h"
#include <FastLED.h>

// FreeRTOS task handles
TaskHandle_t balanceTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

// Robot instance
BalancingRobot robot;

// LED
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Battery monitoring
float batteryVoltage = 0;

// Function prototypes
void balanceTask(void *parameter);
void telemetryTask(void *parameter);
void ledTask(void *parameter);
void readBattery();
void processSerialCommands();

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n=================================");
    Serial.println("ESP32-S3 Balancing Robot");
    Serial.println("=================================");
    
    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    FastLED.addLeds<WS2812B, WS2812B_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Blue;
    FastLED.show();
    
    // Initialize battery ADC
    pinMode(BAT_ADC_PIN, INPUT);
    
    // Initialize robot
    Serial.println("Initializing robot...");
    robot.init();
    
    Serial.println("Creating RTOS tasks...");
    
    // Create balance control task (highest priority)
    xTaskCreatePinnedToCore(
        balanceTask,
        "Balance",
        4096,
        NULL,
        3,  // High priority
        &balanceTaskHandle,
        1   // Core 1
    );
    
    // Create telemetry task
    xTaskCreatePinnedToCore(
        telemetryTask,
        "Telemetry",
        4096,
        NULL,
        1,  // Low priority
        &telemetryTaskHandle,
        0   // Core 0
    );
    
    // Create LED task
    xTaskCreatePinnedToCore(
        ledTask,
        "LED",
        2048,
        NULL,
        1,  // Low priority
        &ledTaskHandle,
        0   // Core 0
    );
    
    Serial.println("Robot ready!");
    Serial.println("Commands: w=forward, s=backward, a=left, d=right, x=stop");
    Serial.println("PID tuning: p=angle_kp, i=angle_ki, d=angle_kd");
    
    leds[0] = CRGB::Green;
    FastLED.show();
}

void loop() {
    // Main loop handles serial commands
    readBattery();
    delay(10);
}

// Balance control task - runs at high frequency
void balanceTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        robot.update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Telemetry task - prints status information
void telemetryTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        Serial.print("Pitch: ");
        Serial.print(robot.getPitch(), 2);
        Serial.print("° | L: ");
        Serial.print(robot.getLeftSpeed(), 1);
        Serial.print(" RPM | R: ");
        Serial.print(robot.getRightSpeed(), 1);
        Serial.print(" RPM | Bat: ");
        Serial.print(batteryVoltage, 2);
        Serial.print("V | ");
        Serial.println(robot.isBalancing() ? "BALANCING" : "STOPPED");
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// LED task - indicates robot status
void ledTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    uint8_t hue = 0;
    
    while (1) {
        if (robot.isBalancing()) {
            leds[0] = CRGB::Green;
        } else {
            // Rainbow effect when not balancing
            leds[0] = CHSV(hue, 255, 255);
            hue += 5;
        }
        
        FastLED.show();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void readBattery() {
    static unsigned long lastRead = 0;
    if (millis() - lastRead > 1000) {
        int rawValue = analogRead(BAT_ADC_PIN);
        // Adjust voltage divider ratio based on your hardware
        batteryVoltage = (rawValue / 4095.0) * 3.3 * 2.0; // Assuming 1:1 divider
        lastRead = millis();
    }
}