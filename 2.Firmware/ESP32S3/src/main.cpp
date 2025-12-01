#include <Arduino.h>
#include <board.h>
#include <BalancingRobot.h>
#include <Web.h>
#include <FastLED.h>

// FreeRTOS task handles
TaskHandle_t balanceTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t webServerTaskHandle = NULL;

// Robot instance
BalancingRobot robot;

// Web Server
WebServerManager* webServer;

// LED
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Battery monitoring
float batteryVoltage = 0;

// Function prototypes
void balanceTask(void *parameter);
void telemetryTask(void *parameter);
void ledTask(void *parameter);
void webServerTask(void *parameter);
void readBattery();
void processSerialCommands();

void setup() {
    Serial.begin(115200);
    delay(2000); // 增加延时确保串口稳定
    
    Serial.println("\n\n=================================");
    Serial.println("ESP32-S3 Balancing Robot");
    Serial.println("=================================");
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("CPU Freq: %d MHz\n", ESP.getCpuFreqMHz());
    
    // Initialize LED - 注意：LED_PIN(48)与RMOTOR_IN1冲突，暂时注释
    Serial.println("Init LED...");
    // pinMode(LED_PIN, OUTPUT);  // 暂时禁用，因为与电机引脚冲突
    // digitalWrite(LED_PIN, HIGH);
    
    Serial.println("Init WS2812B...");
    FastLED.addLeds<WS2812B, WS2812B_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Blue;
    FastLED.show();
    
    // Initialize battery ADC
    Serial.println("Init Battery ADC...");
    pinMode(BAT_ADC_PIN, INPUT);
    
    // Initialize robot
    Serial.println("Initializing robot...");
    
    BalancingRobot::MotorPins leftMotor = {
        LMOTOR_IN1_PIN, LMOTOR_IN2_PIN, LMOTOR_SLEEP_PIN,
        LMOTOR_ENCODER_A_PIN, LMOTOR_ENCODER_B_PIN
    };
    
    BalancingRobot::MotorPins rightMotor = {
        RMOTOR_IN1_PIN, RMOTOR_IN2_PIN, RMOTOR_SLEEP_PIN,
        RMOTOR_ENCODER_A_PIN, RMOTOR_ENCODER_B_PIN
    };
    
    BalancingRobot::IMUPins imuPins = {
        I2C_SDA_PIN, I2C_SCL_PIN
    };
    
    robot.init(leftMotor, rightMotor, imuPins);
    
    Serial.printf("Free heap after init: %d bytes\n", ESP.getFreeHeap());
    
    // Initialize Web Server (STA模式 - 连接到现有WiFi)
    Serial.println("\n=== Initializing Web Server ===");
    webServer = new WebServerManager(&robot);  // 修正类名
    webServer->init("218", "12345678218");  // WiFi SSID和密码
    Serial.println("================================\n");
    
    Serial.println("Creating RTOS tasks...");
    
    // Create balance control task (highest priority)
    xTaskCreatePinnedToCore(
        balanceTask,
        "Balance",
        8192,  // 增加栈大小
        NULL,
        3,  // High priority
        &balanceTaskHandle,
        1   // Core 1
    );
    
    // Create telemetry task
    xTaskCreatePinnedToCore(
        telemetryTask,
        "Telemetry",
        4096,  // 增加栈大小
        NULL,
        1,  // Low priority
        &telemetryTaskHandle,
        0   // Core 0
    );
    
    // Create LED task
    xTaskCreatePinnedToCore(
        ledTask,
        "LED",
        4096,  // 增加栈大小
        NULL,
        1,  // Low priority
        &ledTaskHandle,
        0   // Core 0
    );
    
    // Create web server task
    xTaskCreatePinnedToCore(
        webServerTask,
        "WebServer",
        8192,  // Web服务器需要更大的栈
        NULL,
        1,  // Low priority
        &webServerTaskHandle,
        0   // Core 0
    );
    
    Serial.println("Robot ready!");
    Serial.println("\n========================================");
    Serial.println("WiFi Connection Info:");
    Serial.print("  Connected to: 218\n");
    Serial.print("  Local IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("  Access URL: http://");
    Serial.println(WiFi.localIP());
    Serial.println("========================================\n");
    Serial.println("Commands: w=forward, s=backward, a=left, d=right, x=stop");
    Serial.println("PID tuning: p=angle_kp, i=angle_ki, d=angle_kd");
    
    leds[0] = CRGB::Green;
    FastLED.show();
}

void loop() {
    // Main loop handles serial commands
    processSerialCommands();
    readBattery();
    delay(10);
}

// Balance control task - runs at high frequency
void balanceTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    Serial.println("Balance task started");
    
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
        Serial.print("Pitch: "); // Pitch角（前后倾斜）
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

// Web server task - handles HTTP requests
void webServerTask(void *parameter) {
    while (1) {
        webServer->handleClient();
        vTaskDelay(pdMS_TO_TICKS(5)); // 5ms delay
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

void processSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'w': // Forward
                robot.setTargetSpeed(20);
                Serial.println("Moving forward");
                break;
                
            case 's': // Backward
                robot.setTargetSpeed(-20);
                Serial.println("Moving backward");
                break;
                
            case 'a': // Turn left
                robot.setTargetTurn(-20);
                Serial.println("Turning left");
                break;
                
            case 'd': // Turn right
                robot.setTargetTurn(20);
                Serial.println("Turning right");
                break;
                
            case 'x': // Stop
                robot.setTargetSpeed(0);
                robot.setTargetTurn(0);
                Serial.println("Stopped");
                break;
                
            case 'p': { // Tune angle Kp - 添加大括号作用域
                Serial.println("Enter Angle Kp:");
                while (!Serial.available()) delay(10);
                float kp = Serial.parseFloat();
                robot.setAnglePID(kp, 0, 1.5);
                Serial.print("Angle Kp set to: ");
                Serial.println(kp);
                break;
            }
                
            case 'i': { // Tune angle Ki - 添加大括号作用域
                Serial.println("Enter Angle Ki:");
                while (!Serial.available()) delay(10);
                float ki = Serial.parseFloat();
                robot.setAnglePID(40.0, ki, 1.5);
                Serial.print("Angle Ki set to: ");
                Serial.println(ki);
                break;
            }
                
            case 'k': { // 改用k代替d（避免与'd'转向冲突）
                Serial.println("Enter Angle Kd:");
                while (!Serial.available()) delay(10);
                float kd = Serial.parseFloat();
                robot.setAnglePID(40.0, 0, kd);
                Serial.print("Angle Kd set to: ");
                Serial.println(kd);
                break;
            }
                
            case 'h': // Help
                Serial.println("\nCommands:");
                Serial.println("w - Forward");
                Serial.println("s - Backward");
                Serial.println("a - Turn left");
                Serial.println("d - Turn right");
                Serial.println("x - Stop");
                Serial.println("p - Tune angle Kp");
                Serial.println("i - Tune angle Ki");
                Serial.println("k - Tune angle Kd");
                Serial.println("h - Help");
                break;
        }
    }
}