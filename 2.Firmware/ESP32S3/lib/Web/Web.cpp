#include "Web.h"
#include "BalancingRobot.h"

WebServerManager::WebServerManager(BalancingRobot* robot) {
    _robot = robot;
    _server = new WebServer(80);  // 去掉::前缀
    
    // 默认PID参数（基于Kp/Ki=200经验值）
    _params.angle_kp = 5.0;
    _params.angle_ki = 0.0;
    _params.angle_kd = 0.5;
    
    _params.speed_kp = 2.0;   // Kp/Ki = 200
    _params.speed_ki = 0.01;  // Ki = Kp/200 = 2.0/200
    _params.speed_kd = 0.0;
    
    _params.turn_kp = 0.0;
    _params.turn_ki = 0.0;
    _params.turn_kd = 0.0;
}

void WebServerManager::init(const char* ssid, const char* password) {
    Serial.println("Starting WiFi Station Mode...");
    
    // 确保WiFi完全关闭后再启动
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    // 启动STA模式（连接到现有WiFi）
    WiFi.mode(WIFI_STA);
    delay(100);
    
    // 连接到WiFi
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    
    // 等待连接，最多30秒
    int timeout = 30;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
        delay(1000);
        Serial.print(".");
        timeout--;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected successfully!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal strength (RSSI): ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        Serial.println("-------------------------------");
    } else {
        Serial.println("ERROR: Failed to connect to WiFi!");
        Serial.println("Please check SSID and password.");
        Serial.println("Retrying in AP mode as fallback...");
        
        // 连接失败，切换到AP模式
        WiFi.mode(WIFI_AP);
        WiFi.softAP("BalanceCar-Fallback", "12345678");
        Serial.print("Fallback AP IP: ");
        Serial.println(WiFi.softAPIP());
    }
    
    // 初始化SPIFFS
    Serial.println("Mounting SPIFFS...");
    if (!SPIFFS.begin(true)) {
        Serial.println("ERROR: SPIFFS Mount Failed!");
        return;
    }
    Serial.println("SPIFFS mounted successfully");
    
    // 列出SPIFFS中的文件
    Serial.println("Files in SPIFFS:");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.print("  ");
        Serial.print(file.name());
        Serial.print(" - ");
        Serial.print(file.size());
        Serial.println(" bytes");
        file = root.openNextFile();
    }
    
    // 加载保存的参数
    loadParameters();
    
    // 设置路由
    _server->on("/", HTTP_GET, [this]() { this->handleRoot(); });
    _server->on("/control", HTTP_POST, [this]() { this->handleControl(); });
    _server->on("/setPID", HTTP_POST, [this]() { this->handleSetPID(); });
    _server->on("/status", HTTP_GET, [this]() { this->handleGetStatus(); });
    _server->on("/save", HTTP_POST, [this]() { this->handleSaveParams(); });
    _server->on("/load", HTTP_POST, [this]() { this->handleLoadParams(); });
    _server->onNotFound([this]() { this->handleNotFound(); });
    
    _server->begin();
    Serial.println("HTTP server started");
}

void WebServerManager::handleClient() {
    _server->handleClient();
}

void WebServerManager::loadParameters() {
    _preferences.begin("balance-car", false);
    
    _params.angle_kp = _preferences.getFloat("angle_kp", 5.0);
    _params.angle_ki = _preferences.getFloat("angle_ki", 0.0);
    _params.angle_kd = _preferences.getFloat("angle_kd", 0.5);
    
    _params.speed_kp = _preferences.getFloat("speed_kp", 2.0);
    _params.speed_ki = _preferences.getFloat("speed_ki", 0.01);
    _params.speed_kd = _preferences.getFloat("speed_kd", 0.0);
    
    _params.turn_kp = _preferences.getFloat("turn_kp", 0.0);
    _params.turn_ki = _preferences.getFloat("turn_ki", 0.0);
    _params.turn_kd = _preferences.getFloat("turn_kd", 0.0);
    
    _preferences.end();
    
    // 应用到机器人
    _robot->setAnglePID(_params.angle_kp, _params.angle_ki, _params.angle_kd);
    _robot->setSpeedPID(_params.speed_kp, _params.speed_ki, _params.speed_kd);
    _robot->setTurnPID(_params.turn_kp, _params.turn_ki, _params.turn_kd);
    
    Serial.println("Parameters loaded from NVS");
}

void WebServerManager::saveParameters() {
    _preferences.begin("balance-car", false);
    
    _preferences.putFloat("angle_kp", _params.angle_kp);
    _preferences.putFloat("angle_ki", _params.angle_ki);
    _preferences.putFloat("angle_kd", _params.angle_kd);
    
    _preferences.putFloat("speed_kp", _params.speed_kp);
    _preferences.putFloat("speed_ki", _params.speed_ki);
    _preferences.putFloat("speed_kd", _params.speed_kd);
    
    _preferences.putFloat("turn_kp", _params.turn_kp);
    _preferences.putFloat("turn_ki", _params.turn_ki);
    _preferences.putFloat("turn_kd", _params.turn_kd);
    
    _preferences.end();
    
    Serial.println("Parameters saved to NVS");
}

void WebServerManager::handleRoot() {
    // 从SPIFFS读取HTML文件
    File file = SPIFFS.open("/index.html", "r");
    if (!file) {
        _server->send(404, "text/plain", "File not found");
        return;
    }
    
    _server->streamFile(file, "text/html");
    file.close();
}

void WebServerManager::handleControl() {
    if (_server->hasArg("action")) {
        String action = _server->arg("action");
        
        if (action == "forward") {
            _robot->setTargetSpeed(20);
            _robot->setTargetTurn(0);
        } else if (action == "backward") {
            _robot->setTargetSpeed(-20);
            _robot->setTargetTurn(0);
        } else if (action == "left") {
            _robot->setTargetTurn(-20);
        } else if (action == "right") {
            _robot->setTargetTurn(20);
        } else if (action == "stop") {
            _robot->setTargetSpeed(0);
            _robot->setTargetTurn(0);
        }
        
        _server->send(200, "text/plain", "OK");
    } else {
        _server->send(400, "text/plain", "Missing action");
    }
}

void WebServerManager::handleSetPID() {
    bool updated = false;
    
    // 角度PID
    if (_server->hasArg("angle_kp")) {
        _params.angle_kp = _server->arg("angle_kp").toFloat();
        updated = true;
    }
    if (_server->hasArg("angle_ki")) {
        _params.angle_ki = _server->arg("angle_ki").toFloat();
        updated = true;
    }
    if (_server->hasArg("angle_kd")) {
        _params.angle_kd = _server->arg("angle_kd").toFloat();
        updated = true;
    }
    
    // 速度PID
    if (_server->hasArg("speed_kp")) {
        _params.speed_kp = _server->arg("speed_kp").toFloat();
        updated = true;
    }
    if (_server->hasArg("speed_ki")) {
        _params.speed_ki = _server->arg("speed_ki").toFloat();
        updated = true;
    }
    if (_server->hasArg("speed_kd")) {
        _params.speed_kd = _server->arg("speed_kd").toFloat();
        updated = true;
    }
    
    // 转向PID
    if (_server->hasArg("turn_kp")) {
        _params.turn_kp = _server->arg("turn_kp").toFloat();
        updated = true;
    }
    if (_server->hasArg("turn_ki")) {
        _params.turn_ki = _server->arg("turn_ki").toFloat();
        updated = true;
    }
    if (_server->hasArg("turn_kd")) {
        _params.turn_kd = _server->arg("turn_kd").toFloat();
        updated = true;
    }
    
    if (updated) {
        _robot->setAnglePID(_params.angle_kp, _params.angle_ki, _params.angle_kd);
        _robot->setSpeedPID(_params.speed_kp, _params.speed_ki, _params.speed_kd);
        _robot->setTurnPID(_params.turn_kp, _params.turn_ki, _params.turn_kd);
        _server->send(200, "text/plain", "PID updated");
    } else {
        _server->send(400, "text/plain", "No parameters");
    }
}

void WebServerManager::handleGetStatus() {
    String json = "{";
    json += "\"pitch\":" + String(_robot->getPitch(), 2) + ",";
    json += "\"roll\":" + String(_robot->getRoll(), 2) + ",";
    json += "\"leftSpeed\":" + String(_robot->getLeftSpeed(), 1) + ",";
    json += "\"rightSpeed\":" + String(_robot->getRightSpeed(), 1) + ",";
    json += "\"balancing\":" + String(_robot->isBalancing() ? "true" : "false") + ",";
    json += "\"angle_kp\":" + String(_params.angle_kp, 3) + ",";
    json += "\"angle_ki\":" + String(_params.angle_ki, 3) + ",";
    json += "\"angle_kd\":" + String(_params.angle_kd, 3) + ",";
    json += "\"speed_kp\":" + String(_params.speed_kp, 3) + ",";
    json += "\"speed_ki\":" + String(_params.speed_ki, 3) + ",";
    json += "\"speed_kd\":" + String(_params.speed_kd, 3) + ",";
    json += "\"turn_kp\":" + String(_params.turn_kp, 3) + ",";
    json += "\"turn_ki\":" + String(_params.turn_ki, 3) + ",";
    json += "\"turn_kd\":" + String(_params.turn_kd, 3);
    json += "}";
    
    _server->send(200, "application/json", json);
}

void WebServerManager::handleSaveParams() {
    saveParameters();
    _server->send(200, "text/plain", "Parameters saved to flash");
}

void WebServerManager::handleLoadParams() {
    loadParameters();
    _server->send(200, "text/plain", "Parameters loaded from flash");
}

void WebServerManager::handleNotFound() {
    _server->send(404, "text/plain", "File not found");
}