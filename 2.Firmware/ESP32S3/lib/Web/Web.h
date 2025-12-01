#ifndef BALANCE_CAR_WEBSERVER_H
#define BALANCE_CAR_WEBSERVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <WebServer.h>  // ESP32的WebServer库

class BalancingRobot; // 前向声明

class WebServerManager {
public:
    WebServerManager(BalancingRobot* robot);
    
    void init(const char* ssid, const char* password);
    void handleClient();
    
    // 参数管理
    void loadParameters();
    void saveParameters();
    
private:
    BalancingRobot* _robot;
    WebServer* _server;  // 直接使用WebServer，不用::前缀
    Preferences _preferences;
    
    // PID参数
    struct PIDParams {
        float angle_kp, angle_ki, angle_kd;
        float speed_kp, speed_ki, speed_kd;
        float turn_kp, turn_ki, turn_kd;
    } _params;
    
    // 网页处理函数
    void handleRoot();
    void handleControl();
    void handleSetPID();
    void handleGetStatus();
    void handleSaveParams();
    void handleLoadParams();
    void handleNotFound();
};

#endif