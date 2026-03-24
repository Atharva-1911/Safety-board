#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

// things defined in main.cpp
extern const char* ssid;
extern const char* password;
extern volatile bool otaInProgress;
extern volatile uint16_t relay_state;
void pcf8575_writeAll(uint16_t value);

bool setupOTA() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi");
    unsigned long start = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi failed. OTA disabled.");
        return false;
    }

    WiFi.setSleep(false);

    Serial.println();
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());

    ArduinoOTA.setHostname("esp32-robot");
    ArduinoOTA.setPassword("admin123");

    ArduinoOTA.onStart([]() {
        otaInProgress = true;
        Serial.println("OTA Start");

        // Put machine in safe state
        relay_state = 0xFFFF;
        pcf8575_writeAll(relay_state);

        // Remove current loop task from watchdog during OTA
        esp_task_wdt_delete(NULL);
    });

    ArduinoOTA.onEnd([]() {
        otaInProgress = false;
        Serial.println("\nOTA End");
        delay(500);
        ESP.restart();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress * 100) / total);
    });

    ArduinoOTA.onError([](ota_error_t error) {
        otaInProgress = false;
        Serial.printf("OTA Error[%u]: ", error);

        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        else Serial.println("Unknown Error");

        esp_task_wdt_add(NULL);
    });

    ArduinoOTA.begin();
    Serial.println("OTA Ready");

    return true;
}