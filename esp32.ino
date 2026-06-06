/*
 * ESP32 CAN Controller — Multi-Profile 6 Motors System
 * WiFi AP + HTTP Server ile Python CLI desteği
 */

#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>

// ── Pin tanımları ─────────────────────────────────────────────────────────────
#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_15

// ── CAN komut tipleri ─────────────────────────────────────────────────────────
#define CMD_POSITION  0x00   // float — derece
#define CMD_VELOCITY  0x01   // float — rad/s
#define CMD_KP        0x02
#define CMD_KI        0x03
#define CMD_KD        0x04

// ── WiFi ──────────────────────────────────────────────────────────────────────
const char* AP_SSID = "ESP32_Motor_Net";
const char* AP_PASS = "password123";
WebServer httpServer(80);

// ── Motor yapısı ──────────────────────────────────────────────────────────────
struct Motor {
    float targetDeg  = 0.0f;
    float currentDeg = 0.0f;
    float currentVel = 0.0f;
    float targetVel  = 0.0f;
    float kp         = 0.0f;
    float ki         = 0.0f;
    float kd         = 0.0f;
};

Motor    motors[7];
int      activeMotor    = 1;
uint32_t lastPrintTime  = 0;

// ── Motor profilleri ──────────────────────────────────────────────────────────
void initMotorProfiles() {
    for (int i = 1; i <= 2; i++) { motors[i].kp = 30.0f;  motors[i].ki = 2.0f; motors[i].kd = 0.00f;  }
    for (int i = 3; i <= 4; i++) { motors[i].kp = 0.8f;  motors[i].ki = 0.02f;  motors[i].kd = 0.000001f;  }
    for (int i = 5; i <= 6; i++) { motors[i].kp = 40.00f; motors[i].ki = 10.00f; motors[i].kd = 2.00f;  }
}

// ── CAN kurulum ───────────────────────────────────────────────────────────────
void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        Serial.println("[CAN] Sürücü aktif.");
    } else {
        Serial.println("[CAN] Sürücü BAŞARISIZ!");
    }
}

// ── Float gönderici ───────────────────────────────────────────────────────────
void sendFloat(uint8_t cmdType, float val, int motorId) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = 0x200 + motorId;
    msg.data_length_code = 5;
    msg.data[0]          = cmdType;
    memcpy(&msg.data[1], &val, 4);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
}

// ── CAN alma ──────────────────────────────────────────────────────────────────
void handleCAN() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        int id = msg.identifier;
        if (id >= 0x301 && id <= 0x306 && msg.data_length_code >= 8) {
            int m = id - 0x300;
            memcpy(&motors[m].currentDeg, &msg.data[0], 4);
            memcpy(&motors[m].currentVel, &msg.data[4], 4);
        }
    }
}

// ── HTTP: /status?m=<1-6> ─────────────────────────────────────────────────────
void handleStatus() {
    int m = activeMotor;
    if (httpServer.hasArg("m")) {
        m = httpServer.arg("m").toInt();
    }
    if (m < 1 || m > 6) {
        httpServer.send(400, "text/plain", "Motor ID 1-6 olmali");
        return;
    }

    Motor& mo = motors[m];
    String json = "{";
    json += "\"id\":"  + String(m)                + ",";
    json += "\"tp\":"  + String(mo.targetDeg,  2) + ",";
    json += "\"cp\":"  + String(mo.currentDeg, 2) + ",";
    json += "\"tv\":"  + String(mo.targetVel,  4) + ",";
    json += "\"cv\":"  + String(mo.currentVel, 4) + ",";
    json += "\"kp\":"  + String(mo.kp, 4)         + ",";
    json += "\"ki\":"  + String(mo.ki, 4)         + ",";
    json += "\"kd\":"  + String(mo.kd, 4)         + "}";

    httpServer.send(200, "application/json", json);
}

// ── HTTP: /set?m=<id>&type=<tip>&val=<deger> ──────────────────────────────────
void handleSet() {
    if (!httpServer.hasArg("m") || !httpServer.hasArg("type") || !httpServer.hasArg("val")) {
        httpServer.send(400, "text/plain", "Eksik parametre: m, type, val");
        return;
    }

    int    m    = httpServer.arg("m").toInt();
    String type = httpServer.arg("type");
    float  val  = httpServer.arg("val").toFloat();

    if (m < 1 || m > 6) {
        httpServer.send(400, "text/plain", "Motor ID 1-6 olmali");
        return;
    }

    if      (type == "pos") { motors[m].targetDeg = val; sendFloat(CMD_POSITION, val, m); }
    else if (type == "vel") { motors[m].targetVel = val; sendFloat(CMD_VELOCITY, val, m); }
    else if (type == "kp")  { motors[m].kp = val;        sendFloat(CMD_KP,       val, m); }
    else if (type == "ki")  { motors[m].ki = val;        sendFloat(CMD_KI,       val, m); }
    else if (type == "kd")  { motors[m].kd = val;        sendFloat(CMD_KD,       val, m); }
    else {
        httpServer.send(400, "text/plain", "Bilinmeyen tip: pos/vel/kp/ki/kd");
        return;
    }

    httpServer.send(200, "text/plain", "OK");
}

// ── HTTP: /ping ───────────────────────────────────────────────────────────────
void handlePing() {
    twai_status_info_t info;
    twai_get_status_info(&info);

    String s = "";
    s += "CAN State : " + String(info.state)               + " (0=RUNNING, 2=BUS-OFF)\n";
    s += "TX Error  : " + String(info.tx_error_counter)    + " (128+ hat kopuk)\n";
    s += "RX Error  : " + String(info.rx_error_counter)    + "\n";
    s += "Missed    : " + String(info.rx_missed_count)      + "\n";
    s += "WiFi Clients: " + String(WiFi.softAPgetStationNum());

    httpServer.send(200, "text/plain", s);
}

// ── WiFi kurulum ──────────────────────────────────────────────────────────────
void setupWiFi() {
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("[WiFi] AP IP: ");
    Serial.println(WiFi.softAPIP());

    httpServer.on("/status", HTTP_GET, handleStatus);
    httpServer.on("/set",    HTTP_GET, handleSet);
    httpServer.on("/ping",   HTTP_GET, handlePing);
    httpServer.begin();
    Serial.println("[HTTP] Sunucu aktif.");
}

// ── Seri port komutları ───────────────────────────────────────────────────────
void handleSerial() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd.startsWith("motor")) {
        int m = cmd.substring(5).toInt();
        if (m >= 1 && m <= 6) {
            activeMotor = m;
            Serial.printf("\n>>> SEÇİLEN MOTOR: M%d\n\n", activeMotor);
        }
        return;
    }

    if (cmd.startsWith("pos")) {
        float deg = cmd.substring(3).toFloat();
        motors[activeMotor].targetDeg = deg;
        sendFloat(CMD_POSITION, deg, activeMotor);
        Serial.printf("[M%d] POZ → %.2f°\n", activeMotor, deg);
    }
    else if (cmd.startsWith("vel")) {
        float radps = cmd.substring(3).toFloat();
        motors[activeMotor].targetVel = radps;
        sendFloat(CMD_VELOCITY, radps, activeMotor);
        Serial.printf("[M%d] VEL → %.4f rad/s\n", activeMotor, radps);
    }
    else if (cmd.startsWith("kp")) { float v = cmd.substring(2).toFloat(); motors[activeMotor].kp = v; sendFloat(CMD_KP, v, activeMotor); }
    else if (cmd.startsWith("ki")) { float v = cmd.substring(2).toFloat(); motors[activeMotor].ki = v; sendFloat(CMD_KI, v, activeMotor); }
    else if (cmd.startsWith("kd")) { float v = cmd.substring(2).toFloat(); motors[activeMotor].kd = v; sendFloat(CMD_KD, v, activeMotor); }

    else if (cmd == "ping") {
        twai_status_info_t info;
        twai_get_status_info(&info);
        Serial.println("\n========= CAN BUS DETAYLI DURUM =========");
        Serial.printf("  Hattın Durumu (State)   : %d  (0:RUNNING, 2:BUS-OFF)\n", info.state);
        Serial.printf("  Giden Mesaj Hatası (TX) : %d  (128+ ise hat kopuk)\n",    info.tx_error_counter);
        Serial.printf("  Gelen Mesaj Hatası (RX) : %d\n", info.rx_error_counter);
        Serial.printf("  Kaçırılan Paket Sayısı  : %d\n", info.rx_missed_count);
        Serial.println("=========================================\n");
    }
    else if (cmd == "help") {
        Serial.println("\n--- KOMUTLAR ---");
        Serial.println("  motor<n>       Aktif motoru seç (1-6)");
        Serial.println("  pos <derece>   Açı komutu gönder");
        Serial.println("  vel <rad/s>    Hız komutu gönder");
        Serial.println("  kp/ki/kd <v>   PID parametre güncelle");
        Serial.println("  ping           CAN bus durumu\n");
    }
}

// ── Sürekli durum çıktısı (serial monitor) ───────────────────────────────────
void printContinuousStatus() {
    if (millis() - lastPrintTime < 150) return;
    lastPrintTime = millis();

    Motor& m = motors[activeMotor];
    Serial.printf(
        "[M%d] TarPos: %.2f° | CurPos: %.2f° | TarVel: %.4f rad/s | CurVel: %.4f rad/s | Kp:%.2f Ki:%.3f Kd:%.3f\n",
        activeMotor,
        m.targetDeg, m.currentDeg,
        m.targetVel, m.currentVel,
        m.kp, m.ki, m.kd
    );
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    initMotorProfiles();
    setupCAN();
    setupWiFi();
    Serial.println("Hazir. 'help' yazarak komutlari gorebilirsiniz.");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    httpServer.handleClient();   // WiFi isteklerini işle
    handleSerial();              // Seri port komutları
    handleCAN();                 // CAN mesajlarını al
    printContinuousStatus();     // 150ms'de bir yazdır
}
