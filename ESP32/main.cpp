#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <SPI.h>
#include "comms.h"
#include <Adafruit_NeoPixel.h>
#include <ESPAsyncWebServer.h>
#include <MycilaWebSerial.h>
#include "SPIFFS.h"
#include <ESPmDNS.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_VL53L7CX.h>

// ─── Pinos SPI ───────────────────────────────────────────────────────────────
#define SPI_SS   10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCLK 13

// ─── Pinos I2C (LIDAR) ───────────────────────────────────────────────────────
#define SDA_LIDAR 8
#define SCL_LIDAR 9

// ─── Pinos enable do sensor ───────────────────────────────────────────────────
#define LIDAR_RST  47
#define LIDAR_LPN  48

// ─── LED RGB ─────────────────────────────────────────────────────────────────
#define RGB_LED_PIN 38

// ─── Modo WiFi (1 = Access Point, 0 = Station) ───────────────────────────────
#define WIFI_ACCESS_POINT 1

// ─── Odometria ───────────────────────────────────────────────────────────────
float valorx, valory, valortheta;

// Valores Lidar
float medcentroid = 0;
float DistEsq = 0;
float DistDir = 0;

// ─── Sensor VL53L7CX ─────────────────────────────────────────────────────────
Adafruit_VL53L7CX vl53l7cx;
VL53L7CX_ResultsData lidarResults;
SemaphoreHandle_t lidarMutex;
bool lidarOk = false;

// Matriz global para guardar as distâncias do LIDAR (4x4)
uint16_t distancias[4][4] = {0};

// Função para calcular a média de todo o array de distâncias
float mediaLinha(uint16_t dist[4][4], int linha)
{
    uint32_t soma = 0;
    int count = 0;

    for (int coluna = 1; coluna < 3; coluna++) {
        if (dist[linha][coluna] > 0) {
            soma += dist[linha][coluna];
            count++;
        }
    }
    if (count == 0) return -1;
    if ((float)soma / count < 0.0f) return 0.0f; 
    return (float)soma / count;
}

// ─── Protótipos ──────────────────────────────────────────────────────────────
void Task1code(void *parameter);
void TaskBootBtnCode(void *parameter);
void TaskReadLidar(void *parameter);
void transferReceivePackets(comms_packet_t *packet);

// ─── Handles de tasks ────────────────────────────────────────────────────────
TaskHandle_t Task1;
TaskHandle_t TaskBootBtnHandler;
TaskHandle_t TaskLidarHandler;


// ─── Comunicações ────────────────────────────────────────────────────────────
PacketHandler packetHandler;

// ─── Periféricos ─────────────────────────────────────────────────────────────
Adafruit_NeoPixel rgbLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
AsyncWebServer server(80);
WebSerial webSerial;

// ─── WiFi ────────────────────────────────────────────────────────────────────
const char *ssid     = "SE_robo_RG";
const char *password = "";
const char *mdnsName = "robot";

uint8_t cmd = 0;


// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    // ── Serial primeiro para não perder mensagens ─────────────────────────────
    Serial.begin(115200);
    delay(3000); // espera USB CDC estar pronto
    Serial.println("A iniciar...");

    if (!SPIFFS.begin(true)) {
        Serial.println("Erro ao montar SPIFFS");
        return;
    }

    // ── Pinos SPI ────────────────────────────────────────────────────────────
    pinMode(SPI_SS,   OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, OUTPUT);
    pinMode(SPI_SCLK, INPUT);
    pinMode(GPIO_NUM_0, INPUT_PULLUP);

    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(1000000);

    // ── Power enable do sensor PRIMEIRO (sequência VL53L7CX) ────────────────
    digitalWrite(LIDAR_RST, LOW);
    digitalWrite(LIDAR_LPN, LOW);
    delay(10);

    digitalWrite(LIDAR_RST, HIGH);
    digitalWrite(LIDAR_LPN, HIGH);  
    delay(500);

    // ── I2C: iniciar barramento primeiro ──────────────────────────────────────
    Wire.begin(SDA_LIDAR, SCL_LIDAR,400000);
    delay(1000); // mais tempo para o sensor estabilizar

    // ── Debug: verificar dispositivos I2C ────────────────────────────────────
    Serial.println("A procurar dispositivos I2C...");
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("Dispositivo I2C encontrado no endereco: 0x");
            Serial.println(address, HEX);
            nDevices++;
        }
    }
    if (nDevices == 0) {
        Serial.println("Nenhum dispositivo I2C encontrado!");
    }

    // ── Inicializar VL53L7CX ─────────────────────────────────────────────────
    Serial.println("Inicializando VL53L7CX...");
    if (!vl53l7cx.begin(VL53L7CX_DEFAULT_ADDRESS, &Wire)) {
        Serial.println("VL53L7CX nao encontrado! Verifica ligacoes I2C.");
        Serial.println("Endereco esperado: 0x" + String(VL53L7CX_DEFAULT_ADDRESS, HEX));
        lidarOk = false;
    } else {
        Serial.println("VL53L7CX inicializado com sucesso!");

        vl53l7cx.setResolution(16);       // 4x4 = 16 zonas
        vl53l7cx.setRangingFrequency(10); // 10 Hz

        if (!vl53l7cx.startRanging()) {
            Serial.println("Erro ao iniciar ranging!");
            lidarOk = false;
        } else {
            Serial.println("Ranging iniciado!");
            lidarOk = true;
            lidarMutex = xSemaphoreCreateMutex();

            xTaskCreatePinnedToCore(
                TaskReadLidar,
                "TaskLidar",
                10000,
                NULL,
                1,
                &TaskLidarHandler,
                0); // Core 0
        }
    }

    // ── Task do botao boot ───────────────────────────────────────────────────
    xTaskCreatePinnedToCore(
        TaskBootBtnCode,
        "TaskBootBtn",
        10000,
        NULL,
        0,
        &TaskBootBtnHandler,
        0);

    // ── WiFi ─────────────────────────────────────────────────────────────────
#if WIFI_ACCESS_POINT == 1
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
#else
    WiFi.mode(WIFI_STA);
    WiFi.begin("ssid", "pass");
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
#endif

    // ── mDNS ─────────────────────────────────────────────────────────────────
    if (!MDNS.begin(mdnsName)) {
        Serial.println("Erro ao configurar mDNS!");
        while (1) delay(1000);
    }
    MDNS.addService("_http", "_tcp", 80);

    // ── Task de comunicacao SPI ──────────────────────────────────────────────
    xTaskCreatePinnedToCore(
        Task1code,
        "Task1",
        10000,
        NULL,
        0,
        &Task1,
        1); // Core 1

    // ── WebSerial ────────────────────────────────────────────────────────────
    webSerial.onMessage([](const std::string &msg) {
        cmd = msg[0];
        xTaskNotifyGive(Task1);
    });
    webSerial.begin(&server);
    webSerial.setBuffer(100);

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->redirect("/index.html");
    });
    server.serveStatic("/", SPIFFS, "/");
    server.begin();

    // ── LED RGB ──────────────────────────────────────────────────────────────
    rgbLed.begin();
    rgbLed.setBrightness(10);

    Serial.println("Setup completo!");
}
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    xTaskNotifyGive(TaskLidarHandler);
    xTaskNotifyGive(Task1);
    delay(1000);

    struct timeval tv;
    gettimeofday(&tv, NULL);

    int horas    = (tv.tv_sec / 3600) % 24;
    int minutos  = (tv.tv_sec / 60) % 60;
    int segundos =  tv.tv_sec % 60;

    webSerial.printf("------------------------------------------------------------------------------\n");
    webSerial.printf("[%02d:%02d:%02d] x= %.2f mm | y= %.2f mm | theta= %.4f rad\n",
                     horas, minutos, segundos,
                     valorx, valory, valortheta);

    // Imprimir grelha 4x4 de distancias no webSerial
    if (lidarOk && lidarMutex != NULL &&
        xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        webSerial.printf("LIDAR 4x4 (mm):\n");
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                int idx        = row * 4 + col;
                int16_t dist   = lidarResults.distance_mm[idx];
                uint8_t status = lidarResults.target_status[idx];
                if (status == 5 && dist > 0)
                    webSerial.printf("[%4d] ", dist);
                else
                    webSerial.printf("[----] ");
            }
            webSerial.printf("\n");
        }
        
        xSemaphoreGive(lidarMutex);
    }
    webSerial.printf("Dist Mid = %.2f mm\n", medcentroid);
    webSerial.printf("Dist Dir = %.2f mm\n", DistDir);
    webSerial.printf("Dist Esq = %.2f mm\n", DistEsq);
    
    webSerial.flush();
}

// ─── Task: leitura continua do LIDAR ────────────────────────────────────────
void TaskReadLidar(void *parameter)
{
    for (;;)
    {
        if (vl53l7cx.isDataReady()) {
            VL53L7CX_ResultsData tempResults;
            if (vl53l7cx.getRangingData(&tempResults)) {
                // Guardar distâncias na matriz global
                for (int row = 0; row < 4; row++) {
                    for (int col = 0; col < 4; col++) {
                        int idx = row * 4 + col;
                        distancias[row][col] = tempResults.distance_mm[idx];
                    }
                }

                if (xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    memcpy(&lidarResults, &tempResults, sizeof(VL53L7CX_ResultsData));
                    xSemaphoreGive(lidarMutex);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    } 
}

// ─── Task: comunicacao SPI + odometria ──────────────────────────────────────
void Task1code(void *parameter)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (;;)
    {
        uint32_t value = cmd;

        // Echo
        packetHandler.buildPacket(COMMS_TYPE_ECHO, 0, (uint8_t *)&value);
        transferReceivePackets(&packetHandler.packet);
        if (packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR) {
            value = *((uint32_t *)packetHandler.packet.data);
        }
        vTaskDelay(pdMS_TO_TICKS(10));


        // Theta (registo 3)
        packetHandler.buildPacket(COMMS_TYPE_READ, 3, (uint8_t *)&value);
        transferReceivePackets(&packetHandler.packet);
        if (packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR) {
            valortheta = *((float *)packetHandler.packet.data);
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        // X (registo 4)
        packetHandler.buildPacket(COMMS_TYPE_READ, 4, (uint8_t *)&value);
        transferReceivePackets(&packetHandler.packet);
        if (packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR) {
            valorx = *((float *)packetHandler.packet.data);
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        // Y (registo 5)
        packetHandler.buildPacket(COMMS_TYPE_READ, 5, (uint8_t *)&value);
        transferReceivePackets(&packetHandler.packet);
        if (packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR) {
            valory = *((float *)packetHandler.packet.data);
        }

        float col1 = mediaLinha(distancias, 1);
        float col2 = mediaLinha(distancias, 2); 
        medcentroid = (col1 + col2) / 2.0;

        packetHandler.buildPacket(COMMS_TYPE_WRITE, 6, (uint8_t *)&medcentroid);
        transferReceivePackets(&packetHandler.packet);

        if (packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR) {
            //Serial.printf("Write registo 6 enviado: %.2f mm\n", medcentroid);   
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        // Distância esquerda
        DistEsq = mediaLinha(distancias, 0);
        
        packetHandler.buildPacket(COMMS_TYPE_WRITE, 7, (uint8_t *)&DistEsq);
        transferReceivePackets(&packetHandler.packet);
        vTaskDelay(pdMS_TO_TICKS(10));

        // Distância direita
        DistDir = mediaLinha(distancias, 3); 
        
        packetHandler.buildPacket(COMMS_TYPE_WRITE, 8, (uint8_t *)&DistDir);
        transferReceivePackets(&packetHandler.packet);
        vTaskDelay(pdMS_TO_TICKS(10));

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

// ─── Transferencia SPI ───────────────────────────────────────────────────────
void transferReceivePackets(comms_packet_t *packet)
{
    digitalWrite(SPI_SS, LOW);
    SPI.transferBytes((uint8_t *)packet, NULL, sizeof(comms_packet_t));
    delay(10);
    SPI.transferBytes(NULL, (uint8_t *)packet, sizeof(comms_packet_t));
    digitalWrite(SPI_SS, HIGH);
}

// ─── Task: botao boot (NAO APAGAR) ──────────────────────────────────────────
void TaskBootBtnCode(void *parameter)
{
    static bool toogle = false;
    for (;;)
    {
        if (digitalRead(GPIO_NUM_0) == LOW) {
            toogle = !toogle;
            rgbLed.setPixelColor(0, toogle ? rgbLed.Color(255, 0, 255)
                                           : rgbLed.Color(0, 0, 0));
        } else {
            rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));
        }
        rgbLed.show();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
