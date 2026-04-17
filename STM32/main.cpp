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

#define SPI_SS 10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCLK 13

#define RGB_LED_PIN 38

#define WIFI_ACCESS_POINT 1

float valorx, valory, valortheta;

void Task1code(void *parameter);
void TaskBootBtnCode(void *parameter);
void transferReceivePackets(comms_packet_t *packet);

TaskHandle_t Task1;
TaskHandle_t TaskBootBtnHandler;
PacketHandler packetHandler;
Adafruit_NeoPixel rgbLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
AsyncWebServer server(80);
WebSerial webSerial;

const char *ssid = "SE_robo"; // WiFi AP SSID
const char *password = "";    // WiFi AP Password

// mDNS name (customize this as needed)
const char *mdnsName = "robot"; // http://robot ou http://robot.local in the browser

uint8_t cmd = 0;

void setup()
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    pinMode(SPI_SS, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, OUTPUT);
    pinMode(SPI_SCLK, INPUT);
    pinMode(GPIO_NUM_0, INPUT_PULLUP);

    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(1000000);

    Serial.begin(115200);
    Serial.setDebugOutput(true);

#if WIFI_ACCESS_POINT == 1 // create a wifi network
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    // Once connected, print IP
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP()); // http://192.168.4.1/

#else // connect to a wifi network
    WiFi.mode(WIFI_STA);
    WiFi.begin("ssid", "pass");
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
#endif

    // Set mDNS
    if (!MDNS.begin(mdnsName))
    {
        Serial.println("Error setting up MDNS responder!");
        while (1)
        {
            delay(1000);
        }
    }
    MDNS.addService("_http", "_tcp", 80);

    xTaskCreatePinnedToCore(
        Task1code, /* Function to implement the task */
        "Task1",   /* Name of the task */
        10000,     /* Stack size in words */
        NULL,      /* Task input parameter */
        0,         /* Priority of the task */
        &Task1,    /* Task handle. */
        1);        /* Core where the task should run */

    webSerial.onMessage([](const std::string &msg)
                        {   
                                cmd = msg[0]; 
                                xTaskNotifyGive(Task1); });
    webSerial.begin(&server);
    webSerial.setBuffer(100);

    server.onNotFound([](AsyncWebServerRequest *request)
                      { request->redirect("/index.html"); });

    // Route for root / web page
    server.serveStatic("/", SPIFFS, "/");
    // Start server
    server.begin();

    rgbLed.begin();
    rgbLed.setBrightness(10);
}

void loop()
{
    /// print dos 1 Hz
    xTaskNotifyGive(Task1); 
    delay(1000); 

    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    int horas   = (tv.tv_sec / 3600) % 24;
    int minutos = (tv.tv_sec / 60) % 60;
    int segundos = tv.tv_sec % 60;

    webSerial.printf("------------------------------------------------------------------------------\n");
    webSerial.printf("[%02d:%02d:%03d] Valor x= %.2f mm, Valor y= %.2f mm, Valor theta= %.4f rad\n",
                     horas, minutos, segundos,
                     valorx, valory, valortheta);
    webSerial.flush();
}

void Task1code(void *parameter)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (;;)
    {
        uint32_t value = cmd;
        packetHandler.buildPacket(COMMS_TYPE_ECHO, 0, (uint8_t *)&value);
        Serial.print("original ");
        packetHandler.print();

        transferReceivePackets(&packetHandler.packet);
        Serial.print("echo ");
        packetHandler.print();

        if (packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR){
            value = *((uint32_t *)packetHandler.packet.data);
        //    webSerial.printf("%c, Recebido= %d\n", (uint8_t)value, value-'0');
        //    webSerial.flush();
        }

        vTaskDelay(pdMS_TO_TICKS(10));

        //  meu codigo esta a fazer connecting time out
        /*
        packetHandler.buildPacket(COMMS_TYPE_READ,1,(uint8_t *)&value);
        transferReceivePackets(&packetHandler.packet);
        //Serial.printf("estou aqui");
        webSerial.flush();
        if(packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR){
            float myvalue = *((float *)packetHandler.packet.data);
            //memcpy(&myvalue, packetHandler.packet.data, 4);
            webSerial.printf("left encoder value= %f\n", myvalue);
            webSerial.flush();
        }

        vTaskDelay(pdMS_TO_TICKS(10));

        packetHandler.buildPacket(COMMS_TYPE_READ,2,(uint8_t *)&value);
        transferReceivePackets(&packetHandler.packet);
        webSerial.flush();
        if(packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR){
            float myvalue = *((float *)packetHandler.packet.data);
            webSerial.printf("right encoder value= %f\n", myvalue);
            webSerial.flush();
        }
        // end my code */
        // receber odometria

        vTaskDelay(pdMS_TO_TICKS(10));

        packetHandler.buildPacket(COMMS_TYPE_READ, 3, (uint8_t *)&value);
        transferReceivePackets(&packetHandler.packet);
        webSerial.flush();
        if (packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR)
        {
            valortheta = *((float *)packetHandler.packet.data);
            // webSerial.printf("Valor theta= %f rad\n", myvalue);
            // webSerial.flush();
        }

        vTaskDelay(pdMS_TO_TICKS(10));

        packetHandler.buildPacket(COMMS_TYPE_READ, 4, (uint8_t *)&value);
        transferReceivePackets(&packetHandler.packet);
        webSerial.flush();
        if (packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR)
        {
            valorx = *((float *)packetHandler.packet.data);
            // webSerial.printf("Valor x= %f mm\n", myvalue);
            // webSerial.flush();
        }

        vTaskDelay(pdMS_TO_TICKS(10));

        packetHandler.buildPacket(COMMS_TYPE_READ, 5, (uint8_t *)&value);
        transferReceivePackets(&packetHandler.packet);
        webSerial.flush();

        if (packetHandler.isPacketValid() && packetHandler.packet.packet_type != COMMS_TYPE_ERR)
        {
            valory = *((float *)packetHandler.packet.data);
            // webSerial.printf("Valor y= %f mm\n", myvalue);
            // webSerial.flush();
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void transferReceivePackets(comms_packet_t *packet)
{
    digitalWrite(SPI_SS, LOW); // pull SS slow to prep other for transfer
    SPI.transferBytes((uint8_t *)packet, NULL, sizeof(comms_packet_t));
    delay(10);
    SPI.transferBytes(NULL, (uint8_t *)packet, sizeof(comms_packet_t));
    digitalWrite(SPI_SS, HIGH); // pull ss high to signify end of data transfer
}

/*
 * DO NOT DELETE THIS TASK
 */
void TaskBootBtnCode(void *parameter)
{
    static bool toogle = false;
    if (digitalRead(GPIO_NUM_0) == LOW)
    {
        if (toogle)
        {
            toogle = false;
            rgbLed.setPixelColor(0, rgbLed.Color(255, 0, 255));
        }
        else
        {
            toogle = true;
            rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));
        }
    }
    else
    {
        rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));
    }
    rgbLed.show();
    delay(100);
}
