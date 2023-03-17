/**************************************************************************************************************************************************
* File name     : ESP32DoorSensor.c
* Compiler      : 
* Autor         : VIGASAN    
* Created       : 14/03/2023
* Modified      : 
* Last modified :
*
*
* Description   : 
*
* Other info    : 
**************************************************************************************************************************************************/


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Include Files----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h> // You need to change the value of costant MQTT_MAX_PACKET_SIZE to 600 in the file PubSubClient.h 
                          // because the MQTT payload could exceed standard value of 256

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------I/O Definitions--------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const int KEEP_ON = 19;
const int IN_REED_NO = 5; 
const int IN_REED_NC = 2; 
const int INT_DRV_TPL5111 = 16;
const int OUT_DONE = 4;
const int STATUS_LED = 25;

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------MQTT DISCOVERY PARAMETERS----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const char*         g_ssid = "your Wifi Name";                              // Wifi Name
const char*         g_password = "Wifi Password";                           // Wifi Password
const char*         g_mqtt_server = "192.168.1.25";                         // MQTT Server IP, same of Home Assistant
const char*         g_mqttUser = "mqttUser";                                // MQTT Server User Name
const char*         g_mqttPsw = "password";                                 // MQTT Server password
int                 g_mqttPort = 1883;                                      // MQTT Server Port


// Variables used for MQTT Discovery
const char*         g_deviceModel = "ESP32IotSensor RevE";                  // Hardware Model
const char*         g_swVersion = "1.0";                                    // Firmware Version
const char*         g_manufacturer = "Vigasan";                             // Manufacturer Name
String              g_deviceName = "Door1";                                 // Device Name: YOU SEE THIS NAME ON HOME ASSISTANT DEVICES LIST
String              g_mqttStatusTopic = "esp32iotsensor/" + g_deviceName;   // MQTT Topic
            
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Public variables-------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
WiFiClient          g_WiFiClient;
PubSubClient        g_mqttPubSub(g_WiFiClient);
float               g_BatteryVoltage = 0.0;
int                 g_BatteryLevel = 0;
unsigned long       g_Time = 0;
int                 g_samples = 0;
int                 g_input_NO;   
int                 g_input_NC;  
String              g_strInputStatus;
int                 g_mqttCounterConn = 0;
bool                g_InitSystem = true;
String              g_UniqueId;
bool                g_WorkDone = false;
byte                g_CounterWork = 0;
           
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ SETUP ----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // I/O Configuration
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    analogSetWidth(11);
    analogSetAttenuation(ADC_11db);
    pinMode(IN_REED_NO, INPUT);
    pinMode(IN_REED_NC, INPUT);
    pinMode(INT_DRV_TPL5111, INPUT);
    pinMode(KEEP_ON, OUTPUT);
    pinMode(OUT_DONE, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);

    digitalWrite(KEEP_ON, HIGH);   
    digitalWrite(OUT_DONE, LOW);
    digitalWrite(STATUS_LED, HIGH);

    Serial.begin(115200);
    delay(500);

    Serial.println("");
    Serial.println("");
    Serial.println("----------------------------------------------");
    Serial.print("MODEL: ");
    Serial.println(g_deviceModel);
    Serial.print("DEVICE: ");
    Serial.println(g_deviceName);
    Serial.print("SW Rev: ");
    Serial.println(g_swVersion);
    Serial.println("----------------------------------------------");
   
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Configurazione Wifi
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    setup_wifi();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Configurazione MQTT
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    g_mqttPubSub.setServer(g_mqtt_server, g_mqttPort);
    g_mqttPubSub.setCallback(MqttReceiverCallback);
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ LOOP -----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void loop() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Connessione MQTT
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(WiFi.status() == WL_CONNECTED)
    {
        if(!g_mqttPubSub.connected())
            MqttReconnect();
        else
            g_mqttPubSub.loop();
    }

    if(g_InitSystem)
    {
        delay(100);
        g_InitSystem = false;
        Serial.println("INIT SYSTEM...");
        MqttHomeAssistantDiscovery();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Reads Battery Status and Inputs
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(millis() > g_Time + 500)    // Every 500 [msec]
    {
        g_Time = millis();

        g_samples++;
        g_BatteryVoltage += ((analogRead(35) * 0.9) / 284.0) + 0.381;           // Reads Battery Voltage with ADC
        
        if(g_samples == 4)                                                      // Average on 4 samples
        {
            g_BatteryVoltage = g_BatteryVoltage / g_samples;                    // Battery Volt
            g_BatteryLevel = 100 * (g_BatteryVoltage - 3.2) / (4.0 - 3.2);      // Converts Battery Volt in Battery Level [%]
            g_samples = 0;

            if(g_BatteryLevel > 100)
              g_BatteryLevel = 100;
            else if(g_BatteryLevel < 0)
              g_BatteryLevel = 0; 

            Serial.print("Battery Voltage: ");
            Serial.print(g_BatteryVoltage);
            Serial.println(" V");

            Serial.print("Battery Level: ");
            Serial.print(g_BatteryLevel);
            Serial.println(" %");

            ////////////////////////////////////////////////////////////////
            // READS REED RELAY STATUS
            ////////////////////////////////////////////////////////////////
            g_input_NO = digitalRead(IN_REED_NO); 
            g_input_NC = digitalRead(IN_REED_NC);
            
            if(g_input_NO == 0 && g_input_NC > 0)
            {
                Serial.println("CONTACT OPEN!!!");
                g_strInputStatus = "ON";
                g_WorkDone = true;
            } else if(g_input_NO > 0 && g_input_NC == 0)
            {
                Serial.println("CONTACT CLOSED!!!");
                g_strInputStatus = "OFF";
                g_WorkDone = true;
            } else
            {
                Serial.println("Undefined!!!");
                if(g_CounterWork++ > 2)
                    g_WorkDone = true;
            }

            Serial.print("INPUT_NO: ");
            Serial.println(g_input_NO);
            Serial.print("INPUT_NC: ");
            Serial.println(g_input_NC);
            
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // SEND MQTT DATA
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
            if(g_WorkDone == true)
           
            {
                StaticJsonDocument<200> payload;
                payload["batt"] = g_BatteryLevel;
                payload["volt"] = Round2(g_BatteryVoltage);
                payload["inputstatus"] = g_strInputStatus;

                String strPayload;
                serializeJson(payload, strPayload);

                if(g_mqttPubSub.connected())
                {
                    g_mqttPubSub.publish(g_mqttStatusTopic.c_str(), strPayload.c_str()); 
                    Serial.println("MQTT: Send Data!!!");
                }
                    
                delay(150);
                // SHUTDOWN
                Serial.println("Shutdown");
                Serial.flush(); 
                digitalWrite(OUT_DONE, LOW);
                delay(100);
                digitalWrite(OUT_DONE, HIGH);
                digitalWrite(KEEP_ON, LOW);
                delay(100);
                Serial.println("  ");
                Serial.println("This will never be printed");   
            }
            Serial.println(" ");
            Serial.println(" ");
            Serial.println(" ");
            Serial.println(" ");
            g_BatteryVoltage = 0.0;
            g_input_NO = 0;
            g_input_NC = 0;
        }
    }
}


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Public Functions -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup_wifi() 
{
    int counter = 0;
    byte mac[6];
    delay(10);
    // We start by connecting to a WiFi network
    Serial.print("Connecting to ");
    Serial.println(g_ssid);

    WiFi.begin(g_ssid, g_password);

    WiFi.macAddress(mac);
    g_UniqueId =  String(mac[0],HEX) +String(mac[1],HEX) +String(mac[2],HEX) +String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX);

    Serial.print("Unique ID: ");
    Serial.println(g_UniqueId);    
   
    while(WiFi.status() != WL_CONNECTED && counter++ < 8) 
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("");

    if(WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else
    {
        Serial.println("WiFi NOT connected!!!");
    }
}

void MqttReconnect() 
{
    // Loop until we're reconnected
    while (!g_mqttPubSub.connected()  && (g_mqttCounterConn++ < 4))
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (g_mqttPubSub.connect(g_deviceName.c_str(), g_mqttUser, g_mqttPsw)) 
        {
            Serial.println("connected");
            // Subscribe
            g_mqttPubSub.subscribe("homeassistant/status");
            delay(100);
        } else 
        {
            Serial.print("failed, rc=");
            Serial.print(g_mqttPubSub.state());
            Serial.println(" try again in 1 seconds");
            delay(1000);
        }
    }  
    g_mqttCounterConn = 0;
}

void MqttHomeAssistantDiscovery()
{
    String discoveryTopic;
    String payload;
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        Serial.println("SEND HOME ASSISTANT DISCOVERY!!!");
        StaticJsonDocument<600> payload;
        JsonObject device;
        JsonArray identifiers;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Battery Level
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_batt" + "/config";
        
        payload["name"] = g_deviceName + ".batt";
        payload["uniq_id"] = g_UniqueId + "_batt";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "battery";
        payload["val_tpl"] = "{{ value_json.batt | is_defined }}";
        payload["unit_of_meas"] = "%";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Battery Voltage
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_volt" + "/config";
        
        payload["name"] = g_deviceName + ".volt";
        payload["uniq_id"] = g_UniqueId + "_volt";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "voltage";
        payload["val_tpl"] = "{{ value_json.volt | is_defined }}";
        payload["unit_of_meas"] = "V";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
         
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Binary Door
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        discoveryTopic = "homeassistant/binary_sensor/esp32iotsensor/" + g_deviceName + "_door" + "/config";
        
        payload["name"] = g_deviceName + ".door";
        payload["uniq_id"] = g_UniqueId + "_door";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "door";
        payload["val_tpl"] = "{{ value_json.inputstatus | is_defined }}";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
    }
}

void MqttReceiverCallback(char* topic, byte* inFrame, unsigned int length) 
{
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    byte state = 0;
    String messageTemp;
    
    for (int i = 0; i < length; i++) 
    {
        Serial.print((char)inFrame[i]);
        messageTemp += (char)inFrame[i];
    }
    Serial.println();
  
    if(String(topic) == String("homeassistant/status")) 
    {
        if(messageTemp == "online")
            MqttHomeAssistantDiscovery();
    }
}

float Round(float value) 
{
   return (int)(value * 10 + 0.5) / 10.0;
}

float Round2(float value) 
{
   return (int)(value * 100 + 0.5) / 100.0;
}
