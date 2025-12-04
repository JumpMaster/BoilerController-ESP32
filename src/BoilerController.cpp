#define DIAGNOSTIC_PIXEL

#include "BoilerController.h"

// Stubs
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishboilerActive(bool state);

void setBoiler(bool state)
{
    digitalWrite(RELAY_PIN, state);
    publishboilerActive(state);
    boilerActive = state;

    if (state)
    {
        boilerAutoOffTime = millis() + maxBoilerRuntime;
    }
    else
    {
        boilerAutoOffTime = 0;   
    }
    
    Log.printf("Boiler switched %s\n", boilerActive ?  "on" : "off");
}

void checkBoilerRelay()
{
    bool state = digitalRead(RELAY_SENSOR_PIN);
    if (state != relaySensorState)
    {
        relaySensorState = state;
        standardFeatures.mqttPublish(mqttRelaySensor.getStateTopic().c_str(), relaySensorState ? "ON" : "OFF", true);
        
        if (relaySensorState)
            standardFeatures.setDiagnosticPixelColor(pixelBoilerActiveColor);
        else
            standardFeatures.setDiagnosticPixelColor(StandardFeatures::NEOPIXEL_BLACK);
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    char data[length + 1];
    memcpy(data, payload, length);
    data[length] = '\0';

    if (strcmp(mqttRebootButton.getCommandTopic().c_str(), topic) == 0) // Restart Topic
    {
        ESP.restart();
    }
    else if (strcmp(topic, mqttBoilerControlSwitch.getCommandTopic().c_str()) == 0)
    {
        setBoiler(strcmp(data, "ON") == 0);
    }
}

void publishboilerActive(bool state)
{
    standardFeatures.mqttPublish(mqttBoilerControlSwitch.getStateTopic().c_str(), state ? "ON" : "OFF", true);
}

void checkDeviceConnectionState()
{
    unsigned long currentMillis = millis();

    if (!standardFeatures.isMQTTConnected() && boilerMode != BOILER_SAFTEY_MODE)
    {
        if (boilerMode == BOILER_NORMAL)
        {
            boilerMode = BOILER_SAFTEY_MODE_PENDING;
            safteyModeStartTime = millis() + safteyBufferTime;
            Log.println("Boiler mode set to saftey pending");
        }
        else if (boilerMode == BOILER_SAFTEY_MODE_PENDING && millis() > safteyModeStartTime)
        {
            boilerMode = BOILER_SAFTEY_MODE;
            Log.println("Boiler mode set to saftey");
        }
    }
    else if (standardFeatures.isMQTTConnected() && boilerMode != BOILER_NORMAL)
    {
        boilerMode = BOILER_NORMAL;
        Log.println("Boiler mode set to normal");
    }

    if (currentMillis > nextboilerActivePublish)
    {
        publishboilerActive(boilerActive);
        nextboilerActivePublish = currentMillis + boilerActivePublishFrequency;
    }

    if (boilerAutoOffTime != 0 && boilerActive && currentMillis > boilerAutoOffTime)
    {
        setBoiler(false);
        Log.println("Boiler switched off due to exceeding max runtime");
    }

    if (boilerMode == BOILER_SAFTEY_MODE && boilerActive == true)
    {
        setBoiler(false);
        Log.println("Boiler switched off due to saftey mode");
    }
}

void onMQTTConnect()
{
    standardFeatures.mqttPublish(parentMQTTDevice.getConfigTopic().c_str(), parentMQTTDevice.getConfigPayload().c_str(), true);

    standardFeatures.mqttPublish(mqttBoilerControlSwitch.getStateTopic().c_str(), relaySensorState ? "ON" : "OFF", true);
    standardFeatures.mqttPublish(mqttRelaySensor.getStateTopic().c_str(), relaySensorState ? "ON" : "OFF", true);

    standardFeatures.mqttSubscribe(mqttRebootButton.getCommandTopic().c_str());
    standardFeatures.mqttSubscribe(mqttBoilerControlSwitch.getCommandTopic().c_str());
}

void setupLocalMQTT()
{
    parentMQTTDevice.addHAMqttDevice(&mqttRelaySensor);
    parentMQTTDevice.addHAMqttDevice(&mqttBoilerControlSwitch);
    parentMQTTDevice.addHAMqttDevice(&mqttRebootButton);

    standardFeatures.setMqttCallback(mqttCallback);
    standardFeatures.setMqttOnConnectCallback(onMQTTConnect);
}

void setup()
{
    pinMode(GPIO_NUM_0, INPUT); // BOOT BUTTON
    
    pinMode(GPIO_NUM_38, OUTPUT); // ONBOARD NEOPIXEL POWER
    digitalWrite(GPIO_NUM_38, LOW); // LOW = OFF, HIGH = ON

    pinMode(RELAY_PIN, OUTPUT); // RELAY
    digitalWrite(RELAY_PIN, LOW);

    pinMode(RELAY_SENSOR_PIN, INPUT); // RELAY SENSOR

    standardFeatures.enableLogging(deviceName, syslogServer, syslogPort);
    standardFeatures.enableDiagnosticPixel(18);
    standardFeatures.enableWiFi(wifiSSID, wifiPassword, deviceName);
    standardFeatures.enableOTA(deviceName, otaPassword);
    standardFeatures.enableSafeMode(appVersion);
    standardFeatures.enableMQTT(mqttServer, mqttUsername, mqttPassword, deviceName);

    setupLocalMQTT();
}

void loop()
{
    standardFeatures.loop();

    if (standardFeatures.isOTARunning())
        return;

    checkDeviceConnectionState();

    checkBoilerRelay();
}