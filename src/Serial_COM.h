// version: 1.1.0
#pragma once

#ifndef Serial_COM_H
#define Serial_COM_H
#include <Arduino.h>

class SerialCom
{
private:
    unsigned long time_serial1 = 0;
    unsigned long time_serial2 = 0;
    // Serial2を送るときに使う
    bool sendFlag = false;
    char sendChar = '\0';

    uint64_t serialFrequency = 500000;

    char commandReturn = 'j';
    char commandDelete = 'd';

public:
    void setup(char setCommandReturn = 'j', char setCommandDelete = 'd', uint64_t setSerialFrequency = 500000);
    void sendSerial2();
    void setCommand(char command);
    void stopCommand();
    // マルチタスク用
    static void sendTask(void *pvParameters);
};

// 引数は指定しなくていい。デフォルト値がある
void SerialCom::setup(char setCommandReturn, char setCommandDelete, uint64_t setSerialFrequency)
{
    commandReturn = setCommandReturn;
    commandDelete = setCommandDelete;
    serialFrequency = setSerialFrequency;
    time_serial1 = micros();
}

// この関数はループの中で呼び出す
void SerialCom::sendSerial2()
{
    if (sendFlag)
    {
        time_serial2 = micros();
        if (time_serial2 - time_serial1 > serialFrequency)
        {
            time_serial1 = time_serial2;
            Serial2.write(sendChar);
            Serial.printf("sendChar1: %c", sendChar);
            Serial.print("\n");
        }
    }
}

void SerialCom::setCommand(char command)
{
    sendFlag = true;
    sendChar = command;
}

void SerialCom::stopCommand()
{
    sendFlag = false;
}

// マルチタスク用
// SPI Flashのデータを削除しているときのみ使用
// 呼び出し方は以下の通り。適宜呼び出し側の変数を変える
// xTaskCreatePinnedToCore(Log67Serial1.sendTask, "sendTask1", 8192, NULL, 2, &taskHandle, 0);
void SerialCom::sendTask(void *pvParameters)
{
    SerialCom serialCom;
    serialCom.setup();
    serialCom.setCommand(serialCom.commandDelete);
    while (1)
    {
        serialCom.sendSerial2();

        char pre3 = Serial2.read();
        if (pre3 == serialCom.commandReturn) // 'j'
        {
            Serial.println("return text2");
            serialCom.sendFlag = false;
        }
        vTaskDelay(1);
    }
}

#endif