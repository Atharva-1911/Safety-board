#include <Arduino.h>
#include "ModbusServerRTU.h"
#include "RTUutils.h"
#include "ModbusTypeDefs.h"

// things defined in main.cpp
#define RXD2 16
#define TXD2 17
#define RTS  4
#define REG_ADDR 0x0000

extern volatile uint16_t modbus_value;

ModbusServerRTU mb(2000, RTS);   // 2s timeout, RTS pin for RS485 direction

ModbusMessage FC06(ModbusMessage request) {
    uint16_t address, mask;
    ModbusMessage response;

    request.get(2, address);
    request.get(4, mask);

    if (address != REG_ADDR) {
        response.setError(request.getServerID(),
                          request.getFunctionCode(),
                          ILLEGAL_DATA_ADDRESS);
        return response;
    }

    uint16_t toggle_bits = ~mask & 0xFFFF;
    modbus_value ^= toggle_bits;

    response = request;
    return response;
}

void setup_modbus() {
    RTUutils::prepareHardwareSerial(Serial2);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    mb.registerWorker(1, WRITE_HOLD_REGISTER, &FC06);
    mb.begin(Serial2);
    pinMode(RTS, OUTPUT);

    Serial.println("Modbus ready, waiting...");
}