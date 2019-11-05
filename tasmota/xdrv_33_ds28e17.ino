/*
  xdrv_33_onewire.ino - Support for DS28E17 1-Wire to I2C Master bus bridge

  Copyright (C) 2019  Andre Thomas and Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifdef USE_I2C
#ifdef USE_1WIRE
#ifdef USE_DS28E17

#define XDRV_33                    33

#define DS28E17_CHIPID       0x19  

#define ONEWIRE_TIMEOUT 50

#define W1_WRITE_DATA_WITH_STOP   0x4B
#define W1_READ_DATA_WITH_STOP    0x87
#define W1_WRITE_CONFIG           0xD2
#include <Wire.h>
//#define NO_GLOBAL_TWOWIRE

class OneWireTwoWireBridge: public TwoWire {

private:
    static uint8_t rxBuffer[];
    static uint8_t rxBufferIndex;
    static uint8_t rxBufferLength;

    static uint8_t txAddress;
    static uint8_t txBuffer[];
    static uint8_t txBufferIndex;
    static uint8_t txBufferLength;

    uint8_t bridgeAddress[8];

public:
    void begin(int sda, int scl) {
      //pins ignored
      //initialize
    } 

    void begin(int sda, int scl, uint8_t address) {} //no slave support
    void begin(uint8_t address) {} //no slave support
    
    
    void setClock(uint32_t frequency) {
      //set speed (only applicable)
    }

    size_t requestFrom(uint8_t address, size_t size, bool sendStop) {
      
      if (size > BUFFER_LENGTH)
      {
        size = BUFFER_LENGTH;
      }

      uint8_t header[3];
      uint8_t headerLength = 3;

      header[0] = W1_READ_DATA_WITH_STOP; // DS28E17 Command
      header[1] = address << 1 | 0x01;    // 7 bit i2c Address
      header[2] = size;           // number of bytes to be read

      uint8_t crc[2];
      uint16_t crc16 = OneWireCrc16(&header[0], headerLength);
      crc16 = ~crc16;
      crc[1] = crc16 >> 8;                 
      crc[0] = crc16 & 0xFF;               
      
      OneWireReset();
      OneWireSelect(bridgeAddress);
      OneWireWrite(header, headerLength);
      OneWireWrite(crc, sizeof(crc));

      uint8_t timeout = 0;
      while (OneWireReadBit() == true){
        delay(1);
        timeout++;
        if (timeout > ONEWIRE_TIMEOUT){
          //oneWire->depower();
          return 0;
        }
      }
      
      uint8_t stat = OneWireRead();
      uint8_t writeStat = OneWireRead();
      
      if ((stat != 0x00) || (writeStat != 0x00)) {
        //oneWire->depower();
        return 0;
      }

      rxBufferLength = 0;
      for (int i=0; i<size; i++){
        rxBuffer[i] = OneWireRead();        
        rxBufferLength++;  
      }

      //oneWire->depower(); 
      
      rxBufferIndex = 0;
      return rxBufferLength;
    }

    void beginTransmission(uint8_t address)
    {
        txAddress = address;
        txBufferIndex = 0;
        txBufferLength = 0;
    }    

    size_t write(uint8_t data)
    {
        if (txBufferLength >= BUFFER_LENGTH)
        {
            setWriteError();
            return 0;
        }
        txBuffer[txBufferIndex] = data;
        ++txBufferIndex;
        txBufferLength = txBufferIndex;

        return 1;
    }

    size_t write(const uint8_t *data, size_t quantity)
    {
        for (size_t i = 0; i < quantity; ++i)
        {
            if (!write(data[i]))
            {
                return i;
            }
        }
        return quantity;
    }

    int available(void)
    {
        int result = rxBufferLength - rxBufferIndex;

        if (!result)
        {
            // yielding here will not make more data "available",
            // but it will prevent the system from going into WDT reset
            optimistic_yield(1000);
        }

        return result;
    }

    uint8_t endTransmission(uint8_t sendStop) {

      uint8_t header[3];      

      header[0] = W1_WRITE_DATA_WITH_STOP;        // DS28E17 Command
      header[1] = txAddress << 1;      // 7 bit i2c Address
      header[2] = txBufferLength;           // number of bytes to be written 

      uint8_t crc[2];
      uint16_t crc16 = OneWireCrc16(&header[0], 3);
      crc16 = OneWireCrc16(&txBuffer[0], txBufferLength, crc16);
      crc16 = ~crc16;
      crc[1] = crc16 >> 8;                 
      crc[0] = crc16 & 0xFF;               
      
      OneWireReset();
      OneWireSelect(bridgeAddress);
      OneWireWrite(header, 3);
      OneWireWrite(txBuffer, txBufferLength);
      OneWireWrite(crc, sizeof(crc));

      uint8_t timeout = 0;
      while (OneWireReadBit() == true){
        delay(1);
        timeout++;
        if (timeout > ONEWIRE_TIMEOUT){
          //oneWire->depower();
          return false;
        }
      }

      uint8_t stat = OneWireRead();
      uint8_t writeStat = OneWireRead();
      
      if ((stat != 0x00) || (writeStat != 0x00)) {
        //oneWire->depower();
        return stat;
      }
      
      //oneWire->depower(); 

      txBufferIndex = 0;
      txBufferLength = 0;

      return 0;
    }

    int read(void)
    {
        int value = -1;
        if (rxBufferIndex < rxBufferLength)
        {
            value = rxBuffer[rxBufferIndex];
            ++rxBufferIndex;
        }
        return value;
    }

    int peek(void)
    {
        int value = -1;
        if (rxBufferIndex < rxBufferLength)
        {
            value = rxBuffer[rxBufferIndex];
        }
        return value;
    }

    void flush(void)
    {
        rxBufferIndex = 0;
        rxBufferLength = 0;
        txBufferIndex = 0;
        txBufferLength = 0;
    }

    uint8_t status() {

    }

    void setClockStretchLimit(uint32_t frequency) {} //not supported

    void onReceive(void (*)(int)) {}    //no slave support
    void onReceive(void (*)(size_t)) {} //no slave support
    void onRequest(void (*)(void)) {}   //no slave support
};

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv33(uint8_t function)
{
  bool result = false;

  if (pin[GPIO_1W] < 99) {
    switch (function) {
      case FUNC_PRE_INIT:
        Wire=OneWireTwoWireBridge();
        onewire_pin = pin[GPIO_1W];
        break;
    }
  }
  return result;
}

#endif //USE_DS28E17
#endif //USE_1WIRE
#endif //USE_I2C