/*
  xsns_05_ds18x20.ino - DS18x20 temperature sensor support for Tasmota

  Copyright (C) 2019  Theo Arends

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
#ifdef USE_1WIRE
#ifdef USE_DS18x20
/*********************************************************************************************\
 * DS18B20 - Temperature - Multiple sensors
\*********************************************************************************************/

#define XSNS_05              5

//#define USE_DS18x20_RECONFIGURE    // When sensor is lost keep retrying or re-configure

#define DS18S20_CHIPID       0x10  // +/-0.5C 9-bit
#define DS1822_CHIPID        0x22  // +/-2C 12-bit
#define DS18B20_CHIPID       0x28  // +/-0.5C 12-bit
#define MAX31850_CHIPID      0x3B  // +/-0.25C 14-bit

#define W1_CONVERT_TEMP      0x44
#define W1_WRITE_EEPROM      0x48
#define W1_WRITE_SCRATCHPAD  0x4E
#define W1_READ_SCRATCHPAD   0xBE

#define DS18X20_MAX_SENSORS  8

const char kDs18x20Types[] PROGMEM = "DS18x20|DS18S20|DS1822|DS18B20|MAX31850";

uint8_t ds18x20_chipids[] = { 0, DS18S20_CHIPID, DS1822_CHIPID, DS18B20_CHIPID, MAX31850_CHIPID };

struct DS18X20STRUCT {
  uint8_t address[8];
  uint8_t index;
  uint8_t valid;
  float   temperature;
} ds18x20_sensor[DS18X20_MAX_SENSORS];
uint8_t ds18x20_sensors = 0;
char ds18x20_types[12];
#ifdef W1_PARASITE_POWER
uint8_t ds18x20_sensor_curr = 0;
unsigned long w1_power_until = 0;
#endif


void Ds18x20Init(void)
{
  uint64_t ids[DS18X20_MAX_SENSORS];

  OneWireResetSearch();

  ds18x20_sensors = 0;
  while (ds18x20_sensors < DS18X20_MAX_SENSORS) {
    if (!OneWireSearch(ds18x20_sensor[ds18x20_sensors].address)) {
      break;
    }
    if (OneWireCrc8(ds18x20_sensor[ds18x20_sensors].address) &&
       ((ds18x20_sensor[ds18x20_sensors].address[0] == DS18S20_CHIPID) ||
        (ds18x20_sensor[ds18x20_sensors].address[0] == DS1822_CHIPID) ||
        (ds18x20_sensor[ds18x20_sensors].address[0] == DS18B20_CHIPID) ||
        (ds18x20_sensor[ds18x20_sensors].address[0] == MAX31850_CHIPID))) {
      ds18x20_sensor[ds18x20_sensors].index = ds18x20_sensors;
      ids[ds18x20_sensors] = ds18x20_sensor[ds18x20_sensors].address[0];  // Chip id
      for (uint32_t j = 6; j > 0; j--) {
        ids[ds18x20_sensors] = ids[ds18x20_sensors] << 8 | ds18x20_sensor[ds18x20_sensors].address[j];
      }
      ds18x20_sensors++;
    }
  }
  for (uint32_t i = 0; i < ds18x20_sensors; i++) {
    for (uint32_t j = i + 1; j < ds18x20_sensors; j++) {
      if (ids[ds18x20_sensor[i].index] > ids[ds18x20_sensor[j].index]) {  // Sort ascending
        std::swap(ds18x20_sensor[i].index, ds18x20_sensor[j].index);
      }
    }
  }
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR(D_LOG_DSB D_SENSORS_FOUND " %d"), ds18x20_sensors);
}

void Ds18x20Convert(void)
{
  OneWireReset();
#ifdef W1_PARASITE_POWER
  // With parasite power address one sensor at a time
  if (++ds18x20_sensor_curr >= ds18x20_sensors)
    ds18x20_sensor_curr = 0;
  OneWireSelect(ds18x20_sensor[ds18x20_sensor_curr].address);
#else
  OneWireWrite(W1_SKIP_ROM);           // Address all Sensors on Bus
#endif
  OneWireWrite(W1_CONVERT_TEMP);       // start conversion, no parasite power on at the end
//  delay(750);                          // 750ms should be enough for 12bit conv
}

bool Ds18x20Read(uint8_t sensor)
{
  uint8_t data[9];
  int8_t sign = 1;

  uint8_t index = ds18x20_sensor[sensor].index;
  if (ds18x20_sensor[index].valid) { ds18x20_sensor[index].valid--; }
  for (uint32_t retry = 0; retry < 3; retry++) {
    OneWireReset();
    OneWireSelect(ds18x20_sensor[index].address);
    OneWireWrite(W1_READ_SCRATCHPAD);
    for (uint32_t i = 0; i < 9; i++) {
      data[i] = OneWireRead();
    }
    if (OneWireCrc8(data)) {
      switch(ds18x20_sensor[index].address[0]) {
        case DS18S20_CHIPID: {
          if (data[1] > 0x80) {
            data[0] = (~data[0]) +1;
            sign = -1;                     // App-Note fix possible sign error
          }
          float temp9 = (float)(data[0] >> 1) * sign;
          ds18x20_sensor[index].temperature = ConvertTemp((temp9 - 0.25) + ((16.0 - data[6]) / 16.0));
          ds18x20_sensor[index].valid = SENSOR_MAX_MISS;
          return true;
        }
        case DS1822_CHIPID:
        case DS18B20_CHIPID: {
          if (data[4] != 0x7F) {
            data[4] = 0x7F;                 // Set resolution to 12-bit
            OneWireReset();
            OneWireSelect(ds18x20_sensor[index].address);
            OneWireWrite(W1_WRITE_SCRATCHPAD);
            OneWireWrite(data[2]);          // Th Register
            OneWireWrite(data[3]);          // Tl Register
            OneWireWrite(data[4]);          // Configuration Register
            OneWireSelect(ds18x20_sensor[index].address);
            OneWireWrite(W1_WRITE_EEPROM);  // Save scratchpad to EEPROM
#ifdef W1_PARASITE_POWER
            w1_power_until = millis() + 10; // 10ms specified duration for EEPROM write
#endif
          }
          uint16_t temp12 = (data[1] << 8) + data[0];
          if (temp12 > 2047) {
            temp12 = (~temp12) +1;
            sign = -1;
          }
          ds18x20_sensor[index].temperature = ConvertTemp(sign * temp12 * 0.0625);  // Divide by 16
          ds18x20_sensor[index].valid = SENSOR_MAX_MISS;
          return true;
        }
        case MAX31850_CHIPID: {
          int16_t temp14 = (data[1] << 8) + (data[0] & 0xFC);
          ds18x20_sensor[index].temperature = ConvertTemp(temp14 * 0.0625);  // Divide by 16
          ds18x20_sensor[index].valid = SENSOR_MAX_MISS;
          return true;
        }
      }
    }
  }
  AddLog_P(LOG_LEVEL_DEBUG, PSTR(D_LOG_DSB D_SENSOR_CRC_ERROR));
  return false;
}

void Ds18x20Name(uint8_t sensor)
{
  uint8_t index = sizeof(ds18x20_chipids);
  while (index) {
    if (ds18x20_sensor[ds18x20_sensor[sensor].index].address[0] == ds18x20_chipids[index]) {
      break;
    }
    index--;
  }
  GetTextIndexed(ds18x20_types, sizeof(ds18x20_types), index, kDs18x20Types);
  if (ds18x20_sensors > 1) {
    snprintf_P(ds18x20_types, sizeof(ds18x20_types), PSTR("%s%c%d"), ds18x20_types, IndexSeparator(), sensor +1);
  }
}

/********************************************************************************************/

void Ds18x20EverySecond(void)
{
#ifdef W1_PARASITE_POWER
  // skip access if there is still an eeprom write ongoing
  unsigned long now = millis();
  if (now < w1_power_until)
    return;
#endif
  if (uptime & 1
#ifdef W1_PARASITE_POWER
      // if more than 1 sensor and only parasite power: convert every cycle
      || ds18x20_sensors >= 2
#endif
  ) {
    // 2mS
    Ds18x20Convert();          // Start conversion, takes up to one second
  } else {
    for (uint32_t i = 0; i < ds18x20_sensors; i++) {
      // 12mS per device
      if (!Ds18x20Read(i)) {   // Read temperature
        Ds18x20Name(i);
        AddLogMissed(ds18x20_types, ds18x20_sensor[ds18x20_sensor[i].index].valid);
#ifdef USE_DS18x20_RECONFIGURE
        if (!ds18x20_sensor[ds18x20_sensor[i].index].valid) {
          memset(&ds18x20_sensor, 0, sizeof(ds18x20_sensor));
          Ds18x20Init();       // Re-configure
        }
#endif  // USE_DS18x20_RECONFIGURE
      }
    }
  }
}

void Ds18x20Show(bool json)
{
  for (uint32_t i = 0; i < ds18x20_sensors; i++) {
    uint8_t index = ds18x20_sensor[i].index;

    if (ds18x20_sensor[index].valid) {   // Check for valid temperature
      char temperature[33];
      dtostrfd(ds18x20_sensor[index].temperature, Settings.flag2.temperature_resolution, temperature);

      Ds18x20Name(i);

      if (json) {
        char address[17];
        for (uint32_t j = 0; j < 6; j++) {
          sprintf(address+2*j, "%02X", ds18x20_sensor[index].address[6-j]);  // Skip sensor type and crc
        }
        ResponseAppend_P(PSTR(",\"%s\":{\"" D_JSON_ID "\":\"%s\",\"" D_JSON_TEMPERATURE "\":%s}"), ds18x20_types, address, temperature);
#ifdef USE_DOMOTICZ
        if ((0 == tele_period) && (0 == i)) {
          DomoticzSensor(DZ_TEMP, temperature);
        }
#endif  // USE_DOMOTICZ
#ifdef USE_KNX
        if ((0 == tele_period) && (0 == i)) {
          KnxSensor(KNX_TEMPERATURE, ds18x20_sensor[index].temperature);
        }
#endif  // USE_KNX
#ifdef USE_WEBSERVER
      } else {
        WSContentSend_PD(HTTP_SNS_TEMP, ds18x20_types, temperature, TempUnit());
#endif  // USE_WEBSERVER
      }
    }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns05(uint8_t function)
{
  bool result = false;

  if (pin[GPIO_1W] < 99) {
    switch (function) {
      case FUNC_INIT:
        Ds18x20Init();
        break;
      case FUNC_EVERY_SECOND:
        Ds18x20EverySecond();
        break;
      case FUNC_JSON_APPEND:
        Ds18x20Show(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        Ds18x20Show(0);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif //USE_DS18x20
#endif //USE_1WIRE