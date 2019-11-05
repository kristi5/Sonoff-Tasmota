/*
  xdrv_32_onewire.ino - Support for One Wire Bus

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

#ifdef USE_1WIRE
/*********************************************************************************************\
 * Embedded tuned OneWire library
\*********************************************************************************************/

#define XDRV_32                    32

#define W1_MATCH_ROM         0x55
#define W1_SEARCH_ROM        0xF0
#define W1_SKIP_ROM          0xCC

uint8_t onewire_last_discrepancy = 0;
uint8_t onewire_last_family_discrepancy = 0;
bool onewire_last_device_flag = false;
unsigned char onewire_rom_id[8] = { 0 };
uint8_t onewire_pin = 0;

uint8_t OneWireReset(void)
{
  uint8_t retries = 125;

  //noInterrupts();
#ifdef ONEWIRE_INTERNAL_PULLUP
  pinMode(onewire_pin, INPUT_PULLUP);
#else
  pinMode(onewire_pin, INPUT);
#endif
  do {
    if (--retries == 0) {
      return 0;
    }
    delayMicroseconds(2);
  } while (!digitalRead(onewire_pin));
  pinMode(onewire_pin, OUTPUT);
  digitalWrite(onewire_pin, LOW);
  delayMicroseconds(480);
#ifdef ONEWIRE_INTERNAL_PULLUP
  pinMode(onewire_pin, INPUT_PULLUP);
#else
  pinMode(onewire_pin, INPUT);
#endif
  delayMicroseconds(70);
  uint8_t r = !digitalRead(onewire_pin);
  //interrupts();
  delayMicroseconds(410);
  return r;
}

void OneWireWriteBit(uint8_t v)
{
  static const uint8_t delay_low[2] = { 65, 10 };
  static const uint8_t delay_high[2] = { 5, 55 };

  v &= 1;
  //noInterrupts();
  digitalWrite(onewire_pin, LOW);
  pinMode(onewire_pin, OUTPUT);
  delayMicroseconds(delay_low[v]);
  digitalWrite(onewire_pin, HIGH);
  //interrupts();
  delayMicroseconds(delay_high[v]);
}

uint8_t OneWireReadBit(void)
{
  //noInterrupts();
  pinMode(onewire_pin, OUTPUT);
  digitalWrite(onewire_pin, LOW);
  delayMicroseconds(3);
#ifdef ONEWIRE_INTERNAL_PULLUP
  pinMode(onewire_pin, INPUT_PULLUP);
#else
  pinMode(onewire_pin, INPUT);
#endif
  delayMicroseconds(10);
  uint8_t r = digitalRead(onewire_pin);
  //interrupts();
  delayMicroseconds(53);
  return r;
}

void OneWireWrite(uint8_t v)
{
  for (uint8_t bit_mask = 0x01; bit_mask; bit_mask <<= 1) {
    OneWireWriteBit((bit_mask & v) ? 1 : 0);
  }
}

void OneWireWrite(uint8_t *data, uint8_t length) {
  for (uint8_t i = 0; i < length; i++) {
    OneWireWrite(data[i]);
  }
}

uint8_t OneWireRead(void)
{
  uint8_t r = 0;

  for (uint8_t bit_mask = 0x01; bit_mask; bit_mask <<= 1) {
    if (OneWireReadBit()) {
      r |= bit_mask;
    }
  }
  return r;
}

void OneWireSelect(const uint8_t rom[8])
{
  OneWireWrite(W1_MATCH_ROM);
  for (uint32_t i = 0; i < 8; i++) {
    OneWireWrite(rom[i]);
  }
}

void OneWireResetSearch(void)
{
  onewire_last_discrepancy = 0;
  onewire_last_device_flag = false;
  onewire_last_family_discrepancy = 0;
  for (uint32_t i = 0; i < 8; i++) {
    onewire_rom_id[i] = 0;
  }
}

uint8_t OneWireSearch(uint8_t *newAddr)
{
  uint8_t id_bit_number = 1;
  uint8_t last_zero = 0;
  uint8_t rom_byte_number = 0;
  uint8_t search_result = 0;
  uint8_t id_bit;
  uint8_t cmp_id_bit;
  unsigned char rom_byte_mask = 1;
  unsigned char search_direction;

  if (!onewire_last_device_flag) {
    if (!OneWireReset()) {
      onewire_last_discrepancy = 0;
      onewire_last_device_flag = false;
      onewire_last_family_discrepancy = 0;
      return false;
    }
    OneWireWrite(W1_SEARCH_ROM);
    do {
      id_bit     = OneWireReadBit();
      cmp_id_bit = OneWireReadBit();

      if ((id_bit == 1) && (cmp_id_bit == 1)) {
        break;
      } else {
        if (id_bit != cmp_id_bit) {
          search_direction = id_bit;
        } else {
          if (id_bit_number < onewire_last_discrepancy) {
            search_direction = ((onewire_rom_id[rom_byte_number] & rom_byte_mask) > 0);
          } else {
            search_direction = (id_bit_number == onewire_last_discrepancy);
          }
          if (search_direction == 0) {
            last_zero = id_bit_number;
            if (last_zero < 9) {
              onewire_last_family_discrepancy = last_zero;
            }
          }
        }
        if (search_direction == 1) {
          onewire_rom_id[rom_byte_number] |= rom_byte_mask;
        } else {
          onewire_rom_id[rom_byte_number] &= ~rom_byte_mask;
        }
        OneWireWriteBit(search_direction);
        id_bit_number++;
        rom_byte_mask <<= 1;
        if (rom_byte_mask == 0) {
          rom_byte_number++;
          rom_byte_mask = 1;
        }
      }
    } while (rom_byte_number < 8);
    if (!(id_bit_number < 65)) {
      onewire_last_discrepancy = last_zero;
      if (onewire_last_discrepancy == 0) {
        onewire_last_device_flag = true;
      }
      search_result = true;
    }
  }
  if (!search_result || !onewire_rom_id[0]) {
    onewire_last_discrepancy = 0;
    onewire_last_device_flag = false;
    onewire_last_family_discrepancy = 0;
    search_result = false;
  }
  for (uint32_t i = 0; i < 8; i++) {
    newAddr[i] = onewire_rom_id[i];
  }
  return search_result;
}

bool OneWireCrc8(uint8_t *addr)
{
  uint8_t crc = 0;
  uint8_t len = 8;

  while (len--) {
    uint8_t inbyte = *addr++;          // from 0 to 7
    for (uint32_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) {
        crc ^= 0x8C;
      }
      inbyte >>= 1;
    }
  }
  return (crc == *addr);               // addr 8
}

uint16_t OneWireCrc16(const uint8_t* input, uint16_t len, uint16_t crc = 0)
{
    static const uint8_t oddparity[16] =
        { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    for (uint16_t i = 0 ; i < len ; i++) {
      // Even though we're just copying a byte from the input,
      // we'll be doing 16-bit computation with it.
      uint16_t cdata = input[i];
      cdata = (cdata ^ crc) & 0xff;
      crc >>= 8;

      if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
          crc ^= 0xC001;

      cdata <<= 6;
      crc ^= cdata;
      cdata <<= 1;
      crc ^= cdata;
    }
    return crc;
}

/********************************************************************************************/


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv32(uint8_t function)
{
  bool result = false;

  if (pin[GPIO_1W] < 99) {
    switch (function) {
      case FUNC_PRE_INIT:
        onewire_pin = pin[GPIO_1W];
        break;
    }
  }
  return result;
}

#endif  // USE_1WIRE