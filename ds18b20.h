/* ds18b20.h
 *
 * Copyright (C) 2020 Matjaz Verbole
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#ifndef DS18B20_H_INCLUDED
#define DS18B20_H_INCLUDED

typedef enum {
  RES_9_BIT  = 0x00,
  RES_10_BIT = 0x01,
  RES_11_BIT = 0x02,
  RES_12_BIT = 0x03
} thermRes;

#define USE_EEPROM_FOR_ALARM  1

void ds18b20_init(uint8_t GPIO);

bool ds18b20_single_get_family_code(uint8_t *code);

bool ds18b20_single_get_serial_number(uint8_t *serialNumber);

bool ds18b20_single_get_temperature(float *temperature);

bool ds18b20_single_get_thermometer_resolution(thermRes *res);
bool ds18b20_single_set_thermometer_resolution(thermRes res);

#if USE_EEPROM_FOR_ALARM
bool ds18b20_single_set_alarm_temperature(int8_t temperatureHigh, int8_t temperatureLow);
bool ds18b20_single_set_alarm_temperature_high(int8_t temperatureHigh);
bool ds18b20_single_set_alarm_temperature_low(int8_t temperatureLow);
bool ds18b20_single_get_alarm_temperature(int8_t *temperatureHigh, int8_t *temperatureLow);
bool ds18b20_single_get_alarm_temperature_high(int8_t *temperatureHigh);
bool ds18b20_single_get_alarm_temperature_low(int8_t *temperatureLow);
#else
bool ds18b20_single_set_byte1_in_EEPROM(uint8_t byte1);
bool ds18b20_single_set_byte2_in_EEPROM(uint8_t byte2);
bool ds18b20_single_set_both_bytes_in_EEPROM(uint8_t byte1, uint8_t byte2);
bool ds18b20_single_read_byte1_in_EEPROM(uint8_t *byte1);
bool ds18b20_single_read_byte2_in_EEPROM(uint8_t *byte2);
bool ds18b20_single_read_both_bytes_in_EEPROM(uint8_t *byte1, uint8_t *byte2);
#endif

#endif /* DS18B20_H_INCLUDED */
