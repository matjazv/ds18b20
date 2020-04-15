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

bool ds18b20_init(uint8_t GPIO);

bool ds18b20_is_parasite_power_mode(void);

bool ds18b20_search_ROM(void);

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
bool ds18b20_single_set_data_EEPROM(uint16_t data);
bool ds18b20_single_read_data_EEPROM(uint16_t *data);
#endif

#endif /* DS18B20_H_INCLUDED */
