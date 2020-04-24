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

/**
* Initialize library and set data pin.
* @param GPIO data pin which will be used for communication
* @return 'true' if initialized successfully, 'false' otherwise
*/
bool ds18b20_init(uint8_t GPIO);

/**
* Temperature sensor may use parasite power or have its own power.
* This function returns true if a sensor uses parasite power.
* @return 'true' if a sensor uses parasite power, 'false' otherwise
*/
bool ds18b20_is_parasite_power_mode(void);

/**
* There may be more than one temperature sensor connected to a bus.
* This function searches for all ROM codes of a sensors and saves
* their address in address parameter.
* @param address pointer to an array which will be populated with a 8 byte ROM code of a sensor
* @return 'true' if a next sensor address was found, 'false' otherwise (it returns 'false' also if there are no sensors on a bus)
*/
bool ds18b20_search_ROM(uint8_t *address);

/**
* This function checks if there is a temperature alarm triggered on any of connected sensors.
* It saves their address in address parameter.
* @param address pointer to an array which will be populated with a 8 byte ROM code of a sensor with alarm triggered
* @return 'true' if a next sensor address was found, 'false' otherwise (it returns 'false' also if there are no sensors on a bus)
*/
bool ds18b20_alarm_search(uint8_t *address);

/**
* This function may be use if there is only one sensor connected on a bus. Otherwise there will be data collision because
* multiple sensors will give a reply. Function populates code with a family code of a sensor.
* @param code will be populated with a family code if sensor will be found
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_single_get_family_code(uint8_t *code);

/**
* This function may be use if there is only one sensor connected on a bus. Otherwise there will be data collision because
* multiple sensors will give a reply. Function populates serialNumber with a serial number of a sensor.
* @param serialNumber will be populated with a serial number if sensor will be found
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_single_get_serial_number(uint8_t *serialNumber);

/**
* Function gets and returns a measured temperature of a sensor.
* It saves temperature in a temperature parameter.
* @param temperature parameter will be populated with measured temperature of a sensor
* @param address if only one sensor is attached on a bus NULL parameter may be used, otherwise it is an 8 byte address of a sensor
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_get_temperature(float *temperature, uint8_t *address);

/**
* Function gets thermometer resolution.
* It saves resolution in a res parameter.
* @param res parameter will be populated with a thermometer resolution
* @param address if only one sensor is attached on a bus NULL parameter may be used, otherwise it is an 8 byte address of a sensor
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_get_thermometer_resolution(thermRes *res, uint8_t *address);

/**
* Function sets thermometer resolution.
* @param res wanted resolution for a sensors thermometer (it may be 9, 10, 11 or 12 bit)
* @param address if only one sensor is attached on a bus NULL parameter may be used, otherwise it is an 8 byte address of a sensor
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_set_thermometer_resolution(thermRes res, uint8_t *address);

#if USE_EEPROM_FOR_ALARM

/**
* Function sets high and low alarm temperatures. Temperatures are in a degree Celsius.
* @param temperatureHigh high alarm temperature to be set
* @param temperatureLow low alarm temperature to be set
* @param address if only one sensor is attached on a bus NULL parameter may be used, otherwise it is an 8 byte address of a sensor
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_set_alarm_temperature(int8_t temperatureHigh, int8_t temperatureLow, uint8_t *address);

/**
* Function sets high alarm temperature. Temperature is in a degree Celsius.
* @param temperatureHigh high alarm temperature to be set
* @param address if only one sensor is attached on a bus NULL parameter may be used, otherwise it is an 8 byte address of a sensor
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_set_alarm_temperature_high(int8_t temperatureHigh, uint8_t *address);

/**
* Function sets low alarm temperature. Temperature is in a degree Celsius.
* @param temperatureLow low alarm temperature to be set
* @param address if only one sensor is attached on a bus NULL parameter may be used, otherwise it is an 8 byte address of a sensor
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_set_alarm_temperature_low(int8_t temperatureLow, uint8_t *address);

/**
* Function gets high and low alarm temperatures. Temperatures are in a degree Celsius.
* @param temperatureHigh parameter will be populated with a high alarm temperature of a sensor
* @param temperatureLow parameter will be populated with a low alarm temperature of a sensor
* @param address if only one sensor is attached on a bus NULL parameter may be used, otherwise it is an 8 byte address of a sensor
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_get_alarm_temperature(int8_t *temperatureHigh, int8_t *temperatureLow, uint8_t *address);

/**
* Function gets high alarm temperature. Temperature is in a degree Celsius.
* @param temperatureHigh parameter will be populated with a high alarm temperature of a sensor
* @param address if only one sensor is attached on a bus NULL parameter may be used, otherwise it is an 8 byte address of a sensor
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_get_alarm_temperature_high(int8_t *temperatureHigh, uint8_t *address);

/**
* Function gets low alarm temperature. Temperature is in a degree Celsius.
* @param temperatureLow parameter will be populated with a low alarm temperature of a sensor
* @param address if only one sensor is attached on a bus NULL parameter may be used, otherwise it is an 8 byte address of a sensor
* @return 'true' if succeeded, 'false' otherwise
*/
bool ds18b20_get_alarm_temperature_low(int8_t *temperatureLow, uint8_t *address);

#else
bool ds18b20_single_set_data_EEPROM(uint16_t data);
bool ds18b20_single_read_data_EEPROM(uint16_t *data);
#endif

#endif /* DS18B20_H_INCLUDED */
