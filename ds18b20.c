/* ds18b20.c
 *
 * Copyright (C) 2020 Matjaz Verbole
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"
#include "ds18b20.h"

#define LOGGING_ENABLED     1

#if LOGGING_ENABLED
#include "esp_log.h"
#endif

#define READ_ROM_COMMAND        0x33
#define MATCH_ROM_COMMAND       0x55
#define SKIP_ROM_COMMAND        0xCC
#define SEARCH_ROM_COMMAND      0xF0
#define ALARM_SEARCH_COMMAND    0xEC

#define WRITE_SCRATCHPAD_COMMAND  0x4E
#define READ_SCRATCHPAD_COMMAND   0xBE
#define COPY_SCRATCHPAD_COMMAND   0x48
#define CONVERT_TEMP_COMMAND      0x44
#define RECALL_E2_COMMAND         0xB8
#define READ_POWER_SUPLY_COMMAND  0xB4

static const char *TAG = "ds18b20";

static gpio_num_t DQ_GPIO;
static bool parasitePower;

// global variables for search ROM function
static int8_t lastMatchedBitPosition = -1;
static bool needToWriteOne = false;
static bool lastROMFound = false;

static void wait_us(uint32_t us)
{
  ets_delay_us(us);
}

static uint8_t CRC8(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  while (len--)
  {
    uint8_t inbyte = *addr++;
    for (uint8_t i=8; i>0; i--)
    {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C; //Generator polynomial = 10001100
      inbyte >>= 1;
    }
  }

  return crc;
}

static void write_bit(uint8_t bit)
{
  gpio_set_direction(DQ_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DQ_GPIO, 0);

  wait_us(10);

  if (bit == 1) {
    gpio_set_level(DQ_GPIO, 1);
  }

  wait_us(50);

  gpio_set_level(DQ_GPIO, 1);
}

static uint8_t read_bit(void)
{
  uint8_t bit = 0;

  gpio_set_direction(DQ_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DQ_GPIO, 0);

  wait_us(1);

  gpio_set_direction(DQ_GPIO, GPIO_MODE_INPUT);

  wait_us(12);

  if (gpio_get_level(DQ_GPIO) == 1) {
    bit = 1;
  }

  wait_us(47);

  return bit;
}

static void write_byte(uint8_t data)
{
  uint8_t bit;

  for(uint8_t i=0; i<8; i++)
  {
    bit = data>>i;
    bit &= 0x01;
    write_bit(bit);
  }
}

static uint8_t read_byte(void)
{
  uint8_t data = 0;

  for (uint8_t i=0; i<8; i++)
  {
    if(read_bit()) {
      data |= 0x01<<i;
    }
  }

  return data;
}

static bool initialization_sequence(void)
{
  gpio_set_direction(DQ_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DQ_GPIO, 0);

  wait_us(480);

  gpio_set_level(DQ_GPIO, 1);
  gpio_set_direction(DQ_GPIO, GPIO_MODE_INPUT);

  wait_us(60);

  if(gpio_get_level(DQ_GPIO) != 0) {
    return false;
  }

  wait_us(420);

  if(gpio_get_level(DQ_GPIO) == 1) {
      return true;
  }

  return false;
}

static bool read_ROM(uint8_t *data)
{
  if (data == NULL) {
    return false;
  }

  write_byte(READ_ROM_COMMAND);

  for (uint8_t i=0; i<8; i++)
  {
    data[i] = read_byte();
  }

#if LOGGING_ENABLED
  ESP_LOGI(TAG, "Family code: %x", data[0]);
  ESP_LOGI(TAG, "Serial number: %x%x%x%x%x%x", data[1], data[2], data[3], data[4], data[5], data[6]);
  ESP_LOGI(TAG, "CRC: %x", data[7]);
#endif

  return (data[7] == CRC8(data, 7));
}

static bool match_ROM(uint8_t *ROMData)
{
  if (ROMData == NULL) {
    return false;
  }

  write_byte(MATCH_ROM_COMMAND);

  for (uint8_t i=0; i<8; i++)
  {
    write_byte(ROMData[i]);
  }

  return true;
}

static void skip_ROM(void)
{
  write_byte(SKIP_ROM_COMMAND);
}

static void reset_search(void)
{
  lastMatchedBitPosition = -1;
  needToWriteOne = false;
  lastROMFound = false;
}

static bool search_ROM(bool alarmSearch, uint8_t *address)
{
  if (lastROMFound == true) {
    reset_search();
    return false;
  }
  lastROMFound = true;

  if (!alarmSearch) {
    write_byte(SEARCH_ROM_COMMAND);
  }
  else {
    write_byte(ALARM_SEARCH_COMMAND);
  }

  char data[8] = {0};

  for (uint8_t bitPosition=0; bitPosition<64; bitPosition++)
  {
    uint8_t bitValue = read_bit();
    uint8_t complementValue = read_bit();

    // no device attached to the 1-Wire bus
    if (bitValue && complementValue) {
      reset_search();
      return false;
    }

    // there are still devices attached which have conflicting bits in this position
    if (!bitValue && !complementValue && lastMatchedBitPosition <= bitPosition) {
      lastMatchedBitPosition = bitPosition;
      if (needToWriteOne == false) {
          needToWriteOne = true;
          lastROMFound = false;
      }
      else {
        bitValue = 1;
        needToWriteOne = false;
      }
    }

    data[bitPosition / 8] |= bitValue << (bitPosition % 8);
    write_bit(bitValue);
  }

  for (uint8_t i=0; i<8; i++)
  {
      address[i] = data[i];
  }

#if LOGGING_ENABLED
  if (!alarmSearch) {
    ESP_LOGI(TAG, "Found new ROM code: %x%x%x%x%x%x%x%x", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }
  else {
    ESP_LOGI(TAG, "Alarm for sensor with ROM code: %x%x%x%x%x%x%x%x", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }
#endif

  return true;
}

static bool write_scratchpad(uint8_t *data)
{
  if (data == NULL) {
    return false;
  }

#if LOGGING_ENABLED
  ESP_LOGI(TAG, "Setting Temp. High: %x", data[0]);
  ESP_LOGI(TAG, "Setting Temp. Low: %x", data[1]);
  ESP_LOGI(TAG, "Setting Config: %x", data[2]);
#endif

  write_byte(WRITE_SCRATCHPAD_COMMAND);

  for (uint8_t i=0; i<3; i++)
  {
    write_byte(data[i]);
  }

  return initialization_sequence();
}

static bool read_scratchpad(uint8_t *data)
{
  if (data == NULL) {
    return false;
  }

  write_byte(READ_SCRATCHPAD_COMMAND);

  for (uint8_t i=0; i<9; i++)
  {
    data[i] = read_byte();
  }

#if LOGGING_ENABLED
  ESP_LOGI(TAG, "Temperature: %x%x", data[0], data[1]);
  ESP_LOGI(TAG, "Temp. High User: %x", data[2]);
  ESP_LOGI(TAG, "Temp. Low User: %x", data[3]);
  ESP_LOGI(TAG, "Config: %x", data[4]);
  ESP_LOGI(TAG, "CRC: %x", data[8]);
#endif

  if (initialization_sequence() != true) {
    return false;
  }

  return (data[8] == CRC8(data, 8));
}

static bool copy_scratchpad(void)
{
  write_byte(COPY_SCRATCHPAD_COMMAND);

  if (parasitePower == false) {
    gpio_set_direction(DQ_GPIO, GPIO_MODE_INPUT);

    // max NV write cycle time is 10ms
    for (uint8_t i=0; i<10; i++)
    {
      //vTaskDelay(1 / portTICK_RATE_MS);
      wait_us(1000);
      if (read_bit() == 1) {
        return true;
      }
    }
  }
  else {
    // in parasite mode get chip strong pullup for
    // at least 10ms time
    gpio_set_direction(DQ_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DQ_GPIO, 1);

    vTaskDelay(10 / portTICK_RATE_MS);

    gpio_set_level(DQ_GPIO, 0);

    return true;
  }

  return false;
}

static bool convert_temperature(uint8_t waitTime)
{
  write_byte(CONVERT_TEMP_COMMAND);

  if (parasitePower == false) {
    gpio_set_direction(DQ_GPIO, GPIO_MODE_INPUT);

    // max temperature conversion time is:
    // 94ms for 9 bit resolution
    // 188ms for 10 bit resolution
    // 375ms for 11 bit resolution
    // 750ms for 12 bit resolution
    for (uint8_t i=0; i<10; i++)
    {
      vTaskDelay(waitTime / portTICK_RATE_MS);

      if (read_bit() == 1) {
       return true;
      }
    }
  }
  else {
    // in parasite mode get chip strong pullup for
    // at least conversion time
    gpio_set_direction(DQ_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DQ_GPIO, 1);

    vTaskDelay((waitTime * 10) / portTICK_RATE_MS);

    gpio_set_level(DQ_GPIO, 0);

    return true;
  }

  return false;
}

static bool recall_E2(void)
{
  write_byte(RECALL_E2_COMMAND);

  gpio_set_direction(DQ_GPIO, GPIO_MODE_INPUT);

  // max temperature conversion time is:
  // 94ms for 9 bit resolution
  // 188ms for 10 bit resolution
  // 375ms for 11 bit resolution
  // 750ms for 12 bit resolution
  for (uint8_t i=0; i<10; i++)
  {
    vTaskDelay(75 / portTICK_RATE_MS);
    if (read_bit() == 1) {
      return true;
    }
  }

  return false;
}

static bool read_power_suply(bool *parasite)
{
  write_byte(READ_POWER_SUPLY_COMMAND);

  gpio_set_direction(DQ_GPIO, GPIO_MODE_INPUT);

  if (read_bit() == 1) {
    *parasite = false;
  }
  else {
    *parasite = true;
  }

  return true;
}

static bool get_temperature_wait_time(uint8_t *waitTime, uint8_t *address)
{
  thermRes res;

  if (ds18b20_get_thermometer_resolution(&res, address) != true) {
    return false;
  }

  // in convert_temperature there is a for loop with 10 repetitions
  // because of that maximum waitTime is divided by 10 and rounded up
  switch(res)
  {
  case RES_9_BIT:
    *waitTime = 10;
  break;
  case RES_10_BIT:
    *waitTime = 20;
    break;
  case RES_11_BIT:
    *waitTime = 40;
    break;
  case RES_12_BIT:
    *waitTime = 80;
    break;
  default:
    *waitTime = 80;
    break;
  }

  return true;
}

bool ds18b20_init(uint8_t GPIO)
{
  DQ_GPIO = GPIO;
  gpio_pad_select_gpio(DQ_GPIO);

  if (initialization_sequence() != true) {
    return false;
  }

  skip_ROM();

  if (read_power_suply(&parasitePower) != true) {
    return false;
  }

#if LOGGING_ENABLED
  ESP_LOGI(TAG, "Parasite power: %s", parasitePower ? "enabled" : "disabled");
#endif

  return true;
}

bool ds18b20_is_parasite_power_mode(void)
{
  return parasitePower;
}

bool ds18b20_search_ROM(uint8_t *address)
{
  if (initialization_sequence() != true) {
    return false;
  }

  if (search_ROM(false, address) != true) {
    return false;
  }

  return true;
}

bool ds18b20_alarm_search(uint8_t *address)
{
  if (initialization_sequence() != true) {
    return false;
  }

  if (search_ROM(true, address) != true) {
    return false;
  }

  return true;
}

bool ds18b20_single_get_family_code(uint8_t *code)
{
  if (initialization_sequence() != true) {
    return false;
  }

  uint8_t data[8];
  if (read_ROM(data) != true) {
    return false;
  }

  *code = data[0];

  return true;
}

bool ds18b20_single_get_serial_number(uint8_t *serialNumber)
{
  if (initialization_sequence() != true) {
    return false;
  }

  uint8_t data[8];
  if (read_ROM(data) != true) {
    return false;
  }

  serialNumber[0] = data[1];
  serialNumber[1] = data[2];
  serialNumber[2] = data[3];
  serialNumber[3] = data[4];
  serialNumber[4] = data[5];
  serialNumber[5] = data[6];

  return true;
}

bool ds18b20_get_temperature(float *temperature, uint8_t *address)
{
  uint8_t waitConversionTime;

  if (get_temperature_wait_time(&waitConversionTime, address) != true) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  if (convert_temperature(waitConversionTime) != true) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  *temperature = (float)(scratchpadData[0] + (scratchpadData[1] * 256)) / 16;

  return true;
}

bool ds18b20_get_thermometer_resolution(thermRes *res, uint8_t *address)
{
  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  *res = (scratchpadData[4] & 0x60) >> 5;

  return true;
}

bool ds18b20_set_thermometer_resolution(thermRes res, uint8_t *address)
{
  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  // if old resolution equals new resolution skip setting it
  if ((scratchpadData[4] & 0x60) >> 5 == res) {
    return true;
  }

  uint8_t saveData[3];
  saveData[0] = scratchpadData[2];
  saveData[1] = scratchpadData[3];
  saveData[2] = res << 5;

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  if (write_scratchpad(saveData) != true) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  if (copy_scratchpad() != true) {
    return false;
  }

  return true;
}

#if USE_EEPROM_FOR_ALARM
bool ds18b20_set_alarm_temperature(int8_t temperatureHigh, int8_t temperatureLow, uint8_t *address)
{
  if (temperatureHigh < temperatureLow) {
    return false;
  }

  if (temperatureHigh > 125) {
    temperatureHigh = 125;
  }

  if (temperatureLow < -55) {
    temperatureLow = -55;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  // if both old temperatures equal new temperatures skip setting them
  if ((scratchpadData[2] == temperatureHigh) && (scratchpadData[3] == (uint8_t)temperatureLow)) {
    return true;
  }

  uint8_t saveData[3];
  saveData[0] = temperatureHigh;
  saveData[1] = temperatureLow;
  saveData[2] = scratchpadData[4];

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  if (write_scratchpad(saveData) != true) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  if (copy_scratchpad() != true) {
    return false;
  }

  return true;
}

bool ds18b20_set_alarm_temperature_high(int8_t temperatureHigh, uint8_t *address)
{
  if (temperatureHigh > 125) {
    temperatureHigh = 125;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  };

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  // if old high temperature equals new high temperature skip setting
  if (scratchpadData[2] == temperatureHigh) {
    return true;
  }

  uint8_t saveData[3];
  saveData[0] = temperatureHigh;
  saveData[1] = scratchpadData[3];
  saveData[2] = scratchpadData[4];

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  if (write_scratchpad(saveData) != true) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  };

  if (copy_scratchpad() != true) {
    return false;
  }

  return true;
}

bool ds18b20_set_alarm_temperature_low(int8_t temperatureLow, uint8_t *address)
{
  if (temperatureLow < -55) {
    temperatureLow = -55;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  // if old low temperature equals new low temperature skip setting
  if (scratchpadData[3] == temperatureLow) {
    return true;
  }

  uint8_t saveData[3];
  saveData[0] = scratchpadData[2];
  saveData[1] = temperatureLow;
  saveData[2] = scratchpadData[4];

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  if (write_scratchpad(saveData) != true) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  if (copy_scratchpad() != true) {
    return false;
  }

  return true;
}

bool ds18b20_get_alarm_temperature(int8_t *temperatureHigh, int8_t *temperatureLow, uint8_t *address)
{
  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  *temperatureHigh = scratchpadData[2];
  *temperatureLow = scratchpadData[3];

  return true;
}

bool ds18b20_get_alarm_temperature_high(int8_t *temperatureHigh, uint8_t *address)
{
  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  *temperatureHigh = scratchpadData[2];

  return true;
}

bool ds18b20_get_alarm_temperature_low(int8_t *temperatureLow, uint8_t *address)
{
  if (initialization_sequence() != true) {
    return false;
  }

  if (address == NULL) {
    skip_ROM();
  }
  else {
    if (match_ROM(address) != true) {
      return false;
    }
  }

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  *temperatureLow = scratchpadData[3];

  return true;
}
#endif
