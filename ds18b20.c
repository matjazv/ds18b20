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

#define USE_INTERRUPTS      1
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

gpio_num_t DQ_GPIO;

static void wait_us(uint32_t us)
{
#if USE_INTERRUPTS
  portDISABLE_INTERRUPTS();
#endif
  ets_delay_us(us);
#if USE_INTERRUPTS
  portENABLE_INTERRUPTS();
#endif
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

  for(uint8_t i=0; i<8; i++){
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

  return false;
}

static bool convert_temperature(uint8_t waitTime)
{
  write_byte(CONVERT_TEMP_COMMAND);

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

static bool get_temperature_wait_time(uint8_t *waitTime)
{
  thermRes res;

  if (ds18b20_single_get_thermometer_resolution(&res) != true) {
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

void ds18b20_init(uint8_t GPIO)
{
    DQ_GPIO = GPIO;
    gpio_pad_select_gpio(DQ_GPIO);
}

bool ds18b20_single_get_temperature(float *temperature)
{
  uint8_t waitConversionTime;

  if (get_temperature_wait_time(&waitConversionTime) != true) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  skip_ROM();

  if (convert_temperature(waitConversionTime) != true) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  skip_ROM();

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  *temperature = (float)(scratchpadData[0] + (scratchpadData[1] * 256)) / 16;

  return true;
}

bool ds18b20_single_get_thermometer_resolution(thermRes *res)
{
  if (initialization_sequence() != true) {
    return false;
  }

  skip_ROM();

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  *res = (scratchpadData[4] & 0x60) >> 5;

  return true;
}

bool ds18b20_single_set_thermometer_resolution(thermRes res)
{
  if (initialization_sequence() != true) {
    return false;
  }

  skip_ROM();

  uint8_t scratchpadData[9];
  if (read_scratchpad(scratchpadData) != true) {
    return false;
  }

  uint8_t saveData[3];
  saveData[0] = scratchpadData[2];
  saveData[1] = scratchpadData[3];
  saveData[2] = res << 5;

  if (initialization_sequence() != true) {
    return false;
  }

  skip_ROM();

  if (write_scratchpad(saveData) != true) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  skip_ROM();

  if (copy_scratchpad() != true) {
    return false;
  }

  return true;
}
