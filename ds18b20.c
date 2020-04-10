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
  wait_us(100);
}

static uint8_t read_byte(void)
{
  uint8_t data = 0;

  for (uint8_t i=0; i<8; i++)
  {
    if(read_bit()) {
      data |= 0x01<<i;
    }
    wait_us(15);
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

  return (data[8] == CRC8(data, 8));
}

static void convert_temperature(void)
{
  write_byte(CONVERT_TEMP_COMMAND);

  vTaskDelay(750 / portTICK_RATE_MS);
}

void ds18b20_init(uint8_t GPIO)
{
    DQ_GPIO = GPIO;
    gpio_pad_select_gpio(DQ_GPIO);
}

bool ds18b20_read_ROM(uint8_t *data)
{
  if (data == NULL) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (read_ROM(data) != true) {
      return false;
  }

#if LOGGING_ENABLED
  ESP_LOGI(TAG, "Family code: %x", data[0]);
  ESP_LOGI(TAG, "Serial number: %x%x%x%x%x%x", data[1], data[2], data[3], data[4], data[5], data[6]);
  ESP_LOGI(TAG, "CRC: %x", data[7]);
#endif

  return (data[7] == CRC8(data, 7));
}

bool ds18b20_match_ROM(uint8_t *ROMData)
{
  if (ROMData == NULL) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  return match_ROM(ROMData);
}

bool ds18b20_skip_ROM(void)
{
  if (initialization_sequence() != true) {
    return false;
  }

  skip_ROM();

  return true;
}

bool ds18b20_single_read_scratchpad(uint8_t *data)
{
  if (data == NULL) {
    return false;
  }

  if (initialization_sequence() != true) {
    return false;
  }

  if (ds18b20_skip_ROM() != true) {
    return false;
  }

  if (read_scratchpad(data) != true) {
    return false;
  }

#if LOGGING_ENABLED
  ESP_LOGI(TAG, "Temperature: %x%x", data[0], data[1]);
  ESP_LOGI(TAG, "Temp. High User: %x", data[2]);
  ESP_LOGI(TAG, "Temp. Low User: %x", data[3]);
  ESP_LOGI(TAG, "Config: %x", data[4]);
  ESP_LOGI(TAG, "CRC: %x", data[8]);
#endif

  return (data[8] == CRC8(data, 8));
}

bool ds18b20_single_convert_temperature(void)
{
  if (initialization_sequence() != true) {
    return false;
  }

  if (ds18b20_skip_ROM() != true) {
    return false;
  }

  convert_temperature();

  return true;
}

bool ds18b20_single_get_temperature(float *temperature)
{
  if (ds18b20_single_convert_temperature() != true) {
    return false;
  }

  uint8_t scratchpadData[9];
  if (ds18b20_single_read_scratchpad(scratchpadData) != true) {
    return false;
  }

  *temperature = (float)(scratchpadData[0] + (scratchpadData[1] * 256)) / 16;

  return true;
}
