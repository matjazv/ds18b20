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

#define USE_INTERRUPTS  1

#define READ_ROM_COMMAND        0x33
#define MATCH_ROM_COMMAND       0x55
#define SKIP_ROM_COMMAND        0xCC
#define SEARCH_ROM_COMMAND      0xF0
#define ALARM_SEARCH_COMMAND    0xEC

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

static uint8_t initialization_sequence(void)
{
  uint8_t sensorPresence = 0;

  gpio_set_direction(DQ_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DQ_GPIO, 0);

  wait_us(500);

  gpio_set_level(DQ_GPIO, 1);
  gpio_set_direction(DQ_GPIO, GPIO_MODE_INPUT);

  wait_us(45);

  if(gpio_get_level(DQ_GPIO) == 0) {
    sensorPresence = 1;
  } else {
    sensorPresence = 0;
  }

  wait_us(500);

  if(gpio_get_level(DQ_GPIO) == 1) {
      sensorPresence = 1;
  } else {
      sensorPresence = 0;
  }

  return sensorPresence;
}

static void write_bit(uint8_t bit)
{
  gpio_set_direction(DQ_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DQ_GPIO, 0);

  wait_us(1);

  if (bit == 1) {
    gpio_set_level(DQ_GPIO, 1);
  }

  wait_us(70);

  gpio_set_level(DQ_GPIO, 1);
}

static uint8_t read_bit(void)
{
  uint8_t bit = 0;

  gpio_set_direction(DQ_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(DQ_GPIO, 0);

  wait_us(1);

  gpio_set_direction(DQ_GPIO, GPIO_MODE_INPUT);

  wait_us(15);

  if (gpio_get_level(DQ_GPIO) == 1) {
    bit = 1;
  }

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

void ds18b20_init(uint8_t GPIO)
{
    DQ_GPIO = GPIO;
    gpio_pad_select_gpio(DQ_GPIO);
}

bool ds18b20_read_ROM(void)
{
  initialization_sequence();

  write_byte(READ_ROM_COMMAND);

  return true;
}
