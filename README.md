# Library for single or multiple temperature sensor(s) DS18B20 with alarm support on ESP32
# Usage
In both examples (single and multiple) sensors data pin is connected to GPIO15 on ESP32.

You may disable or enable logging in the library by setting LOGGING_ENABLED in the top of ds18b20.c file:
```
#define LOGGING_ENABLED     0 // logging disabled
#define LOGGING_ENABLED     1 // logging enabled
```

#### Single DS18B20: ####

```
if (ds18b20_init(15) == true) {
  ESP_LOGI(TAG, "Initialization completed.");
}
else {
  ESP_LOGI(TAG, "Initialization failed!");
}

float temperature;
if (ds18b20_get_temperature(&temperature, NULL) == true) {
  ESP_LOGI(TAG, "Temperature: %0.1f", temperature);
}
else {
  ESP_LOGI(TAG, "Error reading temperature!");
}
```

#### Optional to set sensor resolution to 11 bit: ####

```
if (ds18b20_set_thermometer_resolution(RES_11_BIT, NULL) == true) {
  ESP_LOGI(TAG, "Successfully set temperature resolution to 11 bit.");
}
else {
  ESP_LOGI(TAG, "Failed to set temperature resolution to 11 bit!");
}	
```

#### Optional to set low alarm temperature to -6°C and high alarm temerature to 30°C: ####

```
if (ds18b20_set_alarm_temperature(30, -6, NULL) == true) {
  ESP_LOGI(TAG, "Successfully set alarm temperatures.");
}
else {
  ESP_LOGI(TAG, "Failed to set alarm temperatures!");
}
```

#### Optional to get a low alarm temperature for a sensor: ####

```
int8_t temperatureLow;
if (ds18b20_get_alarm_temperature_low(&temperatureLow, NULL) == true) {
  ESP_LOGI(TAG, "Low alarm temperature: %d", temperatureLow);
}
else {
  ESP_LOGI(TAG, "Failed to get a low alarm temperature!");
}
```

#### Multiple DS18B20: ####

```
if (ds18b20_init(15) == true) {
  ESP_LOGI(TAG, "Initialization completed.");
}
else {
  ESP_LOGI(TAG, "Initialization failed!");
}

uint8_t address[8] = {0};
if (ds18b20_search_ROM(address) == true) {
  ESP_LOGI(TAG, "Address was found.");
}
else {
  ESP_LOGI(TAG, "No address was found!");
}

float temperature;
if (ds18b20_get_temperature(&temperature, address) == true) {
  ESP_LOGI(TAG, "Temperature: %0.1f", temperature);
}
else {
  ESP_LOGI(TAG, "Error reading temperature!");
}
```

#### Optional to set sensor resolution to 9 bit: ####

```
if (ds18b20_set_thermometer_resolution(RES_9_BIT, address) == true) {
  ESP_LOGI(TAG, "Successfully set temperature resolution to 9 bit.");
}
else {
  ESP_LOGI(TAG, "Failed to set temperature resolution to 9 bit!");
}	
```

#### Optional to set high alarm temperature to 28°C: ####

```
if (ds18b20_set_alarm_temperature_high(28, address) == true) {
  ESP_LOGI(TAG, "Successfully set alarm temperatures.");
}
else {
  ESP_LOGI(TAG, "Failed to set alarm temperatures!");
}
```

#### Optional to check if any sensor has triggered a temperature alarm: ####

```
uint8_t alarmAddress[8] = {0};
while (ds18b20_alarm_search(alarmAddress) == true) {
  ESP_LOGI(TAG, "ALARM for address: %x%x%x%x%x%x%x%x", alarmAddress[0], alarmAddress[1], alarmAddress[2], alarmAddress[3], alarmAddress[4], alarmAddress[5], alarmAddress[6], alarmAddress[7]);
}
```
