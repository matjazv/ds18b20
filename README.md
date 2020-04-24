# Library for single or multiple temperature sensor(s) DS18B20 with alarm support on ESP32
# Usage
In both examples sensors data pin is connected to GPIO15 on ESP32.

#### Single DS18B20: ####

```
if (ds18b20_init(15) == true) {
  ESP_LOGI(TAG, "Initialization completed.");
}
else {
  ESP_LOGI(TAG, "Initialization failed!");
}

if (ds18b20_get_temperature(&temp, NULL) == true) {
  ESP_LOGI(TAG, "Temperature: %0.1f", temp);
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

#### Optional to set alarm temperature between -6 and 30 degrees Celsius: ####

```
if (ds18b20_set_alarm_temperature(30, -6, NULL) == true) {
  ESP_LOGI(TAG, "Successfully set alarm temperatures.");
}
else {
  ESP_LOGI(TAG, "Failed to set alarm temperatures!");
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

if (ds18b20_get_temperature(&temp, address) == true) {
  ESP_LOGI(TAG, "Temperature: %0.1f", temp);
}
else {
  ESP_LOGI(TAG, "Error reading temperature!");
}
```

#### Optional to set sensor resolution to 11 bit: ####

```
if (ds18b20_set_thermometer_resolution(RES_11_BIT, address) == true) {
  ESP_LOGI(TAG, "Successfully set temperature resolution to 11 bit.");
}
else {
  ESP_LOGI(TAG, "Failed to set temperature resolution to 11 bit!");
}	
```

#### Optional to set alarm temperature between -6 and 30 degrees Celsius: ####

```
if (ds18b20_set_alarm_temperature(30, -6, address) == true) {
  ESP_LOGI(TAG, "Successfully set alarm temperatures.");
}
else {
  ESP_LOGI(TAG, "Failed to set alarm temperatures!");
}
```
