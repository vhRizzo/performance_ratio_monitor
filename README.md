# Performance Ratio Monitor

Implementation of low-cost Performance Ratio (PR) monitor using various IoT sensors that sends data through a LoRaWAN radio module using ESP-IDF v5.0.
It is intended to develop a product that will be attached to a photovoltaic module in order to estimate it's PR.

## Devices used:

  - ESP32 NodeMCU: Microcontroller dev-kit used.
  - BPW34: Photodiode to measure irradiance.
  - ADS1115: Analog-digital converter to convert the irradiance signal.
  - DS18B20: Temperature sensor intended to measure the superficial temperature of the photovoltaic module.
  - NEO-6M: GPS module.
  - BME280: Temperature, humidity and pressure sensor to measure the environment conditions.
  - DSM501A: Dust sensor intended to predict if there is gathered dust on the photovoltaic module.
  - SMW-SX1262M0: LoRaWAN radio used to send the gathered data.
