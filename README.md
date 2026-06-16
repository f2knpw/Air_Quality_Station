# Air_Quality_Station


*A simple yet powerful system for Air Quality monitoring.*


## Overview
This DIY air quality monitoring system integrates precision sensors with a connected processing platform and a low-power e-paper display.

**It monitors:**
* Temperature
* Pressure
* Humidity
* CO2 level
* Air quality (particulates/dust)

show it here
<a href="http://www.youtube.com/watch?feature=player_embedded&v=xjLmPx-Fbjk
" target="_blank"><img src="http://img.youtube.com/vi/xjLmPx-Fbjk/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="1180" height="664" border="10" /></a>

## System Architecture

* **Processing Unit (Core):** The **[ESP32 S2 Lolin mini](https://fr.aliexpress.com/item/1005009711874009.html)** manages the sensors. It offers native Wi-Fi connectivity, ideal for sending data to dashboards (Home Assistant, InfluxDB, or MQTT). 

* **Display:**
    * **[3.7-inch E-Paper](https://fr.aliexpress.com/item/1005009712159337.html) (240x416):** Electronic ink technology allowing for static data display with zero power consumption between refreshes, ensuring a clear and readable interface.

* **Air Quality Sensors:**
    * **[SCD40](https://fr.aliexpress.com/item/1005005379409283.html):** Photoacoustic sensor for CO2 concentration, temperature, and relative humidity.
    * **[Plantower PMS5003](https://fr.aliexpress.com/item/4000036196650.html):** Laser scattering sensor for measuring fine particulate matter (PM1.0, PM2.5, PM10).

* **Complementary Environmental Sensors:**
    * **[BME280](https://fr.aliexpress.com/item/1005006824236173.html):** Precision measurement of barometric pressure, humidity, and temperature.
    * **[DS18B20](https://fr.aliexpress.com/item/1005009070205186.html):** Digital temperature sensor using a 1-Wire interface, ideal for remote measurement or isolated thermal reference.


The full project is described on my hackaday pages : https://hackaday.io/project/205888-air-quality-station
PCB can be ordered at PCBWay : https://www.pcbway.com/project/shareproject/Air_Quality_station_6f7bd036.html
Full project is described on my hackaday pages : https://hackaday.io/project/205888-air-quality-station

Esp32-S2 source code is here : https://github.com/f2knpw/Air_Quality_Station/blob/master/air_quality.ino

