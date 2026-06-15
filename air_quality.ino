/**
 * ============================================================================
 * ESP32-S2 Environmental & Air Quality Monitoring Station
 * ============================================================================
 * 
 * Description:
 * A comprehensive environmental monitoring station utilizing an E-Paper display.
 * The firmware implements a non-blocking asynchronous state machine to manage 
 * sensor warm-up sequences, take stable multi-sample averages, and refresh the UI.
 * 
 * Integrated Hardware:
 * - SCD4x                     : CO2, Temperature & Humidity (I2C)
 * - BME280                    : Pressure, Temperature & Humidity (I2C)
 * - Plantower PMS5003         : PM1.0 & PM2.5 Particulate Matter (HardwareSerial)
 * - DS18B20                   : High-precision Temperature (OneWire)
 * - E-Paper                   : Screen display (SPI via GxEPD2)
 * 
 * Architecture & Power Strategy:
 * This device is designed for MAINS POWER (USB / Wall Adapter). 
 *
 * Workflow:
 * Boot -> Init Sensors -> Async Warm-up & Multi-sampling -> Render E-Ink -> Deep Sleep
 *
 * ============================================================================
 */


#include <Arduino.h>
#include <esp_sleep.h>
#include <OneWire.h>
#include <Wire.h>

//Scd41
#include <SensirionI2cScd4x.h>

//BME280
#include <Adafruit_BME280.h>

//Plantower
#include "PMS.h"

//DS18B20
#include <DallasTemperature.h>

//E INK
#include <SPI.h>
#include <GxEPD2_BW.h>

//Fonts
#include <Fonts/FreeSerifItalic9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>



//*****************************************************************************
// PIN
//*****************************************************************************

#define PIN_EPD_CS     33
#define PIN_EPD_DC     35  
#define PIN_EPD_RST    37
#define PIN_EPD_BUSY   39
#define PIN_EPD_SDA    16   //DIN
#define PIN_EPD_SCL    18   //CLK

#define PIN_SCL 14
#define PIN_SDA 13

#define PIN_BME_POWER 8

#define PIN_TEMP_POWER 40
#define PIN_TEMP_DATA 38


#define PIN_PMS_TX 7
#define PIN_PMS_RX 5
#define PIN_PMS_SET 3


//*****************************************************************************
// STRUCT
//*****************************************************************************

template<int N>
struct SensorAverage {
    float    sum[N]   = {0};
    uint16_t count    = 0;

    void add(float values[N]) {
        for (int i = 0; i < N; i++) sum[i] += values[i];
        count++;
    }

    float get(int index) {
        return count > 0 ? sum[index] / count : 0;
    }

    void reset() {
        for (int i = 0; i < N; i++) sum[i] = 0;
        count = 0;
    }

    bool isReady(uint16_t minSamples) {
        return count >= minSamples;
    }
};

struct SensorReading {
    String value;
    String unit;
    int filledBars;
};

struct DisplayData {
    SensorReading pm1_0 = {"N/A", "µg/m3", 0};
    SensorReading pm2_5 = {"N/A", "µg/m3", 0};
    SensorReading co2 = {"N/A", "ppm", 0};
    SensorReading temp = {"N/A", "°C", 0};
    SensorReading humidity = {"N/A", "%", 0};
    SensorReading pressure = {"N/A", "hPa", 0};
    SensorReading aqi = {"N/A", "Missing", 0};
};

enum SensorState {
  SENSOR_NOT_INIT,    
  SENSOR_WARMING_UP, 
  SENSOR_READING,     
  SENSOR_READY        
};

struct SensorTask {
  SensorState state;

  uint32_t stateChangedAtMs;     // Timestamp of last state change
  uint32_t warmupDurationMs;      // Warm-up duration (0 if none)
  uint32_t readingIntervalMs;    // Time between readings

  uint8_t errorCount;
  bool isEnabled;
};


//*****************************************************************************
// CONST
//*****************************************************************************

#define MAX_SENSOR_ERRORS 3
#define MIN_SAMPLES 10   
#define REFRESH_SCREEN_AT_BOOT_COUNT 10
#define SLEEP_TIME_IN_MIN 5



#define MINUTES_EN_MICROSECONDES(m) ((uint64_t)(m) * 60 * 1000000ULL)


RTC_DATA_ATTR int bootCount = 0;

//E INK display
GxEPD2_BW<GxEPD2_370_GDEY037T03, GxEPD2_370_GDEY037T03::HEIGHT> display( GxEPD2_370_GDEY037T03(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY));


//Scd40
SensirionI2cScd4x scd40;
#define NO_ERROR_SCD4X 0

//BME280
Adafruit_BME280 bme280;

//DS18B20
OneWire oneWire(PIN_TEMP_DATA);
DallasTemperature ds18b20(&oneWire);
#define DS18B20_DISCONNECTED_VALUE_C -127

//Plantower 5003
HardwareSerial pmsSerial(1);
PMS pms(pmsSerial);




// ─── One instance per physical sensor ───
enum PmsIndex : uint8_t { PM1_0, PM2_5, PMS_COUNT };
SensorTask pmsSensorTask = { SENSOR_NOT_INIT, 0, 30000, 6000, 0, false }; //You need to wait 30 s before the first read data
SensorAverage<PMS_COUNT> pmsAvg;     // PM1.0, PM2.5

enum Scd40Index : uint8_t { SCD_CO2, SCD_TEMP, SCD_HUM,  SCD_COUNT };
SensorTask scd40SensorTask = { SENSOR_NOT_INIT, 0, 30000, 5000, 0, false };
SensorAverage<SCD_COUNT> scd40Avg;   // CO2, temp, humidity

enum DsIndex : uint8_t { DS_TEMP, DS_COUNT };
SensorTask ds18b20SensorTask = { SENSOR_NOT_INIT, 0, 50, 1500, 0, false };
SensorAverage<DS_COUNT> ds18Avg;    // temp 

enum BmeIndex : uint8_t { BME_TEMP, BME_HUM, BME_PRESS, BME_COUNT };
SensorTask bme280SensorTask = { SENSOR_NOT_INIT, 0, 50, 500, 0, false };
SensorAverage<BME_COUNT> bmeAvg;     // temp, humidity, pressure



void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.print("Setup");

    //Share I2C 
    Wire.begin(PIN_SDA, PIN_SCL); 

    esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();

    if (wakeupCause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        bootCount = 0; 
    } else {
        bootCount++; 

        //Clear bootCount after 100 cycle
        if (bootCount >= REFRESH_SCREEN_AT_BOOT_COUNT) {
        bootCount = 0; 
        }
    }

    //Init Sensors
    pmsSensorTask.isEnabled = initPMS5003();
    scd40SensorTask.isEnabled = initSCD40();
    bme280SensorTask.isEnabled = initBME280();
    ds18b20SensorTask.isEnabled = initDS18B20();

    Serial.printf("PMS5003 : %s\n",  pmsSensorTask.isEnabled   ? "OK" : "Missing");
    Serial.printf("SCD40   : %s\n",  scd40SensorTask.isEnabled  ? "OK" : "Missing");
    Serial.printf("BME280  : %s\n",  bme280SensorTask.isEnabled ? "OK" : "Missing");
    Serial.printf("DS18B20 : %s\n",  ds18b20SensorTask.isEnabled? "OK" : "Missing");

    //Init E-INK
    initDisplay(bootCount == 0);
}


void loop() {
    
    readSCD40();
    readBME280();
    readDS18B20();
    readPMS5003();


    if (allSensorsReady()) {
        //Display new data
        DisplayData data = getDisplayData();
        printSensorData(data);
        displayAirQualityDashboard(data);


        //Prepare to sleep
        powerDownDS18B20();
        powerDownBME280();
        powerDownSCD40();
        powerDownPMS5003();
        prepareI2CToSleep();
        powerDownDisplay();
        goToSleep(SLEEP_TIME_IN_MIN);
    }


    delay(200);
}

//*****************************************************************************
// FUNCTIONS
//*****************************************************************************

bool allSensorsReady() {
    if (pmsSensorTask.isEnabled && pmsSensorTask.state != SENSOR_READY) return false;  
    if (bme280SensorTask.isEnabled && bme280SensorTask.state != SENSOR_READY) return false;
    if (scd40SensorTask.isEnabled && scd40SensorTask.state != SENSOR_READY) return false;
    if (ds18b20SensorTask.isEnabled && ds18b20SensorTask.state != SENSOR_READY) return false;
    return true;
}


DisplayData getDisplayData() {
    DisplayData data;

    if (pmsSensorTask.isEnabled && pmsAvg.count > 0) {
        float pm1_0_val = pmsAvg.get(PM1_0);
        float pm2_5_val = pmsAvg.get(PM2_5);

        data.pm1_0.value = String(pm1_0_val, 1);
        data.pm2_5.value = String(pm2_5_val, 1);
        
        // Calcul et ajout de l'AQI
        int aqiResult = calculatePM25AQI(pm2_5_val);
        data.aqi.value = String(aqiResult);

        if (aqiResult <= 50) {
            data.aqi.unit = "GOOD";
            data.aqi.filledBars = 0;
        } else if (aqiResult <= 100) {
            data.aqi.unit = "MODERATE";
            data.aqi.filledBars = 1;
        } else if (aqiResult <= 150) {
            data.aqi.unit = "BAD";
            data.aqi.filledBars = 2;
        } else if (aqiResult <= 200) {
            data.aqi.unit = "UNHEALTHY";
            data.aqi.filledBars = 3;
        } else if (aqiResult <= 300) {
            data.aqi.unit = "UNHEALTHY";
            data.aqi.filledBars = 4;
        } else {
            data.aqi.unit = "HAZARDOUS";
            data.aqi.filledBars = 5;
        }
        
    }

    if (scd40SensorTask.isEnabled && scd40Avg.count > 0) {
        data.co2.value = String(scd40Avg.get(SCD_CO2), 0);
        data.temp.value = String(scd40Avg.get(SCD_TEMP), 1);
        data.humidity.value  = String(scd40Avg.get(SCD_HUM),  1);
    }

    if (bme280SensorTask.isEnabled && bmeAvg.count > 0) {
        data.temp.value = String(bmeAvg.get(BME_TEMP), 1);
        data.humidity.value  = String(bmeAvg.get(BME_HUM), 1);
        data.pressure.value = String(bmeAvg.get(BME_PRESS), 1);
    }

    if (ds18b20SensorTask.isEnabled && ds18Avg.count > 0) {
        data.temp.value = String(ds18Avg.get(DS_TEMP), 1);
    }

    return data;
}


int calculatePM25AQI(float pm25) {
    if (pm25 < 0.0) return 0;
    
    // Official US EPA thresholds for PM2.5
    if (pm25 <= 12.0)  return round(((50.0 - 0.0)   / (12.0 - 0.0))   * (pm25 - 0.0)   + 0.0);
    if (pm25 <= 35.4)  return round(((100.0 - 51.0) / (35.4 - 12.1))  * (pm25 - 12.1)  + 51.0);
    if (pm25 <= 55.4)  return round(((150.0 - 101.0)/ (55.4 - 35.5))  * (pm25 - 35.5)  + 101.0);
    if (pm25 <= 150.4) return round(((200.0 - 151.0)/ (150.4 - 55.5)) * (pm25 - 55.5)  + 151.0);
    if (pm25 <= 250.4) return round(((300.0 - 201.0)/ (250.4 - 150.5))* (pm25 - 150.5) + 201.0);
    if (pm25 <= 350.4) return round(((400.0 - 301.0)/ (350.4 - 250.5))* (pm25 - 250.5) + 301.0);
    if (pm25 <= 500.4) return round(((500.0 - 401.0)/ (500.4 - 350.5))* (pm25 - 350.5) + 401.0);
    
    return 500; 
}


void printSensorData(const DisplayData& data) {
    Serial.println("=== Display Data ===");
    Serial.println("  AQI  : " + data.aqi.value);
    Serial.println("  PM1.0  : " + data.pm1_0.value  + " " + data.pm1_0.unit);
    Serial.println("  PM2.5  : " + data.pm2_5.value  + " " + data.pm2_5.unit);
    Serial.println("  Press  : " + data.pressure.value + " " + data.pressure.unit);
    Serial.println("  CO2    : " + data.co2.value     + " " + data.co2.unit);
    Serial.println("  Hum    : " + data.humidity.value  + " " + data.humidity.unit);
    Serial.println("  Temp   : " + data.temp.value + " " + data.temp.unit);
    Serial.println("===================");
}

void prepareI2CToSleep() {
    Wire.end(); 
    delay(10);

    pinMode(PIN_SDA, INPUT);
    pinMode(PIN_SCL, INPUT);
}

void goToSleep(int minutes) {
    gpio_deep_sleep_hold_en();
    esp_sleep_enable_timer_wakeup(MINUTES_EN_MICROSECONDES(minutes));
    Serial.println("Go to sleep");
    Serial.flush();
    esp_deep_sleep_start();
}


//*****************************************************************************
// SCD40
//*****************************************************************************

bool initSCD40(){
    Serial.println("SCD40 start init");
    scd40.begin(Wire, SCD40_I2C_ADDR_62);
    delay(100);

    if (scd40.wakeUp() != NO_ERROR_SCD4X) {
        Serial.println("Error trying to execute wakeUp(): ");
        return false; 
    }
    
    if (scd40.stopPeriodicMeasurement() != NO_ERROR_SCD4X) {
        Serial.println("Error trying to execute stopPeriodicMeasurement(): ");
        return false; 
    }
    
    if (scd40.reinit() != NO_ERROR_SCD4X) {
        Serial.println("Error trying to execute reinit(): ");
        return false; 
    }

    if (scd40.startPeriodicMeasurement() != NO_ERROR_SCD4X) {
        Serial.println("Error trying to execute startPeriodicMeasurement(): ");
        return false; 
    }

    scd40SensorTask.state = SENSOR_WARMING_UP;
    scd40SensorTask.stateChangedAtMs = millis();
    return true;
}

void readSCD40() {
    if (!scd40SensorTask.isEnabled) return;
    uint32_t now = millis();


    switch (scd40SensorTask.state) {

      case SENSOR_READING:
        if (now - scd40SensorTask.stateChangedAtMs >= scd40SensorTask.readingIntervalMs) {
          scd40SensorTask.stateChangedAtMs = now; 

          if (scd40Avg.isReady(MIN_SAMPLES)) {
            scd40SensorTask.state = SENSOR_READY;
            Serial.println("SCD40 SENSOR_READY");
          }else{

            if (!waitForDataReadySCD40(5000)) {
                scd40SensorTask.errorCount++;
            } else {
            
                uint16_t co2 = 0;
                float temp = 0.0f;
                float humidity = 0.0f;
                if (scd40.readMeasurement(co2, temp, humidity) != NO_ERROR_SCD4X) {
                    scd40SensorTask.errorCount++;
                    Serial.println("SCD40 error readMeasurement");
                } else {
                    Serial.println("SCD40 Reading new value");
                    Serial.println("co2 : " + String(co2));
                    Serial.println("temp : " + String(temp));
                    Serial.println("humidity : " + String(humidity));
                   
                   
                    scd40SensorTask.errorCount = 0; 
                    float values[SCD_COUNT];
                    values[SCD_CO2]  = static_cast<float>(co2);
                    values[SCD_TEMP] = temp;
                    values[SCD_HUM]  = humidity;
                    scd40Avg.add(values);
                }
            }

            // Too many errors
            if (scd40SensorTask.errorCount >= MAX_SENSOR_ERRORS) {
                Serial.println("SCD40 too many errors");
                scd40SensorTask.errorCount = 0;
                scd40SensorTask.state = SENSOR_READY;
            }
          }
        }
        break;


      case SENSOR_WARMING_UP:
        if (now - scd40SensorTask.stateChangedAtMs >= scd40SensorTask.warmupDurationMs) {
          Serial.println("SCD40 end warmup");
          scd40SensorTask.state = SENSOR_READING;
          scd40Avg.reset();
        }
        break;

      case SENSOR_NOT_INIT:
        Serial.println("SCD40 SENSOR_NOT_INIT");
        break;

      case SENSOR_READY:
        break;
    }
}

bool waitForDataReadySCD40(uint32_t timeoutMs) {
    bool dataReady = false;
    uint32_t startTime = millis();

    while (!dataReady) {

        if (scd40.getDataReadyStatus(dataReady) != NO_ERROR_SCD4X) {
            Serial.println("SCD40 Error trying to execute getDataReadyStatus()");
            return false;
        }

        if (millis() - startTime >= timeoutMs) {
            Serial.println("Timeout waiting for SCD40 data.");
            return false;
        }

        delay(100);
    }

    return true;
}

void powerDownSCD40() {
    Serial.println("powerDownSCD40");

    if (scd40.stopPeriodicMeasurement() != NO_ERROR_SCD4X) {
        Serial.println("Error stopPeriodicMeasurement: ");
    }

    delay(100);
 
    if (scd40.powerDown() != NO_ERROR_SCD4X) { //power down is only available for SCD41
        Serial.println("Error SCD4X powerDown ");
    }

    delay(100);
}

//*****************************************************************************
// DS18B20
//*****************************************************************************
bool initDS18B20() {
    Serial.println("DS18B20 start init");

    gpio_hold_dis((gpio_num_t)PIN_TEMP_POWER);
    pinMode(PIN_TEMP_POWER, OUTPUT);
    digitalWrite(PIN_TEMP_POWER, HIGH);

    delay(50); 

    ds18b20.begin();
  
    if (ds18b20.getDeviceCount() == 0) return false;

    ds18b20.setResolution(12);
    ds18b20.requestTemperatures();

    ds18b20SensorTask.state = SENSOR_WARMING_UP;
    ds18b20SensorTask.stateChangedAtMs = millis();

    return true;
}


void readDS18B20() {
    if (!ds18b20SensorTask.isEnabled) return;
    uint32_t now = millis();


    switch (ds18b20SensorTask.state) {

      case SENSOR_READING:
        if (now - ds18b20SensorTask.stateChangedAtMs >= ds18b20SensorTask.readingIntervalMs) {
          ds18b20SensorTask.stateChangedAtMs = now; 

          if (ds18Avg.isReady(MIN_SAMPLES)) {
            ds18b20SensorTask.state = SENSOR_READY;
            Serial.println("DS18B20 SENSOR_READY");
          }else{

            float tempC = ds18b20.getTempCByIndex(0);

            if (tempC == DS18B20_DISCONNECTED_VALUE_C) {
                ds18b20SensorTask.errorCount++;
                Serial.println("DS18B20 error readMeasurement");
            } else {
                Serial.println("DS18B20 Reading new value \n Temp : " + String(tempC));
                float values[DS_COUNT];
                values[DS_TEMP]  = tempC;
                ds18Avg.add(values);

                ds18b20SensorTask.errorCount = 0; 
            }
            //Ask and wait 1500 ms
            ds18b20.requestTemperatures();

            // Too many errors
            if (ds18b20SensorTask.errorCount >= MAX_SENSOR_ERRORS) {
                Serial.println("DS18B20 too many errors");
                ds18b20SensorTask.errorCount = 0;
                ds18b20SensorTask.state = SENSOR_READY;
            }
          }
        }
        break;


      case SENSOR_WARMING_UP:
        if (now - ds18b20SensorTask.stateChangedAtMs >= ds18b20SensorTask.warmupDurationMs) {
            Serial.println("DS18B20 end warmup");
            ds18b20SensorTask.state = SENSOR_READING;
            ds18Avg.reset();
        }
        break;

      case SENSOR_NOT_INIT:
        Serial.println("DS18B20 SENSOR_NOT_INIT");
        break;

      case SENSOR_READY:
        break;
    }
}

void powerDownDS18B20() {
    Serial.println("powerDown DS18B20");

    digitalWrite(PIN_TEMP_POWER, LOW);
    pinMode(PIN_TEMP_DATA, INPUT);
    delay(50);

    gpio_hold_en((gpio_num_t)PIN_TEMP_POWER);
}

//*****************************************************************************
// BME 280
//*****************************************************************************
bool initBME280() {
    Serial.println("BME280 start init");

    gpio_hold_dis((gpio_num_t)PIN_BME_POWER);
    pinMode(PIN_BME_POWER, OUTPUT);
    digitalWrite(PIN_BME_POWER, HIGH);

    delay(50); 

    bool initSuccess = false;
    if (bme280.begin(0x76, &Wire)) {
        initSuccess = true;
    } else if (bme280.begin(0x77, &Wire)) {
        initSuccess = true;
    }

    if (!initSuccess) {
        Serial.println("BME280 init FAILED");
        return false;
    }

    bme280.setSampling(
        Adafruit_BME280::MODE_SLEEP,
        Adafruit_BME280::SAMPLING_X4, // temperature
        Adafruit_BME280::SAMPLING_X16, // pressure
        Adafruit_BME280::SAMPLING_X4, // humidity
        Adafruit_BME280::FILTER_OFF // Always OFF we use deep sleep
        ); 

    bme280SensorTask.state = SENSOR_WARMING_UP;  
    bme280SensorTask.stateChangedAtMs = millis();  
    return true; 
}

void readBME280() {
    if (!bme280SensorTask.isEnabled) return;
    uint32_t now = millis();


    switch (bme280SensorTask.state) {

      case SENSOR_READING:
        if (now - bme280SensorTask.stateChangedAtMs >= bme280SensorTask.readingIntervalMs) {
          bme280SensorTask.stateChangedAtMs = now; 

          if (bmeAvg.isReady(MIN_SAMPLES)) {
            bme280SensorTask.state = SENSOR_READY;
            Serial.println("BME280 SENSOR_READY");
          }else{
            if (!bme280.takeForcedMeasurement()) {
                bme280SensorTask.errorCount++;
                Serial.println("BME280 error readMeasurement");
            } else {
                float tempBme = bme280.readTemperature();
                float humBme = bme280.readHumidity();
                float presBme = bme280.readPressure();
                Serial.println("BME280 Reading new value \n Temp : " + String(tempBme) + " / Humidity : " + String(humBme) + " / Pressure : " + String(presBme));
                

                bme280SensorTask.errorCount = 0; 
                float values[BME_COUNT];
                values[BME_TEMP]  = tempBme;
                values[BME_HUM] = humBme;
                values[BME_PRESS]  = presBme;
                bmeAvg.add(values);
            }
            

            // Too many errors
            if (bme280SensorTask.errorCount >= MAX_SENSOR_ERRORS) {
                Serial.println("BME280 too many errors");
                bme280SensorTask.errorCount = 0;
                bme280SensorTask.state = SENSOR_READY;
            }
          }
        }
        break;


      case SENSOR_WARMING_UP:
        if (now - bme280SensorTask.stateChangedAtMs >= bme280SensorTask.warmupDurationMs) {
            Serial.println("BME280 end warmup");
            bme280SensorTask.state = SENSOR_READING;
            bmeAvg.reset();
        }
        break;

      case SENSOR_NOT_INIT:
        Serial.println("BME280 SENSOR_NOT_INIT");
        break;

      case SENSOR_READY:
        break;
    }
}


void powerDownBME280() {
    Serial.println("powerDownBME280");

    //Keep High because I2C is shared
    digitalWrite(PIN_BME_POWER, HIGH);
    gpio_hold_en((gpio_num_t)PIN_BME_POWER);

    delay(50);
}

//*****************************************************************************
// PLANTOWER PMS 5003
//*****************************************************************************

bool initPMS5003(){
    Serial.println("PMS5003 start init");

    gpio_hold_dis((gpio_num_t)PIN_PMS_SET);
    pinMode(PIN_PMS_SET, OUTPUT);
    digitalWrite(PIN_PMS_SET, HIGH); 
    delay(100);
    
    pmsSerial.begin(PMS::BAUD_RATE, SERIAL_8N1, PIN_PMS_RX, PIN_PMS_TX);

    pms.passiveMode(); 
    pms.wakeUp();
    delay(100); 

    uint32_t start = millis();
    bool detected = false;

    // Boucle de détection (2 secondes suffisent largement maintenant)
    while (millis() - start < 2000) { 
        if (pmsSerial.available()) {
            detected = true;
            break; 
        }
        delay(10);
    }
    

    if (detected) {
        pms.passiveMode(); 
        pmsSensorTask.state = SENSOR_WARMING_UP;
        pmsSensorTask.stateChangedAtMs = millis();
        return true;
    }

    return false;
}


void readPMS5003() {
    if (!pmsSensorTask.isEnabled) return;
    uint32_t now = millis();


    switch (pmsSensorTask.state) {

      case SENSOR_READING:
        if (now - pmsSensorTask.stateChangedAtMs >= pmsSensorTask.readingIntervalMs) {
          pmsSensorTask.stateChangedAtMs = now; 

          if (pmsAvg.isReady(MIN_SAMPLES)) {
            pmsSensorTask.state = SENSOR_READY;
            Serial.println("PMS5003 SENSOR_READY");
          }else{
            PMS::DATA data;
            uint32_t pm1_0 = 0, pm2_5 = 0;


            //Clear Serial and read
            while (pmsSerial.available()) { pmsSerial.read(); }
            pms.requestRead();

            if (pms.readUntil(data)){
                pm1_0  += data.PM_AE_UG_1_0;
                pm2_5  += data.PM_AE_UG_2_5;
                Serial.println("PMS5003 Reading new value \n pm1_0 : " + String(pm1_0) + " / pm2_5 : " + String(pm2_5));

                pmsSensorTask.errorCount = 0; 
                float values[PMS_COUNT];
                values[PM1_0] = static_cast<float>(pm1_0);
                values[PM2_5] = static_cast<float>(pm2_5);
                pmsAvg.add(values);

            } else {
                pmsSensorTask.errorCount++;
                Serial.println("PMS5003 error readMeasurement");
            }

            // Too many errors
            if (pmsSensorTask.errorCount >= MAX_SENSOR_ERRORS) {
                Serial.println("PMS5003 too many errors");
                pmsSensorTask.errorCount = 0;
                pmsSensorTask.state = SENSOR_READY;
            }
          }
        }
        break;


      case SENSOR_WARMING_UP:
        if (now - pmsSensorTask.stateChangedAtMs >= pmsSensorTask.warmupDurationMs) {
            Serial.println("PMS5003 end warmup");
            pmsSensorTask.state = SENSOR_READING;
            pmsAvg.reset();
        }
        break;

      case SENSOR_NOT_INIT:
        Serial.println("PMS5003 SENSOR_NOT_INIT");
        break;

      case SENSOR_READY:
        break;
    }
}



void powerDownPMS5003() {
    Serial.println("powerDown PMS5003");

    pmsSerial.end();

    pinMode(PIN_PMS_RX, INPUT);
    pinMode(PIN_PMS_TX, INPUT);

    digitalWrite(PIN_PMS_SET, LOW);
    delay(50);
    
    gpio_hold_en((gpio_num_t)PIN_PMS_SET);
}



//*****************************************************************************
// DISPLAY
//*****************************************************************************

void initDisplay(bool restartDisplay){
    Serial.println("Display start init");
    // SPI pin remap
    SPI.begin(PIN_EPD_SCL, -1, PIN_EPD_SDA, PIN_EPD_CS);
    Serial.println("SPI initialized");

    // Init e-paper display
    Serial.println("Initializing display...");
    display.init(115200, restartDisplay);

    // Display config
    display.setRotation(1);               
    display.setTextColor(GxEPD_BLACK);   
    Serial.println("Display end init");
}



void displayAirQualityDashboard(const DisplayData& data) {

    display.setPartialWindow(0, 0, display.width(), display.height());
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);

        // ==========================================
        // GRID
        // ==========================================
        display.drawFastVLine(160, 0, 240, GxEPD_BLACK);   // Séparateur vertical gauche/droite
        display.drawFastHLine(160, 80,  256, GxEPD_BLACK);  // Ligne 1: TEMP | HUMIDITY
        display.drawFastHLine(160, 160, 256, GxEPD_BLACK);  // Ligne 2: CO2  | PRESSURE
        display.drawFastVLine(288, 0, 240, GxEPD_BLACK);    // Séparateur vertical colonne droite

        // ==========================================
        // AQI BLOCK
        // ==========================================
        display.setFont(NULL);
        display.setCursor(12, 8);
        display.print("AIR QUALITY");
        display.setCursor(12, 19);
        display.print("INDEX");


        display.setFont(&FreeSansBold18pt7b);
        display.setCursor(12, 118);
        display.setTextSize(2); 
        display.print(data.aqi.value);
        display.setTextSize(1); 

    
        int displayLabelWidth = 136; 

        display.fillRoundRect(12, 130, displayLabelWidth, 28, 4, GxEPD_BLACK);
        display.setTextColor(GxEPD_WHITE);
        display.setFont(&FreeSans9pt7b);

        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(data.aqi.unit, 0, 0, &x1, &y1, &w, &h);

        int cursorX = 12 + (136 - w) / 2 - x1;
        int cursorY = 130 + (28 + h) / 2;

        display.setCursor(cursorX, cursorY);
        display.print(data.aqi.unit);
        
        display.setTextColor(GxEPD_BLACK);

        int filledBars = data.aqi.filledBars;
        
        int barCount   = 5;
        int gap        = 4;    
        int barW       = (displayLabelWidth - (gap * (barCount - 1))) / barCount;

        for (int i = 0; i < barCount; i++) {
            int bx = 12 + i * (barW + gap);  
            if (i < filledBars) {
                display.fillRoundRect(bx, 198, barW, 10, 2, GxEPD_BLACK);
            } else {
                display.drawRoundRect(bx, 198, barW, 10, 2, GxEPD_BLACK);
            }
        }


        display.setFont(NULL); display.setTextSize(1);
        display.setCursor(12, 213); display.print("GOOD");
        display.setCursor(112, 213); display.print("SEVERE");

        // ==========================================
        // DATA BLOCK
        // ==========================================

        // LINE 1
        drawTile(160, 0, 128, 80, "TEMP", data.temp.value.c_str(), data.temp.unit.c_str());
        drawTile(288, 0, 128, 80, "HUMIDITY", data.humidity.value.c_str(), data.humidity.unit.c_str());

        // LINE 2
        drawTile(160, 80, 128, 80, "CO2", data.co2.value.c_str(), data.co2.unit.c_str());
        drawTile(288, 80, 128, 80, "PRESSURE", data.pressure.value.c_str(), data.pressure.unit.c_str());

        // LINE 3
        drawTile(160, 160, 128, 80, "PM2.5", data.pm2_5.value.c_str(), data.pm2_5.unit.c_str());
        drawTile(288, 160, 128, 80, "PM1.0", data.pm1_0.value.c_str(), data.pm1_0.unit.c_str());


    } while (display.nextPage());
}


void drawTile(int x, int y, int w, int h,
              const char* label, const char* value, const char* unit) {

    // LABEL
    display.setFont(NULL);
    display.setTextSize(1);
    display.setCursor(x + 8, y + 8);
    display.print(label);

    // DATA
    display.setFont(&FreeSansBold18pt7b);
    display.setTextSize(1);

    int16_t vx, vy;
    uint16_t vw, vh;
    display.getTextBounds(value, 0, 0, &vx, &vy, &vw, &vh);

    int valueCursorY = y + (h + vh) / 2 + 4;
    display.setCursor(x + 8 - vx, valueCursorY);
    display.print(value);

    // UNIT
    display.setFont(&FreeSerifItalic9pt7b);
    display.setTextSize(1);

    int16_t ux, uy;
    uint16_t uw, uh;
    display.getTextBounds(unit, 0, 0, &ux, &uy, &uw, &uh);

    int unitCursorX = x - vx + vw + 14;
    int unitCursorY = valueCursorY + 2;

    
    display.setCursor(unitCursorX , unitCursorY);
    display.print(unit);
}


void powerDownDisplay() {
    Serial.println("powerDown Display");

    display.hibernate();
}



