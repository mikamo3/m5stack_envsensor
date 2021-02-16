
#include <M5StickC.h>
#include "SHT3X.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "bmm150.h"
#include "bmm150_defs.h"
#include "Adafruit_SGP30.h"

SHT3X sht3x;
BMM150 bmm = BMM150();
bmm150_mag_data value_offset;
Adafruit_BMP280 bme;
Adafruit_SGP30 sgp;

float tmp = 0.0;
float hum = 0.0;
float pressure = 0.0;
uint8_t setup_flag = 1;
void calibrate(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();

  while ((millis() - timeStart) < timeout)
  {
    bmm.read_mag_data();

    /* Update x-Axis max/min value */
    if (value_x_min > bmm.raw_mag_data.raw_datax)
    {
      value_x_min = bmm.raw_mag_data.raw_datax;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);

    }
    else if (value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
      // Serial.print("update value_x_max: ");
      // Serial.println(value_x_max);
    }

    /* Update y-Axis max/min value */
    if (value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);

    }
    else if (value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
      // Serial.print("update value_y_max: ");
      // Serial.println(value_y_max);
    }

    /* Update z-Axis max/min value */
    if (value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);

    }
    else if (value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
      // Serial.print("update value_z_max: ");
      // Serial.println(value_z_max);
    }

    Serial.print(".");
    delay(1);

  }

  value_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
  value_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
  value_offset.z = value_z_min + (value_z_max - value_z_min) / 2;
}


void setup() {
  M5.begin();
  Wire1.begin(32, 33);
  Wire.begin(0, 26);
  if (! sgp.begin(&Wire1)) {
    Serial.println("Sensor not found :(");
    while (1);
  }

  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  pinMode(M5_BUTTON_HOME, INPUT);

  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while (1);
  } else {
    Serial.println("Initialize done!");
  }
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  calibrate(10);

  Serial.print("\n\rCalibrate done..");
}

void loop() {
  // put your main code here, to run repeatedly:
  // put your main code here, to run repeatedly:
  if (sht3x.get() == 0) {
    tmp = sht3x.cTemp;
    hum = sht3x.humidity;
  }


  M5.Lcd.setCursor(0, 10);
  M5.Lcd.printf("Temp: %2.1f'c\nHumi: %2.0f%%\n", tmp, hum);
  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
  Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");
  M5.Lcd.printf("TVOC: %6d ppb\n", sgp.TVOC);
  M5.Lcd.printf("eCO2: %6d ppm", sgp.eCO2);
  delay(1000);
  if (!setup_flag) {
    setup_flag = 1;

    if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
      Serial.println("Chip ID can not read!");
      while (1);
    } else {
      Serial.println("Initialize done!");
    }
    if (!bme.begin(0x76)) {
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      while (1);
    }
    calibrate(10);
    Serial.print("\n\rCalibrate done..");
  }
  if (digitalRead(M5_BUTTON_HOME) == LOW) {
    setup_flag = 0;
    while (digitalRead(M5_BUTTON_HOME) == LOW);
  }

}
