/***************************************************************************
  This sketch uses the following hardware from Adafruit:
    Feather M4 Express 
    BME680 
    SCD30 
    128x64 OLED FeatherWing 
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
// #include <bsec.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define VBATPIN A6
#define SEALEVELPRESSURE_HPA (1017.60)

float measuredBatteryVoltage = 0.0;
float temperature, humidity, altitude = 0.0;
float pressure, gas_resistance = 0.0;
float scd30_temperature, scd30_relative_humidity, scd30_CO2 = 0.0;

// BME680 Object
Adafruit_BME680 bme680; // I2C

// SCD30 Object 
Adafruit_SCD30 scd30; 

// OLED Display Object 
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire); // I2C Connection

void setup() {
  Serial.begin(115200);
  // while (!Serial);        // Wait for serial connection
  Serial.println(F("BME680, SCD30, and 128x64 OLED Sensor Node"));

  if (!bme680.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1) { delay(10); }
  }
  Serial.println("BME60 Found!");

  // Try to initialize SCD30
  if (!scd30.begin()) {
    Serial.println("Failed to find SCD30 sensor, check wiring!");
    while (1) { delay(10); }
  }
  Serial.println("SCD30 Found!");

  /*** Adjust the rate at which measurements are taken, from 2-1800 seconds */
  // if (!scd30.setMeasurementInterval(10)){
  //   Serial.println("Failed to set measurement interval");
  //   while(1){ delay(10);}
  // }
  Serial.print("Measurement Interval: "); 
  Serial.print(scd30.getMeasurementInterval()); 
  Serial.println(" seconds");

  Serial.print("Ambient pressure offset: ");
  Serial.print(scd30.getAmbientPressureOffset());
  Serial.println(" mBar");

  Serial.print("Altitude offset: ");
  Serial.print(scd30.getAltitudeOffset());
  Serial.println(" meters");

  Serial.print("Temperature offset: ");
  Serial.print((float)scd30.getTemperatureOffset()/100.0);
  Serial.println(" degrees C");

  Serial.print("Forced Recalibration reference: ");
  Serial.print(scd30.getForcedCalibrationReference());
  Serial.println(" ppm");

  // Set up oversampling and filter initialization for the BME680
  bme680.setHumidityOversampling(BME680_OS_2X);
  bme680.setPressureOversampling(BME680_OS_4X);
  bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme680.setGasHeater(320, 150); // 320*C for 150 ms
  bme680.setTemperatureOversampling(BME680_OS_8X);

  Serial.println("128x64 OLED FeatherWing test");
  display.begin(0x3C, true); // Address 0x3C default

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  // Setup the OLED Display 
  display.setRotation(1);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.display(); // actually display all of the above
}

void loop() {
  // Read the battery voltage. 
  measuredBatteryVoltage = analogRead(VBATPIN);
  measuredBatteryVoltage *= 2;    // Multiply by 2 because of the resistive divide by 2. 
  measuredBatteryVoltage *= 3.3;  // Multiply by the analog reference voltage of 3.3V.
  measuredBatteryVoltage /= 1024; // Divide by 1024 to convert to a voltage. 

  if (! bme680.performReading()) {
    Serial.println("Failed to perform BME680 reading :(");
    return;
  }
  temperature = bme680.temperature;
  humidity = bme680.humidity;
  pressure = bme680.pressure;
  gas_resistance = bme680.gas_resistance;
  altitude = bme680.readAltitude(SEALEVELPRESSURE_HPA);

  if (scd30.dataReady()){
    Serial.println("SCD30 Data available!");
    if (!scd30.read()){ Serial.println("Error reading sensor data"); return; }

    scd30_temperature = scd30.temperature;
    scd30_relative_humidity = scd30.relative_humidity;
    scd30_CO2 = scd30.CO2;
  } else { 
    Serial.println("SCD30 Data in not available :("); 
  }

  // pressure = pressure / 100.0;
  // gas_resistance = gas_resistance / 1000.0;

  // BME680
  Serial.print("BME680 Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");
  // Serial.print("             ");
  temperature = temperature * 9/5 + 32;
  Serial.print(temperature);
  Serial.println(" *F");

  Serial.print("BME680 Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("BME680 Pressure: ");
  Serial.print(pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("BME680 Approx. Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.print("BME680 Gas = ");
  Serial.print(gas_resistance / 1000.0);
  Serial.println(" KOhms");

  // SCD30 
  Serial.print("SCD30 Temperature: ");
  Serial.print(scd30_temperature);
  Serial.println(" *C");
  
  scd30_temperature = scd30_temperature * 9/5 + 32;
  Serial.print(scd30_temperature);
  Serial.println(" *F");

  Serial.print("SCD30 Humidity: ");
  Serial.print(scd30_relative_humidity);
  Serial.println(" %");

  Serial.print("SCD30 CO2 PPM: ");
  Serial.print(scd30_relative_humidity, 2);
  Serial.println(" %");

  // Battery Voltage 
  Serial.print("Battery Voltage: ");
  Serial.print(measuredBatteryVoltage);
  Serial.println(" Volts");

  Serial.println();

  // Display to OLED 
  display.clearDisplay();
  display.setCursor(0,0);
  // display.print("Temperature: ");
  display.print(temperature);
  display.print(" *F   ");
  // display.print("Humidity: ");
  display.print(humidity);
  display.println(" %RH");
  display.print("Pressure: ");
  display.println(pressure / 100.0);
  // display.print("Altitude: ");
  // display.println(altitude);
  display.print("Gas Resistance: ");
  display.println(gas_resistance / 1000.0);
  display.println();  // Skip a line on the display.

  display.print(scd30_temperature);
  display.print(" *F   ");
  display.print(scd30_relative_humidity);
  display.println(" %RH");

  display.print("CO2 PPM: ");
  display.println(scd30_CO2);

  display.print("Battery: ");
  display.print(measuredBatteryVoltage);
  display.print(" Volts");

  delay(10);
  yield();
  display.display(); // actually display all of the above
  delay(4990);
}
