//#include "Adafruit_VL53L0X.h" 
#include <Wire.h>
//#include <Adafruit_VL53L0X.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x09
#define LOX2_ADDRESS 0x19

// set the pins to shutdown
#define SHT_LOX1 9
#define SHT_LOX2 10

// objects for the vl53l0x
//Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
//Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
//VL53L0X_RangingMeasurementData_t measure1;
//VL53L0X_RangingMeasurementData_t measure2;

Adafruit_TSL2561_Unified tsl1 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 1);
//Adafruit_TSL2561_Unified tsl2 = Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, 2);

void displaySensorDetails(void)
{
  sensor_t sensor1, sensor2;
  tsl1.getSensor(&sensor1);
//  tsl2.getSensor(&sensor2);
  
  delay(500);
}

void configureSensor(void)
{

  tsl1.enableAutoRange(true);
//  tsl2.enableAutoRange(true);

  tsl1.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);   
//  tsl2.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);

}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
//  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
//  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
//  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
//  if(!lox1.begin(LOX1_ADDRESS)) {
//    Serial.println(F("Failed to boot first VL53L0X"));
//    while(1);
//  }
//  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
//  if(!lox2.begin(LOX2_ADDRESS)) {
//    Serial.println(F("Failed to boot second VL53L0X"));
//    while(1);
//  }
}

void read_dual_sensors() {
  
//  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
//  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
//  Serial.print(F("1: "));
//  if(measure1.RangeStatus != 4) {     // if not out of range
//    Serial.print(measure1.RangeMilliMeter);
//  } else {
//    Serial.print(F("Out of range"));
//  }
 
  //Serial.print("");
  //Serial.print(F(" "));
 
  // print sensor two reading
//  Serial.print(F("2: "));
//  if(measure2.RangeStatus != 4) {
//    Serial.print(measure2.RangeMilliMeter);
//  } else {
//    Serial.print(F("Out of range"));
//  }
  
  Serial.println();
}

void setup() {
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
//  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
//  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();

  //displaySensorDetails();
  
  //configureSensor();
 
}

void loop() {
   
  read_dual_sensors();
  delay(1000);
}
