#include <dht11.h>

dht11 DHT11;
#define DHT11PIN 2
String input = "";
String cmd = "lux";
String response = "";
float humidity, temprature;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()) {
    input = Serial.readStringUntil("\n");
  }
  //Serial.println(input);
  if(Serial.available()==0 && input == cmd) {
    DHT11.read(DHT11PIN);
    humidity = DHT11.humidity;
    temprature = DHT11.temperature;
    response = "{\"Humidity\":"+(String)humidity + ","+"\"Temperature\":"+(String)temprature +"}";
    Serial.println(response);
    input = "";
  }
}

