#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define SOUND_PIN 18
int lastState = HIGH;
int currentState;



Adafruit_BME280 bme;

//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;



void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(SOUND_PIN, INPUT);


  
  /*
  Serial.println(F("BME280 test"));

  bool status;

  //default settings

  status = bme.begin(0x76);
  if(!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while(1);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();
  */
}

void loop() {
  // put your main code here, to run repeatedly:

  //printValues();
  measureSound();
  delay(delayTime);

}


void measureSound() {

    currentState = digitalRead(SOUND_PIN);

    if(lastState == HIGH && currentState == LOW)
    {
      Serial.println("The sound has been detected");
    }
    else if (lastState == LOW && currentState == HIGH)
    {
      Serial.println("The sound has dissapiered!");
    }

    lastState = currentState;

}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.println("Humitidy = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}
