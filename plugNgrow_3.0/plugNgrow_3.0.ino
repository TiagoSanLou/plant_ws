/*

   Esse sketch conecta minha jardineira a internet. Ele

   lê a temperatura do solo com um DS18B20;
   a temperatura e umidade ambiente com um DHT11;
   a umidade do solo com um higrômetro grove;
   e a luminosidade com um LDR.

   Os dados são enviados via serial do para uma RPi
   utilizando ROS, com o pkg ROSSERIAL.

   Autor: Tiago S. Lourenço
   Data: 02/01/2017

*/

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <math.h>

#include "DHT.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
****** Declarando ******
*/

//Pino para DHT11 - Temp. e umidade ambiente
#define DHTPIN 48             //DHT11 digital pin
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float ambTemp;
float prevAmbTemp;
float ambHumidity;
float prevAmbHumidity;

// Higrômetro
#define higroAnalogPin A0
#define higroPin 44
int higroPowerState = LOW;
int soilMoist;

// DS18B20
#define ONE_WIRE_BUS 52
#define TEMPERATURE_PRECISION 11 // Lower resolution
OneWire oneWire(ONE_WIRE_BUS); // Define uma instancia do oneWire
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1;

// LDR
#define ldrAnalogPin A7

// Pino de Acionamento da Bomba
#define pumpPin 22
int pumpPowerState = LOW;

// Pino de Acionamento da Lâmpada
#define lampPin 24
int lampPowerState = LOW;

// Pino de Acionamento do Ventilador
#define fanPin 26

// Inicializa mensagens e publicações do ROS
ros::NodeHandle  nh;

//void lampCb( const std_msgs::Empty& toggle_msg){
//  digitalWrite(lampPin, HIGH-digitalRead(lampPin));   // blink the led
//}

std_msgs::Float32 ambTemp_msg;
std_msgs::Float32 ambHumidity_msg;
std_msgs::Int32 soilMoist_msg;
std_msgs::Float32 soilTemp_msg;
std_msgs::Int32 light_msg;

ros::Publisher pub_ambTemp("ambTemperature", &ambTemp_msg);
ros::Publisher pub_ambHumidity("ambHumidity", &ambHumidity_msg);
ros::Publisher pub_soilMoist("soilMoisture", &soilMoist_msg);
ros::Publisher pub_soilTemp("soilTemperature", &soilTemp_msg);
ros::Publisher pub_light("ambLight", &light_msg);
//ros::Subscriber<std_msgs::Empty> lamp_sub("lampToggle", &lampCb );

// Intervalos para temporizadores
unsigned long previousReadingMillis = 0;
unsigned long higroReadInterval = 5000; //2 * 60000;         // Interval between reading (milliseconds)
unsigned long higroPoweredTime  = 500;            // power on duration
unsigned long higroInterval = higroReadInterval;

unsigned long previousPumpMillis = 0;
unsigned long pumpStartInterval = 60000;
unsigned long pumpPoweredTime   = 3000;
unsigned long pumpInterval = pumpStartInterval;

unsigned long previousLampMillis = 0;
unsigned long lampStartInterval = 2 * 60 * 60000;       // Interval between reading (milliseconds)
unsigned long lampPoweredTime  = 2 * 60 * 60000;        // power on duration
unsigned long lampInterval = lampStartInterval;

/*
****** Setup *******
*/
void setup()
{
  //Inicia publicações paraa o ROS master
  nh.initNode();
  nh.advertise(pub_ambTemp);
  nh.advertise(pub_ambHumidity);
  nh.advertise(pub_soilMoist);
  nh.advertise(pub_soilTemp);
  nh.advertise(pub_light);

  pinMode(higroAnalogPin, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(higroPin, OUTPUT);
  pinMode(lampPin, OUTPUT);
  pinMode(fanPin, OUTPUT);

  /*
    TESTE DO RELÉ**********************
  */
  //  digitalWrite(pumpPin, LOW);
  //  digitalWrite(lampPin, HIGH);
  //  digitalWrite(fanPin, HIGH);
  //  delay(1000);
  //  digitalWrite(pumpPin, HIGH);
  //  digitalWrite(lampPin, LOW);
  //  digitalWrite(fanPin, HIGH);
  //  delay(1000);
  //  digitalWrite(pumpPin, HIGH);
  //  digitalWrite(lampPin, HIGH);
  //  digitalWrite(fanPin, LOW);
  //  delay(1000);
  //  digitalWrite(pumpPin, HIGH);
  //  digitalWrite(lampPin, HIGH);
  //  digitalWrite(fanPin, HIGH);
  //****************************

  digitalWrite(pumpPin, HIGH);
  digitalWrite(higroPin, LOW);
  digitalWrite(lampPin, HIGH);
  digitalWrite(fanPin, HIGH);

  // Inicia comunicação serial
  Serial.begin(57600);
  Serial.println("Start Plant!");

  // Inicia DHT11
  dht.begin();

  // Inicia sensor DS19B20
  sensors.begin();
  sensors.getAddress(sensor1, 0);
  sensors.setResolution(sensor1, TEMPERATURE_PRECISION); // 9to11 bits
}

/*
****** LOOP ******
*/
void loop()
{
  unsigned long currentMillis = millis();
  /*
     LEITURAS
  */

  // Lê DHT11 - temperatura e umidade ambiente
  ambTemp = dht.readTemperature();
  ambHumidity = dht.readHumidity();
  // Check & log
  if (isnan(ambTemp) || isnan(ambHumidity)) {
    //    Serial.println("Failed to read from DHT sensor!");
    char warn[] = "Failed to read DHT11";
    nh.logwarn(warn);

    ambTemp = prevAmbTemp;
    ambHumidity = prevAmbTemp;
    //return;
  } else {
    prevAmbTemp = ambTemp;
    prevAmbHumidity = ambHumidity;
  }

  Serial.print("Amb. Humidity: ");
  Serial.print(ambHumidity);
  Serial.print("   Amb. Temperature: ");
  Serial.println(ambTemp);

  // Lê sensor DS18B20 de temperatura do solo
  sensors.requestTemperatures();
  float soilTemp = sensors.getTempC(sensor1);
  Serial.print("Temp. do Solo: ");
  Serial.println(soilTemp);

  // Lê o valor do higrometro com frequência ajustada
  if (currentMillis - previousReadingMillis >= higroInterval) {
    previousReadingMillis = currentMillis;
    if (higroPowerState == LOW) {
      higroPowerState = HIGH;
      higroInterval = higroPoweredTime;
    } else {
      int higroAnalogValue = analogRead(higroAnalogPin);
      soilMoist = map(higroAnalogValue, 0, 600, 0, 100);

      //      Serial.print("Umidade do Solo: ");
      //      Serial.println(soilMoist);
      soilMoist_msg.data = soilMoist;
      pub_soilMoist.publish(&soilMoist_msg);

      higroPowerState = LOW;
      higroInterval = higroReadInterval;
    }
    digitalWrite(higroPin, higroPowerState);
  }

  // Lê o valor do LDR
  int ldrAnalogValue = analogRead(ldrAnalogPin);
  int light = map(ldrAnalogValue, 0, 1023, 0, 100);
  //  Serial.print("Luminosidade: ");
  //  Serial.println(light);

  /*
     Comandos
  */

  // Comanda bomba com frequência ajustada, baseado no valor do higro
  if (soilMoist < 30) {
    //    Serial.print("pumpCountdown:  ");
    //    Serial.println(pumpInterval - (currentMillis - previousPumpMillis));
    if (currentMillis - previousPumpMillis >= pumpInterval) {
      previousPumpMillis = currentMillis;
      if (pumpPowerState == LOW ) {
        //        Serial.println("Desliga bomba");
        char info[] = "Desligando a bomba";
        nh.loginfo(info);

        pumpPowerState = HIGH;
        pumpInterval = pumpStartInterval;
      } else {
        //        Serial.println("Liga a Bomba!");
        char info[] = "Ligando a bomba";
        nh.loginfo(info);

        pumpPowerState = LOW;
        pumpInterval = pumpPoweredTime;
      }
      //
      digitalWrite(pumpPin, pumpPowerState);
    }
  } else {
    pumpPowerState = HIGH;
    pumpInterval = pumpStartInterval;
  }

  // Comanda lâmpada com frequência ajustada
  if (currentMillis - previousLampMillis >= lampInterval) {
    previousLampMillis = currentMillis;
    if (lampPowerState == HIGH) {
      char info[] = "Ligando a luz";
      nh.loginfo(info);

      lampPowerState = LOW;
      lampInterval = lampPoweredTime;
    } else {
      char info[] = "Desligando a luz";
      nh.loginfo(info);
      
      lampPowerState = HIGH;
      lampInterval = lampStartInterval;
    }
    digitalWrite(lampPin, lampPowerState);
  }

  /*
     Envia mensagens ao ROS
  */

  ambTemp_msg.data = ambTemp;
  ambHumidity_msg.data = ambHumidity;
  //soilMoist_msg.data = soilMoist;
  soilTemp_msg.data = soilTemp;
  light_msg.data = light;

  pub_ambTemp.publish(&ambTemp_msg);
  pub_ambHumidity.publish(&ambHumidity_msg);
  //pub_soilMoist.publish(&soilMoist_msg);
  pub_soilTemp.publish(&soilTemp_msg);
  pub_light.publish(&light_msg);

  nh.spinOnce();

}
