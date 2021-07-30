#include <FlexCAN.h>
int rpm = 5;
int motortempgauge = 6;
int fuel = 7;
int dcdccontrol = 8;
int ThermistorPin = 18;
int enginefan = 17;
int Vo;
int coolanttemp;
float R1 = 10000;
float logR2, R2, T;
float c1 = 0.9818585903e-03, c2 = 1.995199371e-04, c3 = 1.684445298e-07;
static CAN_message_t rxmsg, txmsg;


void setup() {

Can0.begin(500000);
Serial.begin(115200);
pinMode(rpm,OUTPUT);
analogWrite(rpm, 128);


}
void loop() {
//--------- get 12v voltage
if (Can0.available()) {
Can0.read(rxmsg);
float voltagebig = (( rxmsg.buf[2] << 8) | rxmsg.buf[1]);
float voltage = voltagebig/32;

///------ get rpm and send to gauge

float rpmraw = (( rxmsg.buf[4] << 8) | rxmsg.buf[3]);
float rpm = rpmraw/32;
int rpmpulse = rpm*2;
analogWriteFrequency(rpm, rpmpulse); 



//---------Temperature read

Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15; //in C
  coolanttemp = T;

//--------- Activate engine bay fan

if (coolanttemp > 40) 
{
digitalWrite(enginefan, HIGH); 
}
else
{
digitalWrite(enginefan, LOW);
}




}
}
