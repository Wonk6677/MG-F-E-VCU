#include <FlexCAN.h>
//gauges
int rpm = 5;
int motortempgauge = 6;
int fuel = 7;

int dcdcon = 2;
int dcdccontrol = 8;
//coolant temp and engine bay fan

int ThermistorPin = 18;
int enginefan = 17;
int Vo;
int coolanttemp;
float R1 = 10000;
float logR2, R2, T;
float c1 = 0.9818585903e-03, c2 = 1.995199371e-04, c3 = 1.684445298e-07;

//contactors
int maincontactorsignal = 16;
int precharge = 22;
int maincontactor = 21;

//Charging
int cpwm = 24;
int csdn = 25;
int accontactor = 30;
int simpprox = 26;
int simppilot = 27;
int chargestart = 28;
int chargebutton = 31;
int DCSW = 29;

//HV stuff
int HVbus;
int HVdiff;


static CAN_message_t rxmsg, txmsg;


void setup() {

Can0.begin(500000);
Serial.begin(115200);
//outputs
pinMode(rpm,OUTPUT);
pinMode(enginefan,OUTPUT);
pinMode(motortempgauge, OUTPUT);
pinMode(precharge,OUTPUT);
pinMode(maincontactor,OUTPUT);
pinMode(dcdccontrol, OUTPUT);
pinMode(dcdcon, OUTPUT);
pinMode(chargestart, OUTPUT);
pinMode(cpwm, OUTPUT);
pinMode(csdn, OUTPUT);
pinMode(accontactor, OUTPUT);

//inputs
pinMode(simpprox, INPUT_PULLUP);
pinMode(simppilot, INPUT_PULLUP);
pinMode(chargebutton, INPUT_PULLUP);
pinMode(DCSW, INPUT_PULLUP);
pinMode(maincontactorsignal, INPUT_PULLUP);

delay(3000);


//-------If charge port plugged in on startup run through charging setup.
digitalRead (simpprox);
if (digitalRead(simpprox = LOW)) ///put CPWM and CSDN to High and enable charge mode, disabling drive.
{
//Also send canbus message to inverter to set forward and reverse at same time to enable charge mode
digitalWrite(cpwm, LOW); // Or high? 
digitalWrite(csdn, LOW); // Or high? 

}
else // run normal start up
{
digitalWrite (precharge, LOW);   //activate prehcharge on start up
analogWrite(rpm, 128);
analogWriteFrequency(rpm, 2000); //Start rpm at intial high to simulate engine start.
}
delay(3000);
}
void loop() {



//----Read canbus messages

if (Can0.available()) {

  
//--------- get HV bus voltage and battery voltage
Can0.read(rxmsg);
HVbus = (rxmsg.buf[5]);
HVdiff = (rxmsg.buf[6]) - (rxmsg.buf[5]); //calculates difference between battery voltage and HV bus



///------ get rpm and send to gauge

float rpmraw = (( rxmsg.buf[4] << 8) | rxmsg.buf[3]);
float rpm = rpmraw/32;
int rpmpulse = rpm*2;
if (rpmpulse < 1600) //power steering is expecting to see engine idle at least.
{
rpmpulse = 1602;
}
analogWriteFrequency(rpm, rpmpulse); 
}

//--------contactor close cycle
// if hv bus is within a few volts of battery voltage and OI is sending close main contactor, close main contactor and open precharge. Also activate dc-dc
digitalRead (maincontactorsignal);
if ((maincontactorsignal = LOW) && ( HVdiff < 10) && digitalRead (simpprox = HIGH)) //only run if charge cable is unplugged
{
digitalWrite (maincontactor, LOW);
analogWriteFrequency(dcdccontrol, 200); //change this number to change dcdc voltage output
digitalWrite (dcdcon, LOW);
digitalWrite (precharge, HIGH);
}




//--------Charge process Not done yet
digitalRead (simppilot);
digitalRead (chargebutton);
digitalRead (DCSW);

if ((simppilot = LOW)&& (chargebutton = LOW))
{
digitalWrite (chargestart, LOW); // semd signal to simpcharge to send AC voltage
digitalWrite (precharge, LOW); // close precharge contactor

delay (10000); //delay to allow precharge
}
if ((simppilot = LOW) && (DCSW = LOW) && (chargebutton = LOW)) //needs pilot signal, HV bus precharged and the charge button pressed before charging starts.
{
digitalWrite (accontactor, LOW);
digitalWrite (maincontactor, LOW);
digitalWrite (csdn, HIGH);

}
else
{
digitalWrite (csdn, LOW);
digitalWrite (accontactor, HIGH);
digitalWrite (chargestart, HIGH);

}

//---------Temperature read

Vo = analogRead(ThermistorPin); /// use 10k resistor
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15; //in C
  coolanttemp = T; 

//--------- Activate engine bay fan

if (coolanttemp > 40) 
{
digitalWrite(enginefan, LOW); 
}
else
{
digitalWrite(enginefan, HIGH);

}





}
