#include <FlexCAN_T4.h>
#include <Metro.h>
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6

//////timers
Metro coolanttimer = Metro(1000);



//gauges
int rpm = 5;
int motortempgauge = 6;
int fuel = 7;
float rpmraw;

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
int Batvolt;


void setup() {
Serial.begin(115200); delay(400);
Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i<NUM_RX_MAILBOXES; i++){
    Can0.setMB(i,RX,STD);
  }
  for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
    Can0.setMB(i,TX,STD);
  }
  Can0.setMBFilter(REJECT_ALL);
  Can0.enableMBInterrupts();
  Can0.setMBFilterProcessing(MB0,0x3FF, 0xFF);
  //Can0.setMBFilterProcessing(MB1,0x400, 0xFF);
  //Can0.setMBFilterProcessing(MB2,0x0B,0xFF);
  Can0.enhanceFilter(MB0);
  //Can0.enhanceFilter(MB1);
  Can0.onReceive(MB0,canSniff1);
  //Can0.onReceive(MB1,canSniff2);
  //Can0.onReceive(MB2,canSniff);
  Can0.mailboxStatus();


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

//Switch off contactors on startup
digitalWrite (precharge, LOW);
digitalWrite (maincontactor, LOW);


delay(3000);


//-------If charge port plugged in on startup run through charging setup.
digitalRead (simpprox);
if (digitalRead(simpprox)) ///put CPWM and CSDN to High and enable charge mode, disabling drive.
{
digitalWrite (precharge, HIGH);   //activate prehcharge on start up
analogWrite(rpm, 128);
analogWriteFrequency(rpm, 2000); //Start rpm at intial high to simulate engine start.Serial.print("normal startup");
digitalWrite(csdn, LOW); // Or high? 
Serial.print("normal startup");
}
else // run normal start up
{
 //Also send canbus message to inverter to set forward and reverse at same time to enable charge mode
digitalWrite(cpwm, LOW); // Or high? 
digitalWrite(csdn, LOW); // Or high? 
Serial.print("charge port connected");
}
delay(3000);
}

void canSniff1(const CAN_message_t &msg) {
 if (msg.id == 0x3FF)
  {
  HVbus = msg.buf[5];
  Batvolt = msg.buf[6];
  rpmraw = (( msg.buf[4] << 8) | msg.buf[3]);
  
}
}


void coolant()
{
if(coolanttimer.check()){
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
}

void gauges(){
float rpm = rpmraw/32;
int rpmpulse = rpm*2;
if (rpmpulse < 1600) //power steering is expecting to see engine idle at least.
{
rpmpulse = 1602;
}
analogWriteFrequency(rpm, rpmpulse); 

//To Do
// BMS SoC and temperature from motor

}



void loop() {
Can0.events();
  ///------ get rpm and send to gauge


   


//--------contactor close cycle
// if hv bus is within a few volts of battery voltage and OI is sending close main contactor, close main contactor and open precharge. Also activate dc-dc
HVdiff = Batvolt - HVbus; //calculates difference between battery voltage and HV bus
digitalRead (maincontactorsignal);
if ((maincontactorsignal = HIGH) && ( HVdiff < 10) && digitalRead (simpprox = LOW)) //only run if charge cable is unplugged
{
digitalWrite (maincontactor, HIGH);
analogWriteFrequency(dcdccontrol, 200); //change this number to change dcdc voltage output
digitalWrite (dcdcon, HIGH);
digitalWrite (precharge, LOW);
}




//--------Charge process Not done yet
digitalRead (simppilot);
digitalRead (chargebutton);
digitalRead (DCSW);

if ((simppilot = HIGH)&& (chargebutton = HIGH))
{
digitalWrite (chargestart, HIGH); // semd signal to simpcharge to send AC voltage
digitalWrite (precharge, HIGH); // close  Battery precharge contactor

}
if ((simppilot = HIGH) && (DCSW = HIGH) && (chargebutton = HIGH)) //needs pilot signal, HV bus precharged and the charge button pressed before charging starts.
{
digitalWrite (accontactor, HIGH);
digitalWrite (maincontactor, HIGH);
digitalWrite (csdn, LOW);

}
else
{
digitalWrite (csdn, HIGH);
digitalWrite (accontactor, LOW);
digitalWrite (chargestart, LOW);

}

coolant(); // check coolant temperature and swtich on engine bay fan if needed.
gauges(); //send information to guages



}
