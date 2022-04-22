#include <FlexCAN_T4.h>
#include <Metro.h>
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6

//////timers
Metro coolanttimer = Metro(1000);
Metro chargerEVSE = Metro(100);
Metro charger800 = Metro(800);

//OI inputs
int startbutton = 13;
int brake = 14;
int fwd = 15;
int rev = 16;


//gauges
int rpm = 5;
int motortempgauge = 37;
int fuel = 36;
float rpmraw;
int batterylight = 31;

int dcdcon = 2;
int dcdccontrol = 23;

//coolant temp and engine bay fan

int ThermistorPin = 18;
int enginefan = 17;
int Vo;
int coolanttemp;
float R1 = 10000;
float logR2, R2, T;
float c1 = 0.9818585903e-03, c2 = 1.995199371e-04, c3 = 1.684445298e-07;

//contactors
int maincontactorsignal = 20;
int precharge = 22;
int maincontactor = 21;
int maincontactorsingalvalue = 1;

//Charging
int cpwm = 24;
int csdn = 25;
int accontactor = 32;
int simpprox = 26;
int simppilot = 27;
int chargestart = 28;
int chargebutton = 12;

//HV stuff
int HVbus;
int HVdiff;
int Batvolt;
int Batvoltraw;
int AuxBattVolt;
int Batmax;
int Batmaxraw;

int chargemode;

// car inputs
int Batterysoc;


void setup() {
  Serial.begin(115200); delay(400);
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++) {
    Can0.setMB(i, RX, STD);
  }
  for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++) {
    Can0.setMB(i, TX, STD);
  }
  //  Can0.setMBFilter(REJECT_ALL);
  Can0.enableMBInterrupts();
  //  Can0.setMBFilterProcessing(MB0, 0x3FF, 0xFF);
  //Can0.setMBFilterProcessing(MB1,0x400, 0xFF);
  //Can0.setMBFilterProcessing(MB2,0x0B,0xFF);
  // Can0.enhanceFilter(MB0);
  //Can0.enhanceFilter(MB1);
  Can0.onReceive(MB0, canSniff1);
  //Can0.onReceive(MB1,canSniff2);
  //Can0.onReceive(MB2,canSniff);
  Can0.mailboxStatus();


  //outputs
  pinMode(rpm, OUTPUT);
  pinMode(enginefan, OUTPUT);
  pinMode(motortempgauge, OUTPUT);
  pinMode(precharge, OUTPUT);
  pinMode(maincontactor, OUTPUT);
  pinMode(dcdccontrol, OUTPUT);
  pinMode(dcdcon, OUTPUT);
  pinMode(chargestart, OUTPUT);
  pinMode(cpwm, OUTPUT);
  pinMode(csdn, OUTPUT);
  pinMode(accontactor, OUTPUT);
  pinMode(startbutton, OUTPUT);
  pinMode(fwd, OUTPUT);
  pinMode(rev, OUTPUT);
  pinMode(brake, OUTPUT);
  pinMode(batterylight, OUTPUT);
  //inputs
  pinMode(simpprox, INPUT_PULLUP);
  pinMode(simppilot, INPUT_PULLUP);
  pinMode(chargebutton, INPUT_PULLUP);
  pinMode(maincontactorsignal, INPUT_PULLUP);


  //Switch off contactors on startup
  digitalWrite (precharge, LOW);
  digitalWrite (maincontactor, LOW);
  digitalWrite (accontactor, LOW);

  chargemode = 0;

  delay(3000);


  //-------If charge port plugged in on startup run through charging setup.
  digitalRead (simpprox);
  if (digitalRead(simpprox)) // run normal start up
  {
    digitalWrite (startbutton, HIGH);
    digitalWrite (precharge, HIGH);   //activate prehcharge on start up
    analogWrite(rpm, 128);
    analogWriteFrequency(rpm, 2000); //Start rpm at intial high to simulate engine start.Serial.print("normal startup");
    //digitalWrite(csdn, LOW);
    digitalWrite(fwd, HIGH);
    Serial.print("normal startup");
    chargemode = 1;


  }
  else ///put CPWM and CSDN to High and enable charge mode, disabling drive.
  {

    //digitalWrite(fwd, HIGH);
    //digitalWrite(rev, HIGH);
    delay (1000);
    //digitalWrite(csdn, HIGH);
    // digitalWrite(cpwm, HIGH);
    Serial.print("charge port connected");
    chargemode = 2;

  }
  delay(1000);
}

void canSniff1(const CAN_message_t &msg) {
  if (msg.id == 0x3FF)
  {
    HVbus = (( msg.buf[6] << 8) | msg.buf[5]);
    HVbus = HVbus / 32;
    Batvoltraw = (( msg.buf[2] << 8) | msg.buf[1]);
    Batvolt = Batvoltraw / 32;
    rpmraw = (( msg.buf[4] << 8) | msg.buf[3]);
    Batterysoc = msg.buf[7];

  }
  if (msg.id == 0x400)
  {
    AuxBattVolt = msg.buf[0];
  }
  if (msg.id == 0x3FD)
  {
    Batmaxraw = (( msg.buf[1] << 8) | msg.buf[0]);
    Batmax = Batmaxraw;

  }

}

void coolant()
{
  if (coolanttimer.check()) {
    //---------Temperature read

    Vo = analogRead(ThermistorPin); /// use 10k resistor
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
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

void closecontactor() { //--------contactor close cycle
  // if hv bus is within a few volts of battery voltage and OI is sending close main contactor, close main contactor and open precharge. Also activate dc-dc
  //HVdiff = Batvolt - HVbus; //calculates difference between battery voltage and HV bus
  //Serial.print (HVdiff);
  digitalRead(maincontactorsignal);
  maincontactorsingalvalue = digitalRead(maincontactorsignal);
  // Serial.print (maincontactorsingalvalue);
  if (maincontactorsingalvalue == 0)// & HVdiff < 10)
  {
    digitalWrite (maincontactor, HIGH);
    analogWriteFrequency(dcdccontrol, 200); //change this number to change dcdc voltage output
    digitalWrite (dcdcon, HIGH);
    digitalWrite (precharge, LOW);
  }
  else if (maincontactorsingalvalue == 1)
  {
    digitalWrite (maincontactor, LOW);
    analogWriteFrequency(dcdccontrol, 200); //change this number to change dcdc voltage output
    digitalWrite (dcdcon, LOW);
    digitalWrite (precharge, HIGH);
  }
}

void gauges() {
  // RPM
  float rpm = rpmraw / 32;
  int rpmpulse = rpm * 2;
  if (rpmpulse < 1600) //power steering is expecting to see engine idle at least.
  {
    rpmpulse = 1602;
  }

  analogWriteFrequency(rpm, rpmpulse);
  // Battery light
  if (AuxBattVolt < 13)
  {
    digitalWrite(batterylight, HIGH);
  }
  else
  {
    digitalWrite(batterylight, LOW);
  }
  // Battery Soc
  analogWriteFrequency(fuel, 500);
  int fuelpwm = Batterysoc * 2.43;
  int fuelfreq = fuelpwm + 12.8;
  analogWrite(fuel, fuelfreq);

  //To Do

  // temperature from coolant.


}

void charging() {
  if (chargerEVSE.check()) {
    if (/*simpproxvalue == 0 && simppilotvalue == 0 && */Batmax < 4100)
    {
    //unsigned char evse[8] = {0x00, 0x00, 0xB6, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN_message_t msg1;
    msg1.id = (0x285);
   // memcpy (msg1.buf, evse, 8);
    msg1.buf[2] = 0xB6;
    Can0.write(msg1);
    }
    else
    {
    CAN_message_t msg1;
    msg1.id = (0x285);
   // memcpy (msg1.buf, evse, 8);
    msg1.buf[2] = 0x00;
    Can0.write(msg1);
    }
  }

if (charger800.check()) {
  unsigned char charger800[8] = {0x28, 0x0F, 0x00, 0x37, 0x00, 0x00, 0x0A, 0x00};
 CAN_message_t msg1;
    msg1.id = (0x286);
    memcpy (msg1.buf, charger800, 8);
    msg1.buf[2] = 0x1E;
    Can0.write(msg1);


}
  /*
    //--------Charge process Not done yet
    digitalRead (simppilot);
    digitalRead (simpprox);
    int simpproxvalue = digitalRead(simpprox);
    int simppilotvalue = digitalRead(simppilot);
    maincontactorsingalvalue = digitalRead(maincontactorsignal);
    //digitalRead (chargebutton);
    digitalRead (maincontactorsignal); // main contactor close signal from OI control board
  */

}


void loop() {
  if (chargemode == 1)
  {
    Can0.events();
    closecontactor(); //checks precharge level and close contactor
    coolant(); // check coolant temperature and swtich on engine bay fan if needed.
    gauges(); //send information to guages
  }
  else if (chargemode == 2)
  {
    Can0.events();
    charging();
    gauges(); //send information to guages
  }
  /// To Do

  // Interupt to stop charge.



}
