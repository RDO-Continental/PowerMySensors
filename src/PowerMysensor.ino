/* Mesure de puissance club RDO : MPRDO V1.0
 * Alimenté par un transfo qui sert aussi à la mesure de la puissance reelle.
 * Transmission par module NRF24L01 et protocole Mysensors
 * Basé sur le projet "openenergymonitor.org"
 */
#define MY_DEBUG 
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <SPI.h>

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

SSD1306AsciiAvrI2c oled;

/* RADIO Part */ 
#define MY_RADIO_RF24
#include "MySensors.h"

#include "EmonLib.h"             // Include Emon Library
EnergyMonitor emon1;             // Create an instance
EnergyMonitor emon2;             // Create an instance
EnergyMonitor emon3;             // Create an instance
EnergyMonitor emon4;             // Create an instance

boolean DEBUG = 0;                       // Print serial debug

//http://openenergymonitor.org/emon/buildingblocks/calibration

const float Ical1=                90.9;                                 // (2000 turns / 22 Ohm burden) = 90.9
const float Ical2=                90.9;                                 // (2000 turns / 22 Ohm burden) = 90.9
const float Ical3=                90.9;                                 // (2000 turns / 22 Ohm burden) = 90.9
const float Ical4=                16.67;                               // (2000 turns / 120 Ohm burden) = 16.67

float Vcal=                       220;//545;
const float phase_shift=          1.7;
//----------------------------MPRDO V1.0 cablage---------------------------------------------------------------------------------------------------------------
const int  LEDpin=                 DDB7; // emonTx V3 LED
const byte DIP_switch1=            8; // 
const byte DIP_switch2=            9; // 
const byte pulse_countINT=         1; // INT 1 / Dig 3 Terminal Block / RJ45 Pulse counting pin(emonTx V3.4) - (INT0 / Dig2 emonTx V3.2)
const byte pulse_count_pin=        3; // INT 1 / Dig 3 Terminal Block / RJ45 Pulse counting pin(emonTx V3.4) - (INT0 / Dig2 emonTx V3.2)
const int  AcVin=                 A6; // Tension apres le transfo 9V
const int  Ipow1=                 A0; // Premiere sonde en courant
const int  Ipow2=                 A1; // Seconde sonde en courant
const int  Ipow3=                 A2; // Troisieme sonde en courant
const int  Ipow4=                 A3; // Quatrieme sonde en courant
const int  Fp1OptoNeg=             4; // Fil pilote 1 : Partie negative de la sinusoide
const int  Fp1OptoPos=             5; // Fil pilote 1 : Partie positive de la sinusoide
const int  Fp2OptoNeg=             6; // Fil pilote 2 : Partie negative de la sinusoide
const int  Fp2OptoPos=             7; // Fil pilote 2 : Partie positive de la sinusoide
const int  PulseCnt=               0; // Comptage de pulse du compteur EDF
//-------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------Config MySensor----------------------------------------------------------------------------------------------------
#define CHILD_ID_REAL_POWER 0
#define CHILD_ID_APPARENT_POWER 1
#define CHILD_ID_POWER_FACTOR 2
#define CHILD_ID_VOLTAGE 3
#define CHILD_ID_CURRENT_1 4
#define CHILD_ID_CURRENT_2 5
#define CHILD_ID_CURRENT_3 6
#define CHILD_ID_CURRENT_4 7

unsigned long lastSend;
unsigned long SEND_FREQUENCY = 1000; // Minimum time between send (in milliseconds). We don't wnat to spam the gateway.
unsigned long lastSend2;
unsigned long SEND_FREQUENCY2 = 1000;//300000; // Minimum time between send (in milliseconds). We don't wnat to spam the gateway.
bool metric = true;

MyMessage realPowerMsg(CHILD_ID_REAL_POWER,V_WATT);
MyMessage apparentPowerMsg(CHILD_ID_APPARENT_POWER,V_VA);
MyMessage powerFActorMsg(CHILD_ID_POWER_FACTOR,V_POWER_FACTOR);
MyMessage supplyVoltageMsg(CHILD_ID_VOLTAGE,V_VOLTAGE);
MyMessage IrmsMsg1(CHILD_ID_CURRENT_1,V_CURRENT);
MyMessage IrmsMsg2(CHILD_ID_CURRENT_2,V_CURRENT);
MyMessage IrmsMsg3(CHILD_ID_CURRENT_3,V_CURRENT);
MyMessage IrmsMsg4(CHILD_ID_CURRENT_4,V_CURRENT);


#define MOC_P 4 // MOC phase positive
#define MOC_N 5 // MOC phase négative

#define CHILD_ID_FP 10 // Identifiant du capteur

typedef struct {
   char mode_code;
   int code_p;
   int code_n;
} modeFilPilote;

modeFilPilote modesFilPilote[4] = {
  {'O', 1, 0}, // Mode Off
  {'G', 0, 1}, // Mode hors gel
  {'E', 1, 1}, // Mode éco
  {'C', 0, 0}  // Mode confort
};

void presentation()  
{ 
  DDRB |= (1 << DDB7); // Led output
  BlinkLed();
  // Send the sketch version information to the gateway
  sendSketchInfo("Energy Meter x 3v2", "1.2");
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_REAL_POWER, S_POWER); // Register this device as power sensor
  present(CHILD_ID_APPARENT_POWER, S_POWER); // Register this device as power sensor
  present(CHILD_ID_POWER_FACTOR, S_POWER); // Register this device as power sensor
  present(CHILD_ID_VOLTAGE, S_MULTIMETER); // Register this device as power sensor
  present(CHILD_ID_CURRENT_1, S_MULTIMETER); // Register this device as power sensor
  present(CHILD_ID_CURRENT_2, S_MULTIMETER); // Register this device as power sensor
  present(CHILD_ID_CURRENT_3, S_MULTIMETER); // Register this device as power sensor
  present(CHILD_ID_CURRENT_4, S_MULTIMETER); // Register this device as power sensor
  metric = getControllerConfig().isMetric;

  /* FIL PILOTE */
  //sendSketchInfo("Fil pilote", "2.0");
  present(CHILD_ID_FP, S_CUSTOM);
  pinMode(MOC_P, OUTPUT);
  pinMode(MOC_N, OUTPUT);
  // Restauration du mode sauvegardé
  //digitalWrite(MOC_P, loadState(MOC_P));
  //digitalWrite(MOC_N, loadState(MOC_N));

}

void setup()
{  
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);

  oled.clear();
  oled.set2X();
  oled.println("Hello RDO!");
  oled.println("A long line may be truncated");
  oled.println();
  /*oled.set2X();
  oled.println("2X demo");
  oled.set1X();
  oled.print("\nmicros: ");*/
  
  DDRB |= (1 << DDB7); // Led output
  emon1.voltage(AcVin, Vcal, phase_shift);  // Voltage: input pin, calibration, phase_shift
  emon1.current(Ipow1, Ical1);       // 60.6 Current: input pin, calibration.
  emon2.voltage(AcVin, Vcal, phase_shift);  // Voltage: input pin, calibration, phase_shift
  emon2.current(Ipow2, Ical2);       // 60.6 Current: input pin, calibration.
  emon3.voltage(AcVin, Vcal, phase_shift);  // Voltage: input pin, calibration, phase_shift
  emon3.current(Ipow3, Ical3);       // 60.6 Current: input pin, calibration.
  emon4.voltage(AcVin, Vcal, phase_shift);  // Voltage: input pin, calibration, phase_shift
  emon4.current(Ipow4, Ical4);       // 60.6 Current: input pin, calibration.
  BlinkLed();
}

void loop()
{
  unsigned long now = millis();
  emon1.calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out
  //emon1.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
  emon2.calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out
  //emon2.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
  emon3.calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out
  //emon3.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
  emon4.calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out
  //emon4.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
  bool sendTime = now - lastSend > SEND_FREQUENCY;
  if (sendTime) { 

    float realPower       = emon1.realPower;        //extract Real Power into variable
    if (DEBUG==1)
    { 
      Serial.print("Real power=");
      Serial.println(realPower);
    }
    if(realPower >= 0 && realPower < 10000){
      send(realPowerMsg.set(realPower,1));
    }
    else { 
      send(realPowerMsg.set(0.0,1));
      } 
    lastSend = now;
  }
  BlinkLed();
  bool sendTime2 = now - lastSend2 > SEND_FREQUENCY2;
  if(sendTime2){
    
    float apparentPower   = emon1.apparentPower;    //extract Apparent Power into variable
    float powerFactor     = emon1.powerFactor;      //extract Power Factor into Variable
    float supplyVoltage   = emon1.Vrms;             //extract Vrms into Variable
    float Irms            = emon1.Irms;             //extract Irms into Variable
    if (DEBUG==1)
    { 
      Serial.print("Apparent power=");
      Serial.println(apparentPower);
      Serial.print("Power factor=");
      Serial.println(powerFactor);
      Serial.print("Volts=");
      Serial.println(supplyVoltage);
      Serial.print("Irms=");
      Serial.println(Irms);
    }
    if(apparentPower >= 0 && apparentPower < 10000){
      send(apparentPowerMsg.set(apparentPower,2));
    }
    if(powerFactor >= 0 && powerFactor < 10000){
      send(powerFActorMsg.set(powerFactor,3));
    }
    if(supplyVoltage >= 0 && supplyVoltage < 1000){
      send(supplyVoltageMsg.set(supplyVoltage,5));
    }
    if(Irms >= 0 && Irms < 1000){   
      send(IrmsMsg1.set(Irms,5));
    }
    lastSend2 = now;

  }
    AfficheResult();  
}
void AfficheResult(void) {
    oled.clear();
    /*oled.set2X();
    oled.println("Hello RDO!");*/
    oled.set1X();
    oled.print("Real Power 1 : ");
    oled.println(emon1.realPower);
    oled.print("Apparent power=");
      oled.println(emon1.apparentPower);
      oled.print("Power factor=");
      oled.println(emon1.powerFactor);
      oled.print("Volts=");
      oled.println(emon1.Vrms);
      oled.print("Irms=");
      oled.println(emon1.Irms);
      oled.print("Time=");
      oled.println(millis());
}
void BlinkLed(void) {
  if ((PORTB & (1 << DDB7))==1 ) {
    Serial.println("Led On");
    PORTB &= ~(1 << DDB7);
  }
  else {
    PORTB |= (1 << DDB7);
    Serial.println("Led Off");
  }
}

void receive(const MyMessage &message) {
  if (message.type == V_HVAC_FLOW_STATE) {
    for(int i=0; i < 4; i++) {
      if(message.getString()[0] == modesFilPilote[i].mode_code) {
        digitalWrite(MOC_P, modesFilPilote[i].code_p);
        digitalWrite(MOC_N, modesFilPilote[i].code_n);
        
        // Sauvegarde du mode
        //saveState(MOC_P, modesFilPilote[i].code_p);
        //saveState(MOC_N, modesFilPilote[i].code_n);
        return;
      }
    }
  }
}
