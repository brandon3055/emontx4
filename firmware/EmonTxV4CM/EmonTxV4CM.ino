/*
  emonTxV4.0 Continuous Sampling
  using EmonLibCM https://github.com/openenergymonitor/EmonLibCM
  Authors: Robin Emley, Robert Wall, Trystan Lea
  
  -----------------------------------------
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
*/

/*
Change Log:
v1.0: First release of EmonTxV3 Continuous Monitoring Firmware.
v1.1: First stable release, Set default node to 15
v1.2: Enable RF startup test sequence (factory testing), Enable DEBUG by default to support EmonESP
v1.3: Inclusion of watchdog
v1.4: Error checking to EEPROM config
v1.5: Faster RFM factory test
v1.6: Removed reliance on full jeelib for RFM, minimal rfm_send fuction implemented instead, thanks to Robert Wall
v1.7: Check radio channel is clear before transmit
v1.8: PayloadTx.E1 etc were unsigned long. 
v1.9: Unused variables removed.
v2.0: Power & energy calcs using "Assumed Vrms" added, serial output was switched off when rf output is on.
v2.1: Factory test transmission moved to Grp 1 to avoid interference with recorded data at power-up.  [RW - 30/1/21]
v2.1 (duplicate): printTemperatureSensorAddresses() was inside list_calibration() - reason not recorded [G.Hudson 23/12/21]
v2.3: The two v2.1 versions merged [RW - 9/3/22]

-brandon3055
v2.4: Added suppport for expansion module. Expansion module acts as a seperate device on nodeID+1 and is enabled via DIP switch 2
v2.4  Added support for second pulse input on digital in

emonhub.conf node decoder (nodeid is 15 when switch is off, 16 when switch is on)
See: https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
copy the following in to emonhub.conf:

[[15]]
    nodename = emonTx4cm15
    [[[rx]]]
        names = MSG, Vrms, P1, P2, P3, P4, P5, P6, E1, E2, E3, E4, E5, E6, T1, T2, T3, pulse1, pulse2
        datacodes = L,h,h,h,h,h,h,h,l,l,l,l,l,l,h,h,h,L,L
        scales = 1,0.01,1,1,1,1,1,1,1,1,1,1,1,1,0.01,0.01,0.01,1,1
        units = n,V,W,W,W,W,W,W,Wh,Wh,Wh,Wh,Wh,Wh,C,C,C,p,p
        whitening = 1


[[16]]
    nodename = emonTx4cmExp16
    [[[rx]]]
        names = P7, P8, P9, P10, P11, P12, E7, E8, E9, E10, E11, E12
        datacodes = h,h,h,h,h,h,l,l,l,l,l,l
        scales = 1,1,1,1,1,1,1,1,1,1,1,1
        units = W,W,W,W,W,W,Wh,Wh,Wh,Wh,Wh,Wh
        whitening = 1

        

*/
#define Serial Serial3
#include <Arduino.h>
//#include <avr/wdt.h>

const byte version = 24;                                // Firmware version divide by 10 to get version number e,g 05 = v0.5

// Comment/Uncomment as applicable
#define DEBUG                                           // Debug level print out
#define SHOW_CAL                                        // Uncomment to show current for calibration

#define RFM69CW
#define RFMSELPIN PIN_PB5                               // RFM pins
//#define RFPWR 0x99                                    // RFM Power setting - see rfm.ino for more
#define RFPWR 0x9F
#define FACTORYTESTGROUP 1                              // R.F. group for factory test only
#include "emonLibCM.h"

#include <Wire.h>                                       // Required for RFM & temperature measurement
#include <SPI.h>
#include <util/crc16.h>
#include <OneWire.h>

enum rfband {RF12_433MHZ = 1, RF12_868MHZ, RF12_915MHZ }; // frequency band.

byte RF_freq = RF12_433MHZ;                             // Frequency of radio module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. 
byte nodeID = 15;                                       // node ID for this emonTx.
byte nodeIDExt = 16;                                    // node ID for the extansion module if it is enabled.
int networkGroup = 210;                                 // wireless network group, needs to be same as emonBase / emonPi and emonGLCD. OEM default is 210
const int busyThreshold = -97;                          // Signal level below which the radio channel is clear to transmit
const byte busyTimeout = 15;                            // Time in ms to wait for the channel to become clear, before transmitting anyway
int rf_whitening = 2;                                   // RF & data whitening - 0 = no RF, 1 = RF on, no whitening, default = 2: RF is ON with whitening.

typedef struct {
    unsigned long Msg;
    int Vrms;
    int P[6];
    long E[6];
    int T1,T2,T3;
    unsigned long pulse1, pulse2;
} PayloadTX;
PayloadTX emontx;                                       // create an instance

typedef struct {
    int P[6];
    long E[6];
} PayloadTXExt;
PayloadTXExt emontxext;

static void showString (PGM_P s);
 
DeviceAddress allAddresses[3];                          // Array to receive temperature sensor addresses
/*   Example - how to define temperature sensors, prevents an automatic search
DeviceAddress allAddresses[] = {       
    {0x28, 0x81, 0x43, 0x31, 0x7, 0x0, 0xFF, 0xD9}, 
    {0x28, 0x8D, 0xA5, 0xC7, 0x5, 0x0, 0x0, 0xD5},      // Use the actual addresses, as many as required
    {0x28, 0xC9, 0x58, 0x32, 0x7, 0x0, 0x0, 0x89}       // up to a maximum of 6    
};
*/

int allTemps[3];                                        // Array to receive temperature measurements

//----------------------------emonTx V3 Settings - Shared with config.ino------------------------
#define PHASECAL 1.5

// 25A / 333mV output = 75.075
// 102.461A / 333mv   = 307.69
// 30.135A / 333mv    = 90.495

float iCal[] = {307.69, 90.495, 90.495, 90.495, 90.495, 90.495};
float iLead[] = {PHASECAL, PHASECAL, PHASECAL, PHASECAL, PHASECAL, PHASECAL};
int adcChannels[] = {3, 4, 5, 6, 8, 9};

float iCalExt[] = {90.495, 90.495, 90.495, 90.495, 90.495, 30.786839};
float iLeadExt[] = {PHASECAL, PHASECAL, PHASECAL, PHASECAL, PHASECAL, PHASECAL};
int adcChannelsExt[] = {10, 11, 16, 17, 18, 19};

float vCal  = 810.4;          // (6 x 10000) / 75 = 800.0
const float vCal_USA = 810.4; // Will be the same

bool  USA=false;
float assumedVrms2 = 240.0;  // voltage to use for calculating assumed apparent power if a.c input is absent.
float period = 9.96;         // datalogging period
bool  pulse_enable = true;   // pulse counting
int   pulse_period = 100;    // pulse min period
bool  temp_enable = true;    // enable temperature measurement
byte  temp_addr[24];         // sensor address data
bool  ext_enable = false;    // enable the extension module

//----------------------------emonTx V3 hard-wired connections-----------------------------------
const byte LEDpin      = PIN_PB2;  // emonTx V3 LED
const byte DIP_switch1 = PIN_PA4;  // RF node ID (default no change in node ID, switch on for nodeID + 1) switch off D8 is HIGH from internal pullup
const byte DIP_switch2 = PIN_PA5;  // Voltage selection 240 / 120 V AC (default switch off 240V)  - switch off D9 is HIGH from internal pullup


//----------------------------------------Setup--------------------------------------------------
void setup() 
{  
  //wdt_enable(WDTO_8S);
  
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin,HIGH);
  
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  
  // Serial---------------------------------------------------------------------------------
  Serial.begin(115200);

  // ---------------------------------------------------------------------------------------
  if (digitalRead(DIP_switch1)==LOW) {
    nodeID++;                         // IF DIP switch 1 is switched on (LOW) then add 1 from nodeID
    nodeIDExt++;
  }
  
  #ifdef DEBUG
    Serial.print(F("emonTx V4.0 EmonLibCM Continuous Monitoring V")); Serial.println(version*0.1);
    Serial.println(F("OpenEnergyMonitor.org"));
  #else
    Serial.println(F("describe:EmonTX4CM"));
  #endif
 
  load_config(true);                                                   // Load RF config from EEPROM (if any exists)

  delay(1000);
  pinMode(RFMSELPIN, INPUT_PULLUP);
  if (digitalRead(RFMSELPIN)==LOW) {
    rf_whitening = 0;
    Serial.println(F("RFM Select pin LOW, ESP32 Taking control of RFM69"));
  } else {
    // Serial.println(F("RFM Select pin HIGH"));
  }
  
  if (rf_whitening)
  {
    #ifdef DEBUG
      Serial.print(F("RFM69CW only"));
      Serial.print(F(" Node: ")); Serial.print(nodeID);
      if (ext_enable) {
        Serial.print(F(" NodeExt: ")); Serial.print(nodeIDExt);
      }
      Serial.print(" Freq: ");
      if (RF_freq == RF12_433MHZ) Serial.print(F("433MHz"));
      if (RF_freq == RF12_868MHZ) Serial.print(F("868MHz"));
      if (RF_freq == RF12_915MHZ) Serial.print(F("915MHz"));
      Serial.print(F(" Group: ")); Serial.println(networkGroup);
      Serial.println(" ");
    #endif
  }
  
  // Read status of USA calibration DIP switch----------------------------------------------
  if (digitalRead(DIP_switch2)==LOW) {
      USA=true;                            // IF DIP switch 2 is switched on then activate USA mode
      Serial.print(F("USA Vcal active: ")); Serial.println(vCal_USA);
  }
  
  // ---------------------------------------------------------------------------------------
   readConfigInput();

  if (rf_whitening)
  {
    rfm_init(RF_freq);                                                    // initialize RFM
    for (int i=10; i>=0; i--)                                             // Send RF test sequence (for factory testing)
    {
      emontx.P[0]=i;
      PayloadTX tmp = emontx;
      if (rf_whitening == 2)
      {
          byte WHITENING = 0x55;
          for (byte i = 0, *p = (byte *)&tmp; i < sizeof tmp; i++, p++)
              *p ^= (byte)WHITENING;
      }
      rfm_send((byte *)&tmp, sizeof(tmp), FACTORYTESTGROUP, nodeID, busyThreshold, busyTimeout);
      delay(100);
    }
    emontx.P[0]=0;
  }
  
  // ---------------------------------------------------------------------------------------
  
  digitalWrite(LEDpin,LOW);

  // ----------------------------------------------------------------------------
  // EmonLibCM config
  // ----------------------------------------------------------------------------
  // 12 bit ADC = 4096 divisions
  // Time in microseconds for one ADC conversion: 40 us 
  EmonLibCM_setADC(12,29.5);

  // Using AVR-DB 1.024V internal voltage reference
  EmonLibCM_ADCCal(1.024);
  
  EmonLibCM_SetADC_VChannel(0, vCal);                      // ADC Input channel, voltage calibration
  if (USA) EmonLibCM_SetADC_VChannel(0, vCal_USA);

  for (int i = 0; i < 6; i++) {
    EmonLibCM_SetADC_IChannel(adcChannels[i], iCal[i], iLead[i]);  // ADC Input channel, current calibration, phase calibration
  }

  if (ext_enable) {
    for (int i = 0; i < 6; i++) {
      EmonLibCM_SetADC_IChannel(adcChannelsExt[i], iCalExt[i], iLeadExt[i]); 
    }
  }

  // mains frequency 50Hz
  if (USA) EmonLibCM_cycles_per_second(60);                // mains frequency 60Hz
  EmonLibCM_datalog_period(period);                        // period of readings in seconds - normal value for emoncms.org  

  EmonLibCM_setPulseEnable(0, pulse_enable);                  // Enable pulse counting
  EmonLibCM_setPulseEnable(1, pulse_enable); 
//  EmonLibCM_setPulsePin(PIN_PA6, PIN_PA6); <-- Breaks when you actually supply a propper channelbecause someone did a dumb. (channel != 0 || channel != 1) should be (channel != 0 && channel != 1)
  EmonLibCM_setPulsePin(0, PIN_PA6, digitalPinToInterrupt(PIN_PA6)); //<-- Need to use the method that takes an interrupt because it does not do the "(channel != 0 || channel != 1)" check
  EmonLibCM_setPulsePin(1, PIN_PA7, digitalPinToInterrupt(PIN_PA7));
  EmonLibCM_setPulseMinPeriod(0, pulse_period);
  EmonLibCM_setPulseMinPeriod(1, pulse_period);

  EmonLibCM_setTemperatureDataPin(PIN_PB4);                      // OneWire data pin (emonTx V3.4)
  EmonLibCM_setTemperaturePowerPin(PIN_PB3);                    // Temperature sensor Power Pin - 19 for emonTx V3.4  (-1 = Not used. No sensors, or sensor are permanently powered.)
  EmonLibCM_setTemperatureResolution(11);                  // Resolution in bits, allowed values 9 - 12. 11-bit resolution, reads to 0.125 degC
  EmonLibCM_setTemperatureAddresses(allAddresses);         // Name of array of temperature sensors
  EmonLibCM_setTemperatureArray(allTemps);                 // Name of array to receive temperature measurements
  EmonLibCM_setTemperatureMaxCount(3);                     // Max number of sensors, limited by wiring and array size.
  
  EmonLibCM_TemperatureEnable(temp_enable);  
  EmonLibCM_Init();                                        // Start continuous monitoring.
  printTemperatureSensorAddresses();
  emontx.Msg = 0;
}

void loop()             
{
  static double ENERGY[] = {0,0,0,0,0,0,0,0,0,0,0,0};   // Sketch's own value to use when a.c. fails.  
  getCalibration();
  
  if (EmonLibCM_Ready())   
  {
    #ifdef DEBUG
    if (emontx.Msg==0) {
      Serial.println(EmonLibCM_acPresent()?F("AC present "):F("AC missing "));
      delay(5);
    }
    #endif

    emontx.Msg++;

    // Other options calculated by EmonLibCM
    // RMS Current:    EmonLibCM_getIrms(ch)
    // Apparent Power: EmonLibCM_getApparentPower(ch)
    // Power Factor:   EmonLibCM_getPF(ch)
    

    if (EmonLibCM_acPresent())
    {
      for (int i = 0; i < 6; i++) {
        emontx.P[i] = EmonLibCM_getRealPower(i);
        emontx.E[i] = ENERGY[i]= EmonLibCM_getWattHour(i);
      }
      
      if (ext_enable) {
        for (int i = 0; i < 6; i++) {
          emontxext.P[i] = EmonLibCM_getRealPower(6+i);
          emontxext.E[i] = ENERGY[6+i]= EmonLibCM_getWattHour(6+i);        
        }
      }
    }
    else
    {
      for (int i = 0; i < 6; i++) {
        double irms = EmonLibCM_getIrms(i);
        emontx.P[i] = assumedVrms2 * irms;
        ENERGY[i] += assumedVrms2 * irms * EmonLibCM_getDatalog_period()/3600.0;
        emontx.E[i] = ENERGY[i] + 0.5;                                // rounded value        
      }

      if (ext_enable) {
        for (int i = 0; i < 6; i++) {
          double irms = EmonLibCM_getIrms(6+i);
          emontxext.P[i] = assumedVrms2 * irms;
          ENERGY[6+i] += assumedVrms2 * irms * EmonLibCM_getDatalog_period()/3600.0;
          emontxext.E[i] = ENERGY[6+i] + 0.5;        
        }
      }
    }
    
    emontx.Vrms = EmonLibCM_getVrms() * 100;
    
    emontx.T1 = allTemps[0];
    emontx.T2 = allTemps[1];
    emontx.T3 = allTemps[2];

    emontx.pulse1 = EmonLibCM_getPulseCount(0);
    emontx.pulse2 = EmonLibCM_getPulseCount(1);
    
    if (rf_whitening)
    {
      PayloadTX tmp = emontx;
      if (rf_whitening == 2)
      {
          byte WHITENING = 0x55;
          for (byte i = 0, *p = (byte *)&tmp; i < sizeof tmp; i++, p++)
              *p ^= (byte)WHITENING;
      }
      rfm_send((byte *)&tmp, sizeof(tmp), networkGroup, nodeID, busyThreshold, busyTimeout);     //send data
      delay(50);
      
      if (ext_enable) {
        PayloadTXExt tmpext = emontxext;
        if (rf_whitening == 2)
        {
          byte WHITENING = 0x55;
          for (byte i = 0, *p = (byte *)&tmpext; i < sizeof tmpext; i++, p++)
              *p ^= (byte)WHITENING;
        }
        rfm_send((byte *)&tmpext, sizeof(tmpext), networkGroup, nodeIDExt, busyThreshold, busyTimeout);     //send extension board data
      }
      
      delay(50);
    }

    // ---------------------------------------------------------------------
    // Key:Value format, used by EmonESP & emonhub EmonHubTx3eInterfacer
    // ---------------------------------------------------------------------
    Serial.print(F("MSG:")); Serial.print(emontx.Msg);
    Serial.print(F(",Vrms:")); Serial.print(emontx.Vrms*0.01);

    for (int i = 0; i < 6; i++) {
      Serial.print(",P" + String(i+1) + ":"); Serial.print(emontx.P[i]);
    }
    
    if (ext_enable) {
      for (int i = 0; i < 6; i++) {
        Serial.print(",P" + String(i+7) + ":"); Serial.print(emontxext.P[i]);
      }
    }

    for (int i = 0; i < 6; i++) {
      Serial.print(",E" + String(i+1) + ":"); Serial.print(emontx.E[i]);
    }
    
    if (ext_enable) {
      for (int i = 0; i < 6; i++) {
        Serial.print(",E" + String(i+7) + ":"); Serial.print(emontxext.E[i]);
      }
    }
    
    if (emontx.T1!=30000) { Serial.print(F(",T1:")); Serial.print(emontx.T1*0.01); }
    if (emontx.T2!=30000) { Serial.print(F(",T2:")); Serial.print(emontx.T2*0.01); }
    if (emontx.T3!=30000) { Serial.print(F(",T3:")); Serial.print(emontx.T3*0.01); }

    Serial.print(F(",pulse1:")); Serial.print(emontx.pulse1); 
    Serial.print(F(",pulse2:")); Serial.print(emontx.pulse2);  
    delay(20);

    digitalWrite(LEDpin,HIGH); delay(50);digitalWrite(LEDpin,LOW);

    #ifdef SHOW_CAL
      // to show current & power factor for calibration:
      Serial.println();
      
      for (int i = 0; i < 6; i++) {
        Serial.print(" I" + String(i+1) + ":"); Serial.print(EmonLibCM_getIrms(i),3); Serial.print(", ");
      }
    
      if (ext_enable) {
        for (int i = 0; i < 6; i++) {
          Serial.print(" I" + String(i+7) + ":"); Serial.print(EmonLibCM_getIrms(6+i),3); Serial.print(", ");
        }
      }

      Serial.println();

      for (int i = 0; i < 6; i++) {
        Serial.print("pf" + String(i+1) + ":"); Serial.print(EmonLibCM_getPF(i),4); Serial.print(",");
      }
    
      if (ext_enable) {
        for (int i = 0; i < 6; i++) {
          Serial.print("pf" + String(i+7) + ":"); Serial.print(EmonLibCM_getPF(6+i),4); Serial.print(",");
        }
      }

      Serial.println();
    #endif
    Serial.println();
    
    // End of print out ----------------------------------------------------
  }
  //wdt_reset();
  delay(20);
}
