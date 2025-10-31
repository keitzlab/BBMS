/*
  Bunsen Burner Monitoring System
  Desktop display node
  Connect to monitor hub(server) via UDP and receive the sensor status for display.
  Functions as a sensor to the server.
  Vesrion: 2.6
  Last update date: Oct 30, 2025

  License: MIT
  Copyright (c) 2025 The University of Texas at Austin
  Developed by Yang Gao (McKetta Department of Chemical Engineering)
*/
#include <SPI.h>
#include "epd2in7_V2.h"  // eink 2in7, Waveshare
#include "imagedata.h"  // eink 2in7, Waveshare
#include "epdpaint.h" // eink 2in7, Waveshare
#include <WiFi.h>   // Wifi
#include <EEPROM.h> // 512 bytes, each byte 0-255
#include <NTP.h>  // Network time protocol
////// IO
#define AnaMax     1023   // analog input max, for battery voltage sensing
#define AnaMin     0    // analog input min, for battery voltage sensing
#define LEDOutPin 5  // LED control, output, DO5
#define key1Pin 4  // key 1 on eink display, input, pin 4
#define key2Pin 3  // key 1 on eink display, input, pin 3
#define key3Pin 2  // key 1 on eink display, input, pin 2
#define BatAnaPin A7    // battery positive terminal, analog inputpin
#define DC_AnaRawPin A6    // DC power positive terminal, analog input pin
////// eink screen
#define COLORED     0
#define UNCOLORED   1
#define ScrWdh      176   // pixel, eink screen width
#define ScrHgh 264    // pixel, eink screen height
#define HdrPaintHgh 16    // pixel, height of the header section
#define PriPaintHgh 218   // pixel, height of the primary section
#define BtnPaintHgh 30    // pixel, height of the button legend section
//#define FulImageByte (ScrHgh * ScrWdh / 8) // byte, header image buffer size
#define HdrImageByte (HdrPaintHgh * ScrWdh / 8) // byte, header image buffer size
#define PriImageByte (PriPaintHgh * ScrWdh / 8) // byte, primary area image buffer size
#define BtnImageByte (BtnPaintHgh * ScrWdh / 8) // byte, button legend image buffer size
////// sensor (eink unit) control
#define MaxSenNum 32   // maximum eink unit number
#define MonOnLineCntMax 32 // monitor online count down, decrease by 1 each program cycle.
#define MonUDPCntMax 3 // number of 
////// EEPROM
#define ServerIPEEPROM_adr 33   // server IP EEPROM indexing, index 49 -> EEPROM adr 192, 193 - 195, index 1-16-> 16sensor, index 17-32-> empty,  index 33-48-> 16 eink displays, index 49-> server (monitor)
#define SenStaEEPROM_A_byte_adr 132   // sensor status byte A address in EEPROM, sensor 1 to 8, 132=33*4, 4 bytes for one IP, 33 IPs stored before SenSta bytes, EEPROM address starts with 0
#define SenStaEEPROM_B_byte_adr 133   // sensor status byte B address in EEPROM, sensor 9 to 16
#define SenStaEEPROM_C_byte_adr 134   // sensor status byte C address in EEPROM, eink unit 1 to 8, 
#define SenStaEEPROM_D_byte_adr 135   // sensor status byte D address in EEPROM, eink unit 9 to 16
////// Adjustable variables
const char CodeVersion[] = "v2.6";
const char CoreVersion[] = "0.4.1"; // Ardunio core firmware version
const int EinkNum = 8; // display number
const char ssid[] = "ExampleSSID";             // Sensor network SSID (name)
const char pass[] = "ExamplePASS";               // Sensor network password (use for WPA, or use as key for WEP)
IPAddress serverIP(192,0,2,0);  // server IP, replace with your monitor hub assigned IP
const unsigned int UDPPort = 50505;         // local port to listen on, UDP data
float BatFit_slope = 0.001030;  // sloop from ADC to battery voltage fitting, 0.001436 0.0007654
float BatFitoffset = 1.146;    // y intercept from ADC to battery voltage fitting, 1.213 1.945
float BatMaxVol = 4.10; // maximum battery voltage, 4.1847
float BatMinVol = 3.55; // minimum battery voltage, 3.5295
float BatDCOffSet=0.024;  // 0.076 V, voltage offset when DC 5V is connected. With DC 5V, measured battery voltage would jump up ~0.1V
char SenNam[MaxSenNum/2][3] = {"ZY", "IM", "BT", "Ar", "AH", "YG", "MR","","","","","","","","",""}; // sensor names, starting from senor 1
////// main loop and global task management
bool IniPer = false;    // initial period, true -> program just started, will set to false after the delay
bool IniPerDly = false; // initial period delay. true -> within the delay  period at the start of the program start
bool Flg250ms = 0;   // toggle flag for 250 ms functions
bool Flg500ms = 0;   // toggle flag for 500 ms functions
bool Flg1s = 0;   // toggle flag for 1 s functions
bool Flg2s = 0;   // toggle flag for 1 s functions
int TimCycCnt = 0; // time slot cycle counter
unsigned long CurTim = 0;  // ms, current time
unsigned long PreTim125ms = 0;  // previous time counter, 125 ms
unsigned long starttime =0; 
unsigned long prestarttime=0;
unsigned long endtime = 0;
unsigned long TimMilRecStt  = 0;    // time in ms, Wifi reconnection start time
const unsigned long TimMilRecInt  = 60000;    // time in ms, Wifi reconnection interval
const unsigned long CstTimInt125ms = 125; // ms, time managment 
const unsigned long CstTimInt250ms = 250; // ms, time managment 
const unsigned long CstTimInt500ms = 500; // ms, time managment
const unsigned long CstTimInt25s = 25000; // ms, time managment  
const unsigned long CstTimInt1min = 60000; // ms, time managment  

////// battery monitor
bool DC_Chg = 0;  // 1-> 5V DC charger in
uint8_t BatChgPer = 0; // battery charge percentage
int BatAnaRaw;  // analog read raw value, battery voltage
int DC_AnaRaw;  // analog read raw value, DC input voltage

////// e-ink
bool DplRef = 0; // display refresh, caused by page change or , 1 -> refresh the whole screen
bool DplHdrRef = 0; // display header refresh, 1 -> refresh
bool DplPriRef = 0; // display primary area refresh, for sensor status or device info1->refresh
bool DplBtnRef = 0; // display button legend refresh, 1 -> refresh
bool MonOnLine = false; // monitor is online 1->yes, 0-> no.
int MonOnLineCnt =0;    // monitor online counter, reduce by 1 every 1s,
int PagInd = 0;   // page index, 0-> overview, 1 and 2 -> page with name
unsigned char HdrImage[HdrImageByte];
Paint HdrPaint(HdrImage, ScrWdh, HdrPaintHgh);    //width should be the multiple of 8
unsigned char PriImage[PriImageByte];
Paint PriPaint(PriImage, ScrWdh, PriPaintHgh);    //width should be the multiple of 8
unsigned char BtnImage[BtnImageByte];
Paint BtnPaint(BtnImage, ScrWdh, BtnPaintHgh);    //width should be the multiple of 8
//unsigned char FulImage[FulImageByte];
//Paint FulPaint(FulImage, ScrWdh, ScrHgh);    //width should be the multiple of 8
Epd epd;

////// sensor (eink unit) control
bool Alm_On=false;      // alarm on flag
bool LEDstate = 0;
bool VlvSta[MaxSenNum/2];              // bool arry for valve position status, received from UDP data, 1-> vavle open
bool SenSta[MaxSenNum/2];              // bool arry for sensor alive status, received from UDP data, 1-> sensor alive
bool AlmTimOut[MaxSenNum/2];          // bool array for sensor's alarm on status,
///// Wifi service 
//bool SenIP_Chg[MaxSenNum];              // bool arry for sensor IP change, received from UDP data, 1-> sensor IP changed
//bool SenStaEEPROM_A[8];              // bool arry for sensor alive status, sensor 1 to 8
//bool SenStaEEPROM_B[8];              // bool arry for sensor alive status, sensor 9 to 16
//bool SenStaEERROM[MaxSenNum];              // bool arry for sensor alive status, read from EEPROM, 1-> sensor alive
//uint8_t SenStaEEPROM_A_byte;        // byte for sensor alive status, sensor 1 to 8
//uint8_t SenStaEEPROM_B_byte;        // byte for sensor alive status, sensor 9 to 16
char packetBuffer[64];                 //buffer to hold incoming packet
int status = WL_IDLE_STATUS;
//int UDPRcvDat = 0;                    // received message from UDP
int SenSta_CD[MaxSenNum/2];           // int array for sensor status cool down counter, set to 3 upon new UDP data receiving, decrease by time, when 0 set the sensor status flag to false
int TotSen=0;           // total number of connected devices
int TotSenEEPROM=0;     // total number of connected devices, calculated from EEPROM data
const int SerAck = 9; // code for server received the status message
const int SerIPChgCode = 8; // code for server IP change event, update new server IP address if recevived this code
IPAddress serverIPEEPROM;                // Broadcast IP address
WiFiUDP Udp;
WiFiUDP NTPUdp;
////// NTP
NTP timeClient(NTPUdp);
char DoW[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"}; // days of the week
////// initialization
void setup() {

  // note the start time
  IniPer = true;
  starttime=millis();
  // serial port
  Serial.begin(9600);
  // IO
  pinMode(LEDOutPin,OUTPUT); // key 1 on eink
  pinMode(key1Pin,INPUT_PULLUP); // key 1 on eink
  pinMode(key2Pin,INPUT_PULLUP); // key 2 on eink
  pinMode(key3Pin,INPUT_PULLUP); // key 3 on eink
  ///analogWrite(Alm_OnPin,AnaMax);
  //analogWrite(AlmOffPin,AnaMin);
  //attachInterrupt(digitalPinToInterrupt(key4Pin), Key4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(key1Pin), Key1, RISING);
  attachInterrupt(digitalPinToInterrupt(key2Pin), Key2, RISING);
  attachInterrupt(digitalPinToInterrupt(key3Pin), Key3, RISING);

  // e-ink display
  char lnchar[16];   // line string
  // set line change flags to true
  DplRef=1; 
  Serial.print("e-Paper init\r\n");
  if (epd.Init() != 0) {
    Serial.print("e-Paper init failed\r\n");
    return;
  }
  // clear and initialize the frame
  //// header area
  epd.Clear();
  epd.Display_Base_color(0xff);
  HdrPaint.Clear(UNCOLORED);
  HdrPaint.DrawStringAt(0, 0, "Initializing", &Font16, COLORED);
  // battery symbol
  HdrPaint.DrawFilledRectangle(140, 0, 176, 15, COLORED);
  HdrPaint.DrawFilledRectangle(140, 0, 142, 3, UNCOLORED);
  HdrPaint.DrawFilledRectangle(140, 12, 142, 15, UNCOLORED);
  //batery percentage
  snprintf(lnchar, sizeof(lnchar),"%u%%",BatChgPer);
  HdrPaint.DrawStringAt(144, 2, lnchar, &Font16, UNCOLORED);
  epd.Display_Partial_Not_refresh(HdrPaint.GetImage(), 0, 0,ScrWdh, HdrPaintHgh);  // send header line

  //// primary display area
  // title line
  PriPaint.Clear(UNCOLORED);
  PriPaint.DrawStringAt(0, 0, "  BB Status", &Font20, COLORED);

  // upper divider line
  PriPaint.DrawFilledRectangle(0, 18, ScrWdh, 20, COLORED);

  // line 1-2, SSID
  PriPaint.DrawStringAt(0, 24, "Connecting to", &Font16, COLORED);
  PriPaint.DrawStringAt(0, 24+16, ssid, &Font16, COLORED);

  // line 3-4, received signal strength indicator for target network
  int rssi=0; // signal strength
  int n = WiFi.scanNetworks();  // number of networks
  for (int i = 0; i < n; ++i) {
    if (WiFi.SSID(i) == ssid) { // scan for the target SSID
      rssi = WiFi.RSSI(i);  // get the target network RSSI
      Serial.print(ssid);
      Serial.print(" RSSI: ");
      Serial.print(rssi);
      Serial.println(" dBm");
      break;  // Stop scanning once found
    }
  }
  char buf[10];
  itoa(rssi, buf, 10);  // 10 = base 10
  PriPaint.DrawStringAt(0, 24+1*38, "RSSI (dBm):", &Font16, COLORED);
  PriPaint.DrawStringAt(0, 24+1*38+16, buf, &Font16, COLORED);

  // line 5-6, firmware version
  PriPaint.DrawStringAt(0, 24+2*38, "*Core ver.", &Font16, COLORED);
  PriPaint.DrawStringAt(0, 24+2*38+16, CoreVersion, &Font16, COLORED);

  // line 7-8 code version
  PriPaint.DrawStringAt(0, 24+3*38, "*Code ver.", &Font16, COLORED);
  PriPaint.DrawStringAt(0, 24+3*38+16, CodeVersion, &Font16, COLORED);

  // line 9 device number 
  snprintf(lnchar, sizeof(lnchar), "*Device #%d", EinkNum);
  PriPaint.DrawStringAt(0, 24+4*38, lnchar, &Font16, COLORED);
  epd.Display_Partial_Not_refresh(PriPaint.GetImage(), 0, HdrPaintHgh,ScrWdh, HdrPaintHgh+PriPaintHgh);  // send the primary display section

  //// button legend area
  BtnPaint.Clear(UNCOLORED); //  lower divider line
  BtnPaint.DrawFilledRectangle(0, 0, 176, 2, COLORED);
  BtnPaint.DrawStringAt(0, 3, "Next|Ref. |Back", &Font16, COLORED);
  BtnPaint.DrawStringAt(0, 15, "Page|Scrn.|Light", &Font16, COLORED);
  epd.Display_Partial(BtnPaint.GetImage(), 0, HdrPaintHgh+PriPaintHgh, ScrWdh, HdrPaintHgh+PriPaintHgh+BtnPaintHgh);  // send the button legend section

  // WiFi
  // get mac address
  Serial.print("MAC Address: ");
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print(mac[0],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.println(mac[5],HEX);
  // attempt to connect to WiFi network:
  if(status != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    //delay(10000);
  }
  Serial.println("Connected to WiFi");
  printWiFiStatus();
  // start UDP
  // if you get a connection, report back via serial:
  Udp.begin(UDPPort);

  // EEPROM ini
  if (!EEPROM.begin(512))
  {
    Serial.println("failed to initialise EEPROM"); delay(10000);
  }
  // read the saved server IP, EEPROM
  serverIPEEPROM=readIPEEPROM(ServerIPEEPROM_adr);   // read saved device IP
  // send the IPs to serial port
  Serial.print("serverIPEEPROM: ");
  Serial.println(serverIPEEPROM);

  // NTP
  timeClient.ruleDST("CT", Second, Sun, Mar, 0, -300); // start of DST: second sunday in March 0:00, timetone -300min (GMT-5, or GMT-6 + 1h summertime offset)
  timeClient.ruleSTD("CT", First, Sun, Nov, 0, -360); // end of DST: first sunday in November 0:00, timezone -360min (GMT-6)
  timeClient.begin();

  //timeClient.dstOffset(-18000);
  //timeClient.setTimeOffset(-18000);
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT -5 = -18000
  // GMT -6 = -21600
  // GMT 0 = 0
    
}

void loop() {
  //Time management
  CurTim = millis();  // get current time
  // program inital start flags
  if(IniPer){
    unsigned long tmptim = CurTim-starttime;
    IniPer=tmptim<CstTimInt1min;  // true -> within 10 min of program start
    IniPerDly=tmptim<CstTimInt25s;  // true-> within 25s of program start
  }
  if(CurTim-PreTim125ms>=CstTimInt125ms){
    PreTim125ms=CurTim;  // reset time slot cycle cunter, 100 ms
    // time slot control
    if(TimCycCnt>=1&&TimCycCnt<16){
      //TimCycCnt between 1 to 7
      TimCycCnt=TimCycCnt+1;  // increae counter
    }
    else{
      TimCycCnt=1;    // reset counter
    }
    // call functions according to the time slot cycle counter
    if(TimCycCnt%2==1){
      Fnc250ms();  // call 250 ms functions
      if(TimCycCnt==3){
        Fnc2sA(); // call the 1st 2 s functions
      }
      if(TimCycCnt==11){
        Fnc2sB(); // call the 1st 2 s functions
      }
    }
    else if(TimCycCnt==2||TimCycCnt==6||TimCycCnt==10||TimCycCnt==14){
      Fnc500ms();  // call 500 ms functions
    }
    else if(TimCycCnt==4||TimCycCnt==12){
      Fnc1sA();  // call the first 1 s functions
    }
    else if(TimCycCnt==8||TimCycCnt==16){
      Fnc1sB();  // call the 2nd 1 s functions
    }
  }  
}

// functions
// 100 ms interval functions
void Fnc250ms(){
  Flg250ms = !Flg250ms;
  UDP_Read();   // read UDP package
}

// 250 ms interval functions
void Fnc500ms(){
  Flg500ms = !Flg500ms;
  AlarmControl();
  Outputs();
  
}

// 1st 1 s interval functions
void Fnc1sA(){
  Flg1s = !Flg1s;
  //UDP_Client(); // wifi control, sending udp

}

// 2nd 1 s interval functions
void Fnc1sB(){
  MonitorCheck(); // monitor online and IP address saving
  Eink();
} 

// 1st 2 s interval functions
void Fnc2sA(){
  Flg2s=!Flg2s;
  Inputs();   // ADC read battery and DC voltage
  BatteryStatus();
  UDP_Client(); // wifi control, sending udp
}

// 2nd 2 s interval functions
void Fnc2sB(){
  NTP();
  //MonitorCheck(); // monitor online and IP address saving
}

void Inputs(){
  BatAnaRaw = analogRead(BatAnaPin);
  DC_AnaRaw = analogRead(DC_AnaRawPin);
}

// key 1 event
void Key1(){
  static unsigned long lastpresstime=0;
  if(CurTim-CstTimInt250ms>lastpresstime){  // avoid multiple key event with one press
    PagInd++; // page index change
    DplRef=true; // set the screen refresh flag
    Serial.println("Dpl Ref by page change");
    if(PagInd>1){
      PagInd=0; // reset to 0
    }
    lastpresstime=CurTim; // save the key event time
  }
}
// key 2 event
void Key2(){
  DplRef=true;
  // tmp
  Serial.println("Dly Ref. input key");

}
// key 3 event
void Key3(){
  LEDstate = !LEDstate;
}

// alarm control, LED light warning
void AlarmControl(){
  static bool alm_onold = false;  // previous alarm state
  static bool ledstateold = false;  // previous led state
  //Alm_On=true;
  if(alm_onold!=Alm_On){  // alarm state just changed
    if(Alm_On){ // alarm just on
      ledstateold=LEDstate; // save the current led state
    }
    else{ // alarm just off
      LEDstate=ledstateold; // set the LED state to that before alarm on
    }
    alm_onold=Alm_On;  // save the alarm state
  }
  else if(Alm_On){
    LEDstate = !LEDstate; // toggle LED state
  }
}
// output control, LED, buzzer
void Outputs(){
  // LED light output
  digitalWrite(LEDOutPin, LEDstate);
}

void UDP_Client(){
  // get the current WiFi status
  static int statusold = WL_IDLE_STATUS;  // for saving the previous WiFi status
  status=WiFi.status();
  if(statusold!=status){  // WiFi status changed 
    DplRef = 1;  // refresh the whole screen
    // tmp
    Serial.println("Dly Ref. wifi status change");
    statusold=status;   // save the current WiFi status
  }
  // reconnect to WiFi if connection is lost
  if(status != WL_CONNECTED){
    if(CurTim-TimMilRecStt>TimMilRecInt){    
      status = WiFi.begin(ssid, pass); // initiate WiFi reconnection at TimMilRecInt interval
      Serial.println("Reconnecting to WiFi");
      TimMilRecStt=CurTim;    // record reconnection start time
    }
  }
  else{ // send status via UDP
    // send a message, to the IP address and port that sent us the packet we received
    Udp.beginPacket(serverIP, UDPPort);
    uint32_t sendmessage=0;
    //sendmessage=EinkNum*10;  // assemble the message, 10 to 161 -> sensor 1 to 16, 330 to 480 display 1 to 16 
    sendmessage=EinkNum*10+(MaxSenNum/2)*10;  // assemble the message, 10 to 161 -> sensor 1 to 16, 330 to 480 display 1 to 16 
    //itoa(sendmessage, packetBuffer, 10);
    //uint8_t buffer = 30;
    //Udp.write((const uint8_t*)packetBuffer, strlen(packetBuffer));
    Udp.write((uint8_t*)&sendmessage,sizeof(sendmessage));
    Udp.endPacket();
  }
}
// UDP read
void UDP_Read(){
  static uint32_t sensorstatusold = 0;          // previous received message from UDP
  static uint32_t valvestatusold = 0;          // previous received message from UDP
  static uint32_t timeoutstatusold = 0;          // previous received message from UDP
  int UDPRcvDat=0;
  // get WiFi status
  status = WiFi.status();
  if (status == WL_CONNECTED) {
    // only start UDP listening if connected to WiFi
    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      // only read the package and proceed if the data is from UDPPort(14890)
      if(Udp.remotePort()==UDPPort){
        MonOnLineCnt=MonOnLineCntMax;   // reset monitor online counter
        // read the packet into packetBuffer
        uint32_t sensorstatus = 0;
        uint32_t valvestatus = 0;
        uint32_t timeoutstatus = 0;
        int len = Udp.read(packetBuffer, 64);
        // check data szie, 12 bytes-> 3x4uint32_t. full data from monitor: sensor, valve and timeout status
        if (len == 12) {
          memcpy(&sensorstatus,  packetBuffer, 4);
          memcpy(&valvestatus,  packetBuffer+4, 4);
          memcpy(&timeoutstatus,  packetBuffer+8, 4);
          // check if sensor status changed
          if(sensorstatusold!=sensorstatus||valvestatusold!=valvestatus||timeoutstatusold!=timeoutstatus){  // sensor status changed
            DplPriRef = 1;  //  set the primary display refresh flag for the sensor status
            // tmp
            Serial.println("Dly Ref. sensor status change");

            sensorstatusold=sensorstatus; // save the current UDP data
            valvestatusold=valvestatus;
            timeoutstatusold=timeoutstatus;
          }
          uint32_t2array(sensorstatus,SenSta);
          uint32_t2array(valvestatus,VlvSta);
          uint32_t2array(timeoutstatus,AlmTimOut);
          // set the alarm on flag for LED control
          if(timeoutstatus!=0){ // if any sensor is in timeout state
            Alm_On=true;
          }
          else{
            Alm_On=false;
          }
        }
        else{// single commend
          UDPRcvDat=atoi(packetBuffer); // received UDP message
        }

        // interpret received message
        if(UDPRcvDat==SerIPChgCode){// server IP address change
          //update new server IP
          //compare the saved value to the remote IP, aviode redondant update to EEPROM
          if(serverIP!=Udp.remoteIP()){
            serverIP=Udp.remoteIP();
            writeIPEEPROM(serverIP,ServerIPEEPROM_adr);
            serverIPEEPROM=serverIP;
            Serial.print("new IP address from server, EEPROM updated: ");
            Serial.println(serverIP);
            serverIPEEPROM=readIPEEPROM(ServerIPEEPROM_adr);   // read saved device IP
            Serial.println(serverIPEEPROM);
          }
        }
      }
    }
  }
}
void MonitorCheck(){
  // monitor online detection
  static bool MonOnLineOld = 0; // for saving the previous monitor status
  if(MonOnLineCnt>=1){  // decrease the cool down counter for monitor online status
    MonOnLineCnt--;
    MonOnLine=true;
  }
  else{ // no new data from monitor during cool down, monitor is offline
    MonOnLineCnt=0;
    MonOnLine=false;
  }
  if(MonOnLineOld!=MonOnLine){  // monitor online status changed
    DplRef=1;    // refresh the whole screen
    // tmp
    Serial.println("Dly Ref. monitor off line");
    MonOnLineOld=MonOnLine; // save the current monitor status
    if(!MonOnLine){
      // clear the sensor and valve status flags
      for(int i=0;i<MaxSenNum/2;i++){
        SenSta[i]=0;
        VlvSta[i]=0;
      }
    }
  }
  // server IP address check at the start of program
  if(!IniPerDly&&IniPer){
    // server online, server IP in the code is correct
    if(MonOnLine){
      // check if stored server IP in EEPROM is the same as the one in code
      // if not store the server IP to EEPROM
      if(serverIP!=serverIPEEPROM){
        writeIPEEPROM(serverIP,ServerIPEEPROM_adr);
        serverIPEEPROM=serverIP;
        Serial.print("new IP address from code, EEPROM updated: ");
        Serial.println(serverIP);
      }
    }// server not online, and past the initial delay, use the server IP stored in EEPROM
    else if(!MonOnLine&&serverIP!=serverIPEEPROM){
      serverIP=serverIPEEPROM;
      Serial.print("new IP address from EEPROM: ");
      Serial.println(serverIP);
    }
  }
}
// e-ink control
// the refresh time/sequence dose not change with the size of the plot -> no need to use refreshpartial, just refresh the whole screen each time 
void Eink(){
  char lnchar[16];
  bool fastrefresh = 0; // display fast refresh, 1->refresh, transmit last image data, no content redrawing 
  // only use header and main area refresh flag, avoid unnecessary display refresh e.g. sensor status caused refresh when displaying device info.
  if(DplRef){  // whole screen refresh request, from wifi or monitor status change, and key events
    DplHdrRef=true; // set header refresh flag
    DplPriRef=true; // set primary area refresh flag
    DplBtnRef=true; // set button legend area refresh flag
    DplRef=false;// clear whole screen refresh flag
  }
  else if(DplPriRef&&PagInd==1){  // sensor status change while displaying device info
    DplPriRef=false;  // do not refresh screen
  }

  //// header line
  if(DplHdrRef){
    int xpos = 0;
    int ypos = 0;
    // wifi status
    //epd.Clear();
    HdrPaint.Clear(UNCOLORED);
    if(WiFi.status()==WL_CONNECTED){
      //strcpy(lnchar, "WiFi");
      HdrPaint.DrawFilledRectangle(1, 0, 15, 2, COLORED);  // largest arc
      HdrPaint.DrawFilledRectangle(4, 4, 12, 6, COLORED);  // medium arc
      HdrPaint.DrawFilledRectangle(7, 8, 9, 10, COLORED);  // smallest arc
      
    }
    else{
      strcpy(lnchar, "*");
      HdrPaint.DrawStringAt(0, 0, lnchar, &Font16, COLORED); // 
    }

    // data exchange symbol
    xpos = 18;
    ypos = 0;
    // monitor online, MonOnLine
    if(!MonOnLine){ // monitor offline, draw cross, 24
      HdrPaint.DrawLine(xpos, ypos+11, xpos+21, ypos+1,COLORED);
      HdrPaint.DrawLine(xpos, ypos+1, xpos+21, ypos+11,COLORED);
    }
    HdrPaint.DrawCharAt(xpos, ypos, '|', &Font16, COLORED);
    HdrPaint.DrawLine(xpos+5, ypos+12, xpos+1, ypos+8,COLORED);
    HdrPaint.DrawLine(xpos+6, ypos+12, xpos+10, ypos+8,COLORED);
    HdrPaint.DrawCharAt(xpos+8, ypos, '|', &Font16, COLORED);
    HdrPaint.DrawLine(xpos+13, ypos, xpos+9, ypos+4,COLORED);
    HdrPaint.DrawLine(xpos+14, ypos, xpos+18, ypos+4,COLORED);

    // date and time
    xpos = 42;
    ypos = 1;
    HdrPaint.DrawStringAt(xpos,ypos,DoW[timeClient.weekDay()], &Font16, COLORED);  // day of the week
    snprintf(lnchar, sizeof(lnchar), "%02d", timeClient.hours()); // get the hours to char, two digits
    HdrPaint.DrawStringAt(xpos+38, ypos, lnchar, &Font16, COLORED);
    HdrPaint.DrawCharAt(xpos+58, ypos, ':', &Font16, COLORED);
    snprintf(lnchar, sizeof(lnchar), "%02d", timeClient.minutes());  // get the mins to char, two digits
    HdrPaint.DrawStringAt(xpos+66, ypos, lnchar, &Font16, COLORED);
    
    
    // charging symbol
    if(DC_Chg){ // 5V DC in
      HdrPaint.DrawCharAt(130, 1, '+', &Font16, COLORED); 
    }
    else{ // 5V DC disconnected
      HdrPaint.DrawCharAt(130, 1, ' ', &Font16, COLORED); 
    }
    // battery symbol
    HdrPaint.DrawFilledRectangle(140, 0, 176, 15, COLORED);
    HdrPaint.DrawFilledRectangle(140, 0, 142, 3, UNCOLORED);
    HdrPaint.DrawFilledRectangle(140, 12, 142, 15, UNCOLORED);
    //batery percentage
    snprintf(lnchar, sizeof(lnchar),"%u%%",BatChgPer);
    HdrPaint.DrawStringAt(144, 2, lnchar, &Font16, UNCOLORED);
    
    // output the new display content
    // due to the fading of previous Display_Partial() if excuted sequentially, always refresh the whole screen
    // if no refresh needed for the primary display area, use fast refresh by not redrawing the contents
    epd.Display_Partial_Not_refresh(HdrPaint.GetImage(), 0, 0,ScrWdh, HdrPaintHgh);  // send line 1
    
    // fast refresh the display if only header is changed
    fastrefresh=!DplPriRef;

    // clear display header change flag
    DplHdrRef=false;
  }
  
  //// primary display area, for sensor status or device info
  if(fastrefresh){  // send out the content without redrawing
    epd.Display_Partial_Not_refresh(PriPaint.GetImage(), 0, HdrPaintHgh,ScrWdh, HdrPaintHgh+PriPaintHgh);  // send the primary display section
  }
  else if(DplPriRef){
    int yoffset = 2;
    int ystep = 24;
    int xcolumnoffset = 90;
    if(PagInd==0){  // overview page
      PriPaint.Clear(UNCOLORED);
      // title line
      PriPaint.DrawStringAt(0, 0, "  BB Status", &Font20, COLORED);
      // upper divider line  
      PriPaint.DrawFilledRectangle(0, 18, ScrWdh, 20, COLORED);
      // display contents depending on the WiFi connection
      if(WiFi.status()==WL_CONNECTED){  // wifi connected, show sensor status
        for(int i=0;i<MaxSenNum/2;i++){ // sener and eink display are saved to SenSta, 0 to 15 -> sensors, 16 to 31 -> einks
          // prepare contents according to SenSta and VlvSta
          int xcolumnind = (i+1)/9; // draw 16 sensors into two columns
          if(SenSta[i]){  // sensor i+1 is online
            // display the sensor name if there is one, otherwise display sensor number
            if(SenNam[i][0] == '\0'){ // name string empty, just display the number
              // display the senser number
              if(i<=8){ // single digit numbers
                snprintf(lnchar, sizeof(lnchar),"%d",i+1);  // sensor number
                PriPaint.DrawStringAt(xcolumnind*xcolumnoffset+8, ystep*(i%8+1), lnchar, &Font24, COLORED); // draw sensor number
              }
              else{ // double digit number
                int tmpint = (i+1)%10;  // get the last digit
                snprintf(lnchar, sizeof(lnchar),"%d",tmpint);  // 
                PriPaint.DrawCharAt(xcolumnind*xcolumnoffset, ystep*(i%8+1), '1', &Font24, COLORED); // draw tens digit
                PriPaint.DrawStringAt(xcolumnind*xcolumnoffset+12, ystep*(i%8+1), lnchar, &Font24, COLORED); // draw last digit
              }
            }
            else{// display the name string
              PriPaint.DrawStringAt(xcolumnind*xcolumnoffset+0, ystep*(i%8+1), SenNam[i], &Font20, COLORED); // draw sensor number
            }

            // display valve position 
            if(VlvSta[i]){  // sensor i+1 valve open
              // invert the color to highlight the status
              PriPaint.DrawFilledRectangle(xcolumnind*xcolumnoffset+30, ystep*(i%8+1), xcolumnind*xcolumnoffset+30+54, ystep*(i%8+1)+16, COLORED);
              if(AlmTimOut[i]){ // time out
                strcpy(lnchar, "TMO");
                PriPaint.DrawStringAt(xcolumnind*xcolumnoffset+30, yoffset+ystep*(i%8+1), lnchar, &Font16, UNCOLORED);
              }
              else{ // alarm count down
                strcpy(lnchar, "open");
                PriPaint.DrawStringAt(xcolumnind*xcolumnoffset+30, yoffset+ystep*(i%8+1), lnchar, &Font16, UNCOLORED);
              }
            }
            else{   // sensor i+1 valve closed
              strcpy(lnchar, "off");
              PriPaint.DrawStringAt(xcolumnind*xcolumnoffset+30, yoffset+ystep*(i%8+1), lnchar, &Font16, COLORED);
            }
          }
          else{   // sensor i+1 not online
            strcpy(lnchar, "   ");
            PriPaint.DrawStringAt(xcolumnind*xcolumnoffset,  ystep*(i%8+1), lnchar, &Font24, COLORED);
          }
        }
      }
      else{ // wifi not connected
        // continued from upper divider line
        // line 1-2, display SSID
        PriPaint.DrawStringAt(0, 24, "Connecting to", &Font16, COLORED);
        PriPaint.DrawStringAt(0, 24+16, ssid, &Font16, COLORED);
        // line 3-4, network signal strength:
        int rssi=0; // signal strength
        int n = WiFi.scanNetworks();  // number of networks
        for (int i = 0; i < n; ++i) {
          if (WiFi.SSID(i) == ssid) { // scan for the target SSID
            rssi = WiFi.RSSI(i);  // get the target network RSSI
            Serial.print(ssid);
            Serial.print(" RSSI: ");
            Serial.print(rssi);
            Serial.println(" dBm");
            break;  // Stop scanning once found
          }
        }
        char buf[10];
        itoa(rssi, buf, 10);  // 10 = base 10
        PriPaint.DrawStringAt(0, 24+1*38, "RSSI (dBm):", &Font16, COLORED);
        PriPaint.DrawStringAt(0, 24+1*38+16, buf, &Font16, COLORED);
        // line 5-6, local IP
        char ipStr[16];
        IPAddress ip = WiFi.localIP();
        sprintf(ipStr, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
        PriPaint.DrawStringAt(0, 24+2*38, "Local IP", &Font16, COLORED);
        PriPaint.DrawStringAt(0, 24+2*38+16, ipStr, &Font16, COLORED);
      }  
    }
    else if(PagInd==1){ // info page
      // title line
      PriPaint.Clear(UNCOLORED);
      PriPaint.DrawStringAt(0, 0, "Device Info.", &Font20, COLORED);
      // upper divider line  
      PriPaint.DrawFilledRectangle(0, 18, ScrWdh, 20, COLORED);
      // line 1-2, server IP
      char ipStr[16];
      sprintf(ipStr, "%u.%u.%u.%u", serverIP[0], serverIP[1], serverIP[2], serverIP[3]);
      PriPaint.DrawStringAt(0, 24, "*Monitor IP", &Font16, COLORED);
      PriPaint.DrawStringAt(0, 24+16, ipStr, &Font16, COLORED);
      
      // line 3-5,  Mac address
      char macStr[18];
      byte mac[6];
      WiFi.macAddress(mac);  // Fills the mac[] array
      //sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      sprintf(macStr, "%02X:%02X:%02X:",mac[0], mac[1], mac[2]);
      PriPaint.DrawStringAt(0, 24+1*38, "*Mac adr.", &Font16, COLORED);
      PriPaint.DrawStringAt(0, 24+1*38+16, macStr, &Font16, COLORED);
      sprintf(macStr, "%02X:%02X:%02X",mac[3], mac[4], mac[5]);
      PriPaint.DrawStringAt(0, 24+1*38+32, macStr, &Font16, COLORED);

      // line 6-7, firmware version
      PriPaint.DrawStringAt(0, 40+2*38, "*Core ver.", &Font16, COLORED);
      PriPaint.DrawStringAt(0, 40+2*38+16, CoreVersion, &Font16, COLORED);

      // line 8-9 code version
      PriPaint.DrawStringAt(0, 40+3*38, "*Code ver.", &Font16, COLORED);
      PriPaint.DrawStringAt(0, 40+3*38+16, CodeVersion, &Font16, COLORED);

      // line 11 device number 
      snprintf(lnchar, sizeof(lnchar), "*Device #%d", EinkNum);
      PriPaint.DrawStringAt(0, 40+4*38, lnchar, &Font16, COLORED);
    } 
    // output the new content
    epd.Display_Partial_Not_refresh(PriPaint.GetImage(), 0, HdrPaintHgh,ScrWdh, HdrPaintHgh+PriPaintHgh);  // send the primary display section
    // fast refresh display if no button legend refresh needed
    fastrefresh=!DplBtnRef;
    // clear display sensor status change refresh flag
    DplPriRef=false;
    //epd.Display(paint.GetImage());
  }

  //// button legend area
  if(fastrefresh){
    epd.Display_Partial(BtnPaint.GetImage(), 0, HdrPaintHgh+PriPaintHgh, ScrWdh, HdrPaintHgh+PriPaintHgh+BtnPaintHgh);  // send the button legend section
  }
  else if(DplBtnRef){
    BtnPaint.Clear(UNCOLORED); //  lower divider line
    BtnPaint.DrawFilledRectangle(0, 0, 176, 2, COLORED);
    BtnPaint.DrawStringAt(0, 3, "Next|Ref. |Back", &Font16, COLORED);
    BtnPaint.DrawStringAt(0, 15, "Page|Scrn.|Light", &Font16, COLORED);
    epd.Display_Partial(BtnPaint.GetImage(), 0, HdrPaintHgh+PriPaintHgh, ScrWdh, HdrPaintHgh+PriPaintHgh+BtnPaintHgh);  // send the button legend section
    DplBtnRef=false;  // clear button legend refresh flag
  }
}

// battery status
void BatteryStatus(){
  static bool DC_ChgOld = 0;  // for saving the previous 5V DC charger state
  static uint8_t BatChgPerOld=0;    // for saving the previous battery charge percentage
  static uint8_t batteryPerFluCnt=0;  // battery charge percentage fluctuation count

  //// calculate DC voltage
  float DCV = BatFit_slope*DC_AnaRaw + BatFitoffset; // battery voltage 
  // set the DC input flag
  if(DCV>4.6){ 
    DC_Chg=true;  // 5V in
  }
  else{
    DC_Chg=false;  // 5V disconnected
  }
  if(DC_ChgOld!=DC_Chg){    // change in 5V DC state
    DplHdrRef=1;    // set the display header refresh flag
    DC_ChgOld=DC_Chg; // save the current 5V DC charger state
    // tmp
    Serial.println("Dpl. Ref. 5V in change");
  }
  //// calculate the battery voltage and charge percental
  float batteryV = BatFit_slope*BatAnaRaw + BatFitoffset; // battery voltage 
  // subtract battery offest when DC 5V is connected
  if(DC_Chg){
    batteryV=batteryV-BatDCOffSet;  // subtract offset
  }
  static float batteryVfil=0;   // filtered battery voltage
  // filter the converted voltage as it fluctuates and cause the charge percentage to change over 6 percent between reads
  const float alpha = 0.04;  // smoothing facter, 0.2->20% of new value is added to the filtered results
  // by pass the filter when program just starts
  if(IniPer){
    batteryVfil=batteryV;
    BatChgPerOld=BatChgPer; // save the current battery charge percentage
  }
  else{ // after the initial period, filter the calculated battery voltages
    batteryVfil = alpha*batteryV+(1-alpha)*batteryVfil; 
  }
  
  // tmp
  /*
  Serial.print("voltage read: ");
  Serial.println(BatAnaRaw);
  Serial.print("voltage calculated: ");
  Serial.println(batteryV);
  Serial.print("voltage filtered: ");
  Serial.println(batteryVfil);*/
  
  if(batteryVfil>BatMinVol){ // make sure the battery voltage is larger then the minimum
    float batterychargepercentage=0;
    /*
    // offset the battery voltage if the 5V DC is on
    if(DC_Chg){ // if 5V DC connected
      batterychargepercentage=100*(batteryVfil-BatMinVol+BatDCOffSet)/(BatMaxVol-BatMinVol);
    }
    else{ // no 5V DC
      batterychargepercentage=100*(batteryVfil-BatMinVol)/(BatMaxVol-BatMinVol);
    }*/
    batterychargepercentage=100*(batteryVfil-BatMinVol)/(BatMaxVol-BatMinVol);
    // round the percentage to multiply of 5
    BatChgPer=(((uint8_t)(batterychargepercentage)+2)/5)*5;
    if(BatChgPer>=100){
      BatChgPer=100;  // limit the percentage to 100
    }
    batterychargepercentage=100*(batteryVfil-BatMinVol)/(BatMaxVol-BatMinVol);
  }
  else{// battery voltage lower than minimum, battery error or disconnected
    BatChgPer=0;
  }
  if(BatChgPerOld!=BatChgPer){    // change in battery charge percentage
    // avoid battery percentage to change back and forth when just changed
    // if battery is discharging, only allow the value to increase after repeated calling (e.g. battery changed)
    if(!DC_Chg&&BatChgPerOld<BatChgPer){
      batteryPerFluCnt++;
      if(batteryPerFluCnt>30){
        BatChgPerOld=BatChgPer; // save the current battery charge percentage
        DplHdrRef=1;    // set the display header refresh flag
        batteryPerFluCnt=1; // reset the battery percentage fluctuation counter
      }
    }
    else{ // charging or battery percentge dropping
      BatChgPerOld=BatChgPer; // save the current battery charge percentage
      DplHdrRef=1;    // set the display header refresh flag
      batteryPerFluCnt=1; // reset the battery percentage fluctuation counter
    }
    Serial.println("Dpl Ref by battery percentage");
  }
}

// NTP functions
void NTP(){
  static int8_t minold=0;
  // get updates
  timeClient.update();
  // set the header refresh flag if min changed
  if(minold!=timeClient.minutes()){
    minold=timeClient.minutes();
    DplHdrRef=true;
  }


  /*
  Serial.println(timeClient.month());
  Serial.println(timeClient.day());
  Serial.println(timeClient.hours());
  Serial.println(timeClient.minutes());
  Serial.println(timeClient.weekDay());
  */
  //Serial.println(timeClient.formattedTime("%d. %B %Y")); // dd. Mmm yyyy
  //Serial.println(timeClient.formattedTime("%A %T")); // Www hh:mm:ss
  /*
  Serial.print(timeClient.getDay());
  Serial.print(", ");
  Serial.print(timeClient.getHours());
  Serial.print(":");
  Serial.println(timeClient.getMinutes());
  */
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  int rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

}

// bool array and byte functions
// e.g. sensor byte 64 (b0100 0000, msb->lsb)
void byte2array(uint16_t byte, bool boolArray[8]) {
  for (int i = 0; i < 8; i++) {
    boolArray[i] = (byte >> i) & 0x01;  // Store bits in reverse order
  }
}
void uint32_t2array(uint32_t uintValue, bool boolArray[16]) {
  for (int i = 0; i < 16; i++) {
    boolArray[i] = (uintValue >> i) & 0x01;  // Store bits in reverse order
  }
}
// array 0100 0000 (msb->), byte 64, sensor 8->1
void array2byte(bool boolArray[8], uint8_t &byteValue) {
  byteValue = 0;  // Clear the byte before setting bits
  for (int i = 0; i < 8; i++) {
    if (boolArray[i]) {
      byteValue |= (1 << i);  // Set the i-th bit if boolArray[i] is true
    }
  }
}
// source: 0000 0010 0000 0001 (lsb->msb), target1: 0000 0010 (lsb->msb), target2: 0000 0001 (lsb->msb)
void splitintarraytoboolarray(uint32_t value, bool bitsLow[16], bool bitsHigh[16]) {
  for (int i = 0; i < 16; i++) {
    bitsLow[i]  = (value >> i) & 0x01;       // bits 0–15
    bitsHigh[i] = (value >> (16 + i)) & 0x01; // bits 16–31
  }
}

// EEPROM related functions, sensor 1-16->index 0-15
void writeIPEEPROM(IPAddress ip, int startAddressInd) {
  int startAddress = (startAddressInd-1)*4;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(startAddress + i, ip[i]);  // Save each byte of the IP
    EEPROM.commit();  // need this for ESP32
  }
}
//
IPAddress readIPEEPROM(int startAddressInd) {
  IPAddress ip;
  int startAddress = (startAddressInd-1)*4;
  for (int i = 0; i < 4; i++) {
    ip[i] = EEPROM.read(startAddress + i);  // Read each byte and assign to IPAddress
  }
  return ip;
}
