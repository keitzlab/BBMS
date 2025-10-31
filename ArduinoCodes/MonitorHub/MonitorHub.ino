/*
  Bunsen Burner Monitoring System
  Monitor hub
  UDP hub/server that validates package from sensor nodes, maintains liveness, drives signage (via relay), 
  serves a read-only web UI, and streams status to display units. 
  Vesrion: 2.6

  License: MIT
  Copyright (c) 2025 The University of Texas at Austin
  Developed by Yang Gao (McKetta Department of Chemical Engineering)
 */
#include "WiFiS3.h"
#include <U8g2lib.h>  // OLED control
#include <Wire.h>
#include <EEPROM.h> // 512 bytes, each byte 0-255
#include <cstring>
////// IO, LED flame sign
////// output pins
#define VlvOpnLEDPin 7    // Valve opne LED, in sensor box, red LED
#define VlvOpnRlyPin 6    // Valve opne LED flame sign, rely signal
////// sensor and eink unit status
#define MaxSenNum 32   // maximum sensing device number, 1-16 sensor, 17-32 eink units
#define SenSta_CDMax 8                 // sensor status cool down maximum value, decrease by 1 on each cycle
////// EEPROM
#define ServerIPEEPROM_adr 33   // server IP EEPROM indexing, index 49 -> EEPROM adr 192, 193 - 195, index 1-16-> 16sensor, index 17-32-> empty,  index 33-48-> 16 eink displays, index 49-> server (monitor)
#define SenStaEEPROM_A_byte_adr 132   // sensor status byte A address in EEPROM, sensor 1 to 8, 132=33*4, 4 bytes for one IP, 33 IPs stored before SenSta bytes, EEPROM address starts with 0
#define SenStaEEPROM_B_byte_adr 133   // sensor status byte B address in EEPROM, sensor 9 to 16
#define SenStaEEPROM_C_byte_adr 134   // sensor status byte C address in EEPROM, eink unit 1 to 8, 
#define SenStaEEPROM_D_byte_adr 135   // sensor status byte D address in EEPROM, eink unit 9 to 16

////// display,  OELD 2.4" SPI
#define ScrLen 128  // pixel, screen lenght, x-axis
#define ScrWid 64  // pixel, screen width, y-axis
#define LinChrMax 16 // maximum number of char per line
#define ChrHig 13 // character height
#define ChrWid 7 // character width
#define LinZerOff 0 // zero position offset for display
#define SenStaDplOff 1 // number of pixels between sensor status display

////// Adjustable variables
const char CodeVersion[] = "v2.6";
const char CoreVersion[] = "0.4.1"; // Ardunio core firmware vsersion
const char ssid[] = "ExampleSSID";             // Sensor network SSID (name)
const char pass[] = "ExamplePASS";               // Sensor network password (use for WPA, or use as key for WEP)
const unsigned int UDPPort = 50505;         // local port to listen on, UDP data
const char RmtPsd[] = "ExamplePASS";  // remote access password

////// main loop and global task management
bool IniPer = false;    // initial period, true -> program just started, will set to false after the delay
bool IniPerDly = false; // initial period delay. true -> within the delay  period at the start of the program start
bool Flg125ms = 0;  // toggle flag for 125 ms functions
bool Flg250ms = 0;   // toggle flag for 250 ms functions
bool Flg500ms = 0;   // toggle flag for 500 ms functions
bool Flg1s = 0;   // toggle flag for 1 s functions
int TimCycCnt = 0; // time slot cycle counter
unsigned long CurTim = 0;  // ms, current time
unsigned long PreTim125ms = 0;  // previous time counter, 125 ms
unsigned long StrTim =0;
unsigned long TimWiFiRecStt  = 0;    // time in ms, Wifi reconnection start time
const unsigned long CstTimWiFiRecInt  = 60000;    // 60s, time in ms, Wifi reconnection interval
const unsigned long CstTimHTTPTMO  = 1000;    // 1s, time in ms, HTTP client timeout limit, stop the connection if longer than 1s
const unsigned long CstTimInt125ms = 125; // 125ms, time managment 
const unsigned long CstTimInt25s = 25000; // 25s, ms, time managment  
const unsigned long CstTimInt1min = 60000; // 1min, time managment

////// display, OELD 2.4" SPI
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
const unsigned char wifi[] U8X8_PROGMEM = {
  0x00, 0xff, 0x00, 0x7e, 0x00, 0x18, 0x00, 0x00};

////// Sensors and eink unit status 
bool BrdCst = false;      // True-> no sensor detected, broadcast IP address
bool Alm_On=false;      // alarm on flag
bool VlvSta[MaxSenNum];              // bool array for valve position status, received from UDP data, 1-> vavle open
bool SenSta[MaxSenNum];              // bool array for sensor alive status, received from UDP data, 1-> sensor alive, 0-15 sensor 1-16, 16-31 eink units 1-16
bool SenIP_Chg[MaxSenNum];              // bool array for sensor IP change, received from UDP data, 1-> sensor IP changed
bool AlmTimOut[MaxSenNum/2];          // bool array for sensor's alarm on status,
int SenSta_CD[MaxSenNum];           // int array for sensor status cool down counter, set to 3 upon new UDP data receiving, decrease by time, when 0 set the sensor status flag to false
unsigned long AlmTimOutSec[MaxSenNum]; // s, alarm time out counter, recevied from sensor
////// EEPROM
bool SavSenVlvStaEEPROM = false; // true -> save the sensor and valve status to the EEPROM
bool SenStaEEPROM_A[8];              // bool array for sensor alive status, sensor 1 to 8
bool SenStaEEPROM_B[8];              // bool array for sensor alive status, sensor 9 to 16
bool SenStaEEPROM_C[8];              // bool array for sensor alive status, sensor 1 to 8
bool SenStaEEPROM_D[8];              // bool array for sensor alive status, sensor 9 to 16
bool SenStaEERROM[MaxSenNum];              // bool array for sensor alive status, read from EEPROM, 1-> sensor alive
uint8_t SenStaEEPROM_A_byte;        // byte for sensor alive status, sensor 1 to 8
uint8_t SenStaEEPROM_B_byte;        // byte for sensor alive status, sensor 9 to 16
uint8_t SenStaEEPROM_C_byte;        // byte for sensor alive status, eink unit 1 to 8
uint8_t SenStaEEPROM_D_byte;        // byte for sensor alive status, eink unit 9 to 16
int TotSenNumEEPROM=0;     // total number of connected devices, calculated from EEPROM data

////// Wifi service 
//char packetBuffer[64];                 //buffer to hold incoming packet
int status = WL_IDLE_STATUS;
const char ReplyBroadcast[] = "8"; // Server IP change command string, UDP
const char ReplyStatus[] = "9"; // Server acknowledgement for reciving sensor data, UDP
IPAddress broadcastIP;                // Broadcast IP address
IPAddress SenIP[MaxSenNum];       // sensor IP address, detect IP changes
WiFiUDP Udp;
WiFiServer server(80);            // TCP/IP

////// initialization
void setup() {
  // note the start time
  IniPer = true;
  StrTim=millis();
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Access Point Web Server");
  // IO
  pinMode(VlvOpnLEDPin, OUTPUT);      // set the LED pin mode
  pinMode(VlvOpnRlyPin, OUTPUT);      // set the LED pin mode
  // OLED screen
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);// 0->Black, 1-> white
  //u8g2.setFont(u8g2_font_mozart_nbp_tf); // H 0 
  u8g2.setFont(u8g2_font_7x13_t_symbols); // H 0 
  //u8g2.setFont(u8g2_font_unifont_t_symbols);
  //u8g2.setFont(u8g2_font_simple1_tr);
  //u8g2.setFont(u8g2_font_new3x9pixelfont_tr); // WxH 6x10 
  u8g2.drawStr(0, 12, "Connecting to:");   // line 1, SSID
  u8g2.drawStr(0, 24, ssid);
  u8g2.drawLine(0, 26, ScrLen, 26); // alive indicator line
  u8g2.drawLine(0, 27, ScrLen, 27); // alive indicator line
  u8g2.drawStr(0, 39, "Code:");     // code version
  u8g2.drawStr(35, 39, CodeVersion);
  u8g2.drawStr(0, 51, "Core:");       // core firmware version
  u8g2.drawStr(35, 51, CoreVersion);
  u8g2.sendBuffer();  // update the display
  
  // WiFi
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the WiFi firmware");
  }
  // get mac address
  Serial.print("MAC Address: ");
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  WiFi.macAddress(mac);
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
  // Connect to utexas-iot
  if(status != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    //delay(10000); // 10s
  }
  Serial.println("Connected to WiFi");
  printWiFiStatus();
  // assign broadcast IP
  //broadcastIP = calculateBroadcastIP(WiFi.localIP(), subnet);
  // start the UDP port
  Udp.begin(UDPPort);
  // start the web server on port 80
  server.begin();
  Serial.println("HTTP server started");
  // read saved sensor status, EEPROM
  SenStaEEPROM_A_byte = EEPROM.read(SenStaEEPROM_A_byte_adr); // read the sensor status in byte format from EEPROM, byte A, sensor 1-8
  SenStaEEPROM_B_byte = EEPROM.read(SenStaEEPROM_B_byte_adr);// read the sensor status in byte format from EEPROM, byte B, sensor 9-16
  SenStaEEPROM_C_byte = EEPROM.read(SenStaEEPROM_C_byte_adr); // read the sensor status in byte format from EEPROM, byte C, eink 1-8
  SenStaEEPROM_D_byte = EEPROM.read(SenStaEEPROM_D_byte_adr);// read the sensor status in byte format from EEPROM, byte D, eink 9-16
  byte2array(SenStaEEPROM_A_byte,SenStaEEPROM_A); // convert the byte to bool array, byte A, sensor 1-8
  byte2array(SenStaEEPROM_B_byte,SenStaEEPROM_B); // convert the byte to bool array, byte B, sensor 9-16
  byte2array(SenStaEEPROM_C_byte,SenStaEEPROM_C); // convert the byte to bool array, byte C, eink 1-8
  byte2array(SenStaEEPROM_D_byte,SenStaEEPROM_D); // convert the byte to bool array, byte D, eink 9-16
  mergeboolarrays(SenStaEEPROM_A,SenStaEEPROM_B,SenStaEEPROM_C,SenStaEEPROM_D,SenStaEERROM);  // merge the arrays
  // send the EEPROM saved sensor status to serial port
  Serial.print("SenStaEEPROM_A_byte: ");
  Serial.println(SenStaEEPROM_A_byte);
  Serial.print("SenStaEEPROM_B_byte: ");
  Serial.println(SenStaEEPROM_B_byte);
  Serial.print("SenStaEERROM: ");
  Serial.print(SenStaEERROM[0]);
  Serial.print(SenStaEERROM[1]);
  Serial.print(SenStaEERROM[2]);
  Serial.print(SenStaEERROM[3]);
  Serial.print(" ");
  Serial.print(SenStaEERROM[4]);
  Serial.print(SenStaEERROM[5]);
  Serial.print(SenStaEERROM[6]);
  Serial.print(SenStaEERROM[7]);
  Serial.print(" ");
  Serial.print(SenStaEERROM[8]);
  Serial.print(SenStaEERROM[9]);
  Serial.print(SenStaEERROM[10]);
  Serial.print(SenStaEERROM[11]);
  Serial.print(" ");
  Serial.print(SenStaEERROM[12]);
  Serial.print(SenStaEERROM[13]);
  Serial.print(SenStaEERROM[14]);
  Serial.println(SenStaEERROM[15]);
  // calculate total connected device number from EEPROM data, load saved sensor IP
  // sensors
  for(int ind=0;ind<MaxSenNum;ind++){
    if(SenStaEERROM[ind]){
      SenIP[ind]=readIPEEPROM(ind+1);
      TotSenNumEEPROM++;   // count last connected sensors
    }
  }
  Serial.print("Previous total sensor number: ");
  Serial.println(TotSenNumEEPROM);

}

// main loop
void loop() {
  //Time management
  CurTim = millis();  // get current time
  // program inital start flags
  if(IniPer){
    unsigned long tmptim = CurTim-StrTim;
    IniPer=!(tmptim>CstTimInt1min);  // true -> within 1 min of program start
    IniPerDly=tmptim<CstTimInt25s;  // true-> within 25s of program start
    SavSenVlvStaEEPROM=tmptim>CstTimInt1min;  // save the sensor and valve status after the initialization period.
  } 
  if(CurTim-PreTim125ms>=CstTimInt125ms){
    PreTim125ms=CurTim;  // reset time slot cycle cunter, 100 ms
    Fnc125ms(); // call base function 
    // time slot control
    if(TimCycCnt>=1&&TimCycCnt<8){
      //TimCycCnt between 1 to 7
      TimCycCnt=TimCycCnt+1;  // increae counter
    }
    else{
      TimCycCnt=1;    // reset counter
    }
    // call functions according to the time slot cycle counter

    if(TimCycCnt%2==1){
      Fnc250ms();  // call 250 ms functions
    }
    else if(TimCycCnt==2||TimCycCnt==6){
      Fnc500ms();  // call 500 ms functions
    }
    else if(TimCycCnt==4){
      Fnc1sA();  // call the first 1 s functions
    }
    else if(TimCycCnt==8){
      Fnc1sB();  // call the 2nd 1 s functions
    }
  }  
}
// functions
// 250 ms interval functions
void Fnc125ms(){
  Flg125ms = !Flg125ms;
  UDP_Server(); // read and reply UDP messages from sensors and eink units
  //Serial.println("1s loop");
}

// 250 ms interval functions
void Fnc250ms(){
  Flg250ms = !Flg250ms;
  Inputs();
  //Serial.println("1s loop");
}

// 500 ms interval functions
void Fnc500ms(){
  Flg500ms = !Flg500ms;
  OLED();

}

// 1 s interval functions
void Fnc1sA(){
  Flg1s = !Flg1s;
  HTTPclient();
}
void Fnc1sB(){
  SenStatus();
  // tmp
  /*SenSta[1]=true;
  VlvSta[1]=true;
  SenSta[2]=true;
  VlvSta[2]=false;
  SenSta[6]=true;
  VlvSta[6]=false;
  SenSta[7]=true;
  VlvSta[7]=false;
  SenSta[8]=true;
  VlvSta[8]=true;
  SenSta[9]=true;
  VlvSta[9]=false;
  SenSta[15]=true;
  VlvSta[15]=true;*/
  Output();
} 

// Input functions
void Inputs(){

}

// Wifi, UDP main cycle
void UDP_Server(){
  // tmp
  //Serial.println("UDP server call");
  static int broadcastInd=0;      // broadcast index
  // get WiFi status
  status = WiFi.status();
  if (status == WL_CONNECTED) {
    // only start UDP listening if connected to WiFi
    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      // only read the package and proceed if the data is from UDPPort(50505)
      if(Udp.remotePort()==UDPPort){
        // read the packet into packetBuffer
        uint8_t packetbuffer[12]; // 3xuint32_t
        int len = Udp.read(packetbuffer, sizeof(packetbuffer));
        uint32_t receivedstatus = 0;   // UDP message, sensor status, e.g. sensor 3 -> 30 (valve closed), or 31 (valve open)
        uint32_t receivedtimeout =0; // UDP message, timeout counter in seconds
        int devicestatus=0;       // sensor valve position
        uint32_t deviceID=0;       // sensor ID, sensor ID - 1 to accomendate the array index. e.g. sensor 2 would send 20 or 21, converted deviceID woud be 1
        // packages from sensor and eink have size of 8 and 4, respectively
        if(len==8){ // message from sensor
          // e.g. 10, 11, 20, 21 to 160, 161 -> senosr 1 to 16; 330, 340, to 480 -> eink units 1 to 16
          memcpy(&receivedstatus,packetbuffer,4); // get sensor status 
          memcpy(&receivedtimeout,packetbuffer+4,4); // get sensor status
          devicestatus=receivedstatus%10;       // get sensor valve position
          deviceID=receivedstatus/10-1;       // get sensor ID, sensor ID - 1 to accomendate the array index. e.g. sensor 2 would send 20 or 21, converted deviceID woud be 1
          SenSta[deviceID]=true; // set the sensor alive flag
          VlvSta[deviceID]=devicestatus;
          SenSta_CD[deviceID]=SenSta_CDMax;   // refresh the cool down timer
          // store sensor IP if the new IP is different than the old
          if(SenIP[deviceID]!=Udp.remoteIP()){
            SenIP[deviceID]=Udp.remoteIP();  // store new sensor IP
            SenIP_Chg[deviceID]=true;        // set sensor IP change flag
          } 
          // store the alarm time out
          if(receivedtimeout!=0){
            AlmTimOutSec[deviceID]=receivedtimeout;
            AlmTimOut[deviceID]=true;
          } 
          else{
            AlmTimOutSec[deviceID]=0;
            AlmTimOut[deviceID]=false;
          }
          // send acknowledgement
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(ReplyStatus);
          Udp.endPacket();
        }
        else if(len==4){ // message from eink
          memcpy(&receivedstatus,packetbuffer,sizeof(packetbuffer)); // get sensor status 
          devicestatus=receivedstatus%10;       // get sensor valve position
          deviceID=receivedstatus/10-1;       // get sensor ID, sensor ID - 1 to accomendate the array index. e.g. sensor 2 would send 20 or 21, converted deviceID woud be 1
          // tmp
          //Serial.print("Data received");
          //Serial.println(packetbuffer);
          //Serial.println(receivedstatus);
          //Serial.println(devicestatus);
          // send sensor, valve and timeout status
          uint32_t sensorstatus = 0; // UDP message to be sent to eink unit, sensor status (all sensors)
          uint32_t valvestatus = 0; // UDP message to be sent to eink unit, valve status (all sensors)
          uint32_t alarmtimeout = 0; // UDP message to be sent to eink unit, sensor 
          bool alarmtimeout32[32];  // temporary array for UDP transfer
          memcpy(alarmtimeout32,AlmTimOut,16);
          memset(alarmtimeout32+16,0,16);
          array2uint32(SenSta,sensorstatus);
          array2uint32(VlvSta,valvestatus);
          array2uint32(alarmtimeout32,alarmtimeout);
          //mergeboolarray2uint(SenSta,VlvSta,  sensorstatus);  // only use the sensor data -> [0] to [15]
          //itoa(sensorstatus, packetBuffer, 10);
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write((uint8_t*)&sensorstatus, sizeof(sensorstatus));
          Udp.write((uint8_t*)&valvestatus, sizeof(valvestatus));
          Udp.write((uint8_t*)&alarmtimeout, sizeof(alarmtimeout));
          Udp.endPacket();
          // tmp
          /*Serial.print("Sending sensor status to eink");
          Serial.println(deviceID);
          Serial.println(sensorstatus);
          Serial.println(valvestatus);
          Serial.println(alarmtimeout);*/
        }
      }
    }
    // broadcast* IP if no sensor is detected, UT iot does not allow broadcast, send server IP change message to the last known sensor IP 
    if(BrdCst){
      // only send IP to one sensor per cycle, reduce load, not using the for cycle
      if(broadcastInd<0||broadcastInd>=MaxSenNum){
        broadcastInd=0;
      }
      // if the device was online previously, but not connected now, send the server IP to it
      if(!SenSta[broadcastInd]&&SenStaEERROM[broadcastInd]){
        //broadcastIP=readIPEEPROM(broadcastInd+1);   // read saved device IP, readIPEEPROM starts at 1
        broadcastIP=SenIP[broadcastInd];
        Udp.beginPacket(broadcastIP, UDPPort);
        Udp.write(ReplyBroadcast);
        Udp.endPacket();
        Serial.print("broadcastIP:");
        Serial.println(broadcastIP);
      }
      broadcastInd++;
    }
  } 
  else {  // reconnect to WiFi if connection is lost
    if(CurTim-TimWiFiRecStt>CstTimWiFiRecInt){    
      WiFi.begin(ssid, pass); // initiate WiFi reconnection at CstTimWiFiRecInt interval
      Serial.println("Reconnecting to WiFi");
      TimWiFiRecStt=CurTim;    // record reconnection start time
    }
  }
}
// Wifi, Status count, adjust cool down counter 'SenSta_CDMax' according to cycle frequency, e.g. SenSta_CDMax=3 means 3s if the function is call every second
// also manage the sensor IP list in EEPROM
void SenStatus(){
  int totalSenNum=0;           // total number of connected devices
  Alm_On=false; // clear the alarm on flag
  for(int ind=0;ind<MaxSenNum;ind++){
    if(SenSta[ind]){
      totalSenNum++;   // count connected sensors
    }
    if(VlvSta[ind]){
      Alm_On=true;  // set alarm if one or more sensor detect valve open
    }
    // reset the valve position and sensor alive flags
    if(SenSta_CD[ind]==0){    // only reset after cool down
      VlvSta[ind]=false;  
      SenSta[ind]=false;
    }
    else if(SenSta_CD[ind]>0){
      SenSta_CD[ind]=SenSta_CD[ind]-1;
    }
    // store the new sensor IP address or sensor and valve status to EEPROM if the flag is true
    // sensor 1 ->adr 0-3, sensor 2 -> adr 4-7
    if(SenIP_Chg[ind]||SavSenVlvStaEEPROM){
      // split the new sensor status bool array to four bytes and save to EEPROM 
      splitboolarray(SenSta, SenStaEEPROM_A, SenStaEEPROM_B, SenStaEEPROM_C, SenStaEEPROM_D); 
      array2byte(SenStaEEPROM_A,SenStaEEPROM_A_byte);
      array2byte(SenStaEEPROM_B,SenStaEEPROM_B_byte);
      array2byte(SenStaEEPROM_C,SenStaEEPROM_C_byte);
      array2byte(SenStaEEPROM_D,SenStaEEPROM_D_byte);
      EEPROM.write(SenStaEEPROM_A_byte_adr, SenStaEEPROM_A_byte);
      EEPROM.write(SenStaEEPROM_B_byte_adr, SenStaEEPROM_B_byte);
      EEPROM.write(SenStaEEPROM_C_byte_adr, SenStaEEPROM_C_byte);
      EEPROM.write(SenStaEEPROM_D_byte_adr, SenStaEEPROM_D_byte);
      // sensor new IP 
      if(SenIP_Chg[ind]){
        // sensor IP changed 
        writeIPEEPROM(SenIP[ind],ind+1);  // update the new sensor IP to EEPROM
        SenIP_Chg[ind]=false; // clear sensor IP change flag
        Serial.print("IP changed for sensor #");
        Serial.println(ind+1);
        Serial.print("New status byte A and B: ");
        Serial.print(SenStaEEPROM_A_byte);
        Serial.print(" ");
        Serial.println(SenStaEEPROM_B_byte);
      }
      // only save to the EEPROM at the end on initialzation period by set the save flag on the falling edge of IniPer,
      else{
        Serial.print("Senor and valve status saved:");
        Serial.print(SenStaEEPROM_A_byte);
        Serial.print(" ");
        Serial.println(SenStaEEPROM_B_byte);
        SavSenVlvStaEEPROM=false;
      }
    }
  }
  // check for IP change event caused by restarting
  // within 30s - 10 min at the beginning of operation
  // if no sensor is connected, broadcast monitor IP
  BrdCst=IniPer&&(!IniPerDly)&&(totalSenNum<TotSenNumEEPROM);
}

// OLED display, 2.4" I2C
void OLED(){
  // divider and alive indicating line
  static unsigned int linelength = 0;
  char ln1Chr[LinChrMax];
  // initialize display variables, clear buffer and set color
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);// 0->Black, 1-> white
  // upper section connected SSID, IP
  if (status == WL_CONNECTED) { // only shows if connected to wifi
    u8g2.drawStr(2, 12, "SSID:");   // line 1, SSID
    u8g2.drawStr(42, 12, ssid);
    u8g2.drawStr(-1, 24, "IP:");     // line 2, IP address
    String tmpStr = WiFi.localIP().toString(); // Convert IP address to String
    const char* ipChar = tmpStr.c_str(); // Convert String to const char*
    u8g2.drawStr(20, 24,ipChar);
  }
  else{
    u8g2.drawStr(0, 12, "Connecting to:");   // line 1, SSID
    u8g2.drawStr(2, 24, ssid);     // line 2, IP address
  }

  if(linelength>=ScrLen){
    linelength=0;
  }
  else{
    linelength=linelength+12;
  }
  u8g2.drawLine(0, 26, linelength, 26); // alive indicator line
  u8g2.drawLine(0, 27, linelength, 27); // alive indicator line
  // lower section, valve status
  u8g2.drawStr(21, 39, "Valve Status");

  // draw sensor number accoring to the status
  for(int ind=0;ind<MaxSenNum/2;ind++){
    if(SenSta[ind]){
      if(ind<=8){ //device 1-9
        itoa(ind+1,ln1Chr,10); // sensor number
        u8g2.drawStr(ind*(ChrWid+SenStaDplOff)-LinZerOff, 51, ln1Chr);
      }
      else{ // device 10-16
        switch(ind){
          case 9:
            u8g2.drawStr(ind*(ChrWid+SenStaDplOff)-LinZerOff, 51, "A");
            break;
          case 10:
            u8g2.drawStr(ind*(ChrWid+SenStaDplOff)-LinZerOff, 51, "B");
            break;
          case 11:
            u8g2.drawStr(ind*(ChrWid+SenStaDplOff)-LinZerOff, 51, "C");
            break;
          case 12:
            u8g2.drawStr(ind*(ChrWid+SenStaDplOff)-LinZerOff, 51, "D");
            break;
          case 13:
            u8g2.drawStr(ind*(ChrWid+SenStaDplOff)-LinZerOff, 51, "E");
            break;
          case 14:
            u8g2.drawStr(ind*(ChrWid+SenStaDplOff)-LinZerOff, 51, "F");
            break;
          case 15:
            u8g2.drawStr(ind*(ChrWid+SenStaDplOff)-LinZerOff, 51, "G");
            break;
        }
      }
      // valve position
      if(VlvSta[ind]){  // valve open
        u8g2.drawStr(ind*(ChrWid+SenStaDplOff), 63, "!");
      }
      else{ // valve closed
        u8g2.drawStr(ind*(ChrWid+SenStaDplOff), 63, "-");
      }
    }
  }
  // send display contents
  u8g2.sendBuffer();
}

// WiFi HTTP client for remote monitoring
void HTTPclient(){
  static unsigned long clientstarttime = 0;  // for the clinet connection start time
  static bool lastclient=false; // stores the last client 
  static bool iniconnection=false; // true-> intial connection 
  WiFiClient client = server.available(); // listen for incoming clients
  while(client) { // new client
    // client variables
    static bool newclient = false;  // true -> new client, send the password login request
    static bool loginrequest = false; // true -> the client is repsonding with a password
    static bool gotpassword=false;  // true ->  password extracted
    String enteredpassword="";
    //Serial.println("new client.");
    iniconnection=!lastclient;  // if the previous client state is false (no client), set the intial connection flag
    lastclient=true;  // remember the current client state
    // note the client connection time
    if(iniconnection){
      clientstarttime=CurTim;
    }
    // stop the client connection if timeout limite reached, does not seem to work...
    if((CurTim-clientstarttime)>CstTimHTTPTMO){
      Serial.println("connection timeout, closing client");
      client.stop();
      break;
    }
    // while client connected and data to read
    if (client.connected()) {
      //bool endofrequest=false; // true -> end of client request, proceed with reponds
      if (client.available()) { // available data from client
        //endofrequest=false;  // end of client message
        String currentLine = client.readStringUntil('\n');  // read the currentline
        //Serial.print("currentline:");
        //Serial.println(currentLine);
        //Serial.println(currentLine.length());
        // check if the current line is empty
        /*
        if(currentLine.length()==1){
          endofrequest=true;
          
        }*/
        // proceed according to the verb at the beginning of the request
        if (currentLine.startsWith("GET ")) {  // 1st time connection
          newclient=true; // set the 1st time connection flag
          loginrequest=false;// clear the password responding  flag
        }
        else if(currentLine.startsWith("POST /login")){  // 2nd or later response with password 
          loginrequest=true;  // set the password responding  flag
          newclient=false;  // clear the 1st time connection flag
        }
        else if(iniconnection){ // other type of request, close connection
          Serial.println("unsupported method, closing client");
          client.stop();
          break;
        }

        // listen for the password
        if(loginrequest){
          //Serial.println("login detected"); // tmp
          // keep reading the message line by line for "password"
          if(currentLine.indexOf("password=")>=0){  // key word detected
            enteredpassword = currentLine.substring(currentLine.indexOf("password=") + 9);  // extract the password
            enteredpassword.trim();  // remove any leading/trailing whitespace
            gotpassword=true; // set the extracted password flag
            Serial.println("entered password: " + enteredpassword);
          }
        }
      }
      // reading the message, or key message received, respond to client accordingly
      // new client, send the login request
      if(newclient){ 
        // start HTTP response 
        Serial.println("Responding to new client"); // tmp
        client.println("HTTP/1.1 200 OK");    // HTTP header
        client.println("Content-type:text/html");
        client.println(); // end of header
        client.println("<html><body>"); // body, asking for password
        client.println("<h1>Please Enter the Password</h1>");
        client.println("<form action='/login' method='post'>");
        client.println("Password: <input type='password' name='password' />");
        client.println("<input type='submit' value='Submit' />");
        client.println("</form>");
        client.println("</body></html>");
        client.println(); // end of response
        // stop the connection after responding to the client
        client.stop();
        lastclient=false;  // reset the client state memory, must follow the client.stop()
        newclient=false;
      }
      else if(loginrequest&&gotpassword){ // client reponded to the login request with a password, check the password
        if (enteredpassword == RmtPsd) {  // correct password
          // correct password
          Serial.println("correct password, sending sensor status"); // tmp
          //Serial.println("Responding to client"); // tmp
          client.println("HTTP/1.1 200 OK");    // HTTP header
          client.println("Content-type:text/html");
          client.println();
          SendSenSta(client);  // send the sensor status data
          client.println("</body></html>");
          client.println();
          Serial.println("sensor data sent");
        } else {
          // incorrect password
          Serial.println("incorrect password");
          client.println("<h1>Incorrect password. Please try again.</h1>");
          client.println("<a href='/'>Go back</a>");
        }
        loginrequest=false;
        gotpassword=false;// clear the password correct flag
        // stop the connection after responding to the client
        client.stop();
        lastclient=false;  // reset the client state memory, must follow the client.stop()
      }
      //Serial.println("client connected, but nothing happens"); // tmp
    }
    // no client or client disconnected by themselves
  }
}

// send the sensor status table to the HTTP client
void SendSenSta(WiFiClient &client){
  // start HTML 
  client.println("<!DOCTYPE html><html><head><title>BB Status</title></head><body>"); // HTML 5 and tabe name
  client.println("<h1>Keitz Lab <br> Bunsen Burner Status</h1>");  // tab name
  client.println("<style>");  // set font 
  client.println("body, h1 { font-family: Courier, Consolas, 'Courier New', monospace; }");
  client.println("</style></head>");

  // start table
  client.println("<table border='1' cellpadding='5' cellspacing='0'>");
  client.println("<tr><th>Sensor</th><th>Status</th><th>Time Out HH:MM</th></tr>");

  for (int i = 0; i < MaxSenNum/2; i++) {
    client.print("<tr><td>");
    client.print(i + 1);
    client.print("</td><td>");
    if (SenSta[i]) {
      client.print(VlvSta[i] ? "<span style='color:red;'>Open</span>" : "<span style='color:green;'>Closed</span>");
    } 
    else {
      client.print("<span style='color:grey;'>Disconnected</span>");
    }
    client.print("</td><td>");
    if (AlmTimOut[i]) {
      unsigned long sec = AlmTimOutSec[i];
      unsigned int hh = sec / 3600;
      unsigned int mm = (sec % 3600) / 60;
      char buf[16];
      snprintf(buf, sizeof(buf), "%02u:%02u", hh, mm);
      client.print("<span style='color:red;'>");
      client.print(buf);
      client.print("</span>");
    } 
    else {
      client.print("-");
    }
    client.println("</td></tr>");
  }

  // end table 
  client.println("</table>");
}

// Output functions
void Output(){

  digitalWrite(VlvOpnLEDPin,Alm_On);  // DO for on LED indicator
  digitalWrite(VlvOpnRlyPin,Alm_On);  // DO for relay, LED Sign ,
  
}

// Function to calculate the broadcast IP
IPAddress calculateBroadcastIP(IPAddress localIP, IPAddress subnetMask) {
  // Perform bitwise OR between the local IP and the inverse of the subnet mask
  uint32_t ip = (uint32_t)localIP;
  uint32_t mask = (uint32_t)subnetMask;
  uint32_t broadcast = ip | ~mask;

  return IPAddress(broadcast);
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
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

// EEPROM related functions, sensor 1-16->index 0-15
void writeIPEEPROM(IPAddress ip, int startAddressInd) {
  int startAddress = (startAddressInd-1)*4;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(startAddress + i, ip[i]);  // Save each byte of the IP
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

// bool array and byte functions
// e.g. sensor 7, byte 64 (b0100 0000, msb->lsb)
void byte2array(const uint8_t byte, bool boolArray[8]) {
  for (int i = 0; i < 8; i++) {
    boolArray[i] = (byte >> i) & 0x01;  // Store bits in reverse order
  }
}
// e.g. sensor 7, array 0100 0000 (msb->), byte 64, sensor 8->1
void array2byte(const bool boolArray[8], uint8_t &byteValue) {
  byteValue = 0;  // Clear the byte before setting bits
  for (int i = 0; i < 8; i++) {
    if (boolArray[i]) {
      byteValue |= (1 << i);  // Set the i-th bit if boolArray[i] is true
    }
  }
}

void array2uint32(const bool boolArray[32], uint32_t &uintValue) {
  // only copy the fisrt 16 bits. 
  uintValue = 0;  // Clear the byte before setting bits
  for (int i = 0; i < 16; i++) {
    if (boolArray[i]) {
      uintValue |= (1 << i);  // Set the i-th bit if boolArray[i] is true
    }
  }
}
// convert senser sensor status and valve status to uint for UDP transmission
// only use the first 16 bools in the input arrays
void mergeboolarray2uint(const bool array1[32], const bool array2[32], uint32_t &uintValue) {
  uintValue = 0;  // Clear the uint before setting bits
  bool tmpunitarray[32];
  memcpy(tmpunitarray, array1, 16 * sizeof(bool));        // Copy first array
  memcpy(tmpunitarray + 16, array2, 16 * sizeof(bool));    // Copy second array
  for (int i = 0; i < 32; i++) {
    if (tmpunitarray[i]) {
      uintValue |= (1 << i);  // Set the i-th bit if boolArray[i] is true
    }
  }
}

// source: 0000 0010 0000 0001 (lsb->msb), target1: 0000 0010 (lsb->msb), target2: 0000 0001 (lsb->msb)
void splitboolarray(const bool source[16], bool target1[8], bool target2[8], bool target3[8], bool target4[8]) {
  memcpy(target1, source, 8 * sizeof(bool));       // Copy first 8 bools
  memcpy(target2, source + 8, 8 * sizeof(bool));  // Copy next 8 bools
  memcpy(target3, source + 16, 8 * sizeof(bool));       // Copy first 8 bools
  memcpy(target4, source + 24, 8 * sizeof(bool));  // Copy next 8 bools
}

void mergeboolarrays(const bool array1[8], const bool array2[8],const bool array3[8], const bool array4[8], bool result[16]) {
  memcpy(result, array1, 8 * sizeof(bool));        // Copy first array
  memcpy(result + 8, array2, 8 * sizeof(bool));    // Copy second array
  memcpy(result + 16, array3, 8 * sizeof(bool));    // Copy third array
  memcpy(result + 24, array4, 8 * sizeof(bool));    // Copy fourth array
}
