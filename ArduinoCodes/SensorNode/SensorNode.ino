/*
  Bunsen Burner Monitoring System
  Sensor node
  Local burner valve position sensor with optional motorized gas valve (auto shutoff). 
  Send status over UDP to the monitor hub
  Variable EleGasVlvEna == 0 -> no motorized valve; == 1 -> motorized valve installed 
  Vesrion: 2.6

  License: MIT
  Copyright (c) 2025 The University of Texas at Austin
  Developed by Yang Gao (McKetta Department of Chemical Engineering)
 */
//#include <Arduino.h>    //
#include <U8g2lib.h>  // OLED control
#include <Wire.h>     // SPI communication
#include <Adafruit_Sensor.h>// temperature sensor
#include "AHT20.h"// temperature sensor
#include "WiFiS3.h" // wifi 
#include <EEPROM.h> // 512 bytes, each byte 0-255
////// IO
////// input pins 
#define VlvSenPin 4  // methane valve position, input, pin 2
#define KeyLftPin 3  // Switch key to the left
#define KeyRghPin 2  // Switch key to the right
#define WifAutRecPin 12    // wifi auto reconnection switch
#define GasVlvRlyPin 0  // gas shut off valve relay pin
////// output pins 
#define AlmBzrPin 5    // Buzzer alarm on pin
#define VlvCloLEDPin 6    // Valve closed LED indicator, green LED
#define VlvOpnLEDPin 7    // Valve opne LED indicator, red LED
#define PIRSenPin 1     // body motion sensor input

////// sensor control
#define AlmTimNum 4   // alarm timer preset values, 15 min, 30 min, 45 min, 60 min
#define MonOnLineCntMax 5 // monitor online count down, decrease by 1 each program cycle.
////// alarm contorl
#define EleVlvCloDly 10 // s, time delay, waiting the gas valve to fully close (shut off)
#define EleVlvCloAftTMODly 600 // s, time delay, waiting the gas valve to fully close (shut off)

////// EEPROM
#define ServerIPEEPROM_adr 33   // server IP EEPROM indexing, index 49 -> EEPROM adr 192, 193 - 195, index 1-16-> 16sensor, index 17-32-> empty,  index 33-48-> 16 eink displays, index 49-> server (monitor)
#define SenStaEEPROM_A_byte_adr 132   // sensor status byte A address in EEPROM, sensor 1 to 8, 132=33*4, 4 bytes for one IP, 33 IPs stored before SenSta bytes, EEPROM address starts with 0
#define SenStaEEPROM_B_byte_adr 133   // sensor status byte B address in EEPROM, sensor 9 to 16
#define SenStaEEPROM_C_byte_adr 134   // sensor status byte C address in EEPROM, eink unit 1 to 8, 
#define SenStaEEPROM_D_byte_adr 135   // sensor status byte D address in EEPROM, eink unit 9 to 16
////// display,  OELD 2.4" SPI
#define ScrLen 128  // pixel, screen lenght, x-axis
#define ScrWid 64  // pixel, screen width, y-axis
#define TmpWdx 88  // temperature window x
#define TmpWdy 0   // temperature window y
#define LinChrMax 16 // maximum number of char per line

////// Adjustable variables
const bool EleGasVlvEna = 0; // true -> with active gas shut off valve, false-> passive sensor only audial and visual alarm
const char CodeVersion[] = "v2.6";
const char CoreVersion[] = "0.4.1"; // Ardunio core firmware version
const int SenNum = 6;  // sensor number
const char ssid[] = "exampleSSID";             // Sensor network SSID (name)
const char pass[] = "examplePASS";               // Sensor network password (use for WPA, or use as key for WEP)
IPAddress serverIP(192,0,2,0);  // server IP, replace with your monitor hub assigned IP
const int UDPPort = 50505;         // local port to listen on, UDP data
int AlmTim[4] = {15, 30, 45, 60}; // mins, alarm timer interval

////// main loop and global task management
bool IniPer = false;    // true -> program just started, will set to false after the delay
bool IniPerDly = false; // initial period delay. true -> within the delay  period at the start of the program start
bool Flg250ms = 0;   // toggle flag for 250 ms functions
bool Flg500ms = 0;   // toggle flag for 500 ms functions
bool Flg1s = 0;   // toggle flag for 1 s functions
int TimCycCnt = 0; // time slot cycle counter
unsigned long CurTim = 0;  // ms, current time
unsigned long PreTim125ms = 0;  // previous time counter, 125 ms
unsigned long starttime =0;
unsigned long keystarttime =0;
unsigned long prekeystarttime=0;
unsigned long endtime = 0;
unsigned long TimWiFiRecStt  = 0;    // time in ms, Wifi reconnection start time
const unsigned long CstTimWiFiRecInt  = 60000;    // 60s, time in ms, Wifi reconnection interval
const unsigned long CstTimInt125ms = 125; // ms, time managment 
const unsigned long CstTimInt250ms = 250; // ms, time managment 
const unsigned long CstTimInt500ms = 500; // ms, time managment 
const unsigned long CstTimInt5s = 5000; // ms, time managment 
const unsigned long CstTimInt15s = 15000; // ms, time managment 
const unsigned long CstTimInt45s = 45000; // ms, time managment 
const unsigned long CstTimInt180s = 180000; // ms, time managment   
////// display, OELD 2.4" SPI
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
static const unsigned char wifi[] U8X8_PROGMEM = {
  0x00, 0xff, 0x00, 0x7e, 0x00, 0x18, 0x00, 0x00};
int CurPos;// cursor position

////// sensor control (alarms, beeper, gas valve, monitor status check, EEPROM etc)
const bool PIRSenEna = 0;  // true -> body motion sensor installed, not used, but may be developed for flame out sensor. PIR can be constantly triggered by the flame
bool ManVlvPos = 0;      // valve position, 0->open, 1->closed
bool PreManVlvPos = 0;      // previous valve position, 0->open, 1->closed
bool Alm_On = 0;  // alarm control, 1 -> alarm on
bool FlmOut = 0;  // 1-> flame out detected
bool PIRMotDet=0; // alarm reset by body motion, 1-> reset alarm, 0-> no motion sensed
bool EleVlvClo = 0;  // electronic gas valve close (shut off), 1-> close, 0-> open
bool EleVlvLck = 0; // electronic gas valve lock, 1-> valve is closed and cannot be open, 0-> can change the valve status
bool PreEleVlvClo = 0;  // previous gas valve close (shut off) state
bool WifAutRec = 0; // wifi auto reconnect, 1->auto reconnect, 0-> do not reconnect
bool MonOnLine = false; // monitor is online 1->yes, 0-> no.
int MonOnLineCnt =0;    // monitor online counter, reduce by 1 every 1s,
int EleVlvLckCnt=0; // electronic gas valve lock count down
unsigned long PIRMotDetCnt_Up = 0; // s, PIR sensor count up
unsigned long EleVlvCloDlyCntDwn = 0; // s, gas valve close (shut off) trigger time count down
unsigned long ManVlvAlmCntDwn = 0;  // s, timer alarm for manual valve open (flame on), beeper and LED
unsigned long ManVlvAlmTMOCnt_Up = 0;  // s, timer alarm for manual valve open (flame on), beeper and LED
unsigned long VlvAlmTimSec = 0; // s, alarm timmer in seconds, if valve open and within alarm timmer -> count down for alarm. if alarm on- > count up for alarm time out

////// temperature and humidity sensor 
float Tmp = 0;    // measured temperature
float Hum = 0;    // measured humidity
AHT20 AHT; // I2C

///// Wifi service 
int status = WL_IDLE_STATUS;
const int SerAck = 9; // code for server received the status message
const int SerIPChgCode = 8; // code for server IP change event, update new server IP address if recevived this code

IPAddress serverIPEEPROM;  // server IP
WiFiUDP Udp;

////// initialization
void setup() {
  // note the start time
  IniPer = true;
  starttime=millis();
  // serial port
  Serial.begin(9600);
  // temperature and humidity sensor
  AHT.begin();
 
  // Set up oversampling and filter initialization

  // input pins
  pinMode(WifAutRecPin, INPUT); // wifi auto reconnect switch
  pinMode(VlvSenPin, INPUT);    // set the digital pin 4 as input, gas valve position
  pinMode(KeyLftPin, INPUT_PULLUP);    // set key left pin as input
  pinMode(KeyRghPin, INPUT_PULLUP);    // set key right pin as input
  attachInterrupt(digitalPinToInterrupt(KeyLftPin), KeyLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(KeyRghPin), KeyRight, FALLING);
  pinMode(PIRSenPin, INPUT); // input for body motion sensor
  
  // output pins
  pinMode(AlmBzrPin, OUTPUT);    // alarm buzzer control
  pinMode(VlvCloLEDPin, OUTPUT);    // drive the green LED when valve is closed
  pinMode(VlvOpnLEDPin, OUTPUT);    // drive the red LED when valve is open
  pinMode(GasVlvRlyPin, OUTPUT);   // control the relay for gas shut off valve

  // beeper test
  digitalWrite(AlmBzrPin,true); // buzzer sound
  delay(100);
  digitalWrite(AlmBzrPin,false); // buzzer slient
  delay(50);
  digitalWrite(AlmBzrPin,true); // buzzer sound
  delay(100);
  digitalWrite(AlmBzrPin,false); // buzzer slient
  // OLED
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);// 0->Black, 1-> white
  //u8g2.setFont(u8g2_font_mozart_nbp_tr); // H 0
  u8g2.setFont(u8g2_font_7x13_t_symbols); // H 0 
  //u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawStr(2, 12, "Connecting WiFi");
  u8g2.drawStr(2, 24, "Code:");
  u8g2.drawStr(42, 24, CodeVersion);
  u8g2.drawStr(2, 36, "Core:");
  u8g2.drawStr(42, 36, CoreVersion);
  u8g2.drawStr(2, 48, "Sensor#:");
  char tmpchr[2];
  itoa(SenNum,tmpchr,10);
  u8g2.drawStr(64, 48, tmpchr);

  u8g2.sendBuffer();

  // Wifi
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  // print out mac address
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
  // attempt to connect to WiFi network:
  if(status != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 3 seconds for connection:
    delay(6000);
  }
  Serial.println("Connected to WiFi");
  printWiFiStatus();
  // start UDP
  // if you get a connection, report back via serial:
  Udp.begin(UDPPort);
  // set line change flags to true
  CurPos=1;

  // read the saved server IP, EEPROM
  serverIPEEPROM=readIPEEPROM(ServerIPEEPROM_adr);   // read saved device IP
  // tmp
  Serial.print("serverIPEEPROM");
  Serial.println(serverIPEEPROM);
}
////////////////// end initialization//////////////

////////////////// functions//////////////
void loop() {
  //Time management
  CurTim = millis();  // get current time
  // program inital start flags
  if(IniPer){
    unsigned long tmptim = CurTim-starttime;
    IniPer=!(tmptim>CstTimInt45s);  // true -> within 45 s of program start
    IniPerDly=tmptim<CstTimInt15s;  // true-> within 15 s of program start
  }
  if(CurTim-PreTim125ms>=CstTimInt125ms){
    PreTim125ms=CurTim;  // reset time slot cycle cunter, 125 ms
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
void Fnc250ms(){
  Flg250ms = !Flg250ms;
  Inputs();   // inputs, valve position, WiFi auto-connection switch
  AlarmControl(); // alarm count down, beeper
  Output(); // alarm LED, beeper

}

// 500 ms interval functions
void Fnc500ms(){
  Flg500ms = !Flg500ms;
  OLED();   // display OLED
}

// 1 s interval functions
void Fnc1sA(){
  Flg1s = !Flg1s;
  UDP_Client();   // send status message to the monitor
  //OLED();
  //Serial.println("1s loop");
  CounterControl_1s();  // counter control

} 
void Fnc1sB(){
  UDP_Read();   // read monitor reply, acknowledgement or monitor IP change
  MonitorCheck(); // monitor alive check and save new monitor IP address same call frame with UDP_Read
  //UDP_Client();
  //OLED();
  //Serial.println("1s loop");
} 

void Inputs(){
  // AHT20 sensors, temperautre and humidity readings
  int AHT_OK=AHT.getSensor(&Hum, &Tmp); // AHT return value 0-> get data fail, else-> data OK
  Hum=Hum*100;  // convert to 100%
  /*Serial.print("humidity: ");
  Serial.print(Hum*100);
  Serial.print("%\t temerature: ");
  Serial.println(Tmp);*/
  // digital IO
  
  ManVlvPos = digitalRead(VlvSenPin);   // read the valve position IR output 
  //digitalRead(KeyLftPin);
  //digitalRead(KeyRghPin);
  WifAutRec=digitalRead(WifAutRecPin);  // read wifi auto reconnect pin 1-> auto reconnect
  PIRMotDet=digitalRead(PIRSenPin)&&PIRSenEna;   // read the body motion sensor output
}
void KeyLeft(){
  keystarttime=millis(); // disable trigger for 500ms 
  if(keystarttime-prekeystarttime>CstTimInt500ms){
    prekeystarttime=keystarttime;
    if(CurPos>1){
      CurPos= CurPos-1;
    }
    else{ 
      CurPos=0;
    }
    // electronic gas valve lock release
    if(ManVlvPos==0&&EleVlvLckCnt>0){ // only allow key input when manual valve is closed
      EleVlvLckCnt--;
    }
    if(EleVlvClo&&EleVlvLckCnt==0){ // only release the electronic valve lock when key is pressed while the valve is closed
      EleVlvLck=false;
    }
  }
  // reset valve position to write in the new time
  PreManVlvPos=false;
}
void KeyRight(){
  keystarttime=millis(); // disable trigger for 500ms 
  if(keystarttime-prekeystarttime>CstTimInt250ms){
    prekeystarttime=keystarttime;
    if(CurPos<=2){
      CurPos= CurPos+1;
    }
    else{ 
      CurPos=3;
    }
    // electronic gas valve lock release
    if(ManVlvPos==0&&EleVlvLckCnt>0){// only allow key input when manual valve is closed
      EleVlvLckCnt--;
    }
    if(EleVlvClo&&EleVlvLckCnt==0){ // only release the electronic valve lock when key is pressed while the valve is closed
      EleVlvLck=false;
    }
  }
  // reset valve position to write in the new time
  PreManVlvPos=false;
}

// counter control
void CounterControl_1s(){
  // manual valve open alarm count down
  if(ManVlvAlmCntDwn>0){
    ManVlvAlmCntDwn--;
  }
  else{
    ManVlvAlmCntDwn=0;
  }
  // manual valve open time out count up
  if(Alm_On){
    ManVlvAlmTMOCnt_Up++;
  }
  else{
    ManVlvAlmTMOCnt_Up=0;
  }
  
  // gas valve closing delay count down
  if(EleVlvCloDlyCntDwn>0){
    EleVlvCloDlyCntDwn--;
  }
  else{
    EleVlvCloDlyCntDwn=0;
  }

  // PIR count up
  if(PIRSenEna){
    PIRMotDetCnt_Up++;
  }
  else{
    PIRMotDetCnt_Up=0;
  }
  
}
//

// alarm control
void AlarmControl(){
// valve alarm control logic
  if (ManVlvPos){    // valve open
    // detect change of state in valve position, set the alarm time upon valve opening
    if (!PreManVlvPos){ // set a new timer if manual valve just opend, body motion sensed while electronic shut off vlave is open
      
      ManVlvAlmCntDwn=AlmTim[CurPos]*60; // convert min to s
      Alm_On=0; // reset the alarm as people might just set a new timer when the alarm is on
      //Serial.print("new timer");  // tmp
    }

    // manual valve open timmer time up, set LED and beeper alarms
    if(ManVlvAlmCntDwn==0){  // alarm on
      //Alm_On
      Alm_On=1; // turn on alarm buzzer
      // gas valve close (shut off)
      if(ManVlvAlmTMOCnt_Up>EleVlvCloAftTMODly){  // allow 1 min for people to intervene, afterwards, shut off the gas with the electronic valve 
        EleVlvClo=EleGasVlvEna; // close (shut off) the gas valve
        EleVlvLck=true; // lock the electronic gas valve state, 
        EleVlvLckCnt=2; 
        if(EleVlvClo!=PreEleVlvClo){  // gas valve close commend just set
          EleVlvCloDlyCntDwn=EleVlvCloDly;  // set the gas valve closing delay count down
          PreEleVlvClo=EleVlvClo;
        }
      }
    }

    // flame out detection
    if(PIRMotDet){  // if PIR is triggered, flame can trigger PIR
      PIRMotDetCnt_Up=0;  // reset the counter
    }
    FlmOut=PIRMotDetCnt_Up>3; // if PIR is not triggered in 3 s when valve is open, set the flame out flag
  }
  else{   // valve closed
    // detect valve position change
    Alm_On=0;   // turn off alarm
    EleVlvClo=EleGasVlvEna&&((EleVlvCloDlyCntDwn>0)||EleVlvLck);  // only open the gas valve after the closing delay and valve lock released
    //noTone(AlmBzrPin);// turn off the buzzer if valve is closed
    //digitalWrite(AlmBzrPin,1);
  }
  PreManVlvPos = ManVlvPos;   // store previous valve position for change detection
}
void OLED(){
  char ln1Chr[LinChrMax];
  char ln2Chr[LinChrMax];
  char ln3Chr[LinChrMax];
  // initialize display variables, clear buffer and set color
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);// 0->Black, 1-> white, 
  // draw the content depending on the gas valve state, if closed (shut off) display restart instruction, if open normal contents
  if(EleVlvClo){  // gas valve closed (shut off), EleVlvClo
    u8g2.drawRFrame(0, 0,126,62, 0);  // draw frame
    u8g2.drawStr(20, 11, "Auto shut-off");
    u8g2.drawStr(0, 22, " -----Please-----");
    u8g2.drawStr(0, 34, ">Turn off the gas");
    u8g2.drawStr(0, 46, ">Switch left/right");
    u8g2.drawStr(0, 58, " twice to confirm");
  }
  else{
    // temperature and humidity sensor info
    u8g2.drawRFrame(TmpWdx, TmpWdy,38,26, 0);

    itoa(Tmp,ln1Chr,10);
    //u8g2.setFont(u8g2_font_unifont_t_symbols);
    u8g2.drawStr(TmpWdx+2, 12, ln1Chr);
    u8g2.drawUTF8(TmpWdx+20, 12, "°C");

    itoa(Hum,ln2Chr,10);
    u8g2.drawStr(TmpWdx+2, 24, ln2Chr);    
    u8g2.drawStr(TmpWdx+20, 24, "%");

    // valve position related display content
    static unsigned int linelength = 0; // divider and alive indicating line
    if(ManVlvPos){ // valve open
      if(!Alm_On){  // during count down
        if(FlmOut){
          u8g2.drawStr(16, 12, "Flameout?");
        }
        else{
          u8g2.drawStr(16, 12, "Alarm in");
        }
        
        VlvAlmTimSec=ManVlvAlmCntDwn; // alarm count down in total sec, 
        // draw line lenght according to time left
        linelength=(ScrLen*VlvAlmTimSec)/(AlmTim[CurPos]*60);
      }
      else{ // valve open, alarm time out
        //  count the overtime
        u8g2.drawStr(16, 12, "Time Out");
        VlvAlmTimSec = ManVlvAlmTMOCnt_Up;  // 
        // blink the line between full lenght and nothing
        if(Flg1s){
          linelength=ScrLen;  // draw full length
        }
        else{
          linelength=0;  // draw full length
        }
      }

      // display the alarm time count down or overtime count up
      itoa(VlvAlmTimSec/60,ln1Chr,10); // alarm count down min
      itoa(VlvAlmTimSec%60,ln2Chr,10); // alarm count down sec
      if(VlvAlmTimSec/60>=100){ // three digits minutes
        u8g2.drawStr(16, 24, ln1Chr);
      }
      else if(VlvAlmTimSec/60>=10){  // two digits minutes
        u8g2.drawStr(24, 24, ln1Chr);
      }
      else{ // single digit minutes
        u8g2.drawStr(24, 24, "0");
        u8g2.drawStr(32, 24, ln1Chr);
      }
      u8g2.drawStr(44, 24, ":");
      if(VlvAlmTimSec%60>=10){  // two digits seconds
        u8g2.drawStr(54, 24, ln2Chr);
      }
      else{ // single digits seconds
        u8g2.drawStr(54, 24, "0");
        u8g2.drawStr(62, 24, ln2Chr);
      }

    }
    else{ // valve closed
      u8g2.drawStr(28, 12, "Valve");
      u8g2.drawStr(24, 24, "Closed");
      linelength=ScrLen;  // draw full length
    }

    // wifi indicator
    //u8g2.drawRFrame(WifiWdx, WifiWdy,14,26, 0);
    if(WiFi.status()==WL_CONNECTED){ // wifi connected
      if(Flg1s){
        //u8g2.drawStr(0, 12, "B");
        u8g2.drawXBMP(0, 1, 8, 8, wifi);
      }
      else{
        u8g2.drawStr(0, 12, " ");
      }
    }
    else{
      if(Flg1s){
        //u8g2.drawStr(0, 12, "*");
        u8g2.drawStr(0, 12, "*");
      }
      else{
        u8g2.drawStr(0, 12, " ");
      }
    }
    if(WifAutRec){
      u8g2.drawStr(1, 20, "A");
    }
    else{
    u8g2.drawStr(1, 20, "M");
    }
    // line 3 with line count down
    u8g2.drawLine(0, 28, linelength, 28);
    u8g2.drawLine(0, 29, linelength, 29);
    u8g2.drawLine(0, 30, linelength, 30);
    u8g2.drawStr(2, 44, "Timer Sel.(min)");

    // draw alarm timer according to cursor position
    for (int ind=0;ind<AlmTimNum;ind++){
      
      //u8g2.setBitmapMode(ind==CurPos);// 0->Black, 1-> white
      //u8g2.setDrawColor(ind==CurPos);// 0->Black, 1-> white
      //u8g2.setBitmapMode(1 /* transparent*/); // 0-> black background, 1-> white background
      // draw box or frame according to cursor selection.
      
      if(ind==CurPos){  // selected timer, draw box
        //u8g2.setDrawColor(0);// 0->Black, 1-> white
        u8g2.drawFrame((ind)*(ScrLen/AlmTimNum), 48, ScrLen/AlmTimNum-5, 16); //61,3
        //u8g2.setDrawColor(1);// 0->Black, 1-> white
      }
      //else{
      //  //u8g2.setDrawColor(0);// 0->Black, 1-> white
      //  u8g2.drawFrame((ind)*(ScrLen/AlmTimNum), 48, ScrLen/AlmTimNum-5, 16);
      //}
      //u8g2.setDrawColor(0);// 0->Black, 1-> white
      itoa(AlmTim[ind],ln3Chr,10);
      u8g2.drawStr((ind)*(ScrLen/AlmTimNum)+5, 60, ln3Chr);
      
    }
  }
  // send display contents
  u8g2.sendBuffer();
}

// outputs controls, buzzer and relay
void Output(){

  digitalWrite(VlvCloLEDPin,!ManVlvPos);  // valve closed LED
  digitalWrite(VlvOpnLEDPin,ManVlvPos);  // valve open LED
  if(Alm_On){ // alarm on
    digitalWrite(AlmBzrPin,Flg1s&&Flg250ms); // buzzer sound
    digitalWrite(VlvOpnLEDPin,Flg500ms);  // LED on
  }
  else{ // alarm time reset manually by Joystick swtich button
    //noTone(AlmBzrPin);// turn off the buzzer if valve is closed
    digitalWrite(AlmBzrPin,0);  // buzzer off
    //tone(AlmBzrPin, 3000);
    //noTone(AlmBzrPin);
  } 
  digitalWrite(GasVlvRlyPin,EleVlvClo);  // relay signal for gas shut off valve
  
}

void UDP_Client(){
  // get the current WiFi status
  status=WiFi.status();
  // reconnect to WiFi if connection is lost
  if(status != WL_CONNECTED){
    if(WifAutRec){  // reconnect to wifi if switched on
      if(CurTim-TimWiFiRecStt>CstTimWiFiRecInt){    
        status = WiFi.begin(ssid, pass); // initiate WiFi reconnection at CstTimWiFiRecInt interval
        Serial.println("Reconnecting to WiFi");
        TimWiFiRecStt=CurTim;    // record reconnection start time
      }
    }
  }
  else{ // send status via UDP
    // send a message, to the IP address and port that sent us the packet we received
    uint32_t sendstatus = 0;   // UDP message, sensor status, e.g. sensor 3 -> 30 (valve closed), or 31 (valve open)
    uint32_t sendtimeout =0; // UDP message, timeout counter in seconds
    if(ManVlvPos){
      sendstatus=SenNum*10+1;
    }
    else{
      sendstatus=SenNum*10+0;
    }
    // send non-zero alarm time out when alarm on
    if(Alm_On){
      sendtimeout=VlvAlmTimSec;
    }
    else{
      sendtimeout=0;
    }
    //itoa(sendstatus, packetBuffer, 10);
    //Udp.write(packetBuffer);
    //Serial.print("sending UDP to:");
    //Serial.println(serverIP);
    Udp.beginPacket(serverIP, UDPPort);
    Udp.write((uint8_t*)&sendstatus, sizeof(sendstatus));
    Udp.write((uint8_t*)&sendtimeout, sizeof(sendtimeout));
    Udp.endPacket();
    //Serial.println("Done");
    // tmp
    //Serial.print("sending to IP:");
    //Serial.println(serverIP);
    // receive 
  }
}
// UDP read
void UDP_Read(){
  char packetBuffer[12]; //buffer to hold incoming packet
  int UDPRcvDat=0; // received UDP message
  // get WiFi status
  status = WiFi.status();
  if (status == WL_CONNECTED) {
    // only start UDP listening if connected to WiFi
    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      // only read the package and proceed if the data is from UDPPort(14890)
      if(Udp.remotePort()==UDPPort){
        // read the packet into packetBuffer
        int len = Udp.read(packetBuffer, sizeof(packetBuffer));
        if (len > 0) {
          packetBuffer[len] = 0;
        }
        int UDPRcvDat=atoi(packetBuffer); // received UDP message

        // interpret received message
        if(UDPRcvDat==SerAck){  // acknowledgement from server for the UDP package
          MonOnLineCnt=MonOnLineCntMax;   // reset monitor online counter
        }
        else if(UDPRcvDat==SerIPChgCode){// server IP address change
          //update new server IP
          //compare the saved value to the remote IP, aviode redondant update to EEPROM
          if(serverIP!=Udp.remoteIP()){
            serverIP=Udp.remoteIP();
            writeIPEEPROM(serverIP,ServerIPEEPROM_adr);
            serverIPEEPROM=serverIP;
            Serial.print("new IP address from server, EEPROM updated: ");
            Serial.println(serverIP);
          }
          
        }

      }
    }
  }
}
void MonitorCheck(){
  // monitor online detection
  if(MonOnLineCnt>=1){
    MonOnLineCnt--;
    MonOnLine=true;
  }
  else{
    MonOnLineCnt=0;
    MonOnLine=false;
  }
  // server IP address check at the start of program
  if(!IniPerDly&&IniPer){
    //Serial.println("ini. per.");
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
    }// server not online, use the server IP stored in EEPROM
    else{
      serverIP=serverIPEEPROM;
      Serial.print("new IP address from EEPROM: ");
      Serial.println(serverIP);
    }
  }
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
}

// EEPROM related functions
void writeIPEEPROM(IPAddress ip, int startAddressInd) {
  int startAddress = (startAddressInd-1)*4;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(startAddress + i, ip[i]);  // Save each byte of the IP
  }
}
IPAddress readIPEEPROM(int startAddressInd) {
  IPAddress ip;
  int startAddress = (startAddressInd-1)*4;
  for (int i = 0; i < 4; i++) {
    ip[i] = EEPROM.read(startAddress + i);  // Read each byte and assign to IPAddress
  }
  return ip;
}
////////////////// end functions//////////////