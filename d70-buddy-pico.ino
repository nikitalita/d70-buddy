#include <pico/stdlib.h>
#include <U8g2lib.h>
#include <U8x8lib.h>
#include <MIDI.h>
#include <SPI.h>
#include <pico/util/queue.h>
byte pcBuffer[16];
byte sysEx[9]={0x41,0x10,0x42,0x12,0x40}; //excludes boundaries. array space for channel, upper byte, lower byte, checksum.
const byte gsResetSysEx[9]={0x41,0x10,0x42,0x12,0x40,0x00,0x7F,0x00,0x41};
byte gmResetSysEx[4]={0x7E,0x7F,0x09,0x01};
byte mt32GM[128]={0x0,0x1,0x2,0x4,0x5,0x4,0x5,0x3,0x10,0x11,0x12,0x12,0x13,0x13,0x14,0x15,0x6,0x6,0x6,0x7,0x7,0x7,0x8,0x8,0x3E,0x3F,0x3E,0x3F,0x26,0x27,0x26,0x27,0x5D,0x5E,0x36,0x62,0x61,0x63,0x59,0x65,0x66,0x60,0x44,0x50,0x51,0x53,0x55,0x50,0x30,0x31,0x30,0x2D,0x28,0x29,0x2A,0x2A,0x2B,0x2E,0x2E,0x18,0x19,0x1A,0x1B,0x68,0x20,0x20,0x21,0x22,0x24,0x25,0x23,0x23,0x49,0x49,0x48,0x48,0x4A,0x4B,0x41,0x40,0x42,0x43,0x47,0x47,0x44,0x45,0x46,0x16,0x38,0x38,0x39,0x39,0x3C,0x3C,0x3A,0x3D,0x3D,0xB,0xB,0xB,0xB,0x9,0xE,0xD,0xC,0x6B,0x6A,0x4D,0x4E,0x4E,0x4C,0x4C,0x2F,0x75,0x2F,0x76,0x76,0x74,0x73,0x77,0x73,0x70,0x37,0x7C,0x7B,0x7E,0x7A,0x7A};
//in SC-8820 mode, CC00 127 = MT32 map.
unsigned long lastTime = 0;
unsigned long gfxTime = 0;
boolean lastButton = false;
char *messages[]={"D70-Buddy!","GM Mode","GS Mode","MT-32 Mode","SC-55","SC-88","SC-88Pro","SC-8820","CM-64 C"};
const char strChannel[]="123456789ABCDEFG";
int msgCoords[]={0,4,44,83,2,29,58,97,97};
boolean logEn=false;
#define GFX_EN 1
// #define DEBUG 1

#ifdef DEBUG
#define DEBUG_TIME_START(starttime) unsigned long starttime = micros()
#define DEBUG_TIME_PRINTF(x,starttime) DEBUG_TIME_START(end_##starttime); Serial.print(x); Serial.print(end_##starttime - starttime); Serial.println();
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTF(x,y) Serial.print(x); Serial.print(y); Serial.println();
#else
#define DEBUG_TIME_START(x)
#define DEBUG_TIME_PRINTF(x,y)
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x,y)
#endif

struct MySettings : public MIDI_NAMESPACE::DefaultSettings{
  static const bool UseRunningStatus = true;
  static const bool Use1ByteParsing = false;
  static const long BaudRate = 31250;
  static const unsigned SysExMaxSize = 1024;
};

MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI, MySettings);


//d70 buddy U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 31, 30, 29);
//U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 10,9,8);
//U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 17, /* dc=*/ 20, /* reset=*/ 21); 

//U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8); 
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
U8G2_SSD1312_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);


const int bSC55=2;
const int bSC88=3;
const int bSC88p=6;
const int b8820=7;
const int bGM=8;
const int bGS=9;
const int bMT32=10;
const int buttons[7] = {bSC55, bSC88, bSC88p, b8820, bGM, bGS, bMT32};

#ifdef LED_ENABLED
const int ledSC55=11;
const int ledSC88=12;
const int ledSC88p=13;
const int led8820=14;
const int ledGM=15;
const int ledGS=22;
const int ledMT32=28;
#endif //LED_ENABLED

enum SoundMap: uint8_t{
  SC_55 = 1,
  SC_88 = 2,
  SC_88Pro = 3,
  SC_8820 = 4
};
enum MidiMode: uint8_t{
  GM_MODE,
  GS_MODE,
  MT32_CM64_MODE, // MT32 mode, 8820 map in CM64 mode
  MT32_GMMAP_MODE // MT32 mode, mapped to any map via mt32GM
};

typedef struct MIDIState_struct {
  uint8_t channelLevel[16];
  bool noteActive[16];
  char sysMsg[20];
  SoundMap map; // 1 = sc55, 2 = sc88, 3 = sc88pro, 4 = sc8820
  SoundMap lastMap;
  MidiMode mode; //0 = gm, 1 = gs, 2 = mt32mode
} MIDIState;

MIDIState state;

queue_t stateDisplayQueue;
const int QUEUE_LENGTH = 4; //if the display misses more than 4 updates, something is seriously wrong.

void setup() {
  queue_init(&stateDisplayQueue, sizeof(MIDIState), QUEUE_LENGTH);
  pinMode(bGM, INPUT_PULLUP);
  pinMode(bGS, INPUT_PULLUP);
  pinMode(bMT32, INPUT_PULLUP); 
  pinMode(bSC55, INPUT_PULLUP);
  pinMode(bSC88, INPUT_PULLUP);
  pinMode(bSC88p, INPUT_PULLUP);
  pinMode(b8820, INPUT_PULLUP);
  #ifdef LED_ENABLED
  pinMode(ledSC55, OUTPUT);
  pinMode(ledSC88, OUTPUT);
  pinMode(ledSC88p, OUTPUT);
  pinMode(led8820, OUTPUT);
  pinMode(ledGM, OUTPUT);
  pinMode(ledGS, OUTPUT);
  pinMode(ledMT32, OUTPUT);
  #endif //LED_ENABLED
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOn();
  MIDI.setThruFilterMode(midi::Thru::Full);
  lastTime=millis();
  gfxTime=millis();
  //unsigned int thing;
  //rp2040.fifo.pop_nb(&thing);
  #ifdef DEBUG
  Serial.begin(500000l);
  #endif
  setGSMode();
  setD70Map(SC_8820);
}

void setup1(){
  startDisplay(1000000);
  gfxInit();
  gfxMessage(messages[0]);
  gfxSetMode(GS_MODE);
  gfxSetMap(SC_8820);
}

void startDisplay(uint32_t busClock){
  u8g2.setBusClock(busClock);
  u8g2.begin();
  //u8g2.sendF("c", 0xa0);
  u8g2.clearBuffer();
}

void readMidi(){
  bool read = MIDI.read();
  if (read){
    if (MIDI.getType()==midi::ProgramChange) {
      pcBuffer[MIDI.getChannel()-1]=MIDI.getData1();
      if(state.mode== MT32_GMMAP_MODE){ //if mt32 any map mode
        MIDI.sendProgramChange(mt32GM[pcBuffer[MIDI.getChannel()-1]], MIDI.getChannel()); //send similar sounding chanel
      }
    }
    if (MIDI.getType()==midi::SystemExclusive && MIDI.getSysExArray()[5]==0x20 && MIDI.getSysExArray()[6]==0x00 && MIDI.getSysExArray()[7]==0x00){
      int i;
      for(i=0;i<19;i++){ //19 chars long
        state.sysMsg[i]=MIDI.getSysExArray()[i+8];
      }
      state.sysMsg[i] = '\0'; //null terminator
    }
    if (MIDI.getType()==midi::NoteOn) {
        state.channelLevel[MIDI.getChannel()-1]=MIDI.getData2();
        state.noteActive[MIDI.getChannel()-1] = true;
    }
    if (MIDI.getType()==midi::NoteOff){
        state.noteActive[MIDI.getChannel()-1] = false;
    }
  }
}

void pollButtons() {
  for(int i=0;i<7;i++){
    if(!digitalRead(buttons[i]) && lastButton==false){
      lastButton=true;
      lastTime=millis();
      {
        switch(buttons[i]){
          case bGM:
            exitMT32Mode();
            setGMMode();
            break;
          case bGS:
            exitMT32Mode();
            setGSMode();
            break;
          case bMT32:
            enterMT32Mode();
            break;
          case bSC55: 
            if(!(state.mode==MT32_CM64_MODE)){
              setD70Map(SC_55);
            }
            break;
          case bSC88:
            if(!(state.mode==MT32_CM64_MODE)){
              setD70Map(SC_88);
            }
            break;
          case bSC88p:
            if(!(state.mode==MT32_CM64_MODE)){
              setD70Map(SC_88Pro);
            }
            break;
          case b8820:
            if((state.mode==MT32_CM64_MODE && state.lastMap==4) || (state.mode==MT32_CM64_MODE && state.lastMap<4)){         //if in CM-64 mode already, change to 8820 mode
              state.mode = MT32_GMMAP_MODE;
            }
            else if(state.mode==MT32_GMMAP_MODE){       //if in MT32->8820 mode already -> change to CM-64 mode
              state.mode = MT32_CM64_MODE;
            }
            setD70Map(SC_8820); 
            break;
        }
      }
    }
  }
}

void drawGraphics(const MIDIState * current_state){
  DEBUG_TIME_START(gfxloopdur);
  gfxMessage(messages[0]);
  gfxSetMap(current_state->map);
  gfxSetMode(current_state->mode);
  gfxSetGraphs(current_state->channelLevel);
  gfxSetChannels(current_state->noteActive);
  DEBUG_TIME_PRINTF("gfxSetGraph LOOP duration: ", gfxloopdur);
  DEBUG_TIME_START(sendbufdur);
  u8g2.sendBuffer();
  DEBUG_TIME_PRINTF("u8g2.sendBuffer duration: ", sendbufdur);
}

void loop() {
  readMidi();
  pollButtons();
  if(millis()-lastTime>1000){  //allows new button presses after 1 sec
      lastButton=false;
      lastTime=millis();
  }

#ifdef GFX_EN

  if(millis()-gfxTime>20){    //update screen each 20ms
    if (!queue_try_add(&stateDisplayQueue, &state)){ //non-blocking add
      DEBUG_PRINT("WTF?!");
    } 
    // decrement all the channel levels
    for(int i=0;i<16;i++){
      if(state.channelLevel[i] > 0){
        if (state.noteActive[i] && state.channelLevel[i] < 15){
            state.channelLevel[i] = 12;
        } else if (state.channelLevel[i] < 3){
          state.channelLevel[i] = 0;
        } else {
          state.channelLevel[i] -= 3;
        }
        
      }
    }
    gfxTime=millis();
  } 
#endif //GFX_EN

}    

void loop1(){
  MIDIState current_state;
  queue_remove_blocking(&stateDisplayQueue, &current_state);
  drawGraphics(&current_state);
}

void gfxInit(){
  u8g2.clearBuffer();
  gfxMapInit();
  gfxModeInit();
  u8g2.sendBuffer();
}

void gfxMapInit(){
  u8g2.setDrawColor(0);
  u8g2.drawBox(0,55,128,9);
  u8g2.setDrawColor(1);
  u8g2.drawRFrame(0,55,128,9,2);
  u8g2.setFont(u8g2_font_micro_tr);
  for(int i=0;i<4;i++){
    u8g2.drawStr(msgCoords[i+4]+1,62,messages[i+4]);
  }
}

void gfxModeInit(){
  u8g2.setDrawColor(0);
  u8g2.drawBox(0,45,128,9);
  u8g2.setDrawColor(1);
  u8g2.drawRFrame(0,45,128,9,2);
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(5,52,"GM Mode   GS Mode");
  u8g2.drawStr(84,52,"MT-32 Mode");
  u8g2.drawStr(3,42,"1 2 3 4 5 6 7 8 9 A B C D E F G");
  u8g2.drawRFrame(0,11,128,33,2);
}

void gfxMessage(char *message){
  //u8g2.clearBuffer();
  u8g2.setDrawColor(0);
  u8g2.drawBox(0,0,128,10);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_t0_11b_tf);
  u8g2.drawStr(0,8,message);
}

void gfxSetGraphs(const uint8_t * channelMeter ){ //sets height of channel bar based on vel
  DEBUG_TIME_START(gfxloopdur);
  u8g2.setDrawColor(0);
  u8g2.drawBox(2,12,125,22);
  u8g2.setDrawColor(1);
  for (int i = 0; i < 16; i++){
    uint8_t col_height = channelMeter[i]/6;
    u8g2.drawBox(2+((i)*8),34-col_height,5,col_height);
  }
  DEBUG_TIME_PRINTF("gfxSetGraphs duration: ", gfxloopdur);
}

void gfxSetGraph(uint8_t channel, uint8_t vel){ //sets height of channel bar based on vel
  uint8_t col_height = vel/6;
  DEBUG_TIME_START(gfxloopdur);
  uint8_t cOffset=3+((channel)*8)-1;
  u8g2.setDrawColor(0);
  u8g2.drawBox(cOffset,12,5,22);
  u8g2.setDrawColor(1);
  u8g2.drawBox(cOffset,34-col_height,5,col_height);
  DEBUG_TIME_PRINTF("gfxSetGraph duration: ", gfxloopdur);
}

void gfxSetChannels(const bool * activeChannel){ //highlights or not channel number
    DEBUG_TIME_START(gfxloopdur);
    char p[2];
    p[1] = '\0';
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_micro_tr);
    u8g2.drawBox(2,36,125,7);
    for (int channel = 0; channel < 16; channel++){
      bool cstate = activeChannel[channel];
      uint8_t cOffset=3+((channel)*8);
      if (cstate){
        u8g2.setDrawColor(true);
        u8g2.drawBox(cOffset-1,36,5,7);
      }
      u8g2.setDrawColor(!cstate);
      p[0] = strChannel[channel];
      u8g2.drawStr(cOffset,42,p);
    }
    DEBUG_TIME_PRINTF("gfxSetChannels duration: ", gfxloopdur);
}


void gfxSetChannel(uint8_t channel, uint8_t state){ //highlights or not channel number
    DEBUG_TIME_START(gfxloopdur);
    u8g2.setDrawColor(0);
    uint8_t cOffset=3+((channel)*8);
    u8g2.setDrawColor(state);
    u8g2.drawBox(cOffset-1,36,5,7);
    u8g2.setDrawColor(!state);
    char p[]="";
    strncpy(p,&strChannel[channel],1);
    u8g2.setFont(u8g2_font_micro_tr);
    u8g2.drawStr(cOffset,42,p);
    DEBUG_TIME_PRINTF("gfxSetChannel duration: ", gfxloopdur);
}

void gfxSetMap(SoundMap map){
  gfxMapInit();
  uint8_t l_map = map - 1 + 4;
  u8g2.setDrawColor(1);
  u8g2.drawBox(msgCoords[l_map],56,strlen(messages[l_map])*4+1,7);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(msgCoords[l_map]+1,62,messages[l_map]);
}

void gfxSetMode(MidiMode mode){
  gfxModeInit();
  u8g2.setDrawColor(1);
  u8g2.drawBox(msgCoords[mode+1],46,strlen(messages[mode+1])*4+1,7);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(msgCoords[mode+1]+1,52,messages[mode+1]);
}

//SC-55= 1
//SC-88= 2
//SC-88Pro = 3
//SC-8820 = 4

void setD70Map(SoundMap map){
    state.map = map;
    for(int j=1;j<17;j++){
      sysEx[5]=0x40+j;
      sysEx[6]=0x01;
      sysEx[7]=map;
      sysEx[8]=128-((sysEx[4]+0x40+j+map+1)%128);
      MIDI.sendSysEx(9,sysEx,false);
    }
    for(int k=1;k<17;k++){
      if(state.mode == MT32_CM64_MODE){
        MIDI.sendControlChange(0,127,k);
        MIDI.sendProgramChange(pcBuffer[k-1],k);
      }
      else if(state.mode == MT32_GMMAP_MODE){
        MIDI.sendControlChange(0,0,k);
        MIDI.sendProgramChange(mt32GM[pcBuffer[k-1]],k);        
      }
      else{
        MIDI.sendControlChange(0,0,k);
        MIDI.sendProgramChange(pcBuffer[k-1],k);
      }
    }
    state.lastMap=map;
    #ifdef LED_ENABLED
    switch (map){
      case 1:
        digitalWrite(ledSC55,HIGH);
        digitalWrite(ledSC88,LOW);
        digitalWrite(ledSC88p,LOW);
        digitalWrite(led8820,LOW);
        break;
      case 2:
        digitalWrite(ledSC55,LOW);
        digitalWrite(ledSC88,HIGH);
        digitalWrite(ledSC88p,LOW);
        digitalWrite(led8820,LOW);
        break;
      case 3:
        digitalWrite(ledSC55,LOW);
        digitalWrite(ledSC88,LOW);
        digitalWrite(ledSC88p,HIGH);
        digitalWrite(led8820,LOW);
        break;
      case 4:
        digitalWrite(ledSC55,LOW);
        digitalWrite(ledSC88,LOW);
        digitalWrite(ledSC88p,LOW);
        digitalWrite(led8820,HIGH);
        break;
    }
    #endif
}

void enterMT32Mode(){
  #ifdef LED_ENABLED
  digitalWrite(ledGM,LOW);
  digitalWrite(ledGS,LOW);
  digitalWrite(ledMT32,HIGH);
  #endif
  gsReset();
  state.mode = MT32_CM64_MODE;
  setD70Map(SC_8820);
}

void exitMT32Mode(){
  state.mode = GM_MODE;
  for(int k=0;k<16;k++){
   MIDI.sendControlChange(0,0,k);
   MIDI.sendProgramChange(pcBuffer[k-1],k);
  }  
}

void setGSMode(){
  state.mode = GS_MODE;
  #ifdef LED_ENABLED
    digitalWrite(ledGM,LOW);
    digitalWrite(ledGS,HIGH);
    digitalWrite(ledMT32,LOW);
  #endif
  gsReset();
}

void gsReset() {
  MIDI.sendSysEx(9,gsResetSysEx,false);
  reSendPCs();
}

void setGMMode(){
#ifdef LED_ENABLED
  digitalWrite(ledGM,HIGH);
  digitalWrite(ledGS,LOW);
  digitalWrite(ledMT32,LOW);
#endif
  state.mode = GM_MODE;
  gmReset();
}

void gmReset(){
  MIDI.sendSysEx(4,gmResetSysEx,false);
  reSendPCs();
}

void reSendPCs(){
  for(int k=0;k<16;k++){
    MIDI.sendProgramChange(pcBuffer[k-1],k);
  } 
}
