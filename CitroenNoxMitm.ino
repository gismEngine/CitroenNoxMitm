// ESP32 Citroen Jumper NOx Emulator
// Please check README.H

#include "canMng.h"
#include "webMng.h"
#include "sockMng.h"
#include "dtcMng.h"

const char* PROGMEM FIRMWARE_NAME = "ESP_NOx_Emulator";
const char* PROGMEM FIRMWARE_VER = "0.1";

#define DEBUG Serial

void setup() {
  
  DEBUG.begin(115200);

  // Just development information
  printEspInfo();

  // Init wifi + web interface:
  // ESP32 can create its own wifi to serve web interface (WIFI_AP_MODE) or can connect
  // to an existing wifi. This setting is set at wifiMng.h file
  
  #if defined (WIFI_AP_MODE) || defined (WIFI_STA_MODE)
    webMng.begin(FIRMWARE_NAME, FIRMWARE_VER);                // Init Wifi-AP + Web Server + DNSserver
    websocketBegin();                                         // Init websocket server
  #else

    // If NO wifi flag is set then there is no wifi interface
    DEBUG.println(F("WARNING: No wifi mode defined --> Wifi disabled"));
  #endif

  // Initialize CAN cards MCP2515 connected by SPI
  canBegin();
}

void loop() {

  canTxMng();
  canRxMng();

  debugCmdMng();

  
  #if defined (WIFI_AP_MODE) || defined (WIFI_STA_MODE)
    webMng.manage();                                      // Do wifi stuff
    websocketManage();                                    // Do websocket stuff
  #endif

  if (isNewRemoteData() || isNewClient()) {               // If there is new client (new web connection)
    broadcastUiData();                                    // Send form information to have update NOx information
  }

}

void broadcastUiData(void) {
  
  convModeNox_main = getConversionModeNox();
  DEBUG.print("convModeNox: ");
  DEBUG.println(convModeNox_main);

  switch (convModeNox_main) {
    case CONV_DIS:
      broadcastJsonById("conv_mode_nox", "CONV_DIS");
      break;
    case CONV_FIX:
      broadcastJsonById("conv_mode_nox", "CONV_FIX");
      break;
    case CONV_PROP:
      broadcastJsonById("conv_mode_nox", "CONV_PROP");
      break;
    case CONV_MAP:
      broadcastJsonById("conv_mode_nox", "CONV_MAP");
      break;
    default:
      broadcastJsonById("conv_mode_nox", "UNKNOWN");
      break;
  }
  
  convModeO2_main = getConversionModeO2();
  DEBUG.print("convModeO2: ");
  DEBUG.println(convModeO2_main);

  switch (convModeO2_main) {
    case CONV_DIS:
      broadcastJsonById("conv_mode_o2", "CONV_DIS");
      break;
    case CONV_FIX:
      broadcastJsonById("conv_mode_o2", "CONV_FIX");
      break;
    case CONV_PROP:
      broadcastJsonById("conv_mode_o2", "CONV_PROP");
      break;
    case CONV_MAP:
      broadcastJsonById("conv_mode_o2", "CONV_MAP");
      break;
    default:
      broadcastJsonById("conv_mode_o2", "UNKNOWN");
      break;
  }
  
  nox_fix_bypass_main = getNoxFixBypass();
  o2_fix_bypass_main = getO2FixBypass();
  broadcastJsonById("o2_fix_bypass", o2_fix_bypass_main);
  broadcastJsonById("nox_fix_bypass", nox_fix_bypass_main);

  nox_prop_main = getNoxProp();
  o2_prop_main = getO2Prop();
  broadcastJsonById("nox_prop", (double)nox_prop_main);
  broadcastJsonById("o2_prop", (double)o2_prop_main);
}

// DEBUG - Serial interface
void debugCmdMng(void) {

  if (DEBUG.available()) {                        // Check if there is command received
    
    char c = DEBUG.read();
    if (c == '?') {

      // '?' Char prints UART menu:
      
      DEBUG.print(F("\n== UART MENU ==\n"));
      DEBUG.print(F("\ni: ESP32 Info "));
      DEBUG.print(F("\nb: CAN bridge: "));
      printBool(canBridge);
      
      DEBUG.print(F("\n\np: Print CAN verbose (both channels): "));
      printBool(canPrint);      
      DEBUG.print(F("\n1: Print CAN Channel 1 verbose: "));
      printBool(can1Print);
      DEBUG.print(F("\n2: Print CAN Channel 2 verbose: "));
      printBool(can2Print);
      
      DEBUG.println(F("\n\no: Request supported OBD PId"));
      DEBUG.println(F("n: Request OBD NOx (PId 0x83)"));
      DEBUG.print(F("c: Print NOx conditions reached: "));
      printBool(printTempReached);
      DEBUG.println();
      
      DEBUG.print(F("\nt: Send periodic test frame: "));
      printBool(testFrameTime != 0);
      DEBUG.println();
    } else if (c == 'i'){
      printEspInfo();
    } else if (c == 'b') {
      DEBUG.print("CAN Bridge: ");
      canBridge = !canBridge;
      printBool(canBridge);
      DEBUG.println();
    } else  if (c == 'p') {
      DEBUG.print("CAN print: ");
      canPrint = !canPrint;
      printBool(canPrint);
      DEBUG.println();
    } else  if (c == '1') {
      DEBUG.print("CAN1 print: ");
      can1Print = !can1Print;
      printBool(can1Print);
      DEBUG.println();
    } else  if (c == '2') {
      DEBUG.print("CAN2 print: ");
      can2Print = !can2Print;
      printBool(can2Print);
      DEBUG.println();
    } else  if (c == 'n') {
      DEBUG.print("OBD NOx req: ");
      askNoxObd = !askNoxObd;
      printBool(askNoxObd);
      DEBUG.println();
    } else  if (c == 'o') {
      DEBUG.println(F("Req OBD supported PId"));
      askSupportedPid = true;
    } else if (c == 'c') {
      DEBUG.print(F("Print conditions reached: "));
      printTempReached = !printTempReached;
      printBool(can2Print);
      DEBUG.println();
    } else if (c == 'd') {
       DEBUG.println(F("Read DTC"));
       doDtcRead();
    } else if (c == 'R') {
      DEBUG.println(F("Clear DTC"));
      doDtcClear();
    } else if (c == 'x') {
      doTestDtc();
    } else  if (c == 't') {
      if (testFrameTime == 0){
        testFrameTime = 1000;
      }else{
        testFrameTime = 0;
      }
    }
  }
}


// Only used to display UART menu in human way
void printBool(bool b){
  if(b){
    DEBUG.print(F("TRUE"));
  }else{
    DEBUG.print(F("FALSE"));
  }
}


// ESP32 hardware and sketch information. 
// Just for debuging and development
void printEspInfo(void){
  
  //  Print firmware name and version const string
  DEBUG.print(F("\n\n\n"));
  DEBUG.print(FIRMWARE_NAME);
  DEBUG.print(F(" - "));
  DEBUG.println(FIRMWARE_VER);

  DEBUG.print(F("CPU@ "));
  DEBUG.print(getCpuFrequencyMhz());    // In MHz
  DEBUG.println(F(" MHz"));

  DEBUG.print(F("Xtal@ "));
  DEBUG.print(getXtalFrequencyMhz());    // In MHz
  DEBUG.println(F(" MHz"));

  DEBUG.print(F("APB@ "));
  DEBUG.print(getApbFrequency());        // In Hz
  DEBUG.println(F(" Hz"));

  EspClass e;

  DEBUG.print(F("SDK v: "));
  DEBUG.println(e.getSdkVersion());

  DEBUG.print(F("Flash size: "));
  DEBUG.println(e.getFlashChipSize());

  DEBUG.print(F("Flash Speed: "));
  DEBUG.println(e.getFlashChipSpeed());

  DEBUG.print(F("Chip Mode: "));
  DEBUG.println(e.getFlashChipMode());

  DEBUG.print(F("Sketch size: "));
  DEBUG.println(e.getSketchSize());

  DEBUG.print(F("Sketch MD5: "));
  DEBUG.println(e.getSketchMD5());
  
}
