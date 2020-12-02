#include "Arduino.h"
#include "canMng.h"
#include "obdMng.h"
#include "map.h"

bool canBridge;
bool canPrint;
bool can1Print;
bool can2Print;

bool printTempReached;

bool finish;
bool waitingObdReply;

bool askSupportedPid;
uint8_t askSupportedPidCount;

bool askNoxObd;
uint32_t askNoxTime;

enum conversionMode convModeNox_main;
enum conversionMode convModeO2_main;
uint32_t nox_fix_bypass_main;
uint32_t o2_fix_bypass_main;
float nox_prop_main;
float o2_prop_main;

MCP_CAN CAN1(CAN1_CS);     // Set CS to pin 10
MCP_CAN CAN2(CAN2_CS);     // Set CS to pin 10

unsigned long rxId;
uint8_t len = 0;
uint8_t rxBuf[8];
char msgString[128];                        // Array to store serial string

// For test Tx-frame
uint32_t frameTime = 0;
uint32_t testFrameTime = 0;    // in ms. If set to 0 no frame is send

void canBegin(){

  // Maps initialization
  // Maps are intended to interpolate NOx and O2 information from Citroen Jumper NOx sensor
  // Maps are not tuned. Just as a concept
  initMaps();
  
  // Init CAN (MCP2515) interfaces:
  DEBUG.println(F("CAN-Channel 1 Setup:"));

  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN1.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    DEBUG.println(F("MCP2515 Init: OK!"));
  } else {
    DEBUG.println(F("/!\ ERROR Initializing MCP2515..."));
  }
  CAN1.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  pinMode(CAN1_INT, INPUT);

  Serial.println(F("CAN-Channel 2 Setup:"));

  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN2.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    DEBUG.println(F("MCP2515 Init: OK!"));
  } else {
    DEBUG.println(F("/!\ ERROR Initializing MCP2515..."));
  }
  CAN2.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  pinMode(CAN2_INT, INPUT);

  // By default CAN traffic on channel 1 is send to CAN channel 2
  // and vice versa. So CAN bus is cut but vehile and sensor can't detect it
  // Flag can be changed by UART/Serie Menu
  canBridge = true;

  // Can traffic can be printed to UART
  // Flag can be changed by UART/Serie Menu
  canPrint = false;
  can1Print = false;
  can2Print = false;

  
  printTempReached = false;
  
  finish = true;
  waitingObdReply = false;
  
  askSupportedPid = false;
  askSupportedPidCount = 0;
  
  askNoxObd = false;
  askNoxTime = 0;
  
  dtcMngBegin(getCAN1(), getCAN2());

  frameTime = millis();
}

void canRxMng(void) {

  // If CAN*_INT pin is low: it means CAN traffic has been detected
  // Proceed to read received buffer

  // CAN Channel 1:
  if (!digitalRead(CAN1_INT)) {
    canChRxMng(&CAN1);
  }

  // CAN Channel 2:
  if (!digitalRead(CAN2_INT)) {
    canChRxMng(&CAN2);
  }
  
}


void canTxMng(void) {

  // Task to send periodic frame when flag is send by UART
  // Only for development
  if (testFrameTime != 0) {
    if (millis() >= frameTime + testFrameTime) {
      sendTestFrame();
      frameTime = millis();
    }
  }

//  bool sendOk = false;
//
//  if (askSupportedPid) {
//    //Serial.println("askSupportedPid: ");
//    if (!waitingObdReply) {
//      Serial.print("waitingObdReply: ");
//      Serial.println(waitingObdReply);
//      supPidReq[2] = askSupportedPidCount;
//      sendOk = sendFrame(&CAN1, obdCanIdExt, true, 8, supPidReq);
//      if (sendOk) {
//        Serial.print("SEND OK");
//        if (askSupportedPidCount <= 0xC0) {   // To generate serie: 0x00, 0x20, 0x40, 0x80, 0xA0, 0xC0
//          askSupportedPidCount = askSupportedPidCount + 0x20;
//        } else {
//          askSupportedPidCount = 0x00;
//          askSupportedPid = false;
//        }
//        waitingObdReply = true;
//      }
//    }
//  }
//
//  if (askNoxObd) {
//    if (!waitingObdReply) {
//      if (millis() >= askNoxTime + NOX_REQ_TIME) {
//        bool sendOk = sendFrame(&CAN1, obdCanIdStd, false, 8, noxReq);
//        if (sendOk) {
//          waitingObdReply = true;
//          askNoxTime = millis();  // if not posible to send frame (arbitration, electrical, ...) repeat again next loop
//        }
//      }
//    }
//  }
//
//  dtcMng();
}


void canChRxMng(MCP_CAN* canCh) {

  uint8_t sidpr = 0;
  uint8_t pId = 0;
  uint8_t payloadSize = 0;
  uint8_t rxObdBuf[20];
  uint8_t currPos = 0;

  // Check CAN channel:
  uint8_t chNum = 0;
  if (canCh == &CAN1) {
    chNum = 1;
  } else if (canCh == &CAN2) {
    chNum = 2;
  }

  // Read data:
  // rxId = frame ID
  // len = CAN frame length (DLC)
  // buf = frame payload: data byte(s)
  canCh->readMsgBuf(&rxId, &len, rxBuf);

  unsigned long canId = rxId & 0x3FFFFFFF;
  //unsigned long canId = canCh->getCanId();
  
  // Determine if ID is standard (11 bits) or extended (29 bits)
  uint8_t extId = 0;
  if ((rxId & 0x80000000) == 0x80000000) {
    extId = 1;
  }

  if (canPrint || (can1Print && (chNum == 1)) || (can2Print && (chNum == 2))) {
    printAsc(chNum, extId);
  }


  // Citroen Jumper NOx VALUES
  // 0x264 - NOx Engine side (PreCata-Nox)
  // 0x284 - NOx Exhaust side (PostCata-Nox)
  
  if ((rxId == 0x264) || (rxId == 0x284)) {

    // Different NOx modifications can be done thru web interface
    // - CONV_FIX will fix NOX and O2 values to a fixed value 
    //            Fix value can be changed by web interface
    
    if (convModeNox_main == CONV_FIX) {
      uint8_t nox_msb_fix = nox_fix_bypass_main >> 8;
      uint8_t nox_lsb_fix = (uint8_t)(nox_fix_bypass_main & 0xFF);

      rxBuf[2] = nox_msb_fix;
      rxBuf[3] = nox_lsb_fix;


    // Different NOx modifications can be done thru web interface
    // - CONV_PROP will change NOX and O2 values multiplying to float value  
    //            Proportional value can be changed by web interface

    } else if (convModeNox_main == CONV_PROP) {

      uint16_t nox = ((uint16_t)rxBuf[2] << 8) | rxBuf[3];        // Current Nox value send by sensor

      nox = nox * nox_prop_main;                                  // Manupulate Nox value send by sensor by multiplying to float parameter

      uint8_t nox_msb_prop = nox >> 8;
      uint8_t nox_lsb_prop = (uint8_t)(nox & 0xFF);

      rxBuf[1] = nox_msb_prop;
      rxBuf[2] = nox_lsb_prop;

    
    // Different NOx modifications can be done thru web interface
    // - CONV_MAP will change NOX and O2 values by interpolating receibed values on defined MAPs  
    //            MAPs input and outputs can be seen live on web interface
    
    }//else if(convMode == CONV_MAP){
    
        // Not fully implemented yet
        // MAP interpolation is done OK
        // MAP interpolation interface (functions are done)
        // TODO: MAP definition (size and values)
        // TODO: MAP inputs definition (input values for X and Y)
        
    //}



    // Different NOx modifications can be done thru web interface
    // - CONV_FIX will fix NOX and O2 values to a fixed value 
    //            Fix value can be changed by web interface
    
    if (convModeO2_main == CONV_FIX) {

      uint8_t o2_msb_fix = o2_fix_bypass_main >> 8;
      uint8_t o2_lsb_fix = (uint8_t)(o2_fix_bypass_main & 0xFF);

      rxBuf[4] = o2_msb_fix;
      rxBuf[5] = o2_lsb_fix;


    // Different NOx modifications can be done thru web interface
    // - CONV_PROP will change NOX and O2 values multiplying to float value  
    //            Proportional value can be changed by web interface
    
    } else if (convModeNox_main == CONV_PROP) {

      uint16_t o2 = ((uint16_t)rxBuf[4] << 8) | rxBuf[5];

      o2 =  o2 * o2_prop_main;

      uint8_t o2_msb_prop = o2 >> 8;
      uint8_t o2_lsb_prop = (uint8_t)(o2 & 0xFF);

      rxBuf[3] = o2_msb_prop;
      rxBuf[4] = o2_lsb_prop;


    // Different NOx modifications can be done thru web interface
    // - CONV_MAP will change NOX and O2 values by interpolating receibed values on defined MAPs  
    //            MAPs input and outputs can be seen live on web interface

    
    }//else if(convMode == CONV_MAP){
    
        // Not fully implemented yet
        // MAP interpolation is done OK
        // MAP interpolation interface (functions are done)
        // TODO: MAP definition (size and values)
        // TODO: MAP inputs definition (input values for X and Y)
     
    //}


  // Aparenlty NOx sensors sends random values before drew point is reached
  // When drew point is reahed byte 1 and 0 are set to 0x80
  // Flag to print drew potint reached (temperature reached) can be changed by UART
  }else if (printTempReached){
    if (rxId == 0x264){
      DEBUG.print(F("PreCata-Nox: "));
      DEBUG.print(rxBuf[1] == 0x80);
      DEBUG.print(F(" PostCata-Nox: "));
      DEBUG.print(rxBuf[0] == 0x80);
      DEBUG.println();
    }
  }

  // if CAN bridge is active. 
  // Send all CAN Rx frames to other CAN channel
  // If not set then CAN timeout DTC is set and vehicle will work in fail safe mode
  if (canBridge) {
    if (chNum == 1) {
      sendFrame(&CAN2, canId, extId, len, rxBuf);
    } else if (chNum == 2) {
      sendFrame(&CAN1, canId, extId, len, rxBuf);
    }
  }



  
//
//  if (((rxId & 0x1FFF00FF) == 0x18DA0010) || ((rxId & 0x1FFFFFFF) == 0x18DB3310) || (rxId == obdCanIdStd_ECU)) {
//    pidRead(&sidpr, &pId, &payloadSize, rxObdBuf, rxBuf, &currPos, &finish);
//
//    if (finish) {
//      pidMng(&sidpr, &pId, &payloadSize, rxObdBuf);
//      waitingObdReply = false;
//    }
//  }
//
//  if (!finish) {
//    if (askNoxObd) {
//      sendFrame(&CAN1, obdCanIdStd, false, 8, flowStatusFrame);
//    }
//  }

//  dtcMngRx(rxId, len, rxBuf);
}


bool sendFrame(MCP_CAN* canCh, uint32_t id, uint8_t ext, uint8_t len, uint8_t *payload) {

  // Check CAN channel
  uint8_t chNum = 0;
  if (canCh == &CAN1) {
    chNum = 1;
  } else if (canCh == &CAN2) {
    chNum = 2;
  }


  // When there is hi load of CAN traffic the CAN frame ID arbitration may be lost
  // This is a small work arround to try 20 times
  // If fail after 20 tries then frame is lost and will not be send
  bool result = false;
  uint8_t intento = 0;
  const uint8_t MAX_INTENTOS = 20;
  
  while((intento <= MAX_INTENTOS) && (!result)){
    byte sndStat = canCh->sendMsgBuf(id, ext, len, payload);
    if (sndStat != CAN_OK) {
      intento = intento + 1;
    } else {
      result = true;
    }
  }

  // Print UART error when frame can not be send after 20 attempts
  if (!result){
      Serial.print(F("ERROR SendFrame: x"));
      Serial.print(id, HEX);
      Serial.print(F(" ch: "));
      Serial.print(chNum);
      Serial.print(F(" rty: "));
      Serial.println(intento);
  }


  // UART/Serial Print information for frame send
  // Only print if propper flag is set (can be changed from UART menu)
  
  if (canPrint || (can1Print && (chNum == 1)) || (can2Print && (chNum == 2))) {
    
    // Print timestamp:
    sprintf(msgString, "%f\t", (float)millis() / 1000);
    DEBUG.print(msgString);

    // Print frame ID
    // There are 2 types of frame. Standar and extended.
    // Difference is bit size
    if (ext) {
      // Extended CAN ID:
      sprintf(msgString, "%d\tx%.8lX\tTx\t", chNum, (id & 0x1FFFFFFF));
    } else {
      // Standard CAN ID:
      sprintf(msgString, "%d\t%.3lX\tTx\t", chNum, id);
    }
    DEBUG.print(msgString);

    // Check RTR bit and determine if message is a Remote Request Frame.
    // datatype:
    // r == remote_frame
    // d == datatype
    if ((id & 0x40000000) == 0x40000000) {
      sprintf(msgString, "r\t%d\t", len);
      DEBUG.print(msgString);
    } else {
      sprintf(msgString, "d\t%d\t", len);
      DEBUG.print(msgString);

      // Print CAN frame payload to UART/Serial
      for (byte i = 0; i < len; i++) {
        if (payload[i] < 0x10) DEBUG.print('0');
        DEBUG.print(payload[i], HEX);
        DEBUG.print(" ");
      }
    }
    DEBUG.println();
  }
  return result;
}

// Function to print Rx frame to DEBUG port (Serial/UART)
void printAsc(uint8_t chNum, uint8_t extId) {
  
  // Print timestamp:
  sprintf(msgString, "%f\t", (float)millis() / 1000);
  DEBUG.print(msgString);

  // Print frame ID
  // There are 2 types of frame. Standar and extended.
  // Difference is bit size
  if (extId) {
    // Extended CAN ID:
    sprintf(msgString, "%d\tx%.8lX\tRx\t", chNum, (rxId & 0x1FFFFFFF));
  } else {
    // Standard CAN ID:
    sprintf(msgString, "%d\t%.3lX\tRx\t", chNum, rxId);
  }
  DEBUG.print(msgString);

  // Check RTR bit and determine if message is a remote request frame.
  // datatype:
  // r == remote_frame
  // d == datatype
  if ((rxId & 0x40000000) == 0x40000000) {
    sprintf(msgString, "r\t%d\t", len);
    DEBUG.print(msgString);
  } else {

    sprintf(msgString, "d\t%d\t", len);
    DEBUG.print(msgString);

    // Print CAN frame payload to UART/Serial
    for (byte i = 0; i < len; i++) {
      if (rxBuf[i] < 0x10) DEBUG.print('0');
      DEBUG.print(rxBuf[i], HEX);
      DEBUG.print(" ");
    }
  }
  DEBUG.println();
}


// Arbitrary frame definition to test connection of MCP2515.
// Just for testing
void sendTestFrame(void) {
  
  byte testData[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
  
  // send data:  ID = 0x555, Standard CAN Frame, Data length = 8 bytes, 'testData' = array of data bytes to send
  sendFrame(&CAN1, 0x555, 0, 8, testData);
  sendFrame(&CAN2, 0x555, 0, 8, testData);
  
}


MCP_CAN* getCAN1(void){
  return &CAN1;
}


MCP_CAN* getCAN2(void){
  return &CAN2;
}
