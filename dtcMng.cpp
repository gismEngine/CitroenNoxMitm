#include <ArduinoJson.h>
#include "dtcMng.h"

const uint32_t TIMEOUT = 5000;
const uint32_t MULTIFRAME_SIZE = 70;
uint32_t lastRequestTime = 0;

MCP_CAN* can1;
MCP_CAN* can2;

uint32_t ecuIdIndex = 0;
bool ecuIdFound = false;
bool ecuIdWaiting = false;

bool flowControlRequest = false;

// Citroen --> Tool: 0x18DA10F1 Ecu: 0x18DAF110
uint32_t ecuIdTx[] = {0x7E0, 0x456, 0x18DA10F1, 0x18DAF11B};
uint32_t ecuIdRx[] = {0x7E8, 0x456, 0x18DAF110, 0x18DAF11A};

//  0x01 defaultSession
//  0x02 programmingSession
//  0x03 extendedDiagnosticSession
byte frameChangeSessionDev[] = {0x02, 0x10, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

byte frameReadDtc[] = {0x03, 0x19, 0x02, 0x2C, 0xFF, 0xFF, 0xFF, 0xFF};

byte frameClearDtc[] = {0x04, 0x14, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
byte clearObdDtc[] = {0x01, 0x04, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

byte flowControlFrame[] =  {0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

byte dtcTestFrame[] =  {0x11, 0x33, 0x55, 0x77, 0x99, 0x12, 0x34, 0x56};

bool readDtcRequest = false;
bool clearDtcRequest = false;
bool clearObdDtcRequest = false;

uint8_t multiframe[MULTIFRAME_SIZE];
uint8_t multiframeIndex = 0;
uint8_t multiframeSize = 0;
uint8_t multiframeResponseId = 0;

void dtcMngBegin(MCP_CAN* canA, MCP_CAN* canB){
  can1 = canA;
  can2 = canB;
}

void doTestDtc(void){
  bool sendOk = sendFrame(can1, ecuIdTx[2], true, sizeof(frameChangeSessionDev) / sizeof(frameChangeSessionDev[0]), frameChangeSessionDev);

  Serial.print("Do Test DTC: ");
  Serial.println(sendOk);
}

void dtcMng(void){
  
  if (!ecuIdFound && (readDtcRequest || clearDtcRequest)){
    findEcuId();
  }
  
  if (readDtcRequest & ecuIdFound){
    bool sendOk = sendFrame(can1, ecuIdTx[ecuIdIndex], ecuIdTx[ecuIdIndex] > 0x7ff, sizeof(frameReadDtc) / sizeof(frameReadDtc[0]), frameReadDtc);
    if (sendOk) readDtcRequest = false;
  }

  if (clearDtcRequest & ecuIdFound){
    bool sendOk = sendFrame(can1, ecuIdTx[ecuIdIndex], ecuIdTx[ecuIdIndex] > 0x7ff, sizeof(frameClearDtc) / sizeof(frameClearDtc[0]), frameClearDtc);
    if (sendOk){
      clearDtcRequest = false;
      lastRequestTime = millis();
    }
  }

  if (clearObdDtcRequest & ecuIdFound){
    if (millis() > lastRequestTime + 100){
      bool sendOk = sendFrame(can1, 0x700, false, sizeof(clearObdDtc) / sizeof(clearObdDtc[0]), clearObdDtc);
      if (sendOk) clearObdDtcRequest = false;
    }
  }


  if (flowControlRequest & ecuIdFound) {
    bool sendOk = sendFrame(can1, ecuIdTx[ecuIdIndex], ecuIdTx[ecuIdIndex] > 0x7ff, sizeof(flowControlFrame) / sizeof(flowControlFrame[0]), flowControlFrame);
    if (sendOk) flowControlRequest = false;
  }
  
}


void dtcMngRx(unsigned long rxId, uint8_t len, uint8_t rxBuf[]){

  if (ecuIdWaiting){
    if ((rxId & 0x1FFFFFFF) == ecuIdRx[ecuIdIndex - 1]){      
      if ((rxBuf[0] = 0x50) && (rxBuf[1] = 0x03)){
        ecuIdFound = true;
        ecuIdIndex = ecuIdIndex - 1;
        ecuIdWaiting = false;
        Serial.println(F("ECU ID FOUND!"));
        return;
      }
    }
  }

  if(ecuIdFound){
    if ((rxId & 0x1FFFFFFF) == ecuIdRx[ecuIdIndex]){
      if(rxBuf[0] == 0x7F){
        Serial.println("0x7F - Negative response");
        
      }else if(rxBuf[0] < 0x10){
        // Not a multiframe. Use multiframe buffer anyway
      
        multiframeSize = rxBuf[0];
        multiframeIndex = 0;

        multiframeResponseId = rxBuf[1];
        
        if(multiframeIndex + len < MULTIFRAME_SIZE){        // Check buffer overflow
          if (multiframeIndex < multiframeSize){            // Check multiframe size reached
            for (uint8_t i = 0; i < multiframeSize - 1; i++) {
                multiframe[multiframeIndex] = rxBuf[i + 2];
                multiframeIndex = multiframeIndex + 1;
            }
          }
        }

        if (multiframeIndex == multiframeSize - 1){
          Serial.println(F("dtcMng: Single frame"));
          processMultiframe();
        }
      
      }else if(rxBuf[0] == 0x10){

        if (rxBuf[1] > 2){
          multiframeSize = rxBuf[1];
        }
        
        multiframe[0] = rxBuf[3];
        multiframe[1] = rxBuf[4];
        multiframe[2] = rxBuf[5];
        multiframe[3] = rxBuf[6];
        multiframe[4] = rxBuf[7];
        multiframeIndex = 4;

        flowControlRequest = true; // Request to send flow control frame
                                   // Expected to be send after first frame of multiframe
      }else{
        uint8_t expectedFrameId = 0x21 + ((multiframeIndex - 4) / 7);
        if (expectedFrameId != rxBuf[0]){
          multiframeIndex = 0;
          Serial.println(F("ERROR -  dtcMng: MF wrong index"));
          return;
        }

        if(multiframeIndex + len < MULTIFRAME_SIZE){        // Check buffer overflow
          if (multiframeIndex < multiframeSize){            // Check multiframe size reached
            for (uint8_t i = 1; i < len; i++) {
                multiframeIndex = multiframeIndex + 1;
                multiframe[multiframeIndex] = rxBuf[i];
            }
          }
        }else{
          multiframeIndex = 0;
        }

        if (multiframeIndex == multiframeSize){
          Serial.println(F("dtcMng: End of MF"));
          //printMultiframeDebug();
          processMultiframe();
        }else if (multiframeIndex > multiframeSize){
          Serial.print(F("ERROR -  dtcMng: MF bigger. "));
          Serial.print(multiframeIndex);
          Serial.print(F(" vs "));
          Serial.println(multiframeSize);
        }
        
      }
    }
  }

}

void processMultiframe(void){
  if (multiframeResponseId == 0x59){
    Serial.print(F("reportType: "));
    Serial.println(multiframe[0], HEX);
    Serial.print(F("Mask: "));
    Serial.println(multiframe[1], HEX);
    
    uint8_t i = 2;
    while (i + 3 <= multiframeIndex){
      Serial.print(F("DTC: "));
      Serial.print(multiframe[i], HEX);
      Serial.print(multiframe[i + 1], HEX);
      Serial.print(multiframe[i + 2], HEX);
      Serial.print(F(" - "));
      Serial.println(multiframe[i + 3], HEX);
      i = i + 4;
    }
  }else{
    printMultiframeDebug();
  }
  multiframeIndex = 0;
}

void printMultiframeDebug(void){
  for (uint8_t i = 0; i <= multiframeIndex; i++) {
      Serial.print(multiframe[i], HEX);
      Serial.print(" ");
  }
  Serial.println();
}

void doDtcRead(void){
  readDtcRequest = true;
  ecuIdIndex = 0;
  Serial.println("read");
}

void doDtcClear(void){
  clearDtcRequest = true;
  clearObdDtcRequest = true;
  ecuIdIndex = 0;
  Serial.println("clear");
}

void findEcuId(void){
  
  if (lastRequestTime + TIMEOUT < millis()){

    if (ecuIdIndex > ((sizeof(ecuIdTx) / sizeof(ecuIdTx[0])) - 1)){
        readDtcRequest = false;
        clearDtcRequest = false;

        Serial.println("Ecu ID search FAIL! try in another castle");
        return;
    }
   
    bool sendOk = sendFrame(can1, ecuIdTx[ecuIdIndex], ecuIdTx[ecuIdIndex] > 0x7ff, sizeof(frameChangeSessionDev) / sizeof(frameChangeSessionDev[0]), frameChangeSessionDev);
    if (sendOk){
      ecuIdWaiting = true;
      ecuIdIndex = ecuIdIndex + 1;
    
      lastRequestTime = millis();
    }
    
  }  
}







 
