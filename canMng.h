#ifndef canMng_h
#define canMng_h

  #include <Arduino.h>

  #include <mcp_can.h>
  #include <SPI.h>

  #include "sockMng.h"
  #include "dtcMng.h"

  #define CAN1_INT 15
  #define CAN2_INT 21
  
  #define CAN1_CS 4
  #define CAN2_CS 5

  #define NOX_REQ_TIME 200

  #define DEBUG Serial


  extern bool canBridge;
  extern bool canPrint;
  extern bool can1Print;
  extern bool can2Print;
  
  extern bool printTempReached;
  
  extern bool finish;
  extern bool waitingObdReply;
  
  extern bool askSupportedPid;
  extern uint8_t askSupportedPidCount;
  
  extern bool askNoxObd;
  extern uint32_t askNoxTime;
  
  extern enum conversionMode convModeNox_main;
  extern enum conversionMode convModeO2_main;
  extern uint32_t nox_fix_bypass_main;
  extern uint32_t o2_fix_bypass_main;
  extern float nox_prop_main;
  extern float o2_prop_main;

  extern uint32_t testFrameTime;

  // Intended pubic
  void canBegin();
  void canRxMng(void);
  void canTxMng(void);

  MCP_CAN* getCAN1(void);
  MCP_CAN* getCAN2(void);

  // Intended private
  void canChRxMng(MCP_CAN* canCh);
  bool sendFrame(MCP_CAN* canCh, uint32_t id, uint8_t ext, uint8_t len, uint8_t *payload);
  void printAsc(uint8_t chNum, uint8_t extId);
  void sendTestFrame(void);

#endif
