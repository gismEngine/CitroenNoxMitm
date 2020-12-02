#ifndef dtcmng_h
#define dtcmng_h

  #include "Arduino.h"
  #include "canMng.h"

  void dtcMngBegin(MCP_CAN* canA, MCP_CAN* canB);
  void dtcMng(void);
  void dtcMngRx(unsigned long rxId, uint8_t len, uint8_t rxBuf[]);
  
  void doTestDtc(void);

  void doDtcRead(void);
  void doDtcClear(void);

  void findEcuId(void);

  // Intended private
  void processMultiframe(void);
  void printMultiframeDebug(void);

#endif
