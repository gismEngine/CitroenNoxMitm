#ifndef map_h
#define map_h

  #include "Arduino.h"

  // Intended public
  void initMaps(void);
  void printBuffersSerial(void);

  float interpolateSimp(float x0, float y0, float x1, float y1, float x);
  float interpolateFull(uint32_t x, uint32_t y);

  // Intended private
  void testInterpolateSimp();
  void verboseInterpolateSimp(float x0, float y0, float x1, float y1, float x);
  void testInterpolateFull();

#endif
