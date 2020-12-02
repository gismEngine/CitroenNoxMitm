// ESP32 Citroen Jumper NOx Emulator
// Target to read and modify NOx sensor data from
// Citroen Jumper by using web interface

// Citroen Jumper NOx PIN-OUT:

// NOx connector: ( 1 2 3 4 5 ]
// PIN 1: 12V 
// PIN 2: CAN_H
// PIN 3: CAN_L
// PIN 4: GND
// PIN 5: GND
 
// ESP32 with 2 CAN Channel input
// This is NOT using of ESP32 internal CAN-bus HW.
// 2 MCP2515 Board used instead

// ESP32 prints by UART (DEBUG) CAN traffic as ASC file format
// When bridge flag is set ('b' command on UART):
// CAN traffic from CAN1 is send to CAN2 and vice versa

// Default SPI pins on ESP32
// MOSI:  D23
// MISO:  D19
// SCK:   D18

// MCP2515 Board connected to VIN (5v)
// ESP32 is NOT officialy 5V input tolerant
// however in practive, IT IS (apparently)
// MOSI and MISO can be connected dirrectly from ESP32 and MCP2515 Board :)


//  Citroen Jumper NOx Frames:
//  0x264 - NOx Engine side (PreCata-Nox)
//  0x284 - NOx Exhaust side (PostCata-Nox)
 
// Different NOx and O2 modifications can be done thru web interface
//
// - CONV_FIX will fix NOX and O2 values to a fixed value 
//            Fix value can be changed on web interface
//
// - CONV_PROP will change NOX and O2 values multiplying to float value  
//            Proportional value can be changed by web interface
//
// - CONV_MAP will change NOX and O2 values by interpolating receibed values on defined MAPs  
//            MAPs input and outputs can be seen live on web interface
//
//            CONV_MAP Not fully implemented yet
//            MAP interpolation is done OK
//            MAP interpolation interface (functions are done)
//            TODO: MAP definition (size and values)
//            TODO: MAP inputs definition (input values for X and Y)

// Aparenlty NOx sensors sends random values before drew point is reached
// When drew point is reahed byte 1 and 0 are set to 0x80
// Flag to print drew potint reached (temperature reached) can be changed by UART
