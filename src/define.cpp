// #include <Arduino.h>

//INFO
// CTX3 31
// CRX3 30
// CTX1 22
// CRX1 23
// EPS TO LKAS TX >> UART2 RX
// EPS TO LKAS TX1 >> UART2 TX
// LKAS TO EPS TX >> UART3 RX
// LKAS TO EPS TX1 >> UART3 TX


#define LKAStoEPS_Serial Serial3

#define EPStoLKAS_Serial Serial2
#define outputSerial Serial
// #define LKAStoEPS_MCU_Serial Serial4 // not sure why i defined these (MCU_Serial)... took them out to stop predictive text
// #define EPStoLKAS_MCU_Serial Serial5 
#define PB1_spoofLKASLeft 2
#define PB2_spoofLKASRight 3
#define PB3_spoofLKASStop 4
#define DIP1_spoofFullMCU 6

#define DIP1 6
#define DIP2 32
#define DIP3 33
#define DIP4 9
#define DIP5 10
#define DIP6 11
#define DIP7 12



#define OUTPUTSERIAL_BAUD 921600

#define BLUE_LED 28 //Used for signifying "manipulated" data
#define TIME_BETWEEN_DIGITIAL_READS 250

#define EPStoLKASCanMsgId 0x201
#define LKAStoEPSCanMsgId 0x200