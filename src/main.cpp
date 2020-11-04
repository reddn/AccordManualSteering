#include <Arduino.h>

//INFO
// CTX3 31
// CRX3 30
// CTX1 22
// CRX1 23
// EPS TO LKAS TX >> UART2 RX
// EPS TO LKAS TX1 >> UART2 TX
// LKAS TO EPS TX >> UART3 RX
// LKAS TO EPS TX1 >> UART3 TX

uint8_t lkas_off_array[][4] =  { {0x20,0x80,0xc0,0xa0},{0x00,0x80,0xc0,0xc0} };
//0x20= B‭0010000 0x80= ‭10000000‬0 0xc0=‭11000000‬ 0xa0= ‭10100000‬
//0x00 = 00000000  0x80= ‭10000000‬0 0xc0=‭11000000‬  0xc0=‭11000000‬
int16_t applySteer = 0;

const uint8_t forceLKASPin = 0;	
const uint8_t analogRotaryInputPin = 20;
uint16_t analogRotaryInputCenter = 450;

#define LKAStoEPS_Serial Serial1
#define EPStoLKAS_Serial Serial2
#define outputSerial Serial3
#define LKAStoEPS_MCU_Serial Serial4
#define EPStoLKAS_MCU_Serial Serial5 

#define OUTPUTSERIAL_BAUD 921600


uint8_t counterbit = 0;
uint8_t chksm(uint8_t,uint8_t,uint8_t);

void createKLinMessage(){
	uint8_t msg[4];
	msg[0] = 0;
	msg[1] = 0;
	msg[2] = 0;
	msg[3] = 0;
	//max is 255, min is -256 in the apply steer PID, if its outside of that (it shouldnt be), then put it 
	//back at 255 or -256
	if (applySteer > 255) applySteer = 255; // 0xFF or B0 1111 1111
	else if(applySteer < -256) applySteer = -256; // B? 1111 1111
	// push the 16 bit applySteer over 5 so you can get the 3MSB not inlucding the sign put it at the beginning of the bigSteer
	uint8_t bigSteer = 0x00;
	// extract the sign of apply steer (int16) and push it over to the 3rd offset, where it would be in the 2nd byte of the frame
	uint8_t bigSteerSign = ( (uint8_t) applySteer >> 12 ) & 0x8 ; //0x8 = 0000 1000
	//get the bigsteer (3 MSB, exlcuding the sign bit) and add(OR) the sign
	bigSteer =  ( (applySteer >> 5) & 0x7 ) | bigSteerSign;  // 0x7 = 0000 0111  
	msg[0] = (counterbit << 5) |  bigSteer;
	
	//get the little steer in the 2nd byte. its the last 5 bits of applysteer. also add in the 0xA0 = 1010 0000
	uint8_t littleSteer = (uint8_t) applySteer & 0x1F; // 0x1F = 0001 1111 ... Change only the last 5 bits
	msg[1] = littleSteer | 0xA0;
	
	msg[2] = 0x80; // this is static 0x80 = 1000 0000
	
	msg[3] = chksm(msg[0], msg[1], msg[2]);
}


uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte){
	uint16_t local = firstByte + secondByte + thirdByte ;
	local = local % 512;
	local = 512 - local;
	return (uint8_t)(local % 256);
}

void setup(){
	// EPStoLKAS_Serial.begin(9600,8E1);
	// LKAStoEPS_Serial.begin(9600,8E1);
	outputSerial.begin(OUTPUTSERIAL_BAUD);
	pinMode(analogRotaryInputPin,INPUT);
	pinMode(forceLKASPin, INPUT); 
}


void loop(){
	
}