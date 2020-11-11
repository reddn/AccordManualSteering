#include <Arduino.h>

struct incomingLKASMessage {
	uint8_t totalCounter = 0;
	uint8_t data[4];
	bool sent = false;
	uint8_t counterBit = 0;
	uint8_t bigSteer = 0;
	uint8_t littleSteer = 0;
	uint8_t lkasOn = 0;
	uint8_t checksum = 0;
};
