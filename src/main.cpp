#include <Arduino.h>
// #include <struct.cpp>
#include <define.cpp>
#include <globalvars.cpp>

#include <struct.cpp>


// *****  Global Variables *****

incomingLKASMessage incomingMsg;

uint8_t lkas_off_array[][4] =  { {0x00,0x80,0xc0,0xc0}, {0x20,0x80,0xc0,0xa0} };
uint8_t eps_off_array[][5] = {
	{B00001111, B10011110, B11000000, B10000000, B10010011},  
	{B00101111, B10011110, B11000000, B10000000, B11110011} //    
};

//0x20= B‭0010000 0x80= ‭10000000‬0 0xc0=‭11000000‬ 0xa0= ‭10100000‬
//0x00 = 00000000  0x80= ‭10000000‬0 0xc0=‭11000000‬  0xc0=‭11000000‬
int16_t applySteer_global = 0;  //TODO:change this

const uint8_t forceLKASPin = 0;	
const uint8_t analogRotaryInputPin = 20;
uint16_t analogRotaryInputCenter = 450;

uint8_t PB1_spoofLKASLeftDigitalRead = 1;
uint8_t PB2_spoofLKASRightDigitalRead = 1;

uint32_t lastDigitalReadTime = 0;


//if MCU LKAS is on, force the left torque to the forceLeftApplyTorque 
const int forceLeftPin = 10;
//if MCU LKAS is on, force the left torque to the forceRightApplyTorque 
const int forceRightPin = 11;

int16_t forceLeftApplyTorque = -30;
int16_t forceRightApplyTorque = 40;

uint8_t DIP1_spoofFullMCUDigitalRead = 0;

uint8_t EPStoLKASBuffer[5];
uint8_t EPStoLKASBufferCounter = 0;

uint8_t counterbit = 0;
uint8_t chksm(uint8_t,uint8_t,uint8_t);

uint32_t readLEDblinkLastChange = 0;
uint8_t LKASFrameSentByCreateLinMessage = 0; // check to show if the current LKAS frame was sent by the CreateLinMessage function, if it was
											// do not relay the rest of the frame
//used to keep the last EPStoLKAS frame sent when Full MCU spoofing is occuring. sent when the 5th byte of the EPStoLKAS frame is received from the EPS
uint8_t EPStoLKASLastFrameSent[4]; 

uint8_t nextCounterBit = 0;
 





// 			*****  FUNCTION DECLARATIONS *****

void printuint_t(uint8_t var);
void createKLinMessage(int16_t applySteer);
uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte);
void  deconstructLKASMessage(uint8_t msg);
void handleEPStoLKAS();
void handleDigitalReads();
void sendArrayToSerial(HardwareSerial serial,uint8_t *array,uint8_t arraySize);
void handleMCUtoLKASSpoofMCU(uint8_t rcvdByte);
void handleLKAStoMCUSpoofMCU(uint8_t rcvdByte);


// 				*****	FUNCTIONS ***** 

void createKLinMessage(int16_t applySteer){
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
	msg[0] = (incomingMsg.counterBit << 5) |  bigSteer;
	
	LKAStoEPS_Serial.write(msg[0]);
	
	//get the little steer in the 2nd byte. its the last 5 bits of applysteer. also add in the 0xA0 = 1010 0000
	uint8_t littleSteer = (uint8_t) applySteer & 0x1F; // 0x1F = 0001 1111 ... Change only the last 5 bits
	msg[1] = littleSteer | 0xA0;  // 1010 0000
	//								   ^ lkas on
	msg[2] = 0x80; // this is static 0x80 = 1000 0000
	
	msg[3] = chksm(msg[0], msg[1], msg[2]);

	LKAStoEPS_Serial.write(msg[1]);
	LKAStoEPS_Serial.write(msg[2]);
	LKAStoEPS_Serial.write(msg[3]);
	outputSerial.write('\n');
	outputSerial.print("C-");

	printuint_t(msg[0]);
	outputSerial.write(' ');
	printuint_t(msg[1]);
	outputSerial.write(' ');
	printuint_t(msg[2]);
	outputSerial.write(' ');
	printuint_t(msg[3]);
}

uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte){
	uint16_t local = firstByte + secondByte + thirdByte ;
	local = local % 512;
	local = 512 - local;
	return (uint8_t)(local % 256);
}

void  deconstructLKASMessage(uint8_t msg){ //
	//figure out which message byte your in.. check if first 2 bits are 0's
	if(incomingMsg.totalCounter ==0){ // confirm you are receving the first bit

		if((msg >> 6) != 0) return; 
	}
	incomingMsg.data[incomingMsg.totalCounter] = msg;
	switch (incomingMsg.totalCounter ){
		case 0:
			incomingMsg.counterBit = msg >> 5;
			incomingMsg.bigSteer = msg & B00001111;
			break;
		case 1:
			incomingMsg.littleSteer = msg & B00011111;
			incomingMsg.lkasOn = (msg >> 5) & B00000001;
			break;
		case 2:

			break;

		case 3:
			incomingMsg.checksum = msg;
			break;
	}
	incomingMsg.totalCounter++;
} // deconstructLKASMessage(uint8_t msg)

void handleEPStoLKAS(){
	if(EPStoLKAS_Serial.available()){
		uint8_t mydata = EPStoLKAS_Serial.read();
		if(!DIP1_spoofFullMCUDigitalRead){
			if((mydata >> 6) == 0){
				EPStoLKASBufferCounter = 0;
				EPStoLKAS_Serial.write(eps_off_array[nextCounterBit][0]);
				EPStoLKAS_Serial.write(eps_off_array[nextCounterBit][1]);
				EPStoLKAS_Serial.write(eps_off_array[nextCounterBit][2]);
				EPStoLKAS_Serial.write(eps_off_array[nextCounterBit][3]);
				EPStoLKAS_Serial.write(eps_off_array[nextCounterBit][4]);
			}
		} else{
			EPStoLKAS_Serial.write(mydata);
		}
		EPStoLKASBuffer[EPStoLKASBufferCounter++] = mydata;
		if(EPStoLKASBufferCounter == 5){ // this indicates the end of the LKAStoEPS and EPStoLKAS exchange
			nextCounterBit = !incomingMsg.counterBit;
			outputSerial.print("  ^  ");
			printuint_t(EPStoLKASBuffer[0]);
			outputSerial.print("  ");
			printuint_t(EPStoLKASBuffer[1]);
			outputSerial.print("  ");
			printuint_t(EPStoLKASBuffer[2]);
			outputSerial.print("  ");
			printuint_t(EPStoLKASBuffer[3]);
			outputSerial.print("  ");
			printuint_t(EPStoLKASBuffer[4]);
			// outputSerial.write("\n");
		}
	}
} // handleEPStoLKAS()

void handleLKAStoEPS(){
	if(LKAStoEPS_Serial.available()){
		uint8_t rcvdByte = LKAStoEPS_Serial.read();
		deconstructLKASMessage(rcvdByte);
		uint8_t invertDIP1_spoofFullMCUDigitalRead = !DIP1_spoofFullMCUDigitalRead;
		switch(invertDIP1_spoofFullMCUDigitalRead){
			case 0: // do not spoof MCU... relay or modify
				handleMCUtoLKASSpoofMCU(rcvdByte);
				break;
			case 1: // spoof MCU
				handleLKAStoMCUSpoofMCU(rcvdByte);
		}

		if((rcvdByte >> 6) == 0){ //its the first byte
			incomingMsg.totalCounter = 0;
			
			if(!DIP1_spoofFullMCUDigitalRead){
				if(!PB1_spoofLKASLeftDigitalRead)  {
					createKLinMessage(forceLeftApplyTorque);
					forceLeftApplyTorque = random(-50,-25);
				}
				else if(!PB2_spoofLKASRightDigitalRead) {
					createKLinMessage(forceRightApplyTorque);
					forceRightApplyTorque = random(30,50);
				}
				else {
					uint8_t localCounterBit = 0;
					sendArrayToSerial(EPStoLKAS_Serial,&lkas_off_array[incomingMsg.counterBit][0], 4); //change to nextCounterBit
				}
			}
			
			if(!PB1_spoofLKASLeftDigitalRead && incomingMsg.lkasOn){
				createKLinMessage(forceLeftApplyTorque);
				LKASFrameSentByCreateLinMessage = 1;
				forceLeftApplyTorque = random(-50,-25);
			}else if(!PB2_spoofLKASRightDigitalRead && incomingMsg.lkasOn){
				createKLinMessage(forceRightApplyTorque);
				forceRightApplyTorque = random(30,50);
				LKASFrameSentByCreateLinMessage =1 ;
			} else {
				LKAStoEPS_Serial.write(rcvdByte);
				LKASFrameSentByCreateLinMessage = 0;
				outputSerial.print("\nR-");
				printuint_t(rcvdByte);
				outputSerial.write(' ');
			}
			EPStoLKASBufferCounter = 0;
		} else { //  if first byte of frame
			if(!LKASFrameSentByCreateLinMessage) {
				LKAStoEPS_Serial.write(rcvdByte);
				printuint_t(rcvdByte);
				outputSerial.write(' ');
				// deconstructLKASMessage(mydata);
			}
		} // end else
	}
} // handleLKAStoEPS()

void handleDigitalReads(){
	if( ( millis() - lastDigitalReadTime ) > TIME_BETWEEN_DIGITIAL_READS){
		PB1_spoofLKASLeftDigitalRead = digitalRead(PB1_spoofLKASLeft);
		PB2_spoofLKASRightDigitalRead = digitalRead(PB2_spoofLKASRight);
		// outputSerial.print("reading Digital ");
		// outputSerial.print(PB1_spoofLKASLeftDigitalRead,DEC);
		// outputSerial.print("  ");
		// outputSerial.print(PB2_spoofLKASRightDigitalRead,DEC);
		// outputSerial.print(" \n");
		if((!PB1_spoofLKASLeftDigitalRead) || (!PB2_spoofLKASRightDigitalRead)){
			digitalWrite(BLUE_LED,HIGH);
		} else digitalWrite(BLUE_LED,LOW);
		DIP1_spoofFullMCUDigitalRead = digitalRead(DIP1_spoofFullMCU);
	} // end if true
}


void printuint_t(uint8_t var) { ///i stole this from my program https://github.com/reddn/LIN2LINrepeaterMEGA, i think it should work
  for (uint8_t test = 0x80; test; test >>= 1) {
  	if(test == 0x8) outputSerial.write(' '); // 0x8 is 0000 1000.. so it gives a break between the 2 4 bit writes
    outputSerial.write(var  & test ? '1' : '0');
  }
}

void printLKASArray(uint8_t *msg){ //msg is &array[0]
	for(char zz = 0; zz<4; zz++){
		printuint_t(*(msg+zz));
		if(zz != 3) outputSerial.print("  ");
	}
}

void printArrayInBinary(uint8_t *msg,uint8_t size){ //msg is &array[0]  .. size is sizeof(array)
	for(char zz = 0; zz<size; zz++){
		if(zz != 0) outputSerial.print("  ");
		printuint_t(*(msg+zz));
	}
}

void printEPSArray(uint8_t *msg){ //msg is array[0]
	for(char zz = 0; zz<5; zz++){
		printuint_t(*(msg+zz));
		if(zz != 4) outputSerial.print("  ");
	}
}

void sendArrayToSerial(HardwareSerial serial,uint8_t *array,uint8_t arraySize){
	serial.write(*array);
	serial.write(*(array+1));
	serial.write(*(array+2));
	serial.write(*(array+3));
	if(arraySize == 5) serial.write(*(array+4));
}

void handleLKAStoMCUSpoofMCU(uint8_t rcvdByte){

}

void handleMCUtoLKASSpoofMCU(uint8_t rcvdByte){
	
}

// 			****** 				SETUP 					******
void setup(){	
	EPStoLKAS_Serial.begin(9600,SERIAL_8E1);
	LKAStoEPS_Serial.begin(9600,SERIAL_8E1);
	outputSerial.begin(OUTPUTSERIAL_BAUD);
	pinMode(analogRotaryInputPin,INPUT);
	pinMode(PB1_spoofLKASLeft, INPUT_PULLUP);
	pinMode(PB2_spoofLKASRight, INPUT_PULLUP);
	pinMode(PB3_spoofLKASStop, INPUT_PULLUP);
	pinMode(DIP1_spoofFullMCU, INPUT_PULLUP);
	pinMode(BLUE_LED,OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
}

void loop(){
	handleEPStoLKAS();
	handleLKAStoEPS();
	handleDigitalReads();
	if((millis() - readLEDblinkLastChange) > 2000){
		// outputSerial.println("LED change");
		digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
		readLEDblinkLastChange = millis();
	} 
}