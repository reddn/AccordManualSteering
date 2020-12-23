#include <Arduino.h>
#include <define.cpp>
#include <struct.cpp>
#include <FlexCAN_T4.h>
#include <globalvars.cpp>

// define DEBUG_SEND_TO_SERIAL 1
//define DEBUG_PRINT_LKAStoEPS_OUTPUT 1 
// #define DEBUG_PRINT_EPStoLKAS_TO_SERIAL 0
// #define DEBUG_PRINT_LKAStoEPS_ERRORS_SERIAL 0
// #define DEBUG_PRINT_LKAStoEPS_LIN_INPUT 1
#define DEBUG_PRINT_LKAStoEPS_LIN_OUTPUT 1
// #define DEBUG_PRINT_EPStoLKAS_LIN_OUTPUT 1
// #define DEBUG_PRINT_EPStoLKAS_LIN_INPUT 1
#define DEBUG_PRINT_EPStoLKAS_ERRORS 1
// #define DEBUG_PRINT_OPtoCAN_INPUT 1


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

// const uint8_t forceLKASPin = 0;	
uint16_t analogRotaryInputCenter = 450;

uint8_t PB1_spoofLKASLeftDigitalRead = 1;
uint8_t PB2_spoofLKASRightDigitalRead = 1;
uint8_t PB4_spoofLKASSteerWithPOTEnableDigitalRead = 1;

uint32_t lastDigitalReadTime = 0;

int16_t forceLeftApplyTorque = -30;
int16_t forceRightApplyTorque = 40;
int16_t forceApplyTorqueWithPOT = 0;


// Some DIP switches are hard coded to be on/off.. please check the readDigitalInputs function to see there.
uint8_t DIP1_spoofFullMCUDigitalRead = 1;
uint8_t DIP2_sendOPSteeringTorque = 1;
uint8_t DIP5_disableAllLinOutput = 1;
uint8_t DIP6_passSteeringWheelTorqueData = 1;
uint8_t DIP7_SpoofSteeringWheelTorqueData = 1;

uint8_t EPStoLKASBuffer[5];
uint8_t EPStoLKASBufferCounter = 0;

uint8_t counterbit = 0;
uint8_t chksm(uint8_t,uint8_t,uint8_t);
uint8_t chksm(uint8_t,uint8_t,uint8_t, uint8_t);
uint8_t chksm(uint8_t* data, uint8_t len);

uint32_t readLEDblinkLastChange = 0;
uint8_t LKASFrameSentByCreateLinMessage = 0; // check to show if the current LKAS frame was sent by the CreateLinMessage function, if it was
											// do not relay the rest of the frame
//used to keep the last EPStoLKAS frame sent when Full MCU spoofing is occuring. sent when the 5th byte of the EPStoLKAS frame is received from the EPS
uint8_t EPStoLKASLastFrameSent[4]; 

uint8_t nextCounterBit = 0;
 
int8_t LkasOnIntroCountDown = 5; // sends 5 frames of LKAS on and 0 apply steer.. the stock LKAS does this. but I dont think its needed

FlexCAN_T4<CAN1, RX_SIZE_512, TX_SIZE_128> FCAN;

CAN_message_t canMsg;

int A1_applySteeringPot = 0;

bool spoofSteeringWheelTorqueDataBool = false;

uint8_t spoofSteeringWheelTorqueData_Counter = 0;

bool OPLkasActive = false;
uint8_t OPBigSteer = 0;
uint8_t OPLittleSteer = 0;
unsigned long OPTimeLastCANRecieved = 0;

uint8_t EPStoLKASCanFrameCounter = 0;

uint8_t LkasFromCanCounter = 0;
uint8_t LkasFromCanCounterErrorCount = 0;

uint8_t LkasFromCanChecksumErrorCount = 0;

uint8_t LkasFromCanFatalError = 0;

uint8_t OPCanCounter = 0;

uint16_t mainLedBlinkTimer = 2000;

// 																		*****  FUNCTION DECLARATIONS  aka headers  *****

void printuint_t(uint8_t );
void printArrayInBinary(uint8_t*, uint8_t);

void createKLinMessage(int16_t );
uint8_t chksm(uint8_t , uint8_t , uint8_t );
uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte, uint8_t fourthByte);
void  deconstructLKASMessage(uint8_t );
void handleEPStoLKAS();
void handleInputReads();
void sendArrayToSerial(HardwareSerial ,uint8_t* ,uint8_t);

void handleLKAStoMCUSpoofMCU(uint8_t);
void handleEPStoLKASSpoofMCU(uint8_t);
void handleLKAStoEPSNoSpoof(uint8_t);
void handleEPStoLKASNoSpoof(uint8_t);

void sendEPStoLKASToCAN(uint8_t*);
void sendLKAStoEPSFromCAN(uint8_t*);// might not use this one
void handleLKASFromCan();
void createKLinMessageWBigSteerAndLittleSteer(uint8_t,uint8_t);

void sendArrayToEPStoLKASSerial(uint8_t*);
void sendArrayToLKAStoEPSSerial(uint8_t*);

void passEPStoLKASTorqueData(uint8_t );
void spoofSteeringWheelTorqueData(uint8_t );

void handleLkasFromCanV2(const CAN_message_t &msg);
void handleLkasFromCanV3();
void handleLKAStoEPSUsingOPCan();
void handleEPStoLKASOP(uint8_t rcvdByte);
void handleEPStoLKASKeepMcuHappy(uint8_t rcvdByte);

void buildSteerMotorTorqueCanMsg();
void buildSteerStatusCanMsg();

uint8_t honda_compute_checksum(uint8_t *steerTorqueAndMotorTorque, uint8_t size); // << From Comma.ai Panda safety_honda.h licensed unde MIT


// 	*	*	*	*	*	*	*	*	******	FUNCTIONS ***** 


//This function takes the signed int to make it into a valid LKAStoEPS message for steering.  
//Since everyting is triggered by the stock LKAS(MCU), the counter bits are synced on those messages.
// so if there is no LKAStoEPS message from the stock MCU, nothing will be sent by this device.  i could change this later, but probably not
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
	
	if(LkasOnIntroCountDown > 0){ // 
		applySteer = 0;
		LkasOnIntroCountDown--;
	}
	// push the 16 bit applySteer over 5 so you can get the 3MSB not inlucding the sign put it at the beginning of the bigSteer
	uint8_t bigSteer = 0x00;
	// extract the sign of apply steer (int16) and push it over to the 3rd offset, where it would be in the 2nd byte of the frame
	uint8_t bigSteerSign = 0; // >> old version.. doesn't work( (uint8_t) (applySteer >> 12 ) ) & 0x8 ; //0x8 = 0000 1000
	if(applySteer < 0 ) bigSteerSign = B00001000; 
	//get the bigsteer (3 MSB, exlcuding the sign bit) and add(OR) the sign
	bigSteer =  ( (applySteer >> 5) & 0x7 ) | bigSteerSign;  // 0x7 = 0000 0111  
	msg[0] = (incomingMsg.counterBit << 5) |  bigSteer;
	
	
	//get the little steer in the 2nd byte. its the last 5 bits of applysteer. also add in the 0xA0 = 1010 0000
	uint8_t littleSteer = (uint8_t) applySteer & 0x1F; // 0x1F = 0001 1111 ... Change only the last 5 bits
	msg[1] = littleSteer | 0xA0;  // 1010 0000
	//								   ^ lkas on
	msg[2] = 0x80; // this is static 0x80 = 1000 0000
	
	msg[3] = chksm(msg[0], msg[1], msg[2]);

	
		LKAStoEPS_Serial.write(msg[0]);
		LKAStoEPS_Serial.write(msg[1]);
		LKAStoEPS_Serial.write(msg[2]);
		LKAStoEPS_Serial.write(msg[3]);
	
#ifdef DEBUG_PRINT_LKAStoEPS_OUTPUT
	outputSerial.print("\nC-");
	printArrayInBinary(&msg[0],4);
#endif
}

// same as above 'createKLinMessage', but this accepts only the bigSteer and littleSteer as variables
void createKLinMessageWBigSteerAndLittleSteer(uint8_t bigSteer, uint8_t littleSteer){
	uint8_t msg[4];
	msg[0] = (incomingMsg.counterBit << 5) |  bigSteer;
	
	
	//get the little steer in the 2nd byte. its the last 5 bits of applysteer. also add in the 0xA0 = 1010 0000
	msg[1] = littleSteer | 0xA0;  // 1010 0000
	//								   ^ lkas on
	msg[2] = 0x80; // this is static 0x80 = 1000 0000
	//										 ^ lkas on (1 is off)
	msg[3] = chksm(msg[0], msg[1], msg[2]);


		LKAStoEPS_Serial.write(msg[0]);
		LKAStoEPS_Serial.write(msg[1]);
		LKAStoEPS_Serial.write(msg[2]);
		LKAStoEPS_Serial.write(msg[3]);

#ifdef DEBUG_PRINT_LKAStoEPS_LIN_OUTPUT
	outputSerial.print("\nL-O:");
	printArrayInBinary(&msg[0],4);
#endif
	CAN_message_t thisCanMsg;
	thisCanMsg.id = 0x200;
	thisCanMsg.len = 4;
	thisCanMsg.buf[0] = msg[0];
	thisCanMsg.buf[1] = msg[1];
	thisCanMsg.buf[2] = msg[2];
	thisCanMsg.buf[3] = msg[3];
	FCAN.write(thisCanMsg);
}


//used to figure out what the LKAStoEPS message contained.. mainly the 'counterbit' is the biggest deal on the first byte
void  deconstructLKASMessage(uint8_t msg){ //
	//figure out which message byte your in.. check if first 2 bits are 0's

	uint8_t offset4 = msg >> 4;
	if( offset4 < 4 ){
		incomingMsg.totalCounter = 0;
	} else {
		// outputSerial.write('\n');
		// printuint_t(msg);
		incomingMsg.totalCounter++;
	}
	if(incomingMsg.totalCounter > 3){ // ERROR!! this shouldnt happen
#ifdef DEBUG_PRINT_LKAStoEPS_ERRORS_SERIAL
		outputSerial.print("\nERROR:  incomingMsg.totalCounter is > 3 -- ");
		outputSerial.print(incomingMsg.totalCounter,DEC);
		outputSerial.println(" --   impossible  . resetting to 0 *** ");
#endif
		incomingMsg.totalCounter = 0;
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
} // deconstructLKASMessage(uint8_t msg)



// *	*	*	*	*	*	*	*	*	********** Handle  LKAS  to  EPS

//main loop function to check if any data was received on the stock LKAStoEPS line, then handle it spending on spoof settings 
void handleLKAStoEPS(){
	if(LKAStoEPS_Serial.available()){
		uint8_t rcvdByte = LKAStoEPS_Serial.read();
		deconstructLKASMessage(rcvdByte);

		if(!DIP2_sendOPSteeringTorque) handleLKAStoEPSUsingOPCan();// DIP 2 On
		else if(!DIP1_spoofFullMCUDigitalRead) handleLKAStoMCUSpoofMCU(rcvdByte); //DIP1 On  -  spoof MCU
		else if(DIP1_spoofFullMCUDigitalRead) handleLKAStoEPSNoSpoof(rcvdByte); //DIP1 Off -  do not spoof MCU.. relay or modify
#ifdef DEBUG_PRINT_LKAStoEPS_LIN_INPUT
		outputSerial.print("\nL-I:");
		outputSerial.print(incomingMsg.totalCounter,DEC);
		outputSerial.print("-");
		printuint_t(rcvdByte);
#endif
	} 

} // handleLKAStoEPS()

//if we are only forwarding the LKAStoEPS line, this is called.. ie, no spoofing.
//sends the data to the CAN bus if sendLKAStoEPSRxToCan is true, thsi is for OP to capture for later review
//although this function does print the data.. which probably should be bought out to its own function
void handleLKAStoEPSNoSpoof(uint8_t rcvdByte){
	
	LKAStoEPS_Serial.write(rcvdByte);
#ifdef DEBUG_PRINT_LKAStoEPS_ERRORS
	if(incomingMsg.totalCounter == 0) outputSerial.print("\nR-");
	printuint_t(rcvdByte);
	outputSerial.write(' ');	
#endif
	if(incomingMsg.totalCounter == 3){
		if(sendLKAStoEPSRxToCan){
			canMsg.id = LKAStoEPSLinDataRxMsgId;
			canMsg.len = 4;
			for(uint8_t zz =0 ; zz < 4; zz++){
				canMsg.buf[zz] = incomingMsg.data[zz];
			}
			FCAN.write(canMsg);
		}
	}
	
}


void handleLkasFromCanV2(){
	// BO_ 228 STEERING_CONTROL: 3 EON
//  SG_ STEER_TORQUE_REQUEST : 9|1@0+ (1,0) [0|1] "" EPS
//  SG_ STEER_TORQUE : 0|9@0- (1,0) [-256|255] "" EPS
//  SG_ COUNTER : 21|2@0+ (1,0) [0|3] "" EPS
//  SG_ CHECKSUM : 19|4@0+ (1,0) [0|15] "" EPS
	//new version
	// B B B S  S S S S
	// 0 0 0 0  0 0 R B
	// 0 0 C C  H H H H
	if(canMsg.id != 228) return;
#ifdef DEBUG_PRINT_OPtoCAN_INPUT
	outputSerial.print("\nCANmsg rcvd id: ");
	outputSerial.print(canMsg.id,DEC);
	outputSerial.print(":");
	for(uint8_t bb = 0; bb < canMsg.len; bb++){
		printuint_t(canMsg.buf[bb]);
	}
#endif 

	if((canMsg.buf[1]>> 1) == 1 ){ // if STEER REQUEST (aka LKAS enabled)
		OPLkasActive = true;
	} else {
		OPLkasActive = false;
	}

	uint8_t lclBigSteer = 0;
	uint8_t lclLittleSteer = 0;
	lclBigSteer = (canMsg.buf[1] & 0x01 ) << 3;
	lclBigSteer |= canMsg.buf[0] >> 5;

	lclLittleSteer = canMsg.buf[0] & B00011111;

	// TODO: verify counter is working
	uint8_t lclCounter = canMsg.buf[2] >> 4;
	bool counterVerified = false;  // need global counter   and counter error

	if(LkasFromCanCounter != lclCounter) LkasFromCanCounterErrorCount++;
	else LkasFromCanCounterErrorCount = 0;
	
	if(LkasFromCanCounter < 3) counterVerified = true;


	// TODO: verify checksum
	bool checksumVerified = false;

	if(honda_compute_checksum((uint8_t*) &canMsg.buf[0],3) == (canMsg.buf[3] & B00001111 )) LkasFromCanChecksumErrorCount = 0;
	else LkasFromCanChecksumErrorCount++;
	
	if(LkasFromCanCounterErrorCount < 3 ) checksumVerified = true;
	else checksumVerified = false;

	//canbus data time is checked in the handleLkastoEPS function, if no data has been received within 50ms . LKAS is not allowed to be active

	// set big/small steer in varible and that LKAS is on
	// so when its time to send a LKAS message, it just reads the data, make the checksum and send it
	if(counterVerified && checksumVerified){
		// createKLinMessageWBigSteerAndLittleSteer(lclBigSteer,lclLittleSteer);
		OPLkasActive = true;
		OPBigSteer = lclBigSteer;
		OPLittleSteer = lclLittleSteer;
		
	} else{
		OPLkasActive = false;
		// TODO: send/set/notify something to show there was an error... 
	}
	OPTimeLastCANRecieved = millis();

}

void handleLkasFromCanV3(){
// 	BO_ 228 STEERING_CONTROL: 5 ADAS
//  SG_ STEER_TORQUE : 7|16@0- (1,0) [-3840|3840] "" EPS
//  SG_ STEER_TORQUE_REQUEST : 23|1@0+ (1,0) [0|1] "" EPS
//  SG_ SET_ME_X00 : 31|8@0+ (1,0) [0|0] "" EPS
//  SG_ COUNTER : 37|2@0+ (1,0) [0|3] "" EPS
//  SG_ CHECKSUM : 35|4@0+ (1,0) [0|3] "" EPS

	if(canMsg.id != 228) return;
#ifdef DEBUG_PRINT_OPtoCAN_INPUT
	outputSerial.print("\nCANmsg rcvd id: ");
	outputSerial.print(canMsg.id,DEC);
	outputSerial.print(":");
	for(uint8_t bb = 0; bb < canMsg.len; bb++){
		printuint_t(canMsg.buf[bb]);
	}
#endif 

	if((canMsg.buf[2] >> 7) == 1 ){ // if STEER REQUEST (aka LKAS enabled)
		OPLkasActive = true;
	} else {
		OPLkasActive = false;
	}

	uint8_t lclBigSteer = 0;
	uint8_t lclLittleSteer = 0;
	
	lclBigSteer = ( canMsg.buf[0] >> 4 ) & B00001000;
	lclBigSteer |= ( canMsg.buf[1] >> 5 ) & B00000111;
	


	lclLittleSteer = canMsg.buf[1] & B00011111 ;
	
		// TODO: verify counter is working
	uint8_t lclCounter = canMsg.buf[4] >> 4;
	bool counterVerified = false;  // need global counter   and counter error

	if(LkasFromCanCounter != lclCounter) LkasFromCanCounterErrorCount++;
	else LkasFromCanCounterErrorCount = 0;
	
	if(LkasFromCanCounter < 3) counterVerified = true;


	// TODO: verify checksum
	bool checksumVerified = false;

	if(honda_compute_checksum((uint8_t*) &canMsg.buf[0],5) == (canMsg.buf[5] & B00001111 )) LkasFromCanChecksumErrorCount = 0;
	else LkasFromCanChecksumErrorCount++;
	
	if(LkasFromCanCounterErrorCount < 3 ) checksumVerified = true;
	else checksumVerified = false;


	// TODO: Fix this, hard coded to true for testing
	counterVerified = true;
	checksumVerified = true;

	//canbus data time is checked in the handleLkastoEPS function, if no data has been received within 50ms . LKAS is not allowed to be active

	// set big/small steer in varible and that LKAS is on
	// so when its time to send a LKAS message, it just reads the data, make the checksum and send it
	if(counterVerified && checksumVerified){
		// createKLinMessageWBigSteerAndLittleSteer(lclBigSteer,lclLittleSteer);
		
		OPBigSteer = lclBigSteer;
		OPLittleSteer = lclLittleSteer;
		
	} else{
		OPLkasActive = false;
		// TODO: send/set/notify something to show there was an error... 
	}

	OPTimeLastCANRecieved = millis();
}

void handleLKAStoEPSUsingOPCan(){
	//TODO: need to check last msg rcvd from can to see if LKAS active should be on... check fatal error.  
	//we only sendmessages on totalCounter == 0. so if its not 0, keep going
	if(incomingMsg.totalCounter != 0) return;

	// if fatal error is set, only send LKAS OFF packets, do not engage
	if( (   (millis() - OPTimeLastCANRecieved)   > 100 ) && OPLkasActive )
	{
		// sendArrayToLKAStoEPSSerial(&lkas_off_array[incomingMsg.counterBit][0]);
		LkasFromCanFatalError = 1;
		outputSerial.println("Timeout.  more than 100 ms since last CAN message and LKAS is on ");
	}

	if(OPLkasActive && !LkasFromCanFatalError) 
	{
		createKLinMessageWBigSteerAndLittleSteer(OPBigSteer,OPLittleSteer);
	}else 
	{
		sendArrayToLKAStoEPSSerial(&lkas_off_array[incomingMsg.counterBit][0]);
		// printArrayInBinary(&lkas_off_array[incomingMsg.counterBit][0],4);
	}	
} // end handleLKAStoEPSUsingOPCan();


//the name says it all... 
void sendArrayToLKAStoEPSSerial(uint8_t *array){

	LKAStoEPS_Serial.write(*array);
	LKAStoEPS_Serial.write(*(array+1));
	LKAStoEPS_Serial.write(*(array+2));
	LKAStoEPS_Serial.write(*(array+3));
	
	// if(arraySize == 5) serial.write(*(array+4));
	//outputSerial.print("\nsendArrayToLKAStoEPSSerial ");
	#ifdef DEBUG_PRINT_LKAStoEPS_LIN_OUTPUT
	outputSerial.print("\nL-O:");
	printuint_t(*array);
	outputSerial.print("  ");
	printuint_t(*(array+1));
	outputSerial.print("  ");
	printuint_t(*(array+2));
	outputSerial.print("  ");
	printuint_t(*(array+3));
	outputSerial.print("  ");
	#endif
		CAN_message_t thisCanMsg;
	thisCanMsg.id = 0x200;
	thisCanMsg.len = 4;
	thisCanMsg.buf[0] = *array;
	thisCanMsg.buf[1] = *(array+2);
	thisCanMsg.buf[2] = *(array+3);
	thisCanMsg.buf[3] = *(array+4);
	FCAN.write(thisCanMsg);
}

// *	*	*	*	*	*	*	*	*	******* EPS  to   LKAS

void handleEPStoLKAS(){
	if(EPStoLKAS_Serial.available())
	{
		uint8_t rcvdByte = EPStoLKAS_Serial.read();	

		uint8_t offset4 = rcvdByte >> 4;
		if( (offset4 < 4 )) 
		{
			if(EPStoLKASBufferCounter != 0)
			{
#ifdef DEBUG_PRINT_EPStoLKAS_ERRORS
			outputSerial.print("\nEPStoLKAS Was reset out of sequence   -  ");
			for(uint8_t z = 0; z < EPStoLKASBufferCounter; z++)
			{
				printuint_t(EPStoLKASBuffer[z]);
				outputSerial.print("  ");
			}
				printuint_t(rcvdByte);
				outputSerial.print("  -  ");
				outputSerial.println(EPStoLKASBufferCounter,DEC);
#endif
			}

			EPStoLKASBufferCounter = 0;
		}
#ifdef DEBUG_SEND_TO_SERIAL
	// outputSerial.print("  -  ");
	// outputSerial.print(EPStoLKASBufferCounter,DEC);
	// Serial.print("")
#endif

#ifdef DEBUG_PRINT_EPStoLKAS_LIN_INPUT
	outputSerial.print("\nE-I-");
	outputSerial.print(EPStoLKASBufferCounter,DEC);
	outputSerial.print(":");
	printuint_t(rcvdByte);
	// outputSerial.println();

#endif 
	// if its the 5th byte (index 4).. check checksum,, if its bad.. return
		if(EPStoLKASBufferCounter == 4){
			uint8_t thisChksm = chksm(&EPStoLKASBuffer[0],4);
			if(thisChksm != rcvdByte){
#ifdef DEBUG_PRINT_EPStoLKAS_ERRORS
			outputSerial.print(" Bad Chksm ");
			printuint_t(thisChksm);
			outputSerial.print("  **  ");
			printuint_t(rcvdByte);
#endif 	
			EPStoLKASBufferCounter = 0;
			return;
		}
	}


		EPStoLKASBuffer[EPStoLKASBufferCounter] = rcvdByte;


		if(!DIP2_sendOPSteeringTorque) {
			handleEPStoLKASOP(rcvdByte);
			
		}	
		else if(!DIP1_spoofFullMCUDigitalRead) {
			handleEPStoLKASSpoofMCU(rcvdByte);  //spoof MCU
		}
		else handleEPStoLKASNoSpoof(rcvdByte); // do not spoof.... relay or ???


		EPStoLKASBufferCounter++;
				
		if(EPStoLKASBufferCounter > 4) {
			EPStoLKASBufferCounter = 0;
#ifdef DEBUG_SEND_TO_SERIAL
			// outputSerial.println("EPStoLKASBufferCounter is set to 0 by line 267");
#endif
		}

	} // end .available()
} // handleEPStoLKAS()


// this function is called from the handleEPStoLKAS() which is called when a byte is received from this LIN bus.  
// This function builds the 2 CAN messages for MOTOR_TORQUE and STEER_TORQUE (input) from the EPStoLKAS 5 byte frame
// but only does it after the whole frame is received and checksum'd
void handleEPStoLKASOP(uint8_t rcvdByte){

	// use OPCanCounter.
	handleEPStoLKASKeepMcuHappy(rcvdByte);

	if(EPStoLKASBufferCounter < 4) return; //nothing to do 
	//TODO: run checksum on data
	buildSteerMotorTorqueCanMsg();
	buildSteerStatusCanMsg();

	if(++OPCanCounter > 3) OPCanCounter = 0;
}

void handleEPStoLKASKeepMcuHappy(uint8_t rcvdByte){
	// if(EPStoLKASBufferCounter == 0) EPStoLKAS_Serial.write(rcvdByte);
	if(EPStoLKASBufferCounter == 0){
		
			sendArrayToEPStoLKASSerial(&eps_off_array[incomingMsg.counterBit][0]);

		
		#ifdef DEBUG_PRINT_EPStoLKAS_LIN_OUTPUT
		outputSerial.print("\nE-O:");
		printArrayInBinary(&eps_off_array[incomingMsg.counterBit][0],5);
		outputSerial.print("  ");
		#endif
	}
	// EPStoLKAS_Serial.write(eps_off_array[incomingMsg.counterBit][EPStoLKASBufferCounter]);
}


//called if no spoofing is wanted (dip 1) for the EPStoLKAS line
//it simply forwards the data on the to the LKAS but also prints some to the debug UART
// and sends it to the CAN bus for OP (if enabled) when its at the end of the frame
void handleEPStoLKASNoSpoof(uint8_t rcvdByte){
	
	EPStoLKAS_Serial.write(rcvdByte);
	if(EPStoLKASBufferCounter == 4){
		// outputSerial.print("  ^  ");
		// printArrayInBinary(&EPStoLKASBuffer[0],5);
		// EPStoLKASBufferCounter = 0;
		if(sendEPStoLKASRxToCan){
			canMsg.id = EPStoLKASLinDataRxMsgId;
			canMsg.len = 5;
			for(unsigned char zz =0 ; zz < 5; zz++){
				canMsg.buf[zz] = EPStoLKASBuffer[zz];
			}
			FCAN.write(canMsg);
		}
	}
}


//the name says it all. requites 5 byte array pointer
void sendArrayToEPStoLKASSerial(uint8_t *array){
	
	EPStoLKAS_Serial.write(*array);
	EPStoLKAS_Serial.write(*(array+1));
	EPStoLKAS_Serial.write(*(array+2));
	EPStoLKAS_Serial.write(*(array+3));
	EPStoLKAS_Serial.write(*(array+4));
	// if(arraySize == 5) serial.write(*(array+4));
}



//if we are spoofing the lines.. DIP1, the EPStoLKAS is being spoofed
//we are trying to trick the MCU into thinking nothing is going on, so we only send back data showing no torque input
//on the steering wheel, and matching checksums... this isn't good for the safety logic in the MCU, although it does receive
//steering wheel position data from the CAN bus, so the steering torque data might be minimial.
void handleEPStoLKASSpoofMCU(uint8_t rcvdByte){
	// if( (rcvdByte >> 7 )  == 0 ) EPStoLKASBufferCounter = 0;
	

	if(DIP6_passSteeringWheelTorqueData) passEPStoLKASTorqueData(rcvdByte);
	else if(DIP7_SpoofSteeringWheelTorqueData)spoofSteeringWheelTorqueData(rcvdByte);
	else {
		if( EPStoLKASBufferCounter  == 0 ){ // first byte
			sendArrayToEPStoLKASSerial(&eps_off_array[incomingMsg.counterBit][0]);
			
			
		} else {
			if(EPStoLKASBufferCounter == 4){
				// EPStoLKASBufferCounter = 0;
				
				outputSerial.print("  ^  ");
				printArrayInBinary(&eps_off_array[incomingMsg.counterBit][0],5);
			}// end if
		}// end 2nd else
	} // end 1st else
	if(EPStoLKASBufferCounter == 4){
		canMsg.id = 0x201;
		canMsg.len = 5;
		for(uint8_t zz = 0; zz < 5; zz++){
			canMsg.buf[zz] = EPStoLKASBuffer[zz];
		}
		FCAN.write(canMsg); // ****

		// the format will be
		//B   # # # #  # # # #    # # # #   # # # #    # # # 0  0 # # #    0 0 C C  H H H H       
		//    |DriverSteer Torque9|
		//     3rd byte offest 6-1  |
		//  						  |Motor Steer Torque9 |
		//						  	   3 bits 3rd byte offset 0-2 |   |
		//																Counter     Checksum

		uint8_t steerTorqueAndMotorTorque[4] = {0,0,0,0};
		// uint8_t tempData = 0;
		steerTorqueAndMotorTorque[0] = EPStoLKASBuffer[0] << 5;
		steerTorqueAndMotorTorque[0] |= ( ( EPStoLKASBuffer[1] > 1 ) & B00011111 );

		steerTorqueAndMotorTorque[1] = EPStoLKASBuffer[1] << 7;
		steerTorqueAndMotorTorque[1] |= ( EPStoLKASBuffer[1] & B0100000 );
		steerTorqueAndMotorTorque[1] |= ( EPStoLKASBuffer[2] & B00110000);
		steerTorqueAndMotorTorque[1] |= ( ( EPStoLKASBuffer[3] >> 3 ) & B00001111 );
		
		steerTorqueAndMotorTorque[2] = ( EPStoLKASBuffer[4] << 5 );
		steerTorqueAndMotorTorque[2] |= ( EPStoLKASBuffer[3] & B00000111);

		steerTorqueAndMotorTorque[3] = EPStoLKASCanFrameCounter << 4;
		steerTorqueAndMotorTorque[3] |= honda_compute_checksum(&steerTorqueAndMotorTorque[0], 4);
		
		if(++EPStoLKASCanFrameCounter >3) EPStoLKASCanFrameCounter = 0;
	}
} // end function




// *	*	*	*	*	*	*	*	*	*	*	*	*********  Printing   Functions

//prints the binary representation of a uint8_t (byte/char).  it also adds a space at between the two 4 bit segments, easier to read
//the naming is pretty bad, but need to change it. 
void printuint_t(uint8_t var) { ///i stole this from my program https://github.com/reddn/LIN2LINrepeaterMEGA, i think it should work
  for (uint8_t test = 0x80; test; test >>= 1) {
  	if(test == 0x8) outputSerial.write(' '); // 0x8 is 0000 1000.. so it gives a break between the 2 4 bit writes
    outputSerial.write(var  & test ? '1' : '0');
  }
}

//prints an LKAS array to binary using the printuint_t function. it is almost the same as the printArrayInBinary, but this
//assumes the pointer points to an array that is 4 bytes long, where printArrayInBinary requires a length attribute
void printLKASArray(uint8_t *msg){ //msg is &array[0]
	for(char zz = 0; zz<4; zz++){
		printuint_t(*(msg+zz));
		if(zz != 3) outputSerial.print("  ");
	}
}

//prints any byte array to binary using the printuint_t for each byte, 2 spaces between each byte.
void printArrayInBinary(uint8_t *msg,uint8_t size){ //msg is &array[0]  .. size is sizeof(array)
	for(char zz = 0; zz<size; zz++){
		if(zz != 0) outputSerial.print("  ");
		printuint_t(*(msg+zz));
	}
}

//like printLKASarray, and printArrayInBinary, but only will print a 5 byte array.. this maybe deleted for printArrayInBinary
void printEPSArray(uint8_t *msg){ //msg is array[0]
	for(char zz = 0; zz<5; zz++){
		printuint_t(*(msg+zz));
		if(zz != 4) outputSerial.print("  ");
	}
}

//sends an array to a specified serial... i think this doenst work this way and will crash a program if used.. probably a good idea to remove it TODO
void sendArrayToSerial(HardwareSerial serial,uint8_t *array,uint8_t arraySize){
	serial.write(*array);
	serial.write(*(array+1));
	serial.write(*(array+2));
	serial.write(*(array+3));
	if(arraySize == 5) serial.write(*(array+4));
}










//if we are spoofing the LKAStoMCU line
//only sends data if the byte shows its the first byte of the frame, which only the first byte has 00 as its MSB.
//then its sees if the PB1,PB2,PB4 are press to send those torque request values.  if not. just send a normal
//LKAS off value depending on the counterbit (so all counter bits are in sync)
void handleLKAStoMCUSpoofMCU(uint8_t rcvdByte){
	if((rcvdByte >> 7) == 0){ //its the first byte
		incomingMsg.totalCounter = 0;
		if(!PB1_spoofLKASLeftDigitalRead)  {
			createKLinMessage(forceLeftApplyTorque);
			forceLeftApplyTorque = random(-150,-125);
		}
		else if(!PB2_spoofLKASRightDigitalRead) {
			createKLinMessage(forceRightApplyTorque);
			forceRightApplyTorque = random(130,150);
		} else if(!PB4_spoofLKASSteerWithPOTEnableDigitalRead){
			createKLinMessage(forceApplyTorqueWithPOT);
		} else {
			// sendArrayToSerial(LKAStoEPS_Serial,&lkas_off_array[incomingMsg.counterBit][0], 4); 
			sendArrayToLKAStoEPSSerial(&lkas_off_array[incomingMsg.counterBit][0]);
#ifdef DEBUG_PRINT_LKAStoEPS_ERROR
			outputSerial.print("\nL-");
			printArrayInBinary(&lkas_off_array[incomingMsg.counterBit][0],4);
#endif
			LkasOnIntroCountDown = 5;
		}
	}else{ // its not the first byte
		if(incomingMsg.totalCounter == 3){ //its the last byte of the frame
			if(sendLKAStoEPSRxToCan){
				canMsg.id = LKAStoEPSLinDataRxMsgId;
				canMsg.len = 4;
				for(uint8_t zz = 0; zz < 4; zz++){
					canMsg.buf[zz] = incomingMsg.data[zz];
				}
				FCAN.write(canMsg); // ****
			}
		}
	}
	// no else... as all of the data is sent on when you receive the first byte....
}







//sends a 5 byte array by pointer to the CANbus using the messageID set in define.cpp... 
void sendEPStoLKASToCAN(uint8_t *EPSData){ // Sends the 5 byte frame to CAN for tracking in cabana
	// TODO: send a can message for Steer torque so OP can see it

	CAN_message_t msg;
    msg.id = EPStoLKASCanMsgId;
	msg.len = 5;
	for(unsigned char a = 0; a < 5; a++){
		msg.buf[a] = *( EPSData + a );
	}
	FCAN.write(msg); 
	//CAN message for input steer torque (by driver)
	//Byte 1:  B B B B S S S S
	//byte 2:  S 0 C C H H H H
	// B = Big steer input   S = Small steer input 
	// C = Counter (2 bit)   H = 4 bit checksum
	msg.len = 2;
	msg.buf[0] = 0x00;
	msg.buf[0] = (*EPSData << 4); // big steer
	uint8_t EPS2ndByte = *(EPSData +1);
	msg.buf[0] = ( (EPS2ndByte >> 1 ) & B00001111 )  | msg.buf[0];
	msg.buf[1] = *(EPSData + 1) << 3; // small steer
	msg.id = SteerTorqueSensorCanMsgId;
	FCAN.write(msg);
}







//*	*	*	*	*	*	*	*	*	*	*	*	***					OP functions			***

// BO_ 427 STEER_MOTOR_TORQUE: 3 EPS
//  SG_ MOTOR_TORQUE : 0|10@0+ (1,0) [-256|256] "" EON
//  SG_ OUTPUT_DISABLED_INVERT : 15|1@0+ (1,0) [0|1] "not sure if its invert or some LDW" EON
//  SG_ UNK_3BIT_1 : 10|3@0+ (1,0) [0|7] "" EON
//  SG_ OUTPUT_DISABLED : 14|1@0+ (1,0) [0|1] "" EON
//  SG_ COUNTER : 21|2@0+ (1,0) [0|3] "" EON
//  SG_ CHECKSUM : 19|4@0+ (1,0) [0|15] "" EON

//removed
// SG_ CONFIG_VALID : 7|1@0+ (1,0) [0|1] "" EON   << not used

void buildSteerMotorTorqueCanMsg(){ //TODO: add to decclaration
	//outputSerial.print("\nSendingSteer TOrque Can MSg");
	CAN_message_t msg; // move this to a global to save the assignment of id and len
	msg.id = 427;
	msg.len = 3;
	msg.buf[0] = (EPStoLKASBuffer[2] << 4 ) & B1000000;  //1 LSB bit of bigSteerTorque 
	msg.buf[0] |= EPStoLKASBuffer[3] & B01111111 ; // all of SmallSteerTorque
	msg.buf[1] = ( EPStoLKASBuffer[2] >> 4 ) & B00000011; // 2 MSB of bigSteerTorque
	msg.buf[1] |= ( EPStoLKASBuffer[1] << 2 ) & B10000000;  // this is output_disabled_inverted
	msg.buf[1] |= ( EPStoLKASBuffer[2] << 2 )& B00011100; //UNK_3bit_1
	msg.buf[1] |=  EPStoLKASBuffer[2] & B01000000; //output_disabled
	
	msg.buf[2] = (OPCanCounter << 4 ); // put in the counter
	msg.buf[2] |= honda_compute_checksum(&msg.buf[0],3);
	FCAN.write(msg);
}

///// the only thing in this DBC that should be used is steeor_troque_sensor
// BO_ 399 STEER_STATUS: 3 EPS
//  SG_ STEER_TORQUE_SENSOR : 0|10@0- (-1,0) [-256|255] "tbd" EON
//  SG_ COUNTER : 21|2@0+ (1,0) [0|3] "" EON
//  SG_ CHECKSUM : 19|4@0+ (1,0) [0|3] "" EON

//removed
//  SG_ STEER_STATUS : 39|4@0+ (1,0) [0|15] "" EON
//  SG_ STEER_CONTROL_ACTIVE : 35|1@0+ (1,0) [0|1] "" EON
//  SG_ STEER_CONFIG_INDEX : 43|4@0+ (1,0) [0|15] "" EON
//  SG_ STEER_ANGLE_RATE : 23|16@0- (-0.1,0) [-31000|31000] "deg/s" EON << TODO: check if OP uses this or the other STEER_ANGLE_RATE .. this one will not work

void buildSteerStatusCanMsg(){ //TODO: add to decclaration
	
	// outputSerial.print("\nsending Steer Status Cna MSg");
	CAN_message_t msg; // move this to a global so you dont have to re assign the id and len
	msg.id = 399;
	msg.len = 3;
	msg.buf[0] = EPStoLKASBuffer[0] << 5;   // 3 LSB of BigSteerTorque (4bit)
	msg.buf[0] |= EPStoLKASBuffer[1] & B00011111; // all of smallSteerTorque
	msg.buf[1] = ( EPStoLKASBuffer[0] >>3 ) & B00000001; // 1st MSB of bigSteerTorque (4bit)

	msg.buf[2] = (OPCanCounter << 4 ); // put in the counter

	//add other data from Teensy so OP can record it
	msg.buf[1] |= OPLkasActive << 7;
	msg.buf[1] |= LkasFromCanFatalError << 6;
	
	msg.buf[2] |= honda_compute_checksum(&msg.buf[0],3);
	FCAN.write(msg);
}



//when OP port is finally coded, its going to send a steer torque command to the bus, the teensy will see that data and
//after checking counter and checksum, will take the data and send it on the serial/LIN bus for steering torque application
void handleLKASFromCan(const CAN_message_t &msg){
	// pull apply steer value
	
	// CAN message 
	//     MSB				   LSB
	// Byte 1- B B B S S S S S
	// Byte 2- H H H H C C E B  ******************  TODO: it must be * * C C H H H H   I think i changed this in the DBC but didnt update this... 
	// B = BIg steer
	// S = Small steer (used in createlinmsg)
	// E = Enable LKAS (if zero. send lkas_off_msg)
	// C = Counter. 2 bit 0-3
	// H = Checksum. 4 bit

	if((msg.buf[1] & B00000010) >> 0){ // if STEER REQUEST (aka LKAS enabled)
		// send lkas_off_message
		OPLkasActive = false;
		return;
	}

	uint8_t lclBigSteer = 0;
	uint8_t lclLittleSteer = 0;
	lclBigSteer = (msg.buf[1] & 0x01 ) << 3;
	lclBigSteer = (msg.buf[0] >> 5) | lclBigSteer;

	lclLittleSteer = msg.buf[0] & B00011111;
	// TODO: verify counter is working
	bool counterVerified = true;
	// TODO: verify checksum
	bool checksumVerified = true;

	// set big/small steer in varible and that LKAS is on
	// so when its time to send a LKAS message, it just reads the data, make the checksum and send it
	if(counterVerified && checksumVerified){
		// createKLinMessageWBigSteerAndLittleSteer(lclBigSteer,lclLittleSteer);
		OPLkasActive = true;
		OPBigSteer = lclBigSteer;
		OPLittleSteer = lclLittleSteer;
		OPTimeLastCANRecieved = millis();
	} else{
		OPLkasActive = false;
		// TODO: send/set/notify something to show there was an error... 
	}
}



// function spoofs the 3rd and 4th bytes of data back to the LKAS (MCU), this keeps it happy and in the blind
//but the driver torque data should be sent to it so it knows what is happening (or not happening) for its safety logic
void passEPStoLKASTorqueData(uint8_t rcvdByte){
	switch(EPStoLKASBufferCounter){
		case 0: // first byte received on EPStoLKAS 
			EPStoLKAS_Serial.write(rcvdByte);
			break;
		case 1: // second byte received on EPStoLKAS.  the 5th offset in the byte is the 'LKAS ON' signal back fom the EPS.
		// this needs to be spoofed to '0' incase it raises an error(lkas on when the MCU says it should be)
		// if the 4 second timer to apply a fake driver input is true, XOR the 3 LSB of the input driver to change the values
			uint8_t lcldata;
			if(spoofSteeringWheelTorqueDataBool){
				lcldata = rcvdByte ^ B00100111;// XOR  .. offset 4 needs to be 0
				EPStoLKAS_Serial.write(lcldata);// XOR
				EPStoLKASBuffer[1] = lcldata;
				spoofSteeringWheelTorqueDataBool = false;
				
			} else{
				lcldata = rcvdByte ^ B00100000;// XOR  .. offset 4 needs to be 0
				EPStoLKAS_Serial.write(lcldata);
				EPStoLKASBuffer[1] = lcldata;
			}
			break;
		case 2: // third byte recieved ... on the third byte, the rest of the data doesnt matter as its spoofed. so send the rest now
			
			EPStoLKAS_Serial.write(B11000000);
			EPStoLKAS_Serial.write(B10000000);
			uint8_t lclChecksum = chksm(EPStoLKASBuffer[0],EPStoLKASBuffer[1],B11000000,B1000000);
			EPStoLKAS_Serial.write(lclChecksum);
			break;
		
	}
}

// probably not going to be used as most of the commands in this function are replcated in the passEPStoLKASdata above.. 
//so i just added an if statement in it to handle what this needed.
void spoofSteeringWheelTorqueData(uint8_t rcvdByte){

}




// *	*	*	*	*	*	*	*	*	***					CHECKSUM FUNCTIONS

// creates the checksum for the LKAStoEPS message (its 3 bytes + 1 checksum for the frame)
uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte){
	uint8_t tot = firstByte + secondByte + thirdByte ;
	tot = 256- tot;
    tot %= 128;
    tot += 128;
	return tot;
}

// creates checksum of the EPStoLKAS frame, 4 bytes + 1 checksum byte... overloaded
uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte, uint8_t fourthByte){
	uint8_t tot = firstByte + secondByte + thirdByte + fourthByte;
	tot = 256- tot;
    tot %= 128;
    tot += 128;
	return tot;
}

uint8_t chksm(uint8_t* data, uint8_t len){
	uint8_t tot = 0;
	for(uint8_t zz =0; zz < len; zz ++) 
	{
		tot += *(data+zz);
	}
    tot = 256- tot;
    tot %= 128;
    tot += 128;
	return tot;
}

uint8_t chksm_old(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte){
	uint16_t local = firstByte + secondByte + thirdByte ;
	local = local % 512;
	local = 512 - local;
	return (uint8_t)(local % 256);
}

//code mostly taken from safety_honda.h  Credit Comma.ai MIT license on this function only
uint8_t honda_compute_checksum(uint8_t *steerTorqueAndMotorTorque, uint8_t len) {
//   int len = GET_LEN(to_push);
  uint8_t checksum = 0U;
  unsigned int addr = 399U; //this should be set up top... 399 is STEER STATUS
  while (addr > 0U) {
    checksum += (addr & 0xFU); addr >>= 4;
  }
  for (int j = 0; j < len; j++) {
    uint8_t byte = *(steerTorqueAndMotorTorque + j);
    checksum += (byte & 0xFU) + (byte >> 4U);
    if (j == (len - 1)) {
      checksum -= (byte & 0xFU);  // remove checksum in message
    }
  }
  return (8U - checksum) & 0xFU;
}

//called from the main loop.  reads only when needed by the TIME_BETWEEN_DIGIAL_READS define, in milliseconds
//this helps from too many digitalRead calls slowing down reading from the other buses.
//reads all Pushbuttons and DIP switches to set them into a variable.  
//also reads the POT
void handleInputReads(){
	if( ( millis() - lastDigitalReadTime ) > TIME_BETWEEN_DIGITIAL_READS){
		PB1_spoofLKASLeftDigitalRead = digitalRead(PB1_spoofLKASLeft);
		PB2_spoofLKASRightDigitalRead = digitalRead(PB2_spoofLKASRight);
		PB4_spoofLKASSteerWithPOTEnableDigitalRead = digitalRead(PB4_spoofLKASSteerWithPOTEnablePin);
		// if((!PB1_spoofLKASLeftDigitalRead) || (!PB2_spoofLKASRightDigitalRead)){
		// 	digitalWrite(BLUE_LED,HIGH);
		// } else digitalWrite(BLUE_LED,LOW);

		DIP1_spoofFullMCUDigitalRead = digitalRead(DIP1_spoofFullMCU);
		// DIP2_sendOPSteeringTorque = digitalRead(DIP2);
		DIP2_sendOPSteeringTorque = 0; // Forced into OP mode
		// DIP5_disableAllLinOutput = digitalRead(DIP5);
		DIP5_disableAllLinOutput = 0; // doesn't work
		DIP6_passSteeringWheelTorqueData = digitalRead(DIP6_passSteeringWheelTorqueData_PIN);
		DIP7_SpoofSteeringWheelTorqueData = digitalRead(DIP7_SpoofSteeringWheelTorqueData_PIN);

		//commented out as this is not needed now. but can be easily re enabled for manual control fun
		// A1_applySteeringPot = analogRead(A1_applySteeringPotPin);
		
		forceApplyTorqueWithPOT = (A1_applySteeringPot - 474) / 1.78;

		
		// outputSerial.print(A1_applySteeringPot,DEC);
		// outputSerial.print("  -  ");
		// outputSerial.println(forceApplyTorqueWithPOT,DEC);
		if(spoofSteeringWheelTorqueData_Counter++ > 400){
			spoofSteeringWheelTorqueDataBool = true;
			spoofSteeringWheelTorqueData_Counter = 0;
		}

		if( (millis() - OPTimeLastCANRecieved) < 1000 ) mainLedBlinkTimer = 500;
		else mainLedBlinkTimer = 2000; 
		// digitalWrite(BLUE_LED,!DIP2_sendOPSteeringTorque);
		
		lastDigitalReadTime = millis();

		
		digitalWrite(BLUE_LED,( EPStoLKASBuffer[2] >> 2 ) && B00000001);
		
	} // end if true
}

// 			****** 				SETUP 					******

// helper setup to setup the CAN bus.  
// void canSetup(){
// 	FCAN.begin();
// 	FCAN.setBaudRate(500000);
// 	FCAN.setMaxMB(16);
// 	FCAN.enableFIFO();
// 	FCAN.enableFIFOInterrupt();
// 	FCAN.onReceive(handleLKASFromCanV2);
// 	FCAN.mailboxStatus();

// 	FCAN.setFIFOFilter(REJECT_ALL);
// 	FCAN.setMBFilter(REJECT_ALL);
// 	FCAN.setFIFOFilter(0, LKAStoEPSCanMsgId, STD); // Set filter0 to allow STANDARD CAN ID 0x123 to be collected by FIFO. 
// }

void setup(){	
	EPStoLKAS_Serial.begin(9600,SERIAL_8E1);
	LKAStoEPS_Serial.begin(9600,SERIAL_8E1);
	outputSerial.begin(OUTPUTSERIAL_BAUD);
	// pinMode(analogRotaryInputPin,INPUT);
	pinMode(PB1_spoofLKASLeft, INPUT_PULLUP);
	pinMode(PB2_spoofLKASRight, INPUT_PULLUP);
	pinMode(PB3_spoofLKASStop, INPUT_PULLUP);
	pinMode(PB4_spoofLKASSteerWithPOTEnablePin, INPUT_PULLUP);
	pinMode(A1_applySteeringPotPin, INPUT_PULLUP);
	pinMode(DIP1_spoofFullMCU, INPUT_PULLUP);
	pinMode(DIP2,INPUT_PULLUP);
	pinMode(DIP5,INPUT_PULLUP); // **disabled, removed from code** DIP 5 is used as a test to disable LIN output...
	pinMode(DIP7_SpoofSteeringWheelTorqueData, INPUT_PULLUP);
	pinMode(BLUE_LED,OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	// canSetup();
	FCAN.begin();
	FCAN.setBaudRate(500000);
}

void loop(){
	if(FCAN.read(canMsg)){
		handleLkasFromCanV3();
	}
	handleEPStoLKAS();
	handleLKAStoEPS();
	handleInputReads();
	if((millis() - readLEDblinkLastChange) > mainLedBlinkTimer){
		// outputSerial.println("LED change");
		digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
		// digitalWrite(BLUE_LED,!digitalRead(BLUE_LED));
		readLEDblinkLastChange = millis();
	} 
}