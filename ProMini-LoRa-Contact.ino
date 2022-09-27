/*
	LoRa Simple Gateway/Node Exemple

	This code uses InvertIQ function to create a simple Gateway/Node logic.

	Gateway - Sends messages with enableInvertIQ()
					- Receives messages with disableInvertIQ()

	Node		- Sends messages with disableInvertIQ()
					- Receives messages with enableInvertIQ()

	With this arrangement a Gateway never receive messages from another Gateway
	and a Node never receive message from another Node.
	Only Gateway to Node and vice versa.

	This code receives messages and sends a message every second.

	InvertIQ function basically invert the LoRa I and Q signals.

	See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
	for more on InvertIQ register 0x33.

	created 05 August 2018
	by Luiz H. Cassettari
*/

#include <SPI.h>							 // include libraries
#include <LoRa.h>
#include "LowPower.h"

// #define DEBUG

#define SERIAL_BAUD	 				57600
#define NODE_ID			 			2			// NodeId of this LoRa Node
#define MAX_PACKET_SIZE				10

#define MSG_ID_NODE_STARTUP		1 	 	// Node startup notification
#define MSG_ID_STILL_ALIVE		2  		// Node still alive
#define MSG_ID_CMND_REQUEST		3  		// Node wakeup/cmnd request
#define MSG_ID_SWITCH_CHANGE	4  		// Switch change detected

#define SEND_CMND_REQUEST_MSG_EVERY	228   // 0.5 hour
// #define SEND_STILL_ALIVE_MSG_EVERY	1824  // 4 hours
#define SEND_STILL_ALIVE_MSG_EVERY	3  // 4 hours

//#define SEND_MSG_EVERY	22		// Watchdog is a timerTick on a avg 8,0 sec timebase
															// SEND_MSG_EVERY=8	-> +- 1min
															// SEND_MSG_EVERY=14 -> +- 2min
															// SEND_MSG_EVERY=23 -> +- 3min
															// SEND_MSG_EVERY=30 -> +- 4min
															// SEND_MSG_EVERY=38 -> +- 5min
										// SEND_MSG_EVERY=228 -> 0,5 hours
										// SEND_MSG_EVERY=1824 -> 4 hours

// #define SEND_TEMPERATURE			12    // 16 sec
#define SEND_TEMPERATURE			75    // 10 min
#define SEND_MEASURE_VCC_EVERY		24	// Measure Vcc voltage every N messages
										// MEASURE_EVERY=24 -> +- 4 hour

byte payloadSize = 4; //Without any device

volatile word sendMsgTimer = SEND_STILL_ALIVE_MSG_EVERY - 2;
volatile unsigned char sendMsgVccLevelTimer = SEND_MEASURE_VCC_EVERY;

//Message max 30 bytes
struct Payload {
	byte nodeId;
	byte msgId;
	byte voltageVcc;				//getVcc 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
	byte contactState;				//0=Closed, 1=Open
} txPayload;

const long loRaFrequency = 866E6;			// LoRa loRaFrequency

const int loRaCsPin = 15;					// LoRa radio chip select
const int loRaResetPin = 14;			 	// LoRa radio reset
const int loRaIrqPin = 2;					// change for your board; must be a hardware interrupt pin

const int switchIrqPin = 3;
volatile byte switchState = LOW;
volatile byte switchChangeDetected = false;


void switchChangeDetectedInt() {
	switchChangeDetected = true;
}

void LoRa_rxMode(){
	LoRa.enableInvertIQ();								// active invert I and Q signals
	LoRa.receive();										// set receive mode
}

void LoRa_txMode(){
	LoRa.idle();											// set standby mode
	LoRa.disableInvertIQ();								// normal mode
}

void LoRa_sendMessage(Payload payload, byte payloadLen) {
	LoRa_txMode();											// set tx mode
	LoRa.beginPacket();								 	// start packet
	LoRa.write((byte*) &payload, payloadLen); 	// add payload
	LoRa.endPacket(true);								// finish packet and send it
}

void onReceive(int packetSize) {
	byte rxPayload [MAX_PACKET_SIZE];

	byte i = 0, rxByte;

	while (LoRa.available()) {
		rxByte = (byte)LoRa.read();
		if (i < MAX_PACKET_SIZE) {
			rxPayload[i] = rxByte;
			i++;
		}
	}

	// Only accept messages with our NodeId
	if (rxPayload[0] == NODE_ID) {
#ifdef DEBUG
		Serial.print("Rx packet OK "); // Start received message
		for (char i = 0; i < packetSize; i++) {
				Serial.print(rxPayload[i], DEC);
				Serial.print(' ');
		}
#endif
	}
}

void onTxDone() {
	// Serial.println("TxDone");
	LoRa_rxMode();
}

static byte vccLevelRead()
{
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Let mux settle a little to get a more stable A/D conversion
  
  // Start a conversion
  ADCSRA |= _BV(ADSC);
  
  // Wait for it to complete
  while (bit_is_set(ADCSRA, ADSC));
  
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  // 1.0V = 0, 1.8V = 40, 3.0V = 100, 3.3V = 115, 5.0V = 200, 6.0V = 250
  return (55U * 1023U) / (ADC + 1) - 50;
}

void setup() {
#ifdef DEBUG
	Serial.begin(SERIAL_BAUD);									 // initialize serial
	while (!Serial);

	Serial.println();
	Serial.print("[LORA-NODE.");
	Serial.print(NODE_ID);
	Serial.println("]");
#endif

	LoRa.setPins(loRaCsPin, loRaResetPin, loRaIrqPin);

	if (!LoRa.begin(loRaFrequency)) {
#ifdef DEBUG
		Serial.println("LoRa init failed. Check your connections.");
#endif
		while (true);											 // if failed, do nothing
	}

	//LoRa.setTxPower(20);
	LoRa.enableCrc();
	LoRa.onReceive(onReceive);
	LoRa.onTxDone(onTxDone);
	LoRa_rxMode();
	payloadSize = 4;

	// Switch input external interrupt
	pinMode(switchIrqPin, INPUT_PULLUP);
	switchState = digitalRead(switchIrqPin);

	// Send Node startup msg
	txPayload.nodeId = NODE_ID;
	txPayload.msgId = MSG_ID_NODE_STARTUP;
	// txPayload.voltageVcc = vccLevelRead();
	txPayload.contactState = switchState;
	LoRa_sendMessage(txPayload, payloadSize); // send a message
	delay(40); // [ms] Give RFM95W time to send the message

#ifdef DEBUG
	delay(100); // [ms] Give time to print the debug messages before sleep
#endif //DEBUG
	attachInterrupt(digitalPinToInterrupt(switchIrqPin), switchChangeDetectedInt, CHANGE);
}


void loop() {
	// attachInterrupt(digitalPinToInterrupt(switchIrqPin), switchChangeDetectedInt, CHANGE);

	// Enter power down state with ADC and BOD module disabled. Wake up when wake up pin is low
	// Serial.println("Sleep for 8s....");
	// delay(100);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	// Disable external pin interrupt on wake up pin
	// detachInterrupt(digitalPinToInterrupt(switchIrqPin));

	// Waked up! From timer or movement (external interrupt)
	if (switchChangeDetected) {
		switchChangeDetected = false;
		switchState = digitalRead(switchIrqPin);
		// Waked up from external interrupt
#ifdef DEBUG
		Serial.print("Switch change detected: ");
		Serial.println(switchState);
		delay(200); // [ms] Give time to print the debug messages before sleep
#endif

		txPayload.nodeId = NODE_ID;
		txPayload.msgId = MSG_ID_SWITCH_CHANGE;
		txPayload.contactState = switchState;
		LoRa_sendMessage(txPayload, payloadSize); // send the message

		delay(40); // [ms] Give RFM95W time to send the message
		LoRa.sleep(); // Put RFM95W in sleep mode

		sendMsgTimer = 0; // No need to send still alive message
	} else {
		// Waked up by periodic wakeup timer (8s)
		sendMsgTimer++;

		if (sendMsgTimer >= SEND_CMND_REQUEST_MSG_EVERY) {
			sendMsgTimer = 0;

#ifdef DEBUG
			Serial.println("Send Still Alive message");
#endif
			txPayload.nodeId = NODE_ID;
			txPayload.msgId = MSG_ID_SWITCH_CHANGE;
			txPayload.contactState = switchState;
			sendMsgVccLevelTimer++;

			if (sendMsgVccLevelTimer >= SEND_MEASURE_VCC_EVERY) {
				sendMsgVccLevelTimer = 0;
				txPayload.voltageVcc = vccLevelRead();
			}

			LoRa_sendMessage(txPayload, payloadSize); // send the message
			delay(40); 	// [ms] Wait a while for the server response
			LoRa.sleep(); 	// Put RFM95W in sleep mode
		}
	}
}
