// 4 Channel "Micro RC" Receiver with 4 standard RC Servo Outputs
// ATMEL Mega 328P TQFP 32 soldered directly to the board, 8MHz external resonator,
// 2.4GHz NRF24L01 SMD radio module, TB6612FNG dual dc motor driver
// An MPU-6050 gyro / accelerometer can be used for MRSC stability control or self balancing robots

// See: https://www.youtube.com/playlist?list=PLGO5EJJClJBCjIvu8frS7LrEU3H2Yz_so

// * * * * N O T E ! The vehicle specific configurations are stored in "vehicleConfig.h" * * * *

const float codeVersion = 2.9; // Software revision (see https://github.com/TheDIYGuy999/Micro_RC_Receiver/blob/master/README.md)

//
// =======================================================================================================
// BUILD OPTIONS (comment out uArduinonneeded options)
// =======================================================================================================
//

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

//
// =======================================================================================================
// INCLUDE LIRBARIES
// =======================================================================================================
//

// Libraries
#include <Wire.h>         // I2C library (for the MPU-6050 gyro /accelerometer)
#include <RF24.h>         // Installed via Tools > Board > Boards Manager > Type RF24
#include <printf.h>
#include <Servo.h>
#include <TB6612FNG.h>    // https://github.com/TheDIYGuy999/TB6612FNG ***NOTE*** V1.2 required!! <<<-----
#include <PWMFrequency.h> // https://github.com/TheDIYGuy999/PWMFrequency
// Tabs (header files in sketch directory)
#include "readVCC.h"
#include "vehicleConfig.h"
#include "tone.h"
#include "helper.h"

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

// Radio channels (126 channels are supported)
//byte chPointer = 0; // Channel 1 (the first entry of the array) is active by default
//const byte NRFchannel[] { 100, 102, 103, 104, 105, 106, 107, 108 };
const byte NRFchannel = 110;

// the ID number of the used "radio pipe" must match with the selected ID on the transmitter!
// 10 ID's are available @ the moment
const uint64_t pipeIn[] = { 0xE9E8F0F0B0LL, 0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL,
		0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL, 0xE9E8F0F0B6LL,
		0xE9E8F0F0B7LL, 0xE9E8F0F0B8LL, 0xE9E8F0F0B9LL };
const int maxVehicleNumber = (sizeof(pipeIn) / (sizeof(uint64_t)));

byte pipeNo;

// Hardware configuration: Set up nRF24L01 radio on hardware SPI bus & pins 8 (CE) & 7 (CSN)
RF24 radio(8, 7);

// The size of this struct should not exceed 32 bytes
struct RcData {
	byte axis1; // Aileron (Steering for car)
	byte axis2; // Elevator
	byte axis3; // Throttle
	byte axis4; // Rudder
	boolean mode1 = false; // Mode1 (toggle speed limitation)
	boolean mode2 = false; // Mode2 (toggle acc. / dec. limitation)
	boolean momentary1 = false; // Momentary push button
	byte pot1; // Potentiometer
};
RcData data;

// This struct defines data, which are embedded inside the ACK payload
struct ackPayload {
	float vcc; // vehicle vcc voltage
	float batteryVoltage; // vehicle battery voltage
	boolean batteryOk = true; // the vehicle battery voltage is OK!
	byte channel; // the channel number
//	const char Name[5] = "JEEP";
};
ackPayload payload;

// Battery voltage detection pin
#define BATTERY_DETECT_PIN A7 // The 20k (to battery) & 10k (to GND) battery detection voltage divider is connected to pin A7

// Create Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Special functions
#define DIGITAL_OUT_1 1 // 1 = TXO Pin

// Headlight off delay
unsigned long millisLightOff = 0;

// Indicators
boolean left;
boolean right;
boolean hazard;

// Motors
boolean isDriving; // is the vehicle driving?

// Motor objects
TB6612FNG Motor1;
TB6612FNG Motor2;

// Engine sound
boolean engineOn = false;

//
// =======================================================================================================
// RADIO SETUP
// =======================================================================================================
//
void setupRadio() {
	radio.begin(); // активировать модуль
	radio.setChannel(NRFchannel); // выбираем канал (в котором нет шумов!)

	// Set Power Amplifier (PA) level to one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
	radio.setPALevel(RF24_PA_MAX);  // уровень мощности передатчика
	pipeNo = vehicleNumber;
	radio.setDataRate(RF24_250KBPS);
	radio.setAutoAck(pipeIn[vehicleNumber - 1], true); // Ensure autoACK is enabled
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.setRetries(5, 5); // 5x250us delay (blocking!!), max. 5 retries (время между попыткой достучаться, число попыток)
	//radio.setCRCLength(RF24_CRC_8);          // Use 8-bit CRC for performance

#ifdef DEBUG
  radio.printDetails();
  delay(3000);
#endif

	radio.openReadingPipe(1, pipeIn[vehicleNumber - 1]);
	radio.startListening();
}

//
// =======================================================================================================
// MOTOR DRIVER SETUP
// =======================================================================================================
//

void setupMotors() {

	// TB6612FNG H-Bridge pins
	// ---- IMPORTANT ---- The pin assignment depends on your board revision and is switched here to match!
	const byte motor1_in1 = 4;
	const byte motor1_in2 = 9;
	const byte motor1_pwm = 6;

	byte motor2_in1;
	const byte motor2_in2 = 2;
	byte motor2_pwm;

	// Switchable pins:
	if (boardVersion >= 1.3) { // Board version >= 1.3:
		motor2_in1 = 5;  // 5
		motor2_pwm = 3;  // 3
	} else { // Board Version < 1.3:
		motor2_in1 = 5;  // 3
		motor2_pwm = 3;  // 5
	}

	// SYNTAX: IN1, IN2, PWM, min. input value, max. input value, neutral position width
	// invert rotation direction true or false
	Motor1.begin(motor1_in1, motor1_in2, motor1_pwm, 0, 100, 4, false); // Drive motor
	Motor2.begin(motor2_in1, motor2_in2, motor2_pwm, 0, 100, 4, false); // Steering motor (Drive in "HP" version)

	// Motor PWM frequency prescalers (Requires the PWMFrequency.h library)
	// Differential steering vehicles: locked to 984Hz, to make sure, that both motors use 984Hz.
	if (vehicleType == 1 || vehicleType == 2 || vehicleType == 6)
		pwmPrescaler2 = 32;

	// ----------- IMPORTANT!! --------------
	// Motor 1 always runs @ 984Hz PWM frequency and can't be changed, because timers 0 an 1 are in use for other things!
	// Motor 2 (pin 3) can be changed to the following PWM frequencies: 32 = 984Hz, 8 = 3936Hz, 1 = 31488Hz
	setPWMPrescaler(3, pwmPrescaler2); // pin 3 is hardcoded, because we can't change all others anyway
}

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
  printf_begin();
  delay(3000);
#endif

#ifndef DEBUG
//  Serial.end(); // make sure, serial is off!
	UCSR0B = 0b00000000;
#endif

	// Radio setup
	setupRadio();

	// Servo pins
	servo1.attach(A0);

	if (!tailLights)
		servo2.attach(A1);
	if (!engineSound && !toneOut)
		servo3.attach(A2);
	if (!beacons)
		servo4.attach(A3);

	// All axes to neutral position
	data.axis1 = 50;
	data.axis2 = 50;
	data.axis3 = 50;
	data.axis4 = 50;

	// Special functions
	if (TXO_momentary1)
		pinMode(DIGITAL_OUT_1, OUTPUT);

	// Motor driver setup
	setupMotors();

}

//
// =======================================================================================================
// LED
// =======================================================================================================
//

// Brake light subfunction for ESC vehicles
boolean escBrakeActive() {
	static byte driveState;
	boolean brake = true;

	switch (driveState) { // 0 = neutral, 1 = forward, 2 = reverse, 3 = brake

	case 0: // neutral
		if (data.axis3 > 55)
			driveState = 1; // forward
		if (data.axis3 < 45)
			driveState = 2; // reverse
		brake = false;
		break;

	case 1: // forward
		if (data.axis3 < 45)
			driveState = 3; // brake
		brake = false;
		break;

	case 2: // reverse
		if (data.axis3 > 55)
			driveState = 1; // forward
		brake = false;
		break;

	case 3: // brake
		if (data.axis3 > 45)
			driveState = 2; // go to reverse, if above neutral
		brake = true;
		break;

	}
	return brake;
}

//
// =======================================================================================================
// READ RADIO DATA
// =======================================================================================================
//

void readRadio() {

	static unsigned long lastRecvTime;

	if (radio.available(&pipeNo)) {
		radio.read(&data, sizeof(struct RcData)); // read the radia data and send out the ACK payload

		radio.writeAckPayload(pipeNo, &payload, sizeof(struct ackPayload)); // prepare the ACK payload

		hazard = false;
		lastRecvTime = millis();
#ifdef DEBUG
    Serial.print(data.axis1);
    Serial.print("\t");
    Serial.print(data.axis2);
    Serial.print("\t");
    Serial.print(data.axis3);
    Serial.print("\t");
    Serial.print(data.axis4);
    Serial.println("\t");
#endif
	}

	// Switch channel
	if (millis() - lastRecvTime > 500) {
		radio.setChannel(NRFchannel);
		payload.channel = NRFchannel;
	}

	if (millis() - lastRecvTime > 1000) { // set all analog values to their middle position, if no RC signal is received during 1s!
		data.axis1 = 50; // Aileron (Steering for car)
		data.axis2 = 50; // Elevator
		data.axis3 = 50; // Throttle
		data.axis4 = 50; // Rudder
		hazard = true; // Enable hazard lights
		payload.batteryOk = true; // Clear low battery alert (allows to re-enable the vehicle, if you switch off the transmitter)
#ifdef DEBUG
    Serial.println("No Radio Available - Check Transmitter!");
#endif
	}

	if (millis() - lastRecvTime > 2000) {
		setupRadio(); // re-initialize radio
		lastRecvTime = millis();
	}
}

//
// =======================================================================================================
// WRITE SERVO POSITIONS
// =======================================================================================================
//

void writeServos() {

	// Throttle (for ESC control, if you don't use the internal TB6612FNG motor driver)
	if (data.mode1) { // limited speed!
		servo3.write(map(data.axis1, 100, 0, lim3Llow, lim3Rlow)); // less than +/- 45°
	} else { // full speed!
		servo3.write(map(data.axis1, 100, 0, lim3L, lim3R)); // 45 - 135°
	}
}

//
// =======================================================================================================
// DRIVE MOTORS CAR (for cars, one motor for driving, one for steering)
// =======================================================================================================
//

void driveMotorsCar() {

	int maxPWM;
	byte maxAcceleration;

	// Speed limitation (max. is 255)
	if (data.mode1) {
		maxPWM = maxPWMlimited; // Limited
	} else {
		maxPWM = maxPWMfull; // Full
	}

	if (!payload.batteryOk && liPo)
		data.axis1 = 50; // Stop the vehicle, if the battery is empty!

	// Acceleration & deceleration limitation (ms per 1 step input signal change)
	if (data.mode2) {
		maxAcceleration = maxAccelerationLimited; // Limited
	} else {
		maxAcceleration = maxAccelerationFull; // Full
	}

	// ***************** Note! The ramptime is intended to protect the gearbox! *******************
	// SYNTAX: Input value, max PWM, ramptime in ms per 1 PWM increment
	// false = brake in neutral position inactive

	if (!HP) {
		if (Motor1.drive(data.axis2, minPWM, maxPWM, maxAcceleration, true)
				&& Motor2.drive(data.axis2, minPWM, maxPWM, maxAcceleration,
						true)) { // The drive motor (function returns true, if not in neutral)
			millisLightOff = millis(); // Reset the headlight delay timer, if the vehicle is driving!
		}
	}
}

//
// =======================================================================================================
// CHECK RX BATTERY & VCC VOLTAGES
// =======================================================================================================
//

boolean battSense;

void checkBattery() {

	if (boardVersion < 1.2)
		battSense = false;
	else
		battSense = true;

	// switch between load and no load contition
	if (millis() - millisLightOff >= 1000) { // one s after the vehicle did stop
		isDriving = false; // no load
	} else {
		isDriving = true; // under load
	}

	// Every 1000 ms, take measurements
	static unsigned long lastTrigger;
	if (millis() - lastTrigger >= 1000) {
		lastTrigger = millis();

		// Read both averaged voltages
		payload.batteryVoltage = batteryAverage();
		payload.vcc = vccAverage();

		if (battSense) { // Observe battery voltage
			if (payload.batteryVoltage <= cutoffVoltage)
				payload.batteryOk = false;
		} else { // Observe vcc voltage
			if (payload.vcc <= cutoffVoltage)
				payload.batteryOk = false;
		}
	}
}

// Voltage read & averaging subfunctions -----------------------------------------
// vcc ----
float vccAverage() {
	static int raw[6];

	if (raw[0] == 0) {
		for (int i = 0; i <= 5; i++) {
			raw[i] = readVcc(); // Init array
		}
	}

	raw[5] = raw[4];
	raw[4] = raw[3];
	raw[3] = raw[2];
	raw[2] = raw[1];
	raw[1] = raw[0];
	raw[0] = readVcc();
	float average = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5])
			/ 6000.0;
	return average;
}

// battery ----
float batteryAverage() {
	static int raw[6];

	if (!battSense)
		return 0;

	if (raw[0] == 0) {
		for (int i = 0; i <= 5; i++) {
			raw[i] = analogRead(BATTERY_DETECT_PIN); // Init array
		}
	}

	raw[5] = raw[4];
	raw[4] = raw[3];
	raw[3] = raw[2];
	raw[2] = raw[1];
	raw[1] = raw[0];
	if (isDriving && HP)
		raw[0] = (analogRead(BATTERY_DETECT_PIN) + 31); // add 0.3V while driving (HP version only): 1023 steps * 0.3V / 9.9V = 31
	else
		raw[0] = analogRead(BATTERY_DETECT_PIN); // else take the real voltage (compensates voltage drop while driving)
	float average = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5])
			/ 619.999; // 1023steps / 9.9V * 6 = 619.999
	return average;
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {

	// Read radio data from transmitter
	readRadio();

	// Write the servo positions
	writeServos();

	// Drive the motors
	driveMotorsCar(); // Car

	// Battery check
	checkBattery();
}

