/* 

i have made this code for the LMD18245 motor controller, 
i have merged the pid code of  Josh Kopel 
whith the code of makerbot servo-controller board,
you can use this code on the some board changing some values.
Daniele Poddighe

external ardware require a quadrature encoder, timing slit strip and a dc motor,
all you can find inside an old printer, i have took it from canon and hp printers(psc1510)

for motor controll you can choose different type of H-bridge, i have used LMD18245,
you can order 3 of it on ti.com sample request, the hardware needed is explained on the datasheet but i'm drowing
the schematic and PCB layout on eagle.

read a rotary encoder with interrupts
Encoder hooked up with common to GROUND,
encoder0PinA to pin 2, encoder0PinB to pin 4 (or pin 3 see below)
it doesn't matter which encoder pin you use for A or B 

is possible to change PID costants by sending on SerialUSB interfaces the values separated by ',' in this order: KP,KD,KI
example: 5.2,3.1,0 so we have  KP=5.2 KD=3.1 KI=0 is only for testing purposes, but i will leave this function with eeprom storage

Changes from Frank Herrmann for XMoto

*/ 

#include <PID_v2.h>


// Connect to Hall Sensor PCB
#define encoder0PinA  	PA8
#define encoder0PinB  	PA7

#define MotorPWM    	PA1
#define MotorDIR 		PA0

//from ramps 1.4 stepper driver
#define STEP_PIN		PA9
#define DIR_PIN         PA10
#define EnableLED       PB3


volatile long encoder0Pos = 0;

long ToSteps  = 0;
long GetSteps = 0;

// PID_v2
double kp = 6 , ki = 0.1 , kd = 0.01;
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  


int lastError = 0;
int sumError  = 0;

//Integral term min/max (random value and not yet tested/verified)
int iMax = 20000;
int iMin = 0;

long LastInterval = 0;
long previousMillis = 0;        // will store last time LED was updated
long interval       = 5;        // interval at which to blink (milliseconds)

int amp     	= 32500; // maximum PWN Value
int correction	= 0;	 // PID value
int motorspeed  = 0;

//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir     = false;

void setup() { 

	Serial.begin(9600);

	// Set Pins 
	pinMode(encoder0PinA, INPUT); 
	pinMode(encoder0PinB, INPUT);  

	pinMode(MotorDIR, OUTPUT); 
	pinMode(MotorPWM, OUTPUT);
	pinMode(EnableLED, OUTPUT);

	//motor control Pins
	pinMode(STEP_PIN, INPUT);
	pinMode(DIR_PIN, INPUT);

	// Interupts for position and target steps 
	attachInterrupt(encoder0PinB, doEncoderMotor0, CHANGE);  // encoderA pin on interrupt
	attachInterrupt(STEP_PIN, 	  countStep, 	   RISING);  // interrupt to count steppules

	analogWriteFrequency(350000);	// Set PWM Frequenzy to 35KHz

	myPID.SetMode(AUTOMATIC);   	//set PID in Auto mode
	myPID.SetSampleTime(50);  		// refresh rate of PID controller
	myPID.SetOutputLimits(-255, 255); // this is the MAX PWM value to move motor

	Serial.println("start");
	Serial.println("Position ToPosition MotorPWM");

} 

void loop(){
  
	// Diganostic via serialplotter from arduino ide
	if(millis() - LastInterval > 50 && abs(output) > 29){
		Serial.print(input);
		Serial.print(" ");
		Serial.print(output);
		Serial.print(" ");
		Serial.println(setpoint);
		LastInterval=millis();
	}
        
	setpoint = GetSteps; // setpoint to Steps get from controller
	input = encoder0Pos; // real Position
	myPID.Compute();	 // compute correct value
	pwmOut(output);		 // get PWM Value from PID calculated
}

void pwmOut(int out) {                               
	if (out > 0) {
		analogWrite( MotorPWM, out );      	
		digitalWrite ( MotorDIR ,LOW );
	}
	else {
		analogWrite( MotorPWM, abs(out) );                      
		digitalWrite ( MotorDIR, HIGH );		// if REV < encoderValue motor move in reverse direction.   
  }
}

void doEncoderMotor0(){
	if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
		if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
												 // encoder is turning
			encoder0Pos = encoder0Pos - 1;         // CCW
		} 
		else {
			encoder0Pos = encoder0Pos + 1;         // CW
		}
	}
	else                                        // found a high-to-low on channel A
	{ 
		if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
												  // encoder is turning  
			encoder0Pos = encoder0Pos + 1;          // CW
		} 
		else {
			encoder0Pos = encoder0Pos - 1;          // CCW
		}
	}
}

void countStep(){
	dir = digitalRead(DIR_PIN);
	if (dir) GetSteps++;
	else GetSteps--;
}
