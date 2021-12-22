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

long target  = 0;
long target1 = 0;
int  amp     = 212;

//PID controller constants
float KP = 6.0; //position multiplier (gain) 2.25
float KI = 0.1; // Intergral multiplier (gain) .25
float KD = 1.3; // derivative multiplier (gain) 1.0


int lastError = 0;
int sumError  = 0;

//Integral term min/max (random value and not yet tested/verified)
int iMax = 100;
int iMin = 0;

long previousTarget = 0;
long previousMillis = 0;        // will store last time LED was updated
long interval       = 5;        // interval at which to blink (milliseconds)

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

	// Some Tests
	/* 
	Serial.println("start tests");

		Serial.println("led on");
		digitalWrite(EnableLED, HIGH);

		Serial.println("motor on");
		digitalWrite( MotorDIR, HIGH);
		analogWrite ( MotorPWM,  255);
		delay(10000);
		digitalWrite(EnableLED, LOW);

	Serial.println("end tests");
	*/



	attachInterrupt(encoder0PinB, doEncoderMotor0, CHANGE);  // encoderA pin on interrupt
	attachInterrupt(STEP_PIN, 	  countStep, 	   RISING);  // interrupt to count steppules

	Serial.println("start");
} 

void loop(){
  
	while (Serial.available() > 0) {
		KP = Serial.read();
		KD = Serial.read();
		KI = Serial.read();


		Serial.println(KP);
		Serial.println(KD);
		Serial.println(KI);
	}
  
	if(millis() - previousTarget > 500){ //enable this code only for test purposes
		Serial.print(encoder0Pos);
		Serial.print(',');
		Serial.println(target1);
		previousTarget=millis();
	}
        
	target = target1;
	docalc();
}

void docalc() {
  
	if (millis() - previousMillis > interval) 
	{
		previousMillis = millis();   // remember the last time we blinked the LED

		long error = encoder0Pos - target ; // find the error term of current position - target    

		//generalized PID formula
		//correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)
		long ms = KP * error + KD * (error - lastError) +KI * (sumError);
		   
		lastError = error;    
		sumError += error;

		//scale the sum for the integral term
		if(sumError > iMax) {
			sumError = iMax;
		} else if(sumError < iMin){
			sumError = iMin;
		}

		if(ms > 0){
			digitalWrite ( MotorDIR ,HIGH );      
		}
		if(ms < 0){
			digitalWrite ( MotorDIR , LOW );     
			ms = -1 * ms;
		}

		int motorspeed = map(ms,0,amp,0,255);
		if( motorspeed >= 255) motorspeed=255;
		//analogWrite ( MotorPWM, (255 - motorSpeed) );
		analogWrite ( MotorPWM,  motorspeed );
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
	if (dir) target1++;
	else target1--;
}
