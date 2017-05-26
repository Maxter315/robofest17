#include <AFMotor.h>
#include <stdint.h>
//#include <Servo.h>

#define DELTATIME 100

#define S1 0
#define S2 1
#define S3 2
#define S4 3
#define S5 4

AF_DCMotor leftMotor(2);
AF_DCMotor rightMotor(2);

unsigned long prevMillis, currentMillis;
int16_t s1,s2,s3,s4,s5;		//readings

int16_t defineLine(int16_t s1, int16_t s2, int16_t s3, int16_t s4, int16_t s5);
int16_t calcReaction(int16_t input);
void readSensors(void);

void setup(){
	pinMode(S1, INPUT);
	pinMode(S2, INPUT);
	pinMode(S3, INPUT);
	pinMode(S4, INPUT);
	pinMode(S5, INPUT);

	leftMotor.run(RELEASE);
	rightMotor.run(RELEASE);
}

void loop(){

	currentMillis = millis();
	if (currentMillis - prevMillis >= DELTATIME){
		prevMillis = currentMillis;
		
	}
}

int16_t defineLine(int16_t s1, int16_t s2, int16_t s3, int16_t s4, int16_t s5){
	int16_t out;
	int16_t acc;

	acc += s1 * 0;
	acc += s2 * 15;
	acc += s3 * 30;
	acc += s4 * 45;
	acc += s5 * 60;

	out = acc / (s1+s2+s3+s4+s5);

	return out;
}

int16_t calcReaction(int16_t in){
	int16_t out;

	return out;
}

void readSensors(void){
	s1 = analogRead(S1);
	s2 = analogRead(S2);
	s3 = analogRead(S3);
	s4 = analogRead(S4);
	s5 = analogRead(S5);
}