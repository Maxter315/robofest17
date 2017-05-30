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
int16_t sens[5];			//readings
uint8_t minSens, maxSens;

int16_t defineLine(void);
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

int16_t defineLine(void){
	int16_t out;
	int16_t acc;
	int16_t tmp[5];

	tmp[0] = sens[0] - sens[minSens];
	tmp[1] = sens[1] - sens[minSens];
	tmp[2] = sens[2] - sens[minSens];
	tmp[3] = sens[3] - sens[minSens];
	tmp[4] = sens[4] - sens[minSens];

	acc += tmp[0] * 0;
	acc += tmp[1] * 15;
	acc += tmp[2] * 30;
	acc += tmp[3] * 45;
	acc += tmp[4] * 60;

	out = acc / (tmp[0] + tmp[1] + tmp[2] + tmp[3] + tmp[4]);

	return out;
}

int16_t calcReaction(int16_t in){
	int16_t out;

	return out;
}

void readSensors(void){
	sens[0] = analogRead(S1);
	sens[1] = analogRead(S2);
	sens[2] = analogRead(S3);
	sens[3] = analogRead(S4);
	sens[4] = analogRead(S5);

	minSens = 1;
	maxSens = 1;

for (uint8_t i = 1; i < 5; i++){
	if (sens[i-1] < sens[i]) 	maxSens = i;
	else 						minSens = i;
}


/*
	if (s1 < s2)	maxSens = 2;
	else 			minSens = 2;

	if (s2 < s3)	maxSens = 3;
	else 			minSens = 3;

	if (s3 < s4)	maxSens = 4;
	else 			minSens = 4;

	if (s4 < s5)	maxSens = 5;
	else 			minSens = 5;
*/
}