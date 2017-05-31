#include <AFMotor.h>
#include <stdint.h>
//#include <Servo.h>

#define DELTATIME 100

#define LED1 7
#define LED2 8
#define LED3 9
#define LED4 10
#define LED5 11

#define PWMMAX 100
#define SLOPE_K 2

/* ----- Variables & Objects ----- */
AF_DCMotor leftMotor(2);
AF_DCMotor rightMotor(1);

uint8_t pwm_l, pwm_r;
uint8_t pwm_l_c, pwm_r_c;

unsigned long prevMillis, currentMillis;

int16_t s1,s2,s3,s4,s5;		//readings
int16_t sens[5];			//readings
uint8_t minSens, maxSens;
uint16_t minVal, maxVal;
uint16_t thrL, thrR;



/* ----- Functions ----- */
uint8_t sensInit(void);
void readSensors(void);
int16_t defineLine(void);
int8_t reactPID(int16_t input);
uint8_t modeCtrl(void);
void reactDRV(int8_t in);


/* ----- INITIALIZATION ----- */
void setup(){
/*
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);
	pinMode(A4, INPUT);
	pinMode(A5, INPUT);
*/
	leftMotor.run(RELEASE);
	rightMotor.run(RELEASE);

	//pinMode(LED1, OUTPUT);
	//pinMode(LED2, OUTPUT);
	//pinMode(LED3, OUTPUT);
	//pinMode(LED4, OUTPUT);
	//pinMode(LED5, OUTPUT);


}

/* ----- MAIN LOOP ----- */
void loop(){

	currentMillis = millis();
	if (currentMillis - prevMillis >= DELTATIME){
		prevMillis = currentMillis;
		
	}
}
/* ----- END OF LOOP ----- */


/* ----- Functions ----- */
uint8_t sensInit(void){
	uint8_t out;

	digitalWrite(LED1, HIGH);
	delay(500);
	digitalWrite(LED1, LOW);

	readSensors();
	minVal = sens[minSens];
	maxVal = sens[maxSens];

	if ((maxVal - minVal) < 100){
		out = 1;
	}

	delay(10);
	digitalWrite(LED1, HIGH);
	delay(500);
	digitalWrite(LED1, LOW);

	return out;
}

void readSensors(void){
	sens[0] = 1024 - analogRead(A1);
	sens[1] = 1024 - analogRead(A2);
	sens[2] = 1024 - analogRead(A3);
	sens[3] = 1024 - analogRead(A4);
	sens[4] = 1024 - analogRead(A5);

	minSens = 1;
	maxSens = 1;

	for (uint8_t i = 1; i < 5; i++){
		if (sens[i-1] < sens[i]) 	maxSens = i;
		else 						minSens = i;
	}
}


int16_t defineLine(void){
	int16_t out;
	
	int16_t acc;
	int16_t tmp[5];
	int16_t minv;
	minv = sens[minSens];

	//tmp[0] = sens[0] - minv;
	//tmp[1] = sens[1] - minv;
	//tmp[2] = sens[2] - minv;
	//tmp[3] = sens[3] - minv;
	//tmp[4] = sens[4] - minv;

	/*
	acc += tmp[0] * 0;
	acc += tmp[1] * 15;
	acc += tmp[2] * 30;
	acc += tmp[3] * 45;
	acc += tmp[4] * 60;

	out = acc / (tmp[0] + tmp[1] + tmp[2] + tmp[3] + tmp[4]);
	*/


	return out;
}


int8_t reactPID(int16_t in){
	int8_t out;

	return out;
}


uint8_t modeCtrl(void){
	uint8_t res;

	return res;
}


void reactDRV(int8_t sig){
/*
	if (sig > 0){
		pwm_l = 0;
		//pwm_l = -(KF) * sig + PWMMAX;
		pwm_r = PWMMAX;
	}else if (sig < 0){
		pwm_r = 0;
		//pwm_r = (KF) * sig + PWMMAX;
		pwm_l = PWMMAX;
	}else{
		pwm_r = PWMMAX;
		pwm_l = PWMMAX;
	}
*/
	//pwm_l_c = pwm_l;
	//pwm_r_c = pwm_r;
	
	pwm_l_c += sig;
	pwm_r_c -= sig;

	if(pwm_l_c > PWMMAX) pwm_l_c = PWMMAX;
	if(pwm_r_c > PWMMAX) pwm_r_c = PWMMAX;
	if(pwm_l_c < 0) pwm_l_c = 0;
	if(pwm_l_c < 0) pwm_l_c = 0;
	

	leftMotor.setSpeed(pwm_l_c);
	leftMotor.run(FORWARD);

	rightMotor.setSpeed(pwm_r_c);
	rightMotor.run(FORWARD);
}
