#include <AFMotor.h>
#include <stdint.h>
//#include <Servo.h>

#define DELTATIME 500

#define LED1 2
#define LED2 8
#define LED3 9
#define LED4 10
#define LED5 11

#define PWMMAX 100
#define SLOPE_K 2

/* ----- Variables & Objects ----- */
AF_DCMotor leftMotor(3);
AF_DCMotor rightMotor(4);

uint8_t pwm_l, pwm_r;
uint8_t pwm_l_c, pwm_r_c;

unsigned long prevMillis, currentMillis;

int16_t s1,s2,s3,s4,s5;		//readings
int16_t sens[5];			//readings
float eq[5];				//equal coeffs

uint8_t minSens, maxSens;
uint16_t minVal, maxVal;
uint16_t thrL, thrR;

int8_t tmp_sig;


/* ----- Functions ----- */
uint8_t sensInit(void);
void readSensors(void);
int16_t defineLine(void);
int8_t reactPID(int16_t input);
uint8_t modeCtrl(void);
void reactDRV(int8_t in);


/* ----- INITIALIZATION ----- */
void setup(){
	Serial.begin(9600);

	leftMotor.run(RELEASE);
	rightMotor.run(RELEASE);

	pinMode(LED1, OUTPUT);
	//pinMode(LED2, OUTPUT);
	//pinMode(LED3, OUTPUT);
	//pinMode(LED4, OUTPUT);
	//pinMode(LED5, OUTPUT);


	//leftMotor.setSpeed(125);
	//leftMotor.run(FORWARD);

	//rightMotor.setSpeed(255);
	//rightMotor.run(FORWARD);

	sensInit();
}

/* ----- MAIN LOOP ----- */
void loop(){

	currentMillis = millis();
	if (currentMillis - prevMillis >= DELTATIME){
		prevMillis = currentMillis;
		
		readSensors();/*
	Serial.print(sens[0]); Serial.print("\t");
	Serial.print(sens[1]); Serial.print("\t");
	Serial.print(sens[2]); Serial.print("\t");
	Serial.print(sens[3]); Serial.print("\t");
	Serial.println(sens[4]);

	Serial.print(eq[0]*((float)sens[0])); Serial.print("\t");
	Serial.print(eq[1]*((float)sens[1])); Serial.print("\t");
	Serial.print(eq[2]*((float)sens[2])); Serial.print("\t");
	Serial.print(eq[3]*((float)sens[3])); Serial.print("\t");
	Serial.println(eq[4]*((float)sens[4]));
  
  */
	tmp_sig	= reactPID(defineLine());
	Serial.println(tmp_sig);
		//reactDRV(tmp_sig);

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

  int16_t accum[5] = {0,0,0,0,0};

for(int t = 0; t<10; t++){
    readSensors();
    for(int o = 0; o<5; o++){
      accum[o] += sens[o];
    }
}

for(int i = 0; i<5; i++){
  eq[i] = ((float)accum[2])/((float)accum[i]);
  Serial.print("eq[");
  Serial.print(i);
  Serial.print("]: ");
  Serial.println(eq[i]);
 
}


	if ((maxVal - minVal) < 100){
		out = 1;
	}

  Serial.print("min:");
  Serial.print(minVal);
  Serial.print("\tmax:");
  Serial.println(maxVal);
	delay(10);
	digitalWrite(LED1, HIGH);
	delay(1500);
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
	
	//int16_t acc;
	//int16_t tmp[5];
	float tmp[5], acc;
	int16_t minv;
	minv = sens[minSens];
	if(minv> 200) minv = 200;

	if(0){
		tmp[0] = eq[0] * (float)(sens[0] - minv);
		tmp[1] = eq[1] * (float)(sens[1] - minv);
		tmp[2] = eq[2] * (float)(sens[2] - minv);
		tmp[3] = eq[3] * (float)(sens[3] - minv);
		tmp[4] = eq[4] * (float)(sens[4] - minv);
	}else{
		tmp[0] = (float)(sens[0] - minv);
		tmp[1] = (float)(sens[1] - minv);
		tmp[2] = (float)(sens[2] - minv);
		tmp[3] = (float)(sens[3] - minv);
		tmp[4] = (float)(sens[4] - minv);
	}
/*
  Serial.print(tmp[0]); Serial.print("\t");
  Serial.print(tmp[1]); Serial.print("\t");
  Serial.print(tmp[2]); Serial.print("\t");
  Serial.print(tmp[3]); Serial.print("\t");
  Serial.println(tmp[4]);
*/
  
	acc += tmp[0] * 0;
	acc += tmp[1] * 500;
	acc += tmp[2] * 1000;
	acc += tmp[3] * 1500;
	acc += tmp[4] * 2000;

	out = (int16_t)((acc / (tmp[0] + tmp[1] + tmp[2] + tmp[3] + tmp[4])));
	
/*
	if 		((sens[1] > 600) && (sens[3] < 500)) out = 32;
	else if ((sens[3] > 600) && (sens[1] < 500)) out = -32;
	else out = 0;
*/
	return out;
}


int8_t reactPID(int16_t in){
	int8_t out;
	out = (int8_t)in;
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
