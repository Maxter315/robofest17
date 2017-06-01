#include <AFMotor.h>
#include <stdint.h>
//#include <Servo.h>

#define DELTATIME 200

#define LED1 2

#define PWMMAX 255
#define BASEPWM 200

/* ----- Variables & Objects ----- */
AF_DCMotor leftMotor(1);
AF_DCMotor rightMotor(2);

uint8_t pwm_l, pwm_r;
int16_t pwm_l_c, pwm_r_c;
float Kp = 4.0, Kd = 0;
int16_t pr_error;

unsigned long prevMillis, currentMillis;

int16_t s1,s2,s3,s4,s5;		//readings
int16_t sens[5];			//readings
float eq[5];				//equal coeffs

uint8_t minSens, maxSens;
uint16_t minVal, maxVal;
uint16_t thrL, thrR;

int16_t tmp_sig;
int16_t prev_pos;


/* ----- Functions ----- */
uint8_t sensInit(void);
void readSensors(void);
int16_t defineLine(void);
int16_t reactPID(int16_t input);
uint8_t modeCtrl(void);
void reactDRV(int16_t in);


/* ============= INITIALIZATION ================ */
void setup(){
	Serial.begin(9600);

	leftMotor.run(RELEASE);
	rightMotor.run(RELEASE);

	pinMode(LED1, OUTPUT);

	sensInit();
}

/* ================ MAIN LOOP ================== */
void loop(){
  digitalWrite(LED1,LOW);
	currentMillis = millis();
	if (currentMillis - prevMillis >= DELTATIME){
		prevMillis = currentMillis;
		digitalWrite(LED1,HIGH);
		readSensors();
    
	  tmp_sig	= reactPID(defineLine());
	  //Serial.println(tmp_sig);
		reactDRV(tmp_sig);

	}
}
/* ================ END OF LOOP ================ */


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
/*
  Serial.print("eq[");
  Serial.print(i);
  Serial.print("]: ");
  Serial.println(eq[i]);
 */
}


	if ((maxVal - minVal) < 100){
		out = 1;
	}
/*
  Serial.print("min:");
  Serial.print(minVal);
  Serial.print("\tmax:");
  Serial.println(maxVal);
  */
	delay(10);
	digitalWrite(LED1, HIGH);
	delay(1500);
	digitalWrite(LED1, LOW);

	return out;
}

void readSensors(void){

  if(0){
	sens[0] = 1024 - analogRead(A1);
	sens[1] = 1024 - analogRead(A2);
	sens[2] = 1024 - analogRead(A3);
	sens[3] = 1024 - analogRead(A4);
	sens[4] = 1024 - analogRead(A5);
  }else{
  sens[0] = analogRead(A1);
  sens[1] = analogRead(A2);
  sens[2] = analogRead(A3);
  sens[3] = analogRead(A4);
  sens[4] = analogRead(A5);
    
  }
	minSens = 0;
	maxSens = 0;

	for (uint8_t i = 1; i < 5; i++){
		if (sens[maxSens] < sens[i]) 	maxSens = i;
		if (sens[minSens] > sens[i]) 	minSens = i;
	}
}


int16_t defineLine(void){
	int16_t out;
	
	//int16_t acc;
	//int16_t tmp[5];
	float tmp[5], acc;
	int16_t minv;
	minv = sens[minSens];
	if(minv> 400) minv = 400;
 
uint8_t on_line = 0;
  
	if(0){
		tmp[0] = eq[0] * (float)(sens[0] - minv);
		tmp[1] = eq[1] * (float)(sens[1] - minv);
		tmp[2] = eq[2] * (float)(sens[2] - minv);
		tmp[3] = eq[3] * (float)(sens[3] - minv);
		tmp[4] = eq[4] * (float)(sens[4] - minv);
	}else{
    for(int i=0;i<5;i++){
		tmp[i] = (float)(sens[i] - minv);
		if (tmp[i]>100) on_line = 1;
		}
   }


  Serial.print(tmp[0]); Serial.print("\t");
  Serial.print(tmp[1]); Serial.print("\t");
  Serial.print(tmp[2]); Serial.print("\t");
  Serial.print(tmp[3]); Serial.print("\t");
  Serial.println(tmp[4]);

  
	acc += tmp[0] * -100;
	acc += tmp[1] * -50;
	acc += tmp[2] * 0;
	acc += tmp[3] * 50;
	acc += tmp[4] * 100;

	out = (int16_t)((acc / (tmp[0] + tmp[1] + tmp[2] + tmp[3] + tmp[4])));

 if(!on_line){
  if (prev_pos < 0) out = -50;
  else out = 50;
 }else{
  prev_pos = out;
 }
/*
	if 		((sens[1] > 600) && (sens[3] < 500)) out = 32;
	else if ((sens[3] > 600) && (sens[1] < 500)) out = -32;
	else out = 0;
*/
	return out;
}


int16_t reactPID(int16_t in){
	int16_t out;

out = (int16_t)(Kp * (float)in + Kd * (float)(in - pr_error));
pr_error = in;
 /*
	if(in > 127) in = 127;
	else if(in<-128) in = -128;
	out = (int8_t)in;
	*/
  /*
  Serial.print("in: "); Serial.print(in);
  Serial.print("\tout: "); Serial.print(out); */
	return out;
}


uint8_t modeCtrl(void){
	uint8_t res;

	return res;
}


void reactDRV(int16_t sig){

	pwm_l_c = BASEPWM + sig;
	pwm_r_c = BASEPWM - sig;

	if(pwm_l_c > PWMMAX) pwm_l_c = PWMMAX;
	if(pwm_r_c > PWMMAX) pwm_r_c = PWMMAX;
	if(pwm_l_c < 0) pwm_l_c = 0;
	if(pwm_r_c < 0) pwm_r_c = 0;
  
/*
  Serial.print("\t");
  Serial.print(pwm_l_c);
  Serial.print("\t");
  Serial.println(pwm_r_c);
*/

	leftMotor.setSpeed(pwm_l_c);
	leftMotor.run(FORWARD);

	rightMotor.setSpeed(pwm_r_c);
	rightMotor.run(FORWARD);
}
