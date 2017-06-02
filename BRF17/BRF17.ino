#include <AFMotor.h>
#include <stdint.h>
//#include <Servo.h>

#define DELTATIME 200

#define LED1 2

#define BUTL 9
#define BUTR 13
#define BUTC 0

#define PWMMAX 255
#define BASEPWM 200

/* ---------- Variables & Objects ---------- */
AF_DCMotor leftMotor(1);
AF_DCMotor rightMotor(2);

uint8_t pwm_l, pwm_r;
int16_t pwm_l_c, pwm_r_c;

float Kp = 4.0, Kd = 0;
int16_t pr_error;

uint8_t button_state, prev_bs;
uint8_t pr_bc = 1, pr_bl = 1, pr_br = 1;
uint8_t	r_is_pressed, l_is_pressed, c_is_pressed;
uint8_t var_ptr = 0;
int16_t sig_thr;
int16_t base_pwm = 200;
int16_t pwm_max = 255;

uint8_t cntr = 1000;

unsigned long prevMillis, currentMillis;

int16_t s1,s2,s3,s4,s5;		//readings
int16_t sens[5];			//readings
float eq[5];				//equal coeffs

uint8_t minSens, maxSens;
uint16_t minVal, maxVal;
uint16_t thrL, thrR;

int16_t tmp_sig;
int16_t prev_pos;
/* ----------------------------------------- */

/* --------------- Functions --------------- */
uint8_t sensInit(void);
void readSensors(void);
int16_t defineLine(void);
int16_t reactPID(int16_t input);
void reactDRV(int16_t in);
void varMod(void)

/* ============================================= */
/* ============= INITIALIZATION ================ */
void setup(){
	Serial.begin(9600);

	leftMotor.run(RELEASE);
	rightMotor.run(RELEASE);

	pinMode(LED1, OUTPUT);

	pinMode(BUTC, INPUT);
	pinMode(BUTL, INPUT);
	pinMode(BUTR, INPUT);

	sensInit();
}

/* ================ MAIN LOOP ================== */
void loop(){

	currentMillis = millis();
	if (currentMillis - prevMillis >= DELTATIME){
		prevMillis = currentMillis;
	
		checkButt();
		varMod();
		showSelectedVar();

		readSensors();
		tmp_sig = reactPID(defineLine());
		//Serial.println(tmp_sig);

		if(var_ptr == 5){
			leftMotor.run(RELEASE);
			rightMotor.run(RELEASE);
		}else{
			reactDRV(tmp_sig);
		}
	}
}
/* ================ END OF LOOP ================ */
/* ============================================= */

/* ---------- Functions ---------- */
uint8_t sensInit(void){
	uint8_t out;

	digitalWrite(LED1, HIGH);
	delay(1000);
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
		if (tmp[i]>sig_thr) on_line = 1;
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

	return out;
}


int16_t reactPID(int16_t in){
	int16_t out;

out = (int16_t)(Kp * (float)in + Kd * (float)(in - pr_error));
pr_error = in;

	/*Serial.print("in: "); Serial.print(in);
	Serial.print("\tout: "); Serial.print(out); */
	return out;
}

void reactDRV(int16_t sig){

	pwm_l_c = base_pwm + sig;
	pwm_r_c = base_pwm - sig;

	if(pwm_l_c > pwm_max) pwm_l_c = pwm_max;
	if(pwm_r_c > pwm_max) pwm_r_c = pwm_max;
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

void varMod(void){

	if(c_is_pressed) var_ptr++;
	if(var_ptr > 6) var_ptr = 0;

	if(var_ptr == 0){
		if(r_is_pressed) {	sig_thr += 5;		r_is_pressed = 0;}
		if(l_is_pressed) {	sig_thr -= 5;		l_is_pressed = 0;}
	}else if(var_ptr == 1){
		if(r_is_pressed) {	base_pwm += 5;		r_is_pressed = 0;}
		if(l_is_pressed) {	base_pwm -= 5;		l_is_pressed = 0;}
	}else if(var_ptr == 2){
		if(r_is_pressed) {	pwm_max += 5;		r_is_pressed = 0;}
		if(l_is_pressed) {	pwm_max -= 5;		l_is_pressed = 0;}
	}else if(var_ptr == 3){
		if(r_is_pressed) {	Kp += 0.1;		r_is_pressed = 0;}
		if(l_is_pressed) {	Kp -= 0.1;		l_is_pressed = 0;}
	}else if(var_ptr == 4){
		if(r_is_pressed) {	Kd += 0.1;		r_is_pressed = 0;}
		if(l_is_pressed) {	Kd -= 0.1;		l_is_pressed = 0;}
	}
}

void checkButt(void){
	uint8_t curr_st;

	curr_st = digitalRead(BUTC);
	if (curr_st && !pr_bc) c_is_pressed = 1;
	pr_bc = curr_st;

	curr_st = digitalRead(BUTL);
	if (curr_st && !pr_bl) l_is_pressed = 1;
	pr_bl = curr_st;

	curr_st = digitalRead(BUTR);
	if (curr_st && !pr_br) r_is_pressed = 1;
	pr_br = curr_st;

}


void showSelectedVar(void){
	if(cntr & (0x80>>var_ptr)){
		digitalWrite(LED1, HIGH);
	}else{
		digitalWrite(LED1, LOW);
	}
	cntr++;
}
