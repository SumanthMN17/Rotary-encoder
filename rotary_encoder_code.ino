#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA1 8 // channel a of encoder 1
#define ENCB1 9 // channel b of encoder 1
#define ENCA2 10 // channel a of encoder 2
#define ENCB2 11 // channel b of encoder 2
#define PWM1 6 // controls the speed of the motor 1
#define PWM2 7 // controls the speed of the motor 2
#define IN1 2 // positive input to control the direction of motor 1
#define IN2 3 // negative input to control the direction of motor 1
#define IN3 4 // positive input to control the direction of motor 2
#define IN4 5 // negative input to control the direction of motor 2

volatile int posi1 = 0,posi2 = 0; // specify posi as volatile, initial position
long prevT1 = 0, prevT2=0;// previous value
float eprev1 = 0, eprev2=0; // error difference between previous and present value
float eintegral1 = 0, eintegral2=0; // integral changes between two values.

void setup() {
Serial.begin(9600);
pinMode(ENCA1,INPUT);
pinMode(ENCB1,INPUT);
pinMode(ENCA2,INPUT);
pinMode(ENCB2,INPUT);
attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder,RISING); // this will trigered when encoder channel A is raised or motor rotated in motor 1
attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder,RISING); // this will trigered when encoder channel A is raised or motor rotated in motor 2
pinMode(PWM1,OUTPUT);
pinMode(PWM2,OUTPUT);
pinMode(IN1,OUTPUT);
pinMode(IN2,OUTPUT);
pinMode(IN3,OUTPUT);
pinMode(IN4,OUTPUT);  
Serial.println("target pos");
}

void loop() {
int target1 = 1000, target2 = 500; // we can give any value to set the position of motor shaft here it is 1000 for motor 1 and 500 for motor2

// here we are using PID method for the feedback to loop of encoder and motor to control // the precise position of the shaft

// random constants to check the preciseness of the motor shaft 
float kp = 1; 
float kd = 0.025;
float ki = 0.0;

// time difference
long currT1 = micros(), currT2 = micros(); // current position
float deltaT1 = ((float) (currT1 - prevT1))/( 1.0e6 );
float deltaT2 = ((float) (currT2 - prevT2))/( 1.0e6 );// error from targeted position
prevT1 = currT1;
prevT2 = currT2;

int pos1 = 0, pos2=0; 
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = posi1;
    pos2=posi2;
}
  
// error
int e1 = pos1 - target1;
int e2 = pos2 - target2;

// derivative
float dedt1 = (e1-eprev1)/(deltaT1);
float dedt2 = (e2-eprev2)/(deltaT2);

// integral
eintegral1 = eintegral1 + e1*deltaT1;
eintegral2 = eintegral2 + e2*deltaT2;

// control signal
float u1 = kp*e1 + kd*dedt1 + ki*eintegral1;
float u2 = kp*e2 + kd*dedt2 + ki*eintegral2;

// motor 1 power
float pwr1 = fabs(u1);
if( pwr1 > 255 ){
    pwr1 = 255;
}
// motor 2 power
float pwr2 = fabs(u2);
if( pwr2 > 255 ){
    pwr2 = 255;
}

// motor 1 direction
int dir1 = 1;
if(u1<0){
dir1 = -1;
}
// motor 2 direction
int dir2 = 1;
if(u2<0){
dir2 = -1;
}

// signal the motor
setMotor(dir1,pwr1,PWM1,IN1,IN2);
setMotor(dir2,pwr2,PWM2,IN3,IN4);

// store previous error
eprev1 = e1;
eprev2 = e2; 

Serial.print(target1);
Serial.print(" ");
Serial.print(pos1);
Serial.println();
Serial.print(target2);
Serial.print(" ");
Serial.print(pos2);
Serial.println();
}

// this function takes the pwm, direction of encoder signal obtained and direction of //motor to rotate based on the adjust the motor
 
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
analogWrite(pwm,pwmVal);
if(dir == 1){
digitalWrite(in1,HIGH);
digitalWrite(in2,LOW);
}
else if(dir == -1){
digitalWrite(in1,LOW);
digitalWrite(in2,HIGH);
}
else{
digitalWrite(in1,LOW);
digitalWrite(in2,LOW);
}  
}

void readEncoder(){
int b = digitalRead(ENCB1);
if(b > 0){
posi1++;
}
else{
posi1--;
}
int c = digitalRead(ENCB2);
if(c > 0){
posi2++;
}
else{
posi2--;
}
}
