// link to decent firmware https://github.com/hbrobotics/ros_arduino_bridge/tree/indigo-devel/ros_arduino_firmware/src/libraries
// link to ros side controller http://wiki.ros.org/diff_drive_controller
// link to rosserial udp example https://github.com/ros-drivers/rosserial/blob/dd76994c67c5e4997ef64837c07afb4eb0d4df27/rosserial_arduino/src/ros_lib/examples/TcpHelloWorld/TcpHelloWorld.ino#L15
// arduino PID lib https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp


// try to make this the only arduino dependent File

#include <Arduino.h>

// instantiate motion controller class
// make the motion controller class non arduino specific implementation

void setup() {
  Serial.begin(115200);
  // set all control variables initial value
  // maybe control variables from file also
  // start network connection
  // setup webserver for control parameters
  // handshake control regime unless already setup
  // save settings to File
  // check initial diagnostics (battery etc)
}

void loop() {
  // check diagnostics
  // get control commands
  // convert control commands to motion profile
  // update PID model
  // update motion commands
  // publish motion and diagnostics
}



void setup()
{
   pinMode(M_left, OUTPUT);   //L298P Control port settings DC motor driver board for the output mode
   pinMode(E_left, OUTPUT);
   Setpoint =80;  //Set the output value of the PID
   myPID.SetMode(AUTOMATIC);//PID is set to automatic mode
   myPID.SetSampleTime(100);//Set PID sampling frequency is 100ms
  EncoderInit();//Initialize the module
  initMotor();
}

void loop()
{
  advance();//Motor Forward
  abs_duration=abs(duration);
  result=myPID.Compute();//PID conversion is complete and returns 1
  if(result)
  {
    Serial.print("Pluse: ");
    Serial.println(duration);
    duration = 0; //Count clear, wait for the next count
  }
}

/// or

ISR(TIMER1_OVF_vect)        // interrupt service routine - tick every 0.1sec
{
   TCNT1 = timer1_counter;   // set timer
   pv_speed = 60.0*(encoder/200.0)/0.1;  //calculate motor speed, unit is rpm
   encoder=0;
   //print out speed
   if (Serial.available() <= 0) {
     Serial.print("speed");
     Serial.println(pv_speed);         //Print speed (rpm) value to Visual Studio
     }
  //PID program
   if (motor_start){
     e_speed = set_speed - pv_speed;
     pwm_pulse = e_speed*kp + e_speed_sum*ki + (e_speed - e_speed_pre)*kd;
     e_speed_pre = e_speed;  //save last (previous) error
     e_speed_sum += e_speed; //sum of error
     if (e_speed_sum >4000) e_speed_sum = 4000;
     if (e_speed_sum <-4000) e_speed_sum = -4000;
   }
   else{
     e_speed = 0;
     e_speed_pre = 0;
     e_speed_sum = 0;
     pwm_pulse = 0;
   }
  //update new speed
   if (pwm_pulse <255 & pwm_pulse >0){
     analogWrite(pin_pwm,pwm_pulse);  //set motor speed
   }
   else{
     if (pwm_pulse>255){
       analogWrite(pin_pwm,255);
     }
     else{
       analogWrite(pin_pwm,0);
     }
   }
}

/// or

volatile long count = 0;
boolean A,B;
unsigned long timep, time, etime;
byte state, statep;
#define pwm  11
#define dir1 4
#define dir2 5

int speed_req = 120;
int speed_actual = 0;
double Kp = 0.5;
double Kd = 1;
int PWM_val = 0;
unsigned long lastTime = 0;
unsigned long lastTime_print = 0;
#define LOOPTIME 100

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  // attachInterrupt(0,Achange,CHANGE);
  // attachInterrupt(1,Bchange,CHANGE);
  timep = millis();
  // A = digitalRead(2);
  // B = digitalRead(3);
  // if ((A==HIGH)&&(B==HIGH)) statep = 1;
  // if ((A==HIGH)&&(B==LOW)) statep = 2;
  // if ((A==LOW)&&(B==LOW)) statep = 3;
  // if((A==LOW)&&(B==HIGH)) statep = 4;
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm, OUTPUT);
}

void loop() {
  if((millis() - lastTime) >= LOOPTIME) {
    lastTime = millis();
    findMotorData();
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwm, PWM_val);
    }
   printMotorInfo();
}

void findMotorData() {
  static long count_prev = 0;
  speed_actual = ((count - count_prev)*(60*(1000/LOOPTIME)))/400;
  count_prev = count;

  float pidTerm = 0;
  int error = 0;
  int last_error = 0;
  error = abs(speed_req) - abs(speed_actual);
  pidTerm = (Kp * error) + (Kd * (error - last_error));
  last_error = error;
  PWM_val = constrain(pidTerm, 0, 255);
}
