
void loop() {
   if (stringComplete) {
     // clear the string when COM receiving is completed
     mySt = "";  //note: in code below, mySt will not become blank, mySt is blank until '\n' is received
     stringComplete = false;
   }
  //receive command from Visual Studio
   if (mySt.substring(0,8) == "vs_start"){
     digitalWrite(pin_fwd,1);      //run motor run forward
     digitalWrite(pin_bwd,0);
     motor_start = true;
   }
   if (mySt.substring(0,7) == "vs_stop"){
     digitalWrite(pin_fwd,0);
     digitalWrite(pin_bwd,0);      //stop motor
     motor_start = false;
   }
   if (mySt.substring(0,12) == "vs_set_speed"){
     set_speed = mySt.substring(12,mySt.length()).toFloat();  //get string after set_speed
   }
   if (mySt.substring(0,5) == "vs_kp"){
     kp = mySt.substring(5,mySt.length()).toFloat(); //get string after vs_kp
   }
   if (mySt.substring(0,5) == "vs_ki"){
     ki = mySt.substring(5,mySt.length()).toFloat(); //get string after vs_ki
   }
   if (mySt.substring(0,5) == "vs_kd"){
     kd = mySt.substring(5,mySt.length()).toFloat(); //get string after vs_kd
   }
 }
void detect_a() {
   encoder+=1; //increasing encoder at new pulse
   m_direction = digitalRead(pin_b); //read direction of motor
 }
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
