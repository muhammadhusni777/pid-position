////////////////////////////////////////////
//        PID POSITION MOTOR              //
//    written by : Muhammad Husni         //
// PT SYERGIE AUTOMA TEKNOLOGI            //
////////////////////////////////////////////

///390 seputeran

#define encoder0PinA 2
#define encoder0PinB 3
int motor_plus = 10;
int motor_min = 11;
int encoder0Pos = 0;

int setpoint = 0;
int sensor;
int error;
float prev_error;

float P_control;
float kp = 5; //2.5
float kp_adaptive = 18;
float I_control;
float ki = 0.02; //0.025
float ki_adaptive = 0;
float I_windup_plus = 255;
float I_windup_min = -255;
float kd = 0.05;
float kd_adaptive = 0.05;
float D_control;
float saturation_min = -255;
float saturation_plus = 255;

float pid = 0;
float pid_plus;
float pid_min;


float Time;
float elapsedTime = 0.1;
float timePrev;

String mode;

char data;

String readString;
int index_pin = 7;
int index_state;
int last_dir;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(index_pin, INPUT);
  pinMode(motor_plus, OUTPUT);
  pinMode(motor_min, OUTPUT);
  attachInterrupt(0, doEncoder, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  //setpoint
  //setpoint = 228;
  index_state = digitalRead(index_pin);
 
   while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

    if (readString.length() >0) {
    Serial.println(readString);  //so you can see the captured string
    setpoint = readString.toInt();  //convert readString into a number
    }
 readString=""; //empty for next input

  if ((setpoint == 0) && (sensor == 0) && (index_state == 1) && (last_dir == 1)){
    sensor = 0;
    encoder0Pos = 0;
    error = 1;

  } else if ((setpoint == 0) && (sensor == 0) && (index_state == 1) && (last_dir == -1)){
    sensor = 0;
    encoder0Pos = 0;
    error = -1; 
  } else {
 
  //read sensor
  sensor = encoder0Pos;
  
  //error
  error = setpoint-sensor;
  }
  
  if (5 < abs(error)){
  //pid control
  P_control = kp*error;
  I_control = (ki*error*elapsedTime) + I_control;
  D_control = kd*(error-prev_error)/elapsedTime;  
  mode = "aggresive";
  } else {
  //adaptive pid control
  P_control = kp_adaptive * error;
  I_control = (ki_adaptive*error*elapsedTime) + 0;
  D_control = kd_adaptive*(error-prev_error)/elapsedTime;  
  mode = "calm";
  }
  //Integral Windup
  if (I_control > I_windup_plus){
    I_control = I_windup_plus;
  } 
  else if (I_control < I_windup_min){
    I_control = I_windup_min;
  }
  else {
    I_control = I_control;
  }
  
  //Derivative control + derivative kick
  
  //P+I+D control
  
  pid = P_control + I_control + D_control;
  
  //saturation
  
  if (pid > saturation_plus){
    last_dir = 1;
    pid = saturation_plus;
    I_windup_plus = saturation_plus - (P_control+D_control);
  } 
  else {
    pid = pid;
    I_windup_plus = saturation_plus;
    I_windup_min = saturation_min;
  }
  
  if (pid < saturation_min){
    last_dir = -1;
    pid = saturation_min;
    I_windup_min = saturation_min + (P_control+D_control);
  }
  else {
    pid = pid;
    I_windup_plus = saturation_plus;
    I_windup_min = saturation_min;
  }
  //*/

  //pwm
  if (pid > 0){
   pid_plus = pid;
   pid_min = 0; 
  
  }  else if (pid < 0){
   pid_plus = 0;
   pid_min = abs(pid);
 
  }
  
//  
  analogWrite(motor_min, pid_min);
  analogWrite(motor_plus, pid_plus);

  //timer
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000;
  prev_error = error;

  Serial.print (setpoint);
  Serial.print (" ");
  Serial.print (sensor);
  Serial.print (" ");
  Serial.print(error);
  Serial.print (" ");
  Serial.print(P_control);
  Serial.print (" ");
  Serial.print(I_control);
  Serial.print (" ");
  Serial.print(D_control);
  Serial.print (" ");
  Serial.print(pid_plus);
  Serial.print (" ");
  Serial.print(pid_min);
  Serial.print (" ");
  Serial.print(elapsedTime);
  Serial.print (" ");
  Serial.print (mode);
  Serial.print (" ");
  Serial.print (index_state);
  Serial.print (" ");
  Serial.println (last_dir);
}


void doEncoder()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
  {
  encoder0Pos++;
  }
  else
  {
  encoder0Pos--;
  }
  
}
