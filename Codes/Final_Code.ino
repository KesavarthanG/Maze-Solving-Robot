#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
char output[100];
int j = 0;
const int IN1 = 18;    // Motor 1 IN1
const int IN2 = 19;    // Motor 1 IN2
const int PWM_A = 0;  // Motor 1 PWM
const int IN3 = 17;    // Motor 2 IN3
const int IN4 = 5;    // Motor 2 IN4
const int PWM_B = 2;  // Motor 2 PWM
const int ir = 39;
const int ir_r = 22;
const int ir_l = 36; // pin 33 is interfering
const int push_btn = 21;
const int led = 16;
int junctionvalue;
int lastError = 0;
float KP=0.017;
float KD=1.2;
int M1=120;
int M2=120;
int autorun = 0;
int endFound = 0;
int path_length = 0;
void setup()
{
  attachInterrupt(push_btn, autoenable, RISING);
  pinMode(led,OUTPUT);
  pinMode(push_btn,INPUT);
  pinMode(ir,INPUT);
  pinMode(ir_l,INPUT);
  pinMode(ir_r,INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){25,26,27,14,13,23}, SensorCount);// turn on Arduino's LED to indicate we are in calibration mode {33,25,26,27,14,13,23,22}
  Serial.begin(921600);
    for(int k = 0; k < 100; k++){
    output[k] = 'D';
  }
  sensorcalibrate();
}

void loop()
{ 
  while(!endFound){
    dryrun();
    //simplified();
  }
}
void PID(){
  int position = qtr.readLineWhite(sensorValues);
  int error = position - 2500;
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  int m1Speed = M1 + motorSpeed;
  int m2Speed = M2 - motorSpeed;
  if (m1Speed < 0)
    m1Speed = 0;
  if (m2Speed < 0)
    m2Speed = 0;
  setMotorSpeed(m2Speed,m1Speed);
}
void sensorcalibrate(){
  for (uint16_t i = 0; i < 500; i++)
  {
    qtr.calibrate();
  }

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Control left motor with L298N
  if (leftSpeed > 0) {
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
    analogWrite(PWM_B, leftSpeed);
  } else {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
    analogWrite(PWM_B, -leftSpeed);
  }

  // Control right motor with L298N
  if (rightSpeed > 0) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    analogWrite(PWM_A, rightSpeed);
  } else {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    analogWrite(PWM_A, -rightSpeed);
}
}
void printend(){
  Serial.print(digitalRead(ir_l));
  Serial.print(",");
  Serial.print(digitalRead(ir));
  Serial.print(",");
  Serial.println(digitalRead(ir_r));
}

void forward(){
  digitalWrite(IN1,1);
 digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,1);
  analogWrite(PWM_A,150);
  analogWrite(PWM_B,150);
}
void botstop(){
  digitalWrite(IN1,1);
  digitalWrite(IN2,1);
  digitalWrite(IN3,1);
  digitalWrite(IN4,1);
  analogWrite(PWM_A,0);
  analogWrite(PWM_B,0);
}
void leftturn(){
  output[path_length] = 'L';
  path_length = path_length + 1;
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(PWM_A, 90);
  analogWrite(PWM_B, 90);
  int position = qtr.readLineWhite(sensorValues);
    while(digitalRead(ir_l) == 1){
      digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(PWM_A, 90);
  analogWrite(PWM_B, 90);
    }
    // botstop();
    // delay(1000);
    while(digitalRead(ir) == 1){
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(PWM_A, 90);
  analogWrite(PWM_B, 90);
  }

}
void rightturn(){
  output[path_length] = 'R';
  path_length = path_length + 1;
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(PWM_A, 100);
  analogWrite(PWM_B, 100);
  while(digitalRead(ir_r) == 1){
      digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(PWM_A, 90);
  analogWrite(PWM_B, 90);
    }
    // botstop();
    // delay(1000);
  while(digitalRead(ir) == 1){
    digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(PWM_A, 90);
  analogWrite(PWM_B, 90);
  }
}
void uturn(){
  output[path_length] = 'B';
  path_length = path_length + 1;
    botstop();
    delay(1000);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(PWM_A, 90);
  analogWrite(PWM_B, 90);
  while(digitalRead(ir_l) == 1){
    digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(PWM_A, 90);
  analogWrite(PWM_B, 90);
  }
   while(digitalRead(ir) == 1){
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(PWM_A, 90);
  analogWrite(PWM_B, 90);
   }
   botstop();
  delay(500);
}
int detectJunction(unsigned long debounceTime) {
   bool leftDetected = false;
    bool rightDetected = false;
   unsigned long startTime = millis();
   digitalWrite(led,1);
  while (millis() - startTime <debounceTime) {

      if (digitalRead(ir_l)==0) { // Assuming LOW means line detected
            leftDetected = true;
        }
        if (digitalRead(ir_r)==0) { // Assuming LOW means line detected
            rightDetected = true;
        }
    }

   digitalWrite(led,0);
   if(rightDetected&&leftDetected){
    return 1;
   }else if(leftDetected){
    return 2;
   }else if(rightDetected){
    return 3;
   } else{
    return -1;
   }
}
void assignjunctionvalue( int detectvalue){
  int centre;
  int position = qtr.readLineWhite(sensorValues);
  if(sensorValues[2] <900 || sensorValues[3] <900 || sensorValues[4] <900 || sensorValues[1] <900){
    centre = 0;
  }
  else{
    centre = 1;
  }
  if(detectvalue==1 && centre==0){
      //plus
      junctionvalue=1;
      Serial.println("plus");
    } else if(detectvalue==1 && centre==1){
      //T shape
      junctionvalue=2;
      Serial.println("T shape");
    } else if(detectvalue==2 && centre==0){
      // left T
      junctionvalue=3;
      Serial.println("left T");
    } else if(detectvalue==2 && centre==1){
      // left L
      junctionvalue=4;
      Serial.println("left L");
    } else if(detectvalue==3 && centre==0){
      // right T
      junctionvalue=5;
      Serial.println("right T");
      output[path_length] = 'S';
      path_length = path_length + 1;
    } else if(detectvalue==3 && centre==1){
      // right L
      junctionvalue=6;
      Serial.println("right L");
    }
}
void junctionaction(int junctionvalue){
  switch(junctionvalue){
    case 1:
    leftturn();
    break;
    case 2:
    leftturn();
    break;
    case 3:
    leftturn();
    break;
    case 4:
    leftturn();
    break;
    case 6:
    rightturn();
    break;
    case 7:
    uturn();
    break;
  }
}
void dryrun(){
  int position = qtr.readLineWhite(sensorValues);
  if(digitalRead(ir_l) == 0 || digitalRead(ir_r) == 0 ){

      int detectvalue = detectJunction(180);
            botstop();
    //   forward();
    //    unsigned long startTime = millis();
    //      while (millis() - startTime < 100) {
    //   asm("");
    // }
    // botstop();
    delay(1000);
    assignjunctionvalue(detectvalue);
    junctionaction(junctionvalue);
  }
  else if(sensorValues[0] == 1000 && 
  sensorValues[1] == 1000 &&
  sensorValues[2] == 1000 &&
  sensorValues[3] == 1000 &&
  sensorValues[4] == 1000 &&
  sensorValues[5] == 1000){
    junctionvalue = 7;
    junctionaction(junctionvalue);
  }
  else if(sensorValues[0] < 600 && 
  sensorValues[1] < 600 &&
  sensorValues[2] < 600 &&
  sensorValues[3] < 600 &&
  sensorValues[4] < 600 &&
  sensorValues[5] < 600 &&
  digitalRead(ir) == 0 && digitalRead(ir_l) == 0 && digitalRead(ir_r) == 0){
    endFound = 1;
    digitalWrite(led,1);
    botstop();
    delay(2000);
  }
  else{
    PID();
  }
}
void moveDsToBack(char array[], int size) {
  int i, j;

  for (i = 0; i < size; i++) {
    if (array[i] == 'D') {
      // Find the next non-'B' element
      for (j = i + 1; j < size; j++) {
        if (array[j] != 'D') {
          // Swap 'B' with the next non-'B' element
          char temp = array[i];
          array[i] = array[j];
          array[j] = temp;
          break;  // Move to the next 'B'
        }
      }
    }
  }
}

void simplified() {
  //shortest path
  for (int i = 1; i < 99; i++) {
    if (output[i] == 'B') {
      if (output[i - 1] == 'L' && output[i + 1] == 'L') {
        output[i - 1] = 'S';
        output[i] = 'D';
        output[i + 1] = 'D';
      } else if (output[i - 1] == 'L' && output[i + 1] == 'R') {
        output[i - 1] = 'B';
        output[i] = 'D';
        output[i + 1] = 'D';
      } else if (output[i - 1] == 'L' && output[i + 1] == 'S') {
        output[i - 1] = 'R';
        output[i] = 'D';
        output[i + 1] = 'D';
      } else if (output[i - 1] == 'R' && output[i + 1] == 'L') { 
        output[i - 1] = 'B';
        output[i] = 'D';
        output[i + 1] = 'D';
      } else if (output[i - 1] == 'S' && output[i + 1] == 'L') { 
        output[i - 1] = 'R';
        output[i] = 'D';
        output[i + 1] = 'D';
       } 
        else if (output[i - 1] == 'S' && output[i + 1] == 'S') {
        output[i - 1] = 'B';
        output[i] = 'D';
        output[i + 1] = 'D';
      }
    }
  }
  moveDsToBack(output, 100);
}
void autoenable(){
  autorun = 1;
}
void autorunn(){
  if(digitalRead(ir_r) == 0 || digitalRead(ir_l) == 0){
    if(output[j]=='L'){
    leftturn();
     } 
    else if(output[j]=='R'){
      rightturn();
    }
    else if(output[j]=='S'){
      forward();
    }
    else if(output[j]=='B'){
    uturn();
    }
    j=j+1;
  }
  else{
  PID();
  }
}