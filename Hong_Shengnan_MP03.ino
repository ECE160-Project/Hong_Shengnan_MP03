/*
  Hong_Shengnan_MP03.ino

  PROJECT NAME
  State Machine

  PROGRAM DESCRIPTION
  This program uses Arduino mirocontroller to construct a state
  machine and read the values from sensors. It is implement an
  autonomous control system with&without the input from sensors.
  The program has four different states including FLASH_LED, 
  BUZZER_PLAY, MOTOR_SPIN, and SERVO_MOVE.

  KEY FUNCTION
  setup();
  loop();
  run_machine1();
  read_sensors();
  read_soft();
  read_obs();
  read_flex();
  run_machine2();
  flash_led();
  motor_spin();
  buzzer_play();
  servo_move();
  
                            Created by Steven Hong
                                        09/25/2017
*/
//define the pin for all component
const int servoPin = 11;
const int buzzerPin = 6;
const int redPin = 8;
const int motorPin = 10;
const int buttonPin = 3;
const int flexPin = A0;
const int softPin = A1;

//define the value for different state
#define FLASH_LED 0
#define MOTOR_SPIN 1
#define BUZZER_PLAY 2
#define SERVO_MOVE 3
#define NUMSTATES 4

//define all the global values
int state;
int softValue;
int flexValue;
int pinState;
int flexThresh = 850;
int softThresh = 600;
int pinThresh = 1;
int stateTime = 1000;
int delayTime = 500;
int spinTime = 3000;
int tempo = 113;
int frequency[] = {262, 294, 330, 294, 494};

#include <Servo.h>;   //servo library
Servo servo1;   //servo control object

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Initialize serial port & set rate to 9600 bits per second (bps)
  
  pinMode(buzzerPin, OUTPUT); //set up pin as output
  pinMode(redPin, OUTPUT);    //set up pin as output
  pinMode(motorPin, OUTPUT);  //set up pin as output
  pinMode(buttonPin, INPUT);  //set up pin as input
  
  servo1.attach(servoPin, 900, 2100); //connect the servo to the pin
                                      //with a minimum pulse width of
                                      //900 and a maximum width of 2100
  
  state = 0;   //Initialize the index
}

void loop() {
  // put your main code here, to run repeatedly:
  //run_machine1();
  //read_sensors();
  run_machine2();
}

void run_machine1(){
  switch(state) {   //state machine
    case FLASH_LED:
      flash_led();  //run flash_led function for state 1
      break;
    case MOTOR_SPIN:
      motor_spin();  //run motor_spin function for state 2
      break;
    case BUZZER_PLAY:
      buzzer_play();  //run buzzer_play function for state 3
      break;
    case SERVO_MOVE:
      servo_move(); //run servo_move function for state 4
      break;
  }
  state++;    //increment state variable
  state = state % NUMSTATES;      //keep state between 0 and NUMSTATES - 1
  Serial.print("timer state: ");  //print state label
  Serial.println(state);          //print state
  delay(stateTime);               //wait to run next state

}

void read_sensors(){
  read_obs(); //read the status of pushbutton
  read_flex();  //read the value of flex sensor
  read_soft();  //read the value of soft pot
}

void read_soft(){
  softValue = analogRead(softPin);  //Read the voltage from the softpot (0-1023)
  Serial.print("Soft Pot: ");
  Serial.println(softValue);  //Print the result to the serial monitor
  delay(100);
}

void read_flex(){
  flexValue = analogRead(flexPin);  //Read the voltage from the flex sensor
  Serial.print("Flex Sensor: ");
  Serial.println(flexValue);  //Print the result to the serial monitor
  delay(100);
}

void read_obs(){
  pinState = digitalRead(buttonPin);  //Read the status of red pushbutton
  Serial.print("Pushbutton: ");
  Serial.println(pinState); //Print the result to the serial monitor
}

void run_machine2(){
  read_sensors(); //read the input from sensors
  switch(state){
    case FLASH_LED:
      flash_led();  //run flash_led function for state 1;
      break;
    case BUZZER_PLAY:
      buzzer_play(); //run buzzer_play function for state 2
      break;
    case MOTOR_SPIN:
      motor_spin(); //run motor_spin function for state 3
      break;
    case SERVO_MOVE:
      servo_move(); //run servo_move function for state 4
      break;
    default:
      break;
  }
  
  Serial.print("current state: ");  //print state label
  Serial.print(state);   //print state
  if (state == 0) //when led is flashing
  {
    if (flexValue > flexThresh)
    {
      state = FLASH_LED;  //go to state FLASH_LED
    }
    else if (softValue > softThresh)
    {
      state = MOTOR_SPIN; //go to state MOTOR_SPIN
    }
    else if (pinState == 0)
    {
      state = SERVO_MOVE; //go to state SERVO_MOVE
    }
  }
  else if (state == 1)  //when motor is spinning
  {
    if (flexValue > flexThresh)
    {
      state = FLASH_LED;  //go to state FLASH_LED
    }
    else if (pinState == 0)
    {
      state = FLASH_LED;  //go to state FLASH_LED
    }
    else if (softValue > softThresh)
    {
      state = BUZZER_PLAY;  //go to state BUZZER_PLAY
    }
  }
  else if (state == 2)  //when buzzer is playing
  {
    if (pinState == 0)
    {
      state = MOTOR_SPIN; //go to state BUZZER_PLAY
    }
    else if (flexValue > flexThresh)
    {
      state = FLASH_LED;  //go to state FLASH_LED
    }
    else if (softValue > softThresh)
    {
      state = SERVO_MOVE; //go to state SERVO_MOVE
    }
  }
  else if (state == 3)  //when servo is moving
  {
    if (flexValue > flexThresh)
    {
      state = FLASH_LED;  //go to state FLASH_LED
    }
    else if (softValue > softThresh)
    {
      state = FLASH_LED;  //go to state FLASH_LED
    }
    else if (pinState == 0)
    {
      state = BUZZER_PLAY;  //go to state BUZZER_PLAY
    }
  }
  Serial.print("\tnext state: "); //print state label
  Serial.println(state);  //print state
}

void flash_led(){
  for (int i = 0; i <= 4; i++)  //let LED flash five times
  {
    digitalWrite(redPin, HIGH); //turn the LED on
    delay(delayTime);
    digitalWrite(redPin, LOW);  //turn the LED off
    delay(delayTime);
  }
}

void motor_spin(){
  analogWrite(motorPin, 255); //turn the motor on (full speed)
  delay(spinTime);  //delay for spinTime milliseconds
  analogWrite(motorPin, 0);   //turn the motor off
}

void buzzer_play(){
  for (int j = 0; j<= 4; j++)
  {
    tone(buzzerPin, frequency[j], tempo); //play the buzzer with designed frequency
    delay(tempo);
  }
}

void servo_move(){
  servo1.write(180);  //move the servo to 180 degree position
  delay(delayTime);
  servo1.write(0);    //move the servo to 0 degree position
}

