//Lets say we have values of all ir sensors at any moment 

#include <QTRSensors.h>

//  It prints the sensor values to the serial 
// monitor as numbers from 0 (maximum reflectance) to 1023 (minimum reflectance).

// now sensorValues[] gives reading of ir sensosrs.

//4 DIGITAL PINS FOR MOTOR DRIVER
//SAY DG1 - IN1 , DG2 -IN2 , DG3-IN3 , DG4 - IN4
//2 enable pins as output (pwm) en 1 en2 
//threshold - line standard , define ls 800 (let)
//help in code




#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to analog inputs 0 through 7, respectively
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5,6 ,7}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


void setup()
{

  pinMode(29, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(35, OUTPUT);

  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);

  delay(500);
 // Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  //delay(1000);
}


// make 4 functions for forward left write stop 

void forward(int speed1 , int speed2)
{
	digitalWrite(29, HIGH);
    digitalWrite(31, LOW);
    digitalWrite(33, HIGH);
    digitalWrite(35, LOW);

    analogWrite(11,speed1);
    analogWrite(10,speed2);
}

void right(int speed1, int speed2)
{
	digitalWrite(29, LOW);
    digitalWrite(31, LOW);
    digitalWrite(33, HIGH);
    digitalWrite(35, LOW);

    analogWrite(11,speed1);
    analogWrite(10,speed2);
}

void left(int speed1, int speed2)
{
	digitalWrite(29, HIGH);
    digitalWrite(31, LOW);
    digitalWrite(33, LOW);
    digitalWrite(35, LOW);

    analogWrite(11,speed1);
    analogWrite(10,speed2);
}


void stop(int speed1 , int speed2)
{
	digitalWrite(29, LOW);
    digitalWrite(31, LOW);
    digitalWrite(33, LOW);
    digitalWrite(35, LOW);

    analogWrite(11,speed1);
    analogWrite(10,speed2);
}

void loop()
{
  // read raw sensor values
  qtra.read(sensorValues);
  
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
  // 1023 means minimum reflectance
//  for (unsigned char i = 0; i < NUM_SENSORS; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//  }
//  Serial.println();
//  
  delay(250);


  if(sensorValues[0]<100 && sensorValues[1]<100 && sensorValues[6]<100 && sensorValues[7]<100 && sensorValues[3]>300 && sensorValues[4]>300)     // Move Forward
  {
    forward(255,255);
  }
  
  if(sensorValues[0]>300 && sensorValues[1]>300 && sensorValues[6]<100 && sensorValues[7]<100 && sensorValues[3]>300 && sensorValues[4]>300)     // Turn right
  {
    right(0,255);
  }
  
//  if(sensorvalues 0 1 WHITE & sensorValues 3 4 black )     // turn left
//  {
//    left(0,255);
//  }
//  
//  if(sensorValues ( 01234567) black )     // stop
//  {
//    stop(0,0);
//  }

} 

