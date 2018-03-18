#include <QTRSensors.h>
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 2
#define RMPIN 11
#define LMPIN 10
#include <NewPing.h>

#define SONAR_NUM  3             //define Number of ultrasonic sensor used
#define MAX_DISTANCE 40            //Max distance between the object and robot

#define PING_INTERVAL 33

#define trigPin1    50                  //FRONT
#define echoPin1    52

#define trigPin2    39                 //RIGHT
#define echoPin2    37

#define trigPin3    32                 //LEFT
#define echoPin3    30

int front;
int left;
int right;

int flag = 0;
#define LMPIN 10
# define RMPIN 11

NewPing sonar[SONAR_NUM] = {                                            // Define a Newping array to measure the distance

  NewPing(trigPin1, echoPin1, MAX_DISTANCE),
  NewPing(trigPin2, echoPin2, MAX_DISTANCE),
  NewPing(trigPin3, echoPin3, MAX_DISTANCE),
};


// sensors 0 through 7 are connected to analog inputs 0 through 7, respectively
QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4, 5, 6, 7
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR , EMITTER_PIN);
unsigned int sen[NUM_SENSORS];

float last_proportional = 0;
float proportional = 0;
float derivative = 0;
float integral = 0;
int flagcount = 0;
void setup()
{

  //pinMode(24,OUTPUT);
  //digitalWrite(24,HIGH);
  pinMode(29, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(35, OUTPUT);

  pinMode(LMPIN, OUTPUT);
  pinMode(RMPIN, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  digitalWrite(trigPin1, LOW);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  digitalWrite(trigPin2, LOW);

  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  digitalWrite(trigPin3, LOW);


  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

}


// make 4 functions for forward left write stop

void forward(int rspeed , int lspeed, int RM1, int RM2, int LM1, int LM2)
{
  digitalWrite(29, RM1); //for forward 29 LOW, 31 HIGH  RIGHT Motor
  digitalWrite(31, RM2);
  digitalWrite(33, LM1); // for forward 33 HIGH, 35 LOW  LEFT Motor
  digitalWrite(35, LM2);

  analogWrite(RMPIN, rspeed);
  analogWrite(LMPIN, lspeed);
}
void right_wall() {
  if ((front > 25 || front == 0) && (right < 15) && (right != 0))
    forward(65, 80, LOW, HIGH, HIGH, LOW);

  // else if ((front > 25 || front == 0) && (right > 25 || right == 0))
  // forward(78,100, LOW, LOW, HIGH, LOW  );

  else if (front < 10 && right < 10)
    forward(78, 100, LOW, HIGH, LOW, LOW);

  //delay(250);
}

void left_wall() {
  if ((front == 0) && (left < 35) && (left != 0))
    forward(65, 80, LOW, HIGH, HIGH, LOW);

  else if (( front == 0) && ( left == 0))
    forward(65, 80,  LOW, HIGH, LOW, LOW );

  else if (front < 35 && left < 35)
  {
    forward(65, 80,  LOW, HIGH, LOW, LOW);
  }

  else if ( (front < 35 && front != 0 ) && ( left == 0  ))
  {
    forward(65, 80, LOW, LOW, HIGH, LOW);
  }

};
void loop()
{
  if (flagcount > 30)
  {
    Serial.print("front: ");
    Serial.print(sonar[0].ping_cm());
    Serial.print('\t');
    front = sonar[0].ping_cm();
    Serial.print("right: ");
    Serial.print(sonar[1].ping_cm());
    Serial.print('\t');
    right = sonar[1].ping_cm();
    Serial.print("left: ");
    Serial.print(sonar[2].ping_cm());
    Serial.print('\t');
    Serial.println();
    left = sonar[2].ping_cm();

    // forward(78, 100, LOW, HIGH, HIGH, LOW);
    if (left > right)
      right_wall();
    // forward(100,100);
    else
      left_wall();

  }

  int SENSOR_COUNT = 0;
  float pos;
  float v;
  int sense = 1000;
  float setpoint = 3.5;
  int lspeed = 0;
  const int maxs = 40;
  int rspeed = 0;
  // read raw sensor values
  qtra.read(sen);
  float a[8];
  int threshold = 500 ;
  a[0] = float(sen[0]) / sense;
  a[1] = float(sen[1]) / sense;
  a[2] = float(sen[2]) / sense;
  a[3] = float(sen[3]) / sense;
  a[4] = float(sen[4]) / sense;
  a[5] = float(sen[5]) / sense;
  a[6] = float(sen[6]) / sense;
  a[7] = float(sen[7]) / sense;
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
  // 1023 means minimum reflectance
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if ( a[i] > 0.4)
    {
      SENSOR_COUNT = SENSOR_COUNT + 1;
    }
    Serial.print(a[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
  //delay(250);
  pos = ((7 * a[0] + 6 * a[1] + 5 * a[2] + 4 * a[3] + 3 * a[4] + 2 * a[5] + 1 * a[6] + 0 * a[7]) / (a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7]));
  if ( a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7] == 0)
  {
    pos = setpoint;
  }
  Serial.print("Pos = ");
  Serial.println(pos);
  if ((pos < 2.3) && ( SENSOR_COUNT > 4) && (SENSOR_COUNT < 8)) //Right turn 90 degree
  {
    lspeed = maxs;
    rspeed = maxs;
    forward(rspeed, lspeed,  HIGH, LOW, HIGH, LOW);
    //delay(75);
    flagcount = 0;
  }
  else if ((pos > 4.1) && ( SENSOR_COUNT > 4) && (SENSOR_COUNT < 8)) //Left Turn 90 degree
  {
    lspeed = maxs;
    rspeed = maxs;
    forward(rspeed, lspeed,  LOW, HIGH, LOW, HIGH );
    //delay(75);
    flagcount = 0;
  }
  else if (SENSOR_COUNT  == 8) // Right Hand Turn Rule
  {
    lspeed = maxs;
    rspeed = maxs;
    forward(rspeed, lspeed, HIGH, LOW, HIGH, LOW);
    //delay(75);
    flagcount = flagcount + 1;
  }
  else if (SENSOR_COUNT == 0) // DEAD END
  {
    lspeed = maxs;
    rspeed = maxs;
    forward(rspeed, lspeed, HIGH, LOW, HIGH, LOW );
    //delay(171);
    flagcount = 0;
  }
  else
  {
    proportional = (pos - setpoint); //replace your  value of set point
    derivative = proportional - last_proportional;
    integral = integral + proportional;
    last_proportional = proportional;
    float Kp = 7;
    float Ki = 0.000;
    float Kd = 50;

    //right positive
    //left negative
    int power_difference = proportional * Kp + integral * Ki + derivative * Kd;
    Serial.println();
    Serial.print(power_difference);
    Serial.println();

    if (power_difference > maxs)
    {
      power_difference = maxs;
    }



    if (power_difference < -maxs)
    {
      power_difference = (-1 * maxs);
    }
    //speed1 is right motor speed
    //forward(108,97);
    if ( power_difference < 0)
    {
      lspeed = maxs - power_difference;
      rspeed = maxs - 3;
    }
    else
    {
      lspeed = maxs;
      rspeed = maxs - 3 + power_difference;
    }
    forward(rspeed, lspeed, LOW, HIGH, HIGH, LOW);
    flagcount = 0;
  }
  Serial.print("lspeed = ");
  Serial.println(lspeed);
  Serial.print("rspeed = ");
  Serial.println(rspeed);

  //delay(250);
  // forward(83,90, LOW, HIGH, HIGH, LOW);
}



