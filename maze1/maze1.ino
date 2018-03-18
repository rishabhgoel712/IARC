#include <QTRSensors.h>
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 2
#define RMPIN 11
#define LMPIN 10

// sensors 0 through 7 are connected to analog inputs 0 through 7, respectively
QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4, 5, 6, 7
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR , EMITTER_PIN);
unsigned int sen[NUM_SENSORS];

float last_proportional = 0;
float proportional = 0;
float derivative = 0;
float integral = 0;
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
void loop()
{
  int SENSOR_COUNT = 0;
  float pos;
  float v;
  int sense = 1000;
  float setpoint = 3.5;
  int lspeed = 0;
  const int maxs = 50;
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
    //a[i] = float(sen[i]) / sense;
    if ( a[i] > 0.3)
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
    //delay(50);
  }
  else if ((pos > 4.1) && ( SENSOR_COUNT > 4) && (SENSOR_COUNT < 8)) //Left Turn 90 degree
  {
    lspeed = maxs;
    rspeed = maxs;
    forward(rspeed, lspeed,  LOW, HIGH, LOW, HIGH );
    //delay(70);
  }
  else if (SENSOR_COUNT  == 8) // Right Hand Turn Rule
  {
    lspeed = maxs;
    rspeed = maxs;
    forward(rspeed, lspeed,  HIGH, LOW, HIGH, LOW);
    //delay(70);
  }
  else if (SENSOR_COUNT == 0) // DEAD END
  {
    lspeed = maxs;
    rspeed = maxs;
    forward(rspeed, lspeed, HIGH, LOW, HIGH, LOW );
    //delay(120);
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
  }
  Serial.print("lspeed = ");
  Serial.println(lspeed);
  Serial.print("rspeed = ");
  Serial.println(rspeed);

  //delay(250);
   forward(255,255,LOW, HIGH, HIGH, LOW
   );
}


