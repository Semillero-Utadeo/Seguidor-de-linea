/*
Programa desarrollado viernes 7 de Abril de 2017

Programa desarrollado para pololu Baby Orangutan con Sensores QTR8A

el programa maneja un control PD y controla los sensores QTR8A analogo

falta probarlo y calcular las constantes
*/

#include <QTRSensors.h>
#include <OrangutanMotors.h>



#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 0, 1, 2, 3, 4, 5}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

OrangutanMotors motors;

int lastError = 0;
double KP = 0.18;
double KD = 0.015;
int mV= 40; // minima velocidad


void setup()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
 // digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(115200);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(300);
}

long a=0;
long b=0;
void loop()
{ 
  a=millis();
  unsigned int position = qtra.readLine(sensorValues);
  
//  for (unsigned char i = 0; i < NUM_SENSORS; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t');
//  }

  /*Serial.print(position); // comment this line out if you are using raw values*/
  delay(1);
  int error = position - 3500;
 // Serial.print(",");
  Serial.print(error);
  //int motorSpeed = KP * error + KD * (error - lastError);
  double diferencia = KP * error + KD * (error - lastError);
  lastError = error;
  /*Serial.print(",");
  Serial.print(diferencia);*/
  
  int m1Speed = mV + diferencia;
  int m2Speed =  mV- diferencia;
  //motors.setSpeeds(m1Speed,m2Speed);
  
  
 /* Serial.print(",");
  Serial.print(m1Speed);
  Serial.print(",");
  Serial.print(m2Speed);*/
  b=millis();
  Serial.print(",");
  Serial.println((b-a));
  //delay(20);
  
//  if (diferencia > mV)
// diferencia = mV;
// if (diferencia < -mV)
// diferencia = -mV;
// 
// if (diferencia < 0)
// OrangutanMotors::setSpeeds(mV, mV + diferencia);//el signo da la direccoin de giro y la magnitud da la velocidad
// else
// OrangutanMotors::setSpeeds(mV - diferencia,mV);//va de -255 a 255 o es full freno.
//  
  
   //motors.setSpeeds(50,50);
  
}
