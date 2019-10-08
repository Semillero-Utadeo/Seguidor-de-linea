#include <QTRSensors.h>
#include <OrangutanMotors.h>

// This example is designed for use with six QTR-1A sensors or the first six sensors of a
// QTR-8A module.  These reflectance sensors should be connected to analog inputs 0 to 5.
// The QTR-8A's emitter control pin (LEDON) can optionally be connected to digital pin 2,
// or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to
// QTR_NO_EMITTER_PIN.

// The setup phase of this example calibrates the sensor for ten seconds and turns on
// the LED built in to the Arduino on pin 13 while calibration is going on.
// During this phase, you should expose each reflectance sensor to the lightest and
// darkest readings they will encounter.
// For example, if you are making a line follower, you should slide the sensors across the
// line during the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is.  Improper calibration will result in poor readings.
// If you want to skip the calibration phase, you can get the raw sensor readings
// (analog voltage readings from 0 to 1023) by calling qtra.read(sensorValues) instead of
// qtra.readLine(sensorValues).

// The main loop of the example reads the calibrated sensor values and uses them to
// estimate the position of a line.  You can test this by taping a piece of 3/4" black
// electrical tape to a piece of white paper and sliding the sensor across it.  It
// prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance)
// to 1000 (minimum reflectance) followed by the estimated location of the line as a number
// from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
// under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
// sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.


#define NUM_SENSORS             6  // number of sensors used 8
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2
#define MAX_VM                  60

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {
  6, 0, 1, 2, 3, 4 //7, 6, 0, 1, 2, 3, 4, 5
},
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

OrangutanMotors motors;
int lastError = 0;
double KP = 0.03; //0.03;
double KD = 0.09; //0.06
double KI = 0.003; //0.004
double sumError=0;
int mV = MAX_VM; // minima velocidad
int maxi=0;
int maxw=255-mV;
int sensibilidad = 120;


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
  Serial.begin(9600);
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
  Serial.println("presiona el boton");
}
long a = 0;
long b = 0;
int iniciar = 0;
void loop(){
for(int i= 0; i<NUM_SENSORS; i++){
  if(sensorValues[i]>=sensibilidad){
    sensorValues[i]=1000;
  }
  Serial.print(sensorValues[i]);
  Serial.print("        ");
  
}
Serial.println();

  if (digitalRead(10) == HIGH && iniciar < 10) {
    iniciar++;
    delay(10);
  }
  if (iniciar == 10) {
    Serial.println("esperando");
    delay(5000);
    iniciar++;

  }

  if (iniciar == 11) {
    leermotores();
  }
  //pin 10 es el interruptor
}

//mod 6 503
void leermotores() {
  a = millis();
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtra.read(sensorValues); instead of unsigned int position = qtra.readLine(sensorValues);
  int position = qtra.readLine(sensorValues);
  int error = position - 2500;
if(error>2497)
  maxi++;
else
  maxi=0;
  
if((maxi+1)%10==0){
  mV--;
  if(mV<20)
  mV=20;
  maxw=mV;
}
else if(maxi==0){
  mV++;
  if(mV>MAX_VM)
    mV=MAX_VM;
  maxw=255-mV;
  }
  


  double diferencia = KP * error + KD * (error - lastError)+KI*sumError;
  lastError = error;
  sumError+=error;
  if(sumError>350000){
    sumError=350000;
  }
  else if(sumError<-350000){
    sumError=-350000;
  }
  
  if(diferencia>(maxw)){
    diferencia=maxw;
  }
  else if(diferencia<-(maxw)){
    diferencia=-(maxw);
  }
  int m1Speed = mV + diferencia;
  int m2Speed =  mV - diferencia;
  //motors.setSpeeds(m1Speed, m2Speed);
//    
//  Serial.print(error);
//  Serial.print("\t");
//  Serial.print(diferencia);
//  Serial.print("    \t");
//  Serial.print(m1Speed);
//  Serial.print("\t");
//  Serial.print(m2Speed);
//  Serial.print("\t");  
////b = millis();
//Serial.println((millis() - a));
Serial.println(position);
int b=(millis() - a);
if (b<20)
delay(20-b);

 

}
