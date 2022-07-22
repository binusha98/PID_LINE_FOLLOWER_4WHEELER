
#include <AFMotor.h>    //MotorShield library for motor controlling
#include <QTRSensors.h> //IR sensro managing library
AF_DCMotor motor1(1, MOTOR12_1KHZ ); //Motor 1 initializing with 1kHz PWM frequency
AF_DCMotor motor2(2, MOTOR12_1KHZ ); //Motor 3 initializing with 1kHz PWM frequency
AF_DCMotor motor3(3, MOTOR12_1KHZ ); //Motor 4 initializing with 1kHz PWM frequency
AF_DCMotor motor4(4, MOTOR12_1KHZ ); //Motor 5 initializing with 1kHz PWM frequency



#define KP 2 //Pretend to start robot at lower speed
#define KD 3 //Slowly increase speed of robot ( Note: Kp < Kd) 
#define M1_minumum_speed 120  //minimum speed of the Motor1 and Motor2
#define M3_minumum_speed 120  //minimum speed of the Motor3 and Motor4
#define M1_maximum_speed 180 //max. speed of the Motor1 and Motor2
#define M3_maximum_speed 180 //max. speed of the Motor3 and Motor4
#define TIMEOUT 2500          //waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN 2         //emitterPin is the Arduino digital pin that controls whether the IR LEDs are on or off. Emitter is controlled by digital pin 2
#define DEBUG 0

//sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensors qtr;

const uint8_t SensorCount = 5;  //number of sensors used
unsigned int sensorValues[SensorCount];
char preState = 's';
char currState = 's';

void setup()
{

  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const unsigned char[]) {
    A0, A1, A2, A3, A4
  }, SensorCount);
  qtr.setEmitterPin(EMITTER_PIN);

  manual_calibration();

  set_motors(0, 0);
}

int lastError = 0;

//  Break down speed of the robot
void handBreak(const int16_t delayTime = 100) {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(delayTime);
}

//  keep robot on line
void keepOnLine(int error) {

  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  int leftMotorSpeed = M1_minumum_speed + motorSpeed;
  int rightMotorSpeed = M3_minumum_speed - motorSpeed;

  // set motor speeds using the two motor speed variables above
  set_motors(leftMotorSpeed, rightMotorSpeed);
}

//  send output to the motors
void set_motors(int motor1speed, int motor3speed)
{
  if (motor1speed < (M1_minumum_speed * -1) ) {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor1speed = M1_minumum_speed;
  } else {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }
  if (motor3speed < (M3_minumum_speed * -1)) {
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    motor3speed = M3_minumum_speed;
  } else {
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }

  if (motor1speed > M1_maximum_speed ) motor1speed = M1_maximum_speed;
  if (motor3speed > M3_maximum_speed ) motor3speed = M3_maximum_speed;
  if (motor1speed < 0) motor1speed = 0;
  if (motor3speed < 0) motor3speed = 0;
  motor1.setSpeed(motor1speed);
  motor2.setSpeed(motor1speed);
  motor3.setSpeed(motor3speed);
  motor4.setSpeed(motor3speed);

}

//  drive robot through crossed lines
void walkThroughCross() {
  handBreak(500);

  motor1.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);
  motor2.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);
  motor3.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);
  motor4.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(250);

  handBreak();
  (void)qtr.readLineBlack(sensorValues);
  detectState();
  if (currState == 'o') {
    preState = 'e';
  }

}

//  drive robot through Y junctoins
void walkJointY() {

  handBreak(500);

  motor1.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);
  motor2.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);
  motor3.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);
  motor4.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(100);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(100);

  handBreak();
}

//  turn robot in the Y junction without go forward
void jointYTurn() {
  motor1.setSpeed( (int)(M1_minumum_speed + M1_maximum_speed) / 2 );
  motor2.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);
  motor3.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);
  motor4.setSpeed((int)(M1_minumum_speed + M1_maximum_speed) / 2);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(50);
}

//  Detect current shape/state of the line
void detectState() {
  uint16_t sum = 0;
  for ( int i = 0; i < SensorCount; i++ ) sum += sensorValues[ i ];
  if (sum < 4000 && sum > 2000) {
    if (sensorValues[ 1 ] > 0 && sensorValues[ 3 ] > 0 && sensorValues[ 0 ] < 500 && sensorValues[ 4 ] < 500) {
      currState = 'y';
    }
  }
  else if (sum > 3000) {
    currState = '-';
  }
  else if (sum < 500) {
    currState = 'o';
  }
}


void loop()
{
  unsigned int sensors[5];
  int position = qtr.readLineBlack(sensorValues); //get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 2000;

  //  if (preState != 'e') {
  //    detectState();
  //    if (currState == 'y') {
  //      walkJointY();
  //    } else if (currState == '-') {
  //      walkThroughCross();
  //    } else keepOnLine(error);
  //  }

  detectState();
  if (currState == 'y') {
    jointYTurn();
    currState = 'l';
  }
  keepOnLine(error);


  if (DEBUG) {
    Serial.begin(9600);
    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.print(preState);
    Serial.print('\t');
    Serial.print(currState);
    Serial.println(' ');

    delay(250);
  }

}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void manual_calibration() {

  int i;
  for (i = 0; i < 150; i++)
  {
    qtr.calibrate();
    delay(20);
  }
}
