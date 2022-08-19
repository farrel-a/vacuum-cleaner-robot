
#include <ros.h>
#include <vac_msgs/HardwareCommand.h>
#include <vac_msgs/HardwareState.h>

//compute period in ms
double compute_period = 50.0;

ros::NodeHandle nh;

vac_msgs::HardwareCommand hardware_command_msg;
vac_msgs::HardwareState hardware_state_msg;

volatile long duration,duration2, duration3;
volatile float distance, distance2, distance3;
int trig = 4, echo = 3, trig2 = 8, echo2 = 2, trig3 = 12, echo3 = 18;
volatile unsigned long timestart, timeend, timestart2, timeend2, timestart3, timeend3;
//directio pin
int cwM1=8, ccwM1=11, cwM2, ccwM2, cwM3, ccwM3;
//pwm value
float pwm1, pwm2, pwm3;
//pwm output pin
int motor1Pwm=10, motor2Pwm, motor3Pwm;

void UpdateHardwareCommand(const vac_msgs::HardwareCommand& msg){
  hardware_command_msg.motor1 = msg.motor1;
  hardware_command_msg.motor2 = msg.motor2;
  hardware_command_msg.motor3 = msg.motor3;
}

ros::Publisher arduino_state_pub("/arduino/state/hardware", &hardware_state_msg);
ros::Subscriber<vac_msgs::HardwareCommand> hardware_command_sub("/control/command/hardware", &UpdateHardwareCommand );

void setup()
{
  PCICR |= B00000001;
  PCMSK0 |= B00000001;  
  Serial.begin(57600);
  attachInterrupt(digitalPinToInterrupt(echo), readSensorInt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echo2), readSensorInt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echo3), readSensorInt3, CHANGE);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  nh.initNode();
  nh.advertise(arduino_state_pub);
  nh.subscribe(hardware_command_sub);
}

void loop()
{
  readSensor(trig, echo);
  hardware_state_msg.sensor1 = distance;
  hardware_state_msg.sensor2 = distance2;
  hardware_state_msg.sensor3 = distance3;
  pwm1 = hardware_command_msg.motor1;
  pwm2 = hardware_command_msg.motor2;
  pwm3 = hardware_command_msg.motor3;
  arduino_state_pub.publish(&hardware_state_msg);
  motorDirection(cwM1, ccwM1, motor1Pwm, pwm1);
  motorDirection(cwM2, ccwM2, motor2Pwm, pwm2);
  motorDirection(cwM3, ccwM3, motor3Pwm, pwm3);
  nh.spinOnce();
  delay(compute_period);
}
//
void readSensor(int trig, int echo){
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
}

void readSensorInt(){
    if(digitalRead(echo)){
      timestart = micros(); 
    }
    else{
      timeend = micros();
      int elapsedTime = timeend - timestart;
      distance = elapsedTime * 0.0343/2;
      }
  }

void readSensorInt2(){
    if(digitalRead(echo2)){
      timestart2 = micros(); 
    }
    else{
      timeend2 = micros();
      int elapsedTime2 = timeend2 - timestart2;
      distance2 = elapsedTime2 * 0.0343/2;
      }
  }

void readSensorInt3(){
  if(digitalRead(echo3)){
      timestart3 = micros(); 
    }
    else{
      timeend3 = micros();
      int elapsedTime3 = timeend3 - timestart3;
      distance3 = elapsedTime3 * 0.0343/2;
      }
  }

void motorDirection(int motorpinCW, int motorpinCCW, int pwmpin, int pwm){
  if(pwm < 0){
    int mapped = -pwm*255;
    digitalWrite(motorpinCW, LOW);
    digitalWrite(motorpinCCW, HIGH);
    analogWrite(pwmpin, mapped);
  }
  else{
    int mapped = pwm*255;
    digitalWrite(motorpinCW, HIGH);
    digitalWrite(motorpinCCW, LOW);
    analogWrite(pwmpin, mapped);
  }
}
