#include <Arduino.h>
#line 1 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
#include <PID_v1.h>
#include "MotionProfile.h"

// The sample code for driving one way motor encoder
const byte encoderRpinA = 2;  // A pin -> the interrupt pin 0
const byte encoderRpinB = 16; // B pin -> the digital pin 16
const byte encoderLpinA = 3;  // A pin -> the interrupt pin 1
const byte encoderLpinB = 18; // B pin -> the digital pin 18

const float radius = 1.26;
#include <AFMotor.h>

//double Kp = 4, Ki = 50, Kd = 0;
double Kp = 5, Ki = 30, Kd = 0;
double kf = 0;
// double Kp = 2, Ki = 20, Kd = 0;
class wheel
{
public:
  int count = 0;
  double rpm = 0;
  uint64_t ticks = 0;
  float radius;
  double rotations = 0;
  double output;
  double targetSpeed;
  boolean reverse;
  byte encoderPinA;
  byte encoderPinB;
  byte encoderPinALast;
  boolean Direction = true;
  AF_DCMotor motor = AF_DCMotor(1);
  PID speedController = PID(&rpm, &output, &targetSpeed, Kp, Ki, Kd, DIRECT);
  wheel(byte encoderPinA, byte encoderPinB, int motorNumber, float radius, boolean reverse = false) : radius(radius), reverse(reverse), encoderPinA(encoderPinA), encoderPinB(encoderPinB)
  {
    speedController.SetSampleTime(100);
    this->motor = AF_DCMotor(motorNumber);
    speedController.SetMode(AUTOMATIC);
    speedController.SetSampleTime(100);
    pinMode(encoderPinA, INPUT);
  };
  void run(int speed)
  {
    motor.setSpeed(abs(speed));
    if (reverse)
    {
      speed = -1 * speed;
    }
    if (speed > 0)
    {
      motor.run(FORWARD);
    }
    else if (speed < 0)
    {
      motor.run(BACKWARD);
    }
    else
    {
      motor.run(RELEASE);
    }
  };
  void setTargetSpeed(double targetSpeed)
  {
    this->targetSpeed = targetSpeed;
  };
  void runPID()
  {
    bool e = speedController.Compute();
    e = true;
    if (e)
    {
      run((reverse ? -1 : 1) * (output + targetSpeed * kf));
      // Serial.print("output: ");
      // Serial.println(output + targetSpeed * kf);
    }
  };

  void runPID(float targetSpeed)
  {
    setTargetSpeed(targetSpeed);
    bool e = speedController.Compute();
    e = true;
    if (e)
    {
      run((reverse ? -1 : 1) * (output + targetSpeed * kf));
      // Serial.print("output: ");
      // Serial.println(output + targetSpeed * kf);
    }
  };

  void updateSpeed()
  {
    int Lstate = digitalRead(encoderPinA);
    if ((encoderPinALast == LOW) && Lstate == HIGH)
    {
      int val = digitalRead(encoderPinB);
      if (val == LOW && Direction)
      {
        Direction = false; // Reverse
      }
      else if (val == HIGH && !Direction)
      {
        Direction = true; // Forward
      }
    }
    encoderPinALast = Lstate;

    if (!Direction)
    {
      if (!reverse)
      {
        count++;
        ticks++;
      }
      else
      {
        count--;
        ticks--;
      }
    }
    else
    {
      if (!reverse)
      {
        count--;
        ticks--;
      }
      else
      {
        count++;
        ticks++;
      }
    }
  };

  double getSpeed(){
    rpm = (count/1920.0)/0.1 * 60 ;
    return rpm;
  }

  void updateRotations()
  {
    getSpeed();
    rotations = (double)ticks / 1920.0;
    count = 0;
  };

  double distTravelled()
  {
    rotations = (double)ticks / 1920.0;
    return rotations * 2 * 3.1415926535897932384626433832795 * radius;
  };
};

/*

  */
wheel wheelL = wheel(encoderLpinA, encoderLpinB, 1, radius, true);
wheel wheelR = wheel(encoderRpinA, encoderRpinB, 4, radius);

#line 161 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
double RPMtoIPS(double rpm);
#line 165 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
double IPStoRPM(double ips);
#line 169 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
void setup();
#line 211 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
void loop();
#line 271 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
void EncoderInit();
#line 280 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
void wheelSpeedR();
#line 285 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
void wheelSpeedL();
#line 161 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
double RPMtoIPS(double rpm)
{
  return rpm * 2 * 3.1415926535897932384626433832795 * radius / 60;
}
double IPStoRPM(double ips)
{
  return ips * 60 / (2 * 3.1415926535897932384626433832795 * radius);
}
void setup()
{
  Serial.begin(115200); // Initialize the serial port

  EncoderInit(); // Initialize the module
  float target = 4.3;
  SCurveProfile profile = SCurveProfile(target, 1.0, 1.2, 3.42, 3.6, 0.2, 0.5, 1.17);
  delay(0); /*
   for (float i = 0; i <= profile.target + 0.01; i += 0.01)
   {
     auto start = millis();

     float output = profile.getOutputDist(i);

     auto end = millis();

     unsigned long diff = end - start;

     //std::cout << "Time taken " << diff.count() << " s\n";
     Serial.print(i);
     Serial.print(",");
     Serial.print(output);
     Serial.print(",");
     Serial.println(diff);
   }
   //delay(500);
   /*
   wheelL.setTargetSpeed(20);
   wheelR.setTargetSpeed(20);
   */
}
float target = 15;
float a1 = 5;
float a2 = 3;
float j1 = 50;
float j2 = 10;
float v0 = 1;
float vf = 1;
float vmax = 6;
// SCurveProfile profile = SCurveProfile(target, 50, 22, 400, 100, 10, 5, 20);
SCurveProfile profile = SCurveProfile(target, a1, a2, j1, j2, v0, vf, vmax);
// SCurveProfile profile2(10, 50,15,500,100,2,4,7);
void loop()
{
  
  
  float output = profile.getOutputDist((wheelR.distTravelled()+wheelL.distTravelled())/2);
  if (output != 0)
  {
    wheelL.runPID(IPStoRPM(output));
    wheelR.runPID(IPStoRPM(output));


   // Serial.print("dist:");
    Serial.print((wheelR.distTravelled()+wheelL.distTravelled())/2);
        Serial.print(", ");
    Serial.print(millis()/1000.0);
    Serial.print(", ");
    Serial.print(output);
    Serial.print(", ");
    Serial.print(profile.getOutputTime(millis()/1000.0));
    Serial.print(", ");
    Serial.print(RPMtoIPS(wheelL.rpm));
    Serial.print(", ");
    Serial.println(RPMtoIPS(wheelR.rpm));
    //Serial.print(", ");
    //Serial.println(((wheelR.speed+wheelL.speed)/2) - IPStoRPM(output));

    wheelL.updateRotations();
    wheelR.updateRotations();
    delay(100);
  }

  else{
  Serial.println("Done");
  Serial.print("rotations:");
  Serial.print((wheelR.rotations+wheelL.rotations)/2);
  wheelL.run(0);
  wheelR.run(0);
  delay(1000000000);
  }
  /*
  Serial.print("Left: ");
  Serial.print(wheelR.speed);
  Serial.print(" Right: ");
  Serial.println(wheelR.speed);
  *//*
  wheelR.run(255);
  Serial.print(millis() / 1000.0);
  Serial.print(", ");
  Serial.print(wheelR.getSpeed());
  Serial.print(", ");
  Serial.print(wheelR.rotations);
  Serial.print(", ");
  Serial.println(wheelR.rpm);
  wheelL.updateRotations();
  wheelR.updateRotations();
  
  delay(100);
  */
}

void EncoderInit()
{

  pinMode(encoderRpinB, INPUT);
  attachInterrupt(0, wheelSpeedR, CHANGE);
  pinMode(encoderLpinB, INPUT);
  attachInterrupt(1, wheelSpeedL, CHANGE);
}

void wheelSpeedR()
{
  wheelR.updateSpeed();
}

void wheelSpeedL()
{
  wheelL.updateSpeed();
}
