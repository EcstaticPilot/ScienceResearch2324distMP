#include <PID_v1.h>
#include "MotionProfile.h"

// The sample code for driving one way motor encoder
const byte encoderRpinA = 2;  // A pin -> the interrupt pin 0
const byte encoderRpinB = 16; // B pin -> the digital pin 16
const byte encoderLpinA = 3;  // A pin -> the interrupt pin 1
const byte encoderLpinB = 18; // B pin -> the digital pin 18

const float radius = 1.26;
#include <AFMotor.h>

double Kp = 2.5, Ki = 20, Kd = 0.25;

class wheel
{
public:
  double speed = 0;
  int ticks = 0;
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
  PID speedController = PID(&speed, &output, &targetSpeed, Kp, Ki, Kd, DIRECT);
  wheel(byte encoderPinA, byte encoderPinB, int motorNumber, float radius, boolean reverse = false): radius(radius), reverse(reverse),encoderPinA(encoderPinA), encoderPinB(encoderPinB)
  {
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
    if (e)
    {
      run((reverse ? -1 : 1) * output);
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
        speed++;
        ticks++;
      }
      else
      {
        speed--;
        ticks--;
      }
    }
    else
    {
      if (!reverse)
      {
        speed--;
        ticks--;
      }
      else
      {
        speed++;
        ticks++;
      }
    }
  };

 void updateRotations()
  {
    rotations = (double)ticks / 1920.0;
    speed = 0;
  };

  double distTravelled()
  {
    rotations = (double)ticks / 1920.0;
    return rotations * 2 * 3.1415926535897932384626433832795 * radius;
  };
};


/* 

  */
wheel wheelL = wheel(encoderLpinA, encoderLpinB, 1,radius, true);
wheel wheelR = wheel(encoderRpinA, encoderRpinB, 4,radius);

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
  delay(0);/*
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
float target = 20;
float a1 = 50;
float a2 = 30;
float j1 = 500;
float j2 = 100;
float v0 = 6;
float vf = 10;
float vmax = 22;
  //SCurveProfile profile = SCurveProfile(target, 50, 22, 400, 100, 10, 5, 20);
  SCurveProfile profile = SCurveProfile(target, a1,a2,j1,j2,v0,vf,vmax);
void loop()
{
  
  float output = profile.getOutputDist(wheelR.distTravelled());
  if (output != 0)
  {

    output = profile.getOutputDist((wheelR.distTravelled()+wheelL.distTravelled())/2);

    wheelL.setTargetSpeed(IPStoRPM(output));
    wheelR.setTargetSpeed(IPStoRPM(output));
    wheelL.runPID();
    wheelR.runPID();
   // Serial.print("dist:");
    Serial.print((wheelR.distTravelled()+wheelL.distTravelled())/2);
    Serial.print(", ");
    Serial.print(output);
    Serial.print(", ");
    // Serial.print(time);
      Serial.print(", ");
    Serial.print(IPStoRPM(output));
    Serial.print(", ");
    Serial.print(wheelL.speed);
    Serial.print(", ");
    Serial.print(wheelR.speed);
    Serial.print(", ");
    Serial.println(((wheelR.speed+wheelL.speed)/2) - IPStoRPM(output));

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