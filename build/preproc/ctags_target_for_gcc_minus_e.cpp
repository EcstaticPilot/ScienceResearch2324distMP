# 1 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino"
# 2 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino" 2
# 3 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino" 2

// The sample code for driving one way motor encoder
const byte encoderRpinA = 2; // A pin -> the interrupt pin 0
const byte encoderRpinB = 16; // B pin -> the digital pin 16
const byte encoderLpinA = 3; // A pin -> the interrupt pin 1
const byte encoderLpinB = 18; // B pin -> the digital pin 18

const float radius = 1.26;
# 12 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\ScienceResearch2324.ino" 2

// double Kp = 4, Ki = 50, Kd = 0;
double Kp = 2, Ki = 40, Kd = 0.55;
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
  PID speedController = PID(&rpm, &output, &targetSpeed, Kp, Ki, Kd, 0);
  wheel(byte encoderPinA, byte encoderPinB, int motorNumber, float radius, boolean reverse = false) : radius(radius), reverse(reverse), encoderPinA(encoderPinA), encoderPinB(encoderPinB)
  {
    speedController.SetSampleTime(100);
    this->motor = AF_DCMotor(motorNumber);
    speedController.SetMode(1);
    speedController.SetSampleTime(100);
    pinMode(encoderPinA, 0x0);
  };
  void run(int speed)
  {
    motor.setSpeed(((speed)>0?(speed):-(speed)));

    if (reverse)
    {
      speed = -1 * speed;
    }
    if (speed > 0)
    {
      motor.run(1);
    }
    else if (speed < 0)
    {
      motor.run(2);
    }
    else
    {
      motor.run(4);
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
      float out = (((output + targetSpeed * kf))<(255)?((output + targetSpeed * kf)):(255));
      run((reverse ? -1 : 1) * (out));
      // Serial.print("output: ");
      // Serial.println(output + targetSpeed * kf);
    }
  };

  void runPID(float targetSpeed)
  {
    setTargetSpeed(targetSpeed);
    runPID();
  };

  void updateSpeed()
  {
    int Lstate = digitalRead(encoderPinA);
    if ((encoderPinALast == 0x0) && Lstate == 0x1)
    {
      int val = digitalRead(encoderPinB);
      if (val == 0x0 && Direction)
      {
        Direction = false; // Reverse
      }
      else if (val == 0x1 && !Direction)
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

  double getSpeed()
  {
    rpm = (count / 1920.0) / 0.1 * 60;
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
    return rotations * 2 * 3.141592 * radius;
  };
};

/*

  */
wheel wheelL = wheel(encoderLpinA, encoderLpinB, 1, radius, true);
wheel wheelR = wheel(encoderRpinA, encoderRpinB, 4, radius);

double RPMtoIPS(double rpm)
{
  return rpm * 2 * 3.141592 * radius / 60;
}
double IPStoRPM(double ips)
{
  return ips * 60 / (2 * 3.141592 * radius);
}
void setup()
{
  Serial.begin(115200); // Initialize the serial port

  EncoderInit(); // Initialize the module
  float target = 4.3;
  // SCurveProfile profile = SCurveProfile(target, 1.0, 1.2, 3.42, 3.6, 0.2, 0.5, 1.17);
  wheelL.rotations = 0;
  wheelR.rotations = 0;
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
float target = 24;
float a1 = 5;
float a2 = 2.5;
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

  float output = profile.getOutputDist((wheelR.distTravelled() + wheelL.distTravelled()) / 2.0);
  // float output = profile.getOutputTime(millis()/1000.0);
  if (output != 0)
  {
    wheelL.runPID(IPStoRPM(output));
    wheelR.runPID(IPStoRPM(output));

    // Serial.print("dist:");
    Serial.print((wheelR.distTravelled() + wheelL.distTravelled()) / 2.0);
    Serial.print(", ");
    Serial.print(millis() / 1000.0);
    Serial.print(", ");
    Serial.print(output);
    Serial.print(", ");
    Serial.print(profile.getOutputTime(millis() / 1000.0));
    Serial.print(", ");
    Serial.println(RPMtoIPS((wheelL.rpm + wheelR.rpm) / 2.0));

    // Serial.print(", ");
    // Serial.println(((wheelR.speed+wheelL.speed)/2) - IPStoRPM(output));

    wheelL.updateRotations();
    wheelR.updateRotations();
    delay(100);
  }

  else
  {
    Serial.println("Done");
    Serial.print("rotations:");
    Serial.print((wheelR.rotations + wheelL.rotations) / 2.0);
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
  /*
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

  pinMode(encoderRpinB, 0x0);
  attachInterrupt(0, wheelSpeedR, 1);
  pinMode(encoderLpinB, 0x0);
  attachInterrupt(1, wheelSpeedL, 1);
}

void wheelSpeedR()
{
  wheelR.updateSpeed();
}

void wheelSpeedL()
{
  wheelL.updateSpeed();
}
