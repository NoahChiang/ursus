#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_BMP280 bmp;

const int servoy = 15;
const int servox = 14;
const int xzero = 330;
const int yzero = 300;
const int servorange = 72;
const int TVCrange = 5;
const int ignitionPin = 40;
const int chuteServo = 12;
const int chuteZero = 630;
const int chuteOpen = 360;  

const int GreenLED = 6;
const int RedLED = 7;
const int BlueLED = 20;
const int Buzzer = 21;
const int ESP32CAM = 3;

const float constantP = 0.2;
const float constantI = 0.05;
const float constantD = 0.05;
const float maxIntegral = 50.0;

float outputX = 0;
float outputY = 0;

const float lowPassFilter = 0.5;
const float desiredAngle = 0.0;
float altZero = 0.0;

bool exactLaunchTime = false;
bool abortedFlight = false;

float thetaX = 0.0, thetaY = 0.0;  
float integralX = 0.0, integralY = 0.0; 
float previousErrorX =0.0, previousErrorY =0.0;

unsigned long exactLaunchMillis;
unsigned long lastMillis;

void buzz(int frequency, int duration) {
  tone(Buzzer, frequency);
  delay(duration);
  noTone(Buzzer);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(400000);
  SD.begin(BUILTIN_SDCARD);

  pinMode(GreenLED, OUTPUT);
  pinMode(RedLED, OUTPUT);
  pinMode(BlueLED, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(ESP32CAM, OUTPUT);
  pinMode(ignitionPin, INPUT);

  digitalWrite(BlueLED, LOW);
  digitalWrite(RedLED, LOW);
  digitalWrite(GreenLED, HIGH);
  digitalWrite(ESP32CAM, HIGH);

  buzz(600, 100);
  delay(50);
  buzz(800, 100);
  delay(50);
  buzz(1000, 100);
  delay(50);
  buzz(1200, 100);

  digitalWrite(GreenLED, LOW);
  digitalWrite(RedLED, HIGH);

  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  altZero = bmp.readAltitude(1022.5);

  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);

  pwm.begin();
  pwm.setPWMFreq(60);
  pwm.setPWM(servox, 0, xzero);
  pwm.setPWM(servoy, 0, yzero);
  pwm.setPWM(chuteServo, 0, chuteZero);

  File estimationLog = SD.open("StateEstimation.txt", FILE_WRITE);
  estimationLog.println("POWER_ON");
  Serial.println("POWER_ON");

  delay(10000);

  digitalWrite(RedLED, HIGH);
  digitalWrite(BlueLED, HIGH);
  digitalWrite(GreenLED, LOW);
}

void loop() {
  if (digitalRead(ignitionPin) == LOW) {
    String attitudeData = "";
    delay(50);
    File estimationLog = SD.open("StateEstimation.txt", FILE_WRITE);

    digitalWrite(BlueLED, LOW);
    digitalWrite(RedLED, LOW);
    digitalWrite(GreenLED, HIGH);

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    estimationLog.println("LAUNCH DETECTED");
    thetaX = 0.0;
    thetaY = 0.0;
    exactLaunchMillis = millis();
    lastMillis = millis()-50;

    while ((((millis() - exactLaunchMillis)/1000.0) < 5.2) && (abortedFlight == false)) {
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);
        float timeStep = (millis() - lastMillis) * 0.001;
        lastMillis = millis();

        if (!exactLaunchTime && (a.acceleration.y + 9.81 > 2)) {
            exactLaunchTime = true;
            exactLaunchMillis = millis();
        }

        float angularXrate = (g.gyro.x * RAD_TO_DEG) + 2.6;
        thetaX += angularXrate * timeStep;
        

        float angularYrate = (g.gyro.z * RAD_TO_DEG) + 0.4;
        thetaY += angularYrate * timeStep;


        if (abs(thetaX) > 90 || abs(thetaY) > 90) {
            estimationLog.println(attitudeData);
            pwm.setPWM(chuteServo, 0, chuteOpen);
            estimationLog.println("OFF-COURSE; ABORTING");
            Serial.println("OFF-COURSE, ABORTING");
            estimationLog.print(bmp.readAltitude(1022.5) - altZero);
            Serial.print(bmp.readAltitude(1022.5) - altZero);
            estimationLog.println(" HEIGHT OF ABORT");
            Serial.println(" HEIGHT OF ABORT");
            estimationLog.print(millis() - exactLaunchMillis);
            Serial.print(millis() - exactLaunchMillis);
            estimationLog.println(" TIME OF ABORT");
            Serial.println(" TIME OF ABORT");
            estimationLog.close();
            digitalWrite(RedLED, HIGH);
            digitalWrite(BlueLED, LOW);
            digitalWrite(GreenLED, LOW);
            abortedFlight = true;
        }

        float errorX = desiredAngle - thetaX;
        float Px = constantP * errorX;

        integralX += errorX * timeStep;
        if (integralX > maxIntegral) integralX = maxIntegral;
        else if (integralX < -maxIntegral) integralX = -maxIntegral;

        float derivativeX = (errorX - previousErrorX) / timeStep;
        float Ix = constantI * integralX;
        float Dx = constantD * derivativeX;

        outputX = Px + Ix + Dx;
        if (outputX > TVCrange) outputX = TVCrange;
        else if (outputX < -TVCrange) outputX = -TVCrange;

        int pwmxoutput = map(outputX, -TVCrange, TVCrange, xzero - servorange, xzero + servorange);
        pwm.setPWM(servox, 0, pwmxoutput);

        previousErrorX = errorX;

        float errorY = desiredAngle - thetaY;
        float Py = constantP * errorY;

        integralY += errorY * timeStep;
        if (integralY > maxIntegral) integralY = maxIntegral;
        else if (integralY < -maxIntegral) integralY = -maxIntegral;

        float derivativeY = (errorY - previousErrorY) / timeStep;
        float Iy = constantI * integralY;
        float Dy = constantD * derivativeY;

        outputY = Py + Iy + Dy;
        if (outputY > TVCrange) outputY = TVCrange;
        else if (outputY < -TVCrange) outputY = -TVCrange;

        outputY = -1 * outputY;

        int pwmyoutput = map(outputY, -TVCrange, TVCrange, yzero - servorange, yzero + servorange);
        pwm.setPWM(servoy, 0, pwmyoutput);

        previousErrorY = errorY;

        attitudeData += thetaX;
        attitudeData += " ";
        attitudeData += thetaY;
        attitudeData += " ";
        attitudeData += angularXrate;
        attitudeData += " ";
        attitudeData += angularYrate;
        attitudeData += " ";
        attitudeData += outputX;
        attitudeData += " ";
        attitudeData += outputY;
        attitudeData += " ";
        attitudeData += (a.acceleration.y + 9.81);
        attitudeData += " ";
        attitudeData += (bmp.readAltitude(1022.5) - altZero);
        attitudeData += " ";
        attitudeData += (timeStep);
        attitudeData += ("\n");
        delay(35);
    }

    if (abortedFlight == false) {
      digitalWrite(GreenLED, LOW);
      digitalWrite(BlueLED, HIGH);
      digitalWrite(GreenLED, LOW);
      digitalWrite(BlueLED, HIGH);

      estimationLog.println(attitudeData);
      Serial.println(attitudeData);

      pwm.setPWM(chuteServo, 0, chuteOpen);

      float apogeeTime = (millis() - exactLaunchMillis)/1000.0;

      estimationLog.print("APOGEE DETECTED AT ");
      Serial.print("APOGEE DETECTED AT: ");
      estimationLog.print(apogeeTime);
      Serial.print(apogeeTime);
      estimationLog.println("; CHUTES DEPLOYED");
      Serial.println("; CHUTES DEPLOYED");

      float maxAlt = bmp.readAltitude(1022.5) - altZero;
      estimationLog.print("MAX ALTITUDE: ");
      Serial.print("MAX ALTITUDE: ");
      estimationLog.println(maxAlt);
      Serial.println(maxAlt);
      estimationLog.close();

      buzz(900, 500);
      delay(50);
      buzz(1300, 750);

      delay(8750);
    }
    digitalWrite(GreenLED, LOW);
    digitalWrite(BlueLED, LOW);
    digitalWrite(ESP32CAM, LOW);

    for (int i = 0; i<10000; i++) {
      buzz(1000, 100);
      delay(1000);
    }

  }
}