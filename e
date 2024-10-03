#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>


// Motor pins
const int enA = 9;
const int in1 = 2;
const int in2 = 3;
const int enB = 10;
const int in3 = 4;
const int in4 = 5;


MPU6050 mpu;


// PID variables
double Setpoint = 0;  // 0 deg for moving straight
double Input, Output;


// PID constants
double Kp = 0.9;  // proportional
double Ki = 0.0;  // integral
double Kd = 0.1;  // derivative


PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


unsigned long startTime;
int state = 0;  // 0: forward, 1: right, 2: forward, 3: left, 4: stop


void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();


    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }


    // offsets
    mpu.setXAccelOffset(-1894);
    mpu.setYAccelOffset(-937);
    mpu.setZAccelOffset(1398);
    mpu.setXGyroOffset(32);
    mpu.setYGyroOffset(-36);
    mpu.setZGyroOffset(47);


    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-25, 25);  // motor speed


    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);


    startTime = millis();
}


void loop() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


    // time (sec)
    double dt = 0.01;
    Input += (double)gz * dt / 131.0;  // 131 LSB per degree/sec scaling factor


    pid.Compute();


    int leftSpeed, rightSpeed;
    unsigned long currentTime = millis();


    switch (state) {
        case 0:
            leftSpeed = 25 + Output;
            rightSpeed = 25 - Output;


            if (currentTime - startTime > 3000) {  // forward 1 sec
                state = 1;
                Setpoint = 67;
                startTime = currentTime;
                Input = 0;
            }
            break;


        case 1:  // left
            leftSpeed = 15 + Output;  
            rightSpeed = -15 - Output;


            if (abs(Input - 67) < 5) {  
                state = 2;
                leftSpeed = 0;
                rightSpeed = 0;
                startTime = currentTime;  //time for move forward
            }
            break;


        case 2: //forward
            leftSpeed = 25 + Output;
            rightSpeed = 25 - Output;


            if (currentTime - startTime > 4000) {  
                state = 3;
                Setpoint = -67;
                startTime = currentTime;
                Input = 0;  
            }
            break;


        case 3:  //turn right
            leftSpeed = -15 + Output;  
            rightSpeed = 15 - Output;


            if (abs(Input - (-67)) < 5) {  
                state = 4;
                leftSpeed = 0;
                rightSpeed = 0;
                startTime = currentTime;  
            }
            break;


        case 4: //forward
            leftSpeed = 25 + Output;
            rightSpeed = 25 - Output;


            if (currentTime - startTime > 4000) {  
                state = 5;
                Setpoint = -67;
                startTime = currentTime;
                Input = 0;  
            }
            break;


         case 5:  //turn right
            leftSpeed = -15 + Output;  
            rightSpeed = 15 - Output;


            if (abs(Input - (-67)) < 5) {  
                state = 6;
                leftSpeed = 0;
                rightSpeed = 0;
                startTime = currentTime;  
            }
            break;


         case 6: //forward
            leftSpeed = 25 + Output;
            rightSpeed = 25 - Output;


            if (currentTime - startTime > 4200) {  
                state = 7;
                Setpoint = -67;
                startTime = currentTime;
                Input = 0;  
            }
            break;


        case 7:  //turn right
            leftSpeed = -15 + Output;  
            rightSpeed = 15 - Output;


            if (abs(Input - (-67)) < 5) {  
                state = 8;
                leftSpeed = 0;
                rightSpeed = 0;
                startTime = currentTime;  
            }
            break;


        case 8: //forward
            leftSpeed = 25 + Output;
            rightSpeed = 25 - Output;


            if (currentTime - startTime > 6500) {  
                state = 9;
                Setpoint = -67;
                startTime = currentTime;
                Input = 0;  
            }
            break;


        case 9:  //stop
            leftSpeed = 0;
            rightSpeed = 0;


            if (currentTime - startTime > 4000) {
                while (true) {
                    delay(200);
                }
            }
            break;
    }


    leftSpeed = constrain(leftSpeed, -254, 254);
    rightSpeed = constrain(rightSpeed, -254, 254);


    analogWrite(enA, abs(leftSpeed));
    digitalWrite(in1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(in2, leftSpeed > 0 ? LOW : HIGH);


    analogWrite(enB, abs(rightSpeed));
    digitalWrite(in3, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(in4, rightSpeed > 0 ? LOW : HIGH);


    delay(10);
}


