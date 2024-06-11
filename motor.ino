#include <AFMotor.h>

// DC motors on M1, M2, M3, M4
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

void setup() {
    Serial.begin(9600); // set up Serial library at 9600 bps

    motor1.setSpeed(200);
    motor2.setSpeed(200);
    motor3.setSpeed(200);
    motor4.setSpeed(200);
}

void loop() {

    if (Serial.available() > 0) {
        // String command = Serial.readStringUntil('\n');
        char command = Serial.read();
        
        if (command == '1') {

            motor1.setSpeed(200);
            motor2.setSpeed(200);
            motor3.setSpeed(200);
            motor4.setSpeed(200);
            motor1.run(FORWARD);
            motor2.run(BACKWARD);
            motor3.run(FORWARD);
            motor4.run(BACKWARD);
        } else if (command == '4') {
            motor1.run(BACKWARD);
            motor2.run(BACKWARD);
            motor3.run(BACKWARD);
            motor4.run(BACKWARD);
        } else if (command == '2') {

            motor1.setSpeed(200); 
            motor2.setSpeed(100);
            motor3.setSpeed(200); 
            motor4.setSpeed(100);
            motor1.run(FORWARD);
            motor2.run(BACKWARD);
            motor3.run(FORWARD);
            motor4.run(BACKWARD);
        } else if (command == '3') {
            motor1.setSpeed(100); // 좌측 바퀴 속도 그대로
            motor2.setSpeed(200);
            motor3.setSpeed(100); // 우측 바퀴 속도 줄이기
            motor4.setSpeed(200);
            motor1.run(FORWARD);
            motor2.run(BACKWARD);
            motor3.run(FORWARD);
            motor4.run(BACKWARD);
        } else if (command == "stop") {
            motor1.run(RELEASE);
            motor2.run(RELEASE);
            motor3.run(RELEASE);
            motor4.run(RELEASE);
        }
    }
}