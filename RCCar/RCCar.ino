#include <SoftwareSerial.h>
#include <Servo.h>
#include "utils.h"

#define PIN_PWM     6
#define PIN_IN1     7
#define PIN_IN2     8
#define PIN_STEER   9

#define DIR_STOP    0
#define DIR_FWD     1
#define DIR_REV     2

#define DEG_CENTER  90
#define DEG_AMOUNT  35
#define DEG_MIN     (DEG_CENTER - DEG_AMOUNT)
#define DEG_MAX     (DEG_CENTER + DEG_AMOUNT)

static Servo   servoSteer;
static uint8_t angle_dir = 1;
static int     angle = 90;

void setup()
{
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);

	Serial.begin(115200);
	while (!Serial) {
		; // wait for serial port to connect. Needed for Leonardo only
	}
	servoSteer.attach(PIN_STEER);
    servoSteer.write(angle);
    printf(F("angle : %d\n"), angle);
    delay(3000);
}

void setDir(int dir)
{
    uint8_t in1;
    uint8_t in2;

    if (dir == DIR_FWD) {
        in1 = HIGH;
        in2 = LOW;
    } else if (dir == DIR_REV) {
        in1 = LOW;
        in2 = HIGH;
    } else {
        in1 = LOW;
        in2 = LOW;
    }
    digitalWrite(PIN_IN1, in1);
    digitalWrite(PIN_IN2, in2);
}

void setSpeed(int speed)
{
    analogWrite(PIN_PWM, speed);
    servoSteer.write(angle);
}


void loop()
{
    setDir(DIR_FWD);
#if 1
    for (int i = 100; i <= 255; i += 5) {
        printf(F("Speed : %d angle : %d\n"), i, angle);
        setSpeed(i);

        if (angle > DEG_MAX) {
            angle_dir = 0;
        } else if (angle < DEG_MIN) {
            angle_dir = 1;
        }

        if (angle_dir == 1) {
            angle += 5;
        } else {
            angle -= 5;
        }
        servoSteer.write(angle);
        delay(1000);
    }

    setDir(DIR_REV);
    for (int i = 100; i <= 255; i += 5) {
        printf(F("Speed : %d angle : %d\n"), i, angle);
        setSpeed(i);

        if (angle > DEG_MAX) {
            angle_dir = 0;
        } else if (angle < DEG_MIN) {
            angle_dir = 1;
        }

        if (angle_dir == 1) {
            angle += 5;
        } else {
            angle -= 5;
        }
        servoSteer.write(angle);
        delay(1000);
    }
#else
        printf(F("angle : %d\n"), angle);
        if (angle > DEG_MAX) {
            angle_dir = 0;
        } else if (angle < DEG_MIN) {
            angle_dir = 1;
        }

        if (angle_dir == 1) {
            angle += 5;
        } else {
            angle -= 5;
        }
        servoSteer.write(angle);
        delay(200);
#endif

}

