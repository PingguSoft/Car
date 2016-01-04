#include <Arduino.h>
#include <Servo.h>
#include "utils.h"
#include "SerialProtocol.h"

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

#define FP_SHIFT    7
#define FP_VAL      35  // (DEG_AMOUNT * 2) / 255 * 128 ( << 7)

static Servo            servoSteer;
static SerialProtocol   *ctrl;

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
}

static u8 scale = 30;
s8 inputCallback(u8 cmd, u8 *data, u8 size, u8 *res)
{
    s8 ret = -1;

    switch (cmd) {
        case SerialProtocol::MSP_ANALOG:
/*
            if (core) {
                u8 *ptr = (u8*)res;

                *ptr = core->getBattLevel(scale);
                *(ptr + 3) = core->getBattLevel();
                ret = 7;
            }
*/
            break;

        case SerialProtocol::MSP_SET_MISC:
            scale = *(data + 18);
            ret = 0;
            break;

        case SerialProtocol::MSP_SET_RAW_RC:
            {
                int  angle;
                u8   btn, lx, ly, rx, ry;

                btn = ctrl->getButtons();
                if (btn & 0x01) {
                    setDir(DIR_REV);
                } else {
                    setDir(DIR_FWD);
                }

                ctrl->getStick(&lx, &ly, &rx, &ry);
                angle = 90 + (((lx - 128) * FP_VAL) >> FP_SHIFT);
                servoSteer.write(angle);
                setSpeed(ry);
            }
            break;
    }
    return ret;
}

void setup()
{
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);

    setDir(DIR_FWD);
	servoSteer.attach(PIN_STEER);
    servoSteer.write(DEG_CENTER);

    ctrl = new SerialProtocol();
	ctrl->init(inputCallback);

//	Serial.begin(57600);
//	while (!Serial);
//  printf(F("angle : %d\n"), angle);
}


static uint8_t car_dir = DIR_FWD;
static int     angle   = DEG_CENTER;

void loop()
{
    ctrl->handleRX();
#if 0

    setDir(DIR_FWD);
    for (int i = 100; i <= 255; i += 5) {
        //printf(F("Speed : %d angle : %d\n"), i, angle);
        setSpeed(i);

        if (angle > DEG_MAX) {
            car_dir = 0;
        } else if (angle < DEG_MIN) {
            car_dir = 1;
        }

        if (car_dir == 1) {
            angle += 5;
        } else {
            angle -= 5;
        }
        servoSteer.write(angle);
        delay(1000);
    }

    setDir(DIR_REV);
    for (int i = 100; i <= 255; i += 5) {
        //printf(F("Speed : %d angle : %d\n"), i, angle);
        setSpeed(i);

        if (angle > DEG_MAX) {
            car_dir = 0;
        } else if (angle < DEG_MIN) {
            car_dir = 1;
        }

        if (car_dir == 1) {
            angle += 5;
        } else {
            angle -= 5;
        }
        servoSteer.write(angle);
        delay(1000);
    }
#endif
}

