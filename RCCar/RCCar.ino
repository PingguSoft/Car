#include <Arduino.h>
#include <Servo.h>
#include <math.h>
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
#define DEG_AMOUNT  45

static Servo            servoSteer;
static SerialProtocol   *ctrl;
static uint8_t          dirCar = DIR_STOP;

void setDir(int dir)
{
    uint8_t in1;
    uint8_t in2;

    if (dirCar == dir)
        return;

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

    dirCar = dir;
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
                u16  btn, lx, ly, rx, ry;

                ctrl->getStick(&lx, &ly, &rx, &ry);
                btn = ctrl->getButtons();

                if (btn & 0x01) {
                    ry = ly;
                    rx = lx;
                } else if (btn & 0x02) {
                    rx = lx;
                }

                if (ry > 500) {
                    setDir(DIR_FWD);
                    ry = constrain(ry - 500, 0, 500);
                    ry = map(ry, 0, 500, 50, 254);
                } else if (ry < 500) {
                    setDir(DIR_REV);
                    ry = constrain(-(ry - 500), 0, 500);
                    ry = map(ry, 0, 500, 50, 254);
                } else {
                    setDir(DIR_STOP);
                    ry = 0;
                }
                angle = 90 + map(rx, 0, 1000, -DEG_AMOUNT, DEG_AMOUNT);
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


void loop()
{
    ctrl->handleRX();
}

