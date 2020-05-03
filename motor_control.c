#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio_processing.h>
#include <motor_control.h>

#define PERIOD_MS 	100

static int16_t target;
static int16_t current;
static int16_t error;
static float kp;

static MOTOR_STATE_t motor_state;

void rotate(void)
{

}

void rotate_const(void)
{

}

void forward_steer(void)
{

}
void stop(void)
{
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}



static THD_WORKING_AREA(waMotorController, 256);
static THD_FUNCTION(MotorController, arg)
{

	chRegSetThreadName("MotorController");
	(void) arg;

	systime_t time;

	while (1) {
		time = chVTGetSystemTime();

		switch (motor_state) {

			case ROTATE:
				rotate();
				break;
			case ROTATE_CONST:
				rotate_const();
				break;
			case FORWARD_STEER:
				forward_steer();
				break;
			case STOP:
				stop();
				break;
		}

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(PERIOD_MS));
	}
}

void motor_set_state(MOTOR_STATE_t state)
{
	motor_state = state;
}

void motor_set_target(int16_t _target)
{
	target = _target;
}

void motor_set_current(int16_t _current)
{
	current = _current;
}

void motor_control_start(void)
{
	motors_init();
	motor_state = STOP;
	chThdCreateStatic(waMotorController, sizeof(waMotorController),
	NORMALPRIO, MotorController, NULL);

}

