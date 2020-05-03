#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

typedef enum {
	ROTATE,
	ROTATE_CONST,
	FORWARD_STEER,
	STOP
} MOTOR_STATE_t;



void motor_control_start(void);

void motor_set_state(MOTOR_STATE_t state);



#endif
