#ifndef PROXIMITY_PROCESSING_H
#define PROXIMITY_PROCESSING_H

typedef enum {
	FORWARD_RIGHT = 0,
	FRONT_RIGHT,
	RIGHT,
	BACK_RIGHT,
	BACK_LEFT,
	LEFT,
	FRONT_LEFT,
	FORWARD_LEFT
} SENSOR_NAME_t;

void proximity_processing_start(void);

uint16_t * get_values_ptr(void);

uint16_t * get_distances_ptr(void);

#endif

