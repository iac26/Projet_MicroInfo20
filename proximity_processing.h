#ifndef PROXIMITY_PROCESSING_H
#define PROXIMITY_PROCESSING_H

typedef enum {
	S_FORWARD_RIGHT = 0,
	S_FRONT_RIGHT,
	S_RIGHT,
	S_BACK_RIGHT,
	S_BACK_LEFT,
	S_LEFT,
	S_FRONT_LEFT,
	S_FORWARD_LEFT
} SENSOR_NAME_t;

/*
 * @brief		start the proximity processing module
 */
void proximity_processing_start(void);

/*
 * @brief 		get the pointer to the raw results array
 *
 * @return 		pointer to the raw results array
 */
uint16_t * get_values_ptr(void);

/*
 * @brief 		get the pointer to the adjusted results array
 *
 * @return 		pointer to the adjusted results array
 */
uint16_t * get_distances_ptr(void);

#endif

