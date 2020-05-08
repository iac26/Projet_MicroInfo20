#ifndef PICKUP_DETECTOR_H
#define PICKUP_DETECTOR_H

typedef enum {
	PD_PICKED_UP,
	PD_RESTING
} PICKUP_DETECTOR_STATE_t;

/*
 * @brief		start the pickup detector
 */
void pickup_detector_start(void);

/*
 * @brief		get the state of the detector
 *
 * @return		state of the robot (picked up or resting)
 */
PICKUP_DETECTOR_STATE_t get_pd_state(void);

/*
 * @brief		function used to notify the pickup detector of an incomming strong movement
 */
void strong_movement_expected(void);

#endif
