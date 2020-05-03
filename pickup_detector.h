#ifndef PICKUP_DETECTOR_H
#define PICKUP_DETECTOR_H

typedef enum {
	PD_PICKED_UP,
	PD_RESTING
} PICKUP_DETECTOR_STATE_t;

void pickup_detector_start(void);
PICKUP_DETECTOR_STATE_t get_pd_state(void);

#endif
