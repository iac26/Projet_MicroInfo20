#ifndef NAVIGATION_H
#define NAVIGATION_H

typedef enum {
	N_SOUND_SEARCH,
	N_MOVE_FORWARD,
	N_FOLLOW_WALL,
	N_PAUSED,
	N_TARGET_INSIGHT,
	N_ARRIVED,
	N_DEBUG
} NAVIGATION_STATE_t;

typedef enum {
	FW_ALIGN,
	FW_ADJUST,
	FW_ROTATE_P,
	FW_ROTATE_S,
	FW_FOLLOW,
	FW_TURN_AROUND
} FOLLOW_WALL_STATE_t;

typedef enum {
	TI_ALIGN,
	TI_APPROACH
} TARGET_INSIGHT_STATE_t;

void navigation_start(void);

#endif
