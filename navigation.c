#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <leds.h>
#include <motors.h>

#include <pickup_detector.h>
#include <audio_processing.h>
#include <image_processing.h>
#include <proximity_processing.h>
#include <navigation.h>

//#define DEBUG

#define PERIOD_MS 	100
#define KP 		1

//#define DEBUG

#define DISTANCE_TO_WALL	50 //mm
#define DISTANCE_TO_ADJUST	30
#define FOLLOW_WALL_SPEED	300
#define SOUND_SEARCH_SPEED	200
#define APPROACH_TARGET_SPEED	350
#define	MOVE_SPEED		500
#define ROTATION_SPEED		200
#define TI_ALIGN_TOL		15
#define ALIGN_TOL		3
#define ADJUST_TOL		3
#define DISTANCE_TO_WALL_TOL_F	30
#define DISTANCE_TO_WALL_TOL_N	10
#define FAR			200

#define ROBOT_RAD		(60/2)
#define ROBOT_PERIM		(2*M_PI*ROBOT_RAD)
#define WHEEL_PERIM		130
#define STEPS_TURN		1000
#define TURN			(STEPS_TURN/WHEEL_PERIM*ROBOT_PERIM)
#define QUART_TURN		(TURN/4)
#define HALF_TURN		(TURN/2)

#define DIST_PROB_T		10
#define NO_SIDE_T		10
#define END_OF_WALL_T		20
#define CORRECT_WALL_T		5
#define ALIGN_FAR_T		5

#define TARGET_SPOTTED_T	5
#define TARGET_LOST_T		5

#define NO_WALL_T		80
#define SOUND_DIR_T		15
#define SOUND_DIR_RED_T		5

#define COLOR_BLUE		0, 0, 128	//ARRIVED
#define COLOR_GREEN		0, 128, 28	//SOUND SEACH
#define COLOR_LBLUE		0, 128, 90	//SOUND SEACH
#define COLOR_TURKOISE		0, 128, 90	//SOUND SEACH
#define COLOR_PINK		128, 0, 128	//ALIGN TARGET
#define COLOR_VIOLET		64, 0, 128	//APPROACH TARGET
#define COLOR_BORDO		128, 32, 0	//ALIGN WALL
#define COLOR_RED		128, 0, 0	//MOVE
#define COLOR_YELLO		128, 128, 0	//FOLLOW_WALL
#define COLOR_DYELO		128, 90, 0	//FOLLOW_WALL WALLSIDE
#define COLOR_ORANG		128, 64, 0	//ADJUST_WALL
#define COLOR_DORANG		128, 32, 0	//ADJUST_WALL
#define COLOR_BLACK		0, 0, 0		//PICKED UP

#define COLOR_DDORAN		128, 16, 0		//PICKED UP

#define LED_T1			30
#define LED_T2			75
#define LED_T3			105
#define LED_T4			150

static NAVIGATION_STATE_t navigation_state;
static FOLLOW_WALL_STATE_t follow_wall_state;
static TARGET_INSIGHT_STATE_t target_insight_state;
static SENSOR_NAME_t wall_side;

static int16_t sound_angle;

static uint16_t wall_dist;
static uint16_t align_far_c;
static uint16_t end_of_wall_c;
static uint16_t correct_wall_c;
static uint16_t dist_prob_c;
static uint16_t target_spotted_c;
static uint16_t target_lost_c;
static uint16_t no_side_c;
static uint16_t sound_dir_red_c;

static uint16_t* distances;

static int16_t l_speed;
static int16_t r_speed;

void leds_angle(int16_t angle)
{
	if (angle > 0) {
		if (angle > LED_T4) {
			//L5
			set_led(LED5, 1);

		} else if (angle > LED_T3) {
			//L6
			set_rgb_led(LED6, COLOR_RED);

		} else if (angle > LED_T2) {
			//L7
			set_led(LED7, 1);

		} else if (angle > LED_T1) {
			//L8
			set_rgb_led(LED8, COLOR_RED);

		} else {
			//l1
			set_led(LED1, 1);
		}
	} else {
		if (angle < -LED_T4) {
			//L5
			set_led(LED5, 1);

		} else if (angle < -LED_T3) {
			//L4
			set_rgb_led(LED4, COLOR_RED);

		} else if (angle < -LED_T2) {
			//3
			set_led(LED3, 1);

		} else if (angle < -LED_T1) {
			//L2
			set_rgb_led(LED2, COLOR_RED);

		} else {
			//l1
			set_led(LED1, 1);
		}
	}
}

//essaye de se tourner vers le son
//donc on a besoin d'un PID -> target proche de 0 et current phase +/-
//si on est en dessous d'un threshold proche de 0 on entre en mode move formward.
//si pas de son valide-> wait
void sound_search(void)
{

	sound_angle = get_sound_angle();
	leds_angle(sound_angle);
	if (get_sound_valid()) {

		int16_t error = sound_angle;
		float kp = 5;
		int16_t speed = kp * error;
		if (speed > SOUND_SEARCH_SPEED) {
			speed = SOUND_SEARCH_SPEED;
		}
		if (speed < -SOUND_SEARCH_SPEED) {
			speed = -SOUND_SEARCH_SPEED;
		}
		l_speed = -speed;
		r_speed = speed;
#ifdef DEBUG
		chprintf((BaseSequentialStream *) &SD3, "sound search: %d freq: %d\n", sound_angle, get_sound_freq());
#endif
		if (abs(error) < SOUND_DIR_T) {
			l_speed = 0;
			r_speed = 0;
			sound_dir_red_c++;
			if (sound_dir_red_c > SOUND_DIR_RED_T) {
				l_speed = MOVE_SPEED;
				r_speed = MOVE_SPEED;
				navigation_state = N_MOVE_FORWARD;
				sound_dir_red_c = 0;
			}
		} else {
			sound_dir_red_c = 0;
		}
	} else {
#ifdef DEBUG
		chprintf((BaseSequentialStream *) &SD3, "no valid sound\n");
#endif
		l_speed = 0;
		r_speed = 0;
	}

	//l_speed = MOVE_SPEED;
	//r_speed = MOVE_SPEED;
	//navigation_state = N_MOVE_FORWARD;
}

//le robot avance en checkant si il y a un mur devant
// si mur: il entre en mode align_wall
//si N secondes se sont ecoulées il entre en mode sound search.
// detection d'arrivee-> arrived
void move_forward(void)
{
#ifdef DEBUG
	chprintf((BaseSequentialStream *) &SD3, "moving  %5d %5d\n", distances[S_FORWARD_LEFT], distances[S_FORWARD_RIGHT]);
#endif

	if (distances[S_FORWARD_LEFT] < NO_WALL_T || distances[S_FORWARD_RIGHT] < NO_WALL_T) {
		l_speed = distances[S_FORWARD_RIGHT] * 5;
		r_speed = distances[S_FORWARD_RIGHT] * 5;
	}
	if (distances[S_FORWARD_LEFT] < DISTANCE_TO_WALL || distances[S_FORWARD_RIGHT] < DISTANCE_TO_WALL) {
		l_speed = 0;
		r_speed = 0;
		if (distances[S_FORWARD_RIGHT] > distances[S_FORWARD_LEFT]) {
			wall_side = S_LEFT;

		} else {
			wall_side = S_RIGHT;
		}
		navigation_state = N_FOLLOW_WALL;
		follow_wall_state = FW_ALIGN;
	}
}

//le robot se tourne de 90deg droite ou gauche? event. selon les deux capteurs devant.
//ensuite on entre en mode follow_wall
void align_wall(void)
{
	if (distances[S_FORWARD_RIGHT] > FAR && distances[S_FORWARD_LEFT] > FAR) {
		left_motor_set_speed(0);
		right_motor_set_speed(0);
#ifdef DEBUG
		chprintf((BaseSequentialStream *) &SD3, "align far\n");
#endif
		align_far_c++;
		if (align_far_c > ALIGN_FAR_T) {
			align_far_c = 0;
			navigation_state = N_SOUND_SEARCH;
		}
		return;
	} else {
		align_far_c = 0;
	}

	int16_t target = 0;
	int16_t current = distances[S_FORWARD_RIGHT] - distances[S_FORWARD_LEFT];
	int16_t error = target - current;
	float kp = 5;
	int16_t speed = kp * error;
	l_speed = speed;
	r_speed = -speed;
#ifdef DEBUG
	chprintf((BaseSequentialStream *) &SD3, "aligning: %5d %5d err %d\n", distances[S_FORWARD_LEFT], distances[S_FORWARD_RIGHT], error);
#endif
	if (abs(error) < ALIGN_TOL) {
		l_speed = 0;
		r_speed = 0;
		follow_wall_state = FW_ADJUST;
	}

}

void adjust_wall(void)
{
	if (distances[S_FORWARD_RIGHT] > FAR && distances[S_FORWARD_LEFT] > FAR) {
		l_speed = 0;
		r_speed = 0;
#ifdef DEBUG
		chprintf((BaseSequentialStream *) &SD3, "ajust_wall: dist_prob\n");
#endif
		dist_prob_c++;
		if (dist_prob_c > DIST_PROB_T) {
			dist_prob_c = 0;
			navigation_state = N_SOUND_SEARCH;
		}
		return;
	} else {
		dist_prob_c = 0;
	}
	int16_t target = DISTANCE_TO_ADJUST;
	int16_t current = (distances[S_FORWARD_RIGHT] + distances[S_FORWARD_LEFT]) / 2;
	int16_t error = current - target;
	float kp = 5;
	int16_t speed = kp * error;
	l_speed = speed;
	r_speed = speed;
#ifdef DEBUG
	chprintf((BaseSequentialStream *) &SD3, "ajusting: %5d %5d err %d\n", distances[S_FORWARD_LEFT], distances[S_FORWARD_RIGHT], error);
#endif
	if (abs(error) < ALIGN_TOL) {
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		if (abs(distances[S_FORWARD_RIGHT] - distances[S_FORWARD_LEFT]) < ALIGN_TOL) {
			follow_wall_state = FW_ROTATE_P;
		} else {
			follow_wall_state = FW_ALIGN;
		}
	}

}

void rotate_90p(void)
{
	int16_t current = abs(left_motor_get_pos());
	int16_t target = QUART_TURN;
	int16_t error = current - target;
	float kp = 10;
	int16_t speed = kp * error;

	if (speed > ROTATION_SPEED) {
		speed = ROTATION_SPEED;
	}
	if (speed < -ROTATION_SPEED) {
		speed = -ROTATION_SPEED;
	}

	if (wall_side == S_RIGHT) {
		l_speed = speed;
		r_speed = -speed;
	} else {
		l_speed = -speed;
		r_speed = speed;
	}

	if (error == 0) {
		wall_dist = distances[wall_side];
		if (abs(DISTANCE_TO_ADJUST - wall_dist) < DISTANCE_TO_WALL_TOL_N) {
			l_speed = FOLLOW_WALL_SPEED;
			r_speed = FOLLOW_WALL_SPEED;
			follow_wall_state = FW_FOLLOW;
			no_side_c = 0;
		} else {
			no_side_c++;
			if (no_side_c > NO_SIDE_T) {
				navigation_state = N_SOUND_SEARCH;
				no_side_c = 0;
			}
		}
#ifdef DEBUG
		chprintf((BaseSequentialStream *) &SD3, "calibrating side %d, %d\n", wall_side, wall_dist);
#endif

	} else {
#ifdef DEBUG
		chprintf((BaseSequentialStream *) &SD3, "rotate parallel %d, %d\n", left_motor_get_pos(), right_motor_get_pos());
#endif
	}
}

void rotate_90s(void)
{
	int16_t current = abs(left_motor_get_pos());
	int16_t target = QUART_TURN;
	int16_t error = current - target;
	float kp = 10;
	int16_t speed = kp * error;

	if (speed > ROTATION_SPEED) {
		speed = ROTATION_SPEED;
	}
	if (speed < -ROTATION_SPEED) {
		speed = -ROTATION_SPEED;
	}

	if (wall_side == S_LEFT) {
		l_speed = speed;
		r_speed = -speed;
	} else {
		l_speed = -speed;
		r_speed = speed;
	}

	if (error == 0) {
		l_speed = 0;
		r_speed = 0;
		follow_wall_state = FW_ALIGN;
	}
#ifdef DEBUG
	chprintf((BaseSequentialStream *) &SD3, "rotate senkrecht %d, %d\n", left_motor_get_pos(), right_motor_get_pos());
#endif
}
// avance tout droit + correction directionnelle selon la distance sur le capteur droite ou gauche
// variable "WALL_SIDE"
//au meme temps on check le capteur avant: si mur devant -> turn around
void follow_wall(void)
{
	uint16_t front_dist = (distances[S_FORWARD_LEFT] + distances[S_FORWARD_RIGHT]) / 2;
#ifdef DEBUG
	chprintf((BaseSequentialStream *) &SD3, "following wall%d: %d/%d f: %d\n", wall_side, distances[wall_side], wall_dist, front_dist);
#endif

	if (distances[wall_side] > NO_WALL_T) {
		end_of_wall_c++;
		if (end_of_wall_c > END_OF_WALL_T) {
			l_speed = 0;
			r_speed = 0;
			left_motor_set_pos(0);
			right_motor_set_pos(0);
			end_of_wall_c = 0;
			follow_wall_state = FW_ROTATE_S;
		}

	} else {
		end_of_wall_c = 0;
		if (distances[wall_side] - wall_dist > DISTANCE_TO_WALL_TOL_F || distances[wall_side] - wall_dist < -DISTANCE_TO_WALL_TOL_N) {
			correct_wall_c++;
			if (correct_wall_c > CORRECT_WALL_T) {
				l_speed = 0;
				r_speed = 0;
				left_motor_set_pos(0);
				right_motor_set_pos(0);
				follow_wall_state = FW_ROTATE_S;
				correct_wall_c = 0;
			}

		} else {
			correct_wall_c = 0;
		}

	}

	if (front_dist < DISTANCE_TO_WALL) {
		l_speed = 0;
		r_speed = 0;
		if (wall_side == S_LEFT) {
			wall_side = S_RIGHT;
		} else {
			wall_side = S_LEFT;
		}
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		follow_wall_state = FW_TURN_AROUND;
	}
}

//rotation 180deg + change la valeur de wall_side -> retourne a follow_wall
void turn_around(void)
{
	int16_t current = abs(left_motor_get_pos());
	int16_t target = HALF_TURN;
	int16_t error = current - target;
	float kp = 10;
	int16_t speed = kp * error;

	if (speed > ROTATION_SPEED) {
		speed = ROTATION_SPEED;
	}
	if (speed < -ROTATION_SPEED) {
		speed = -ROTATION_SPEED;
	}

	if (wall_side == S_RIGHT) {
		l_speed = -speed;
		r_speed = speed;
	} else {
		l_speed = speed;
		r_speed = -speed;
	}

	if (error == 0) {
		wall_dist = distances[wall_side];
		if (abs(
		DISTANCE_TO_ADJUST - wall_dist) < DISTANCE_TO_WALL_TOL_N) {
			l_speed = FOLLOW_WALL_SPEED;
			r_speed = FOLLOW_WALL_SPEED;
			follow_wall_state = FW_FOLLOW;
			no_side_c = 0;
		} else {
			no_side_c++;
			if (no_side_c > NO_SIDE_T) {
				navigation_state = N_SOUND_SEARCH;
				no_side_c = 0;
			}
		}
#ifdef DEBUG
		chprintf((BaseSequentialStream *) &SD3, "calibrating side %d, %d\n", wall_side, wall_dist);
#endif

	} else {
#ifdef DEBUG
		chprintf((BaseSequentialStream *) &SD3, "rotate around %d, %d\n", left_motor_get_pos(), right_motor_get_pos());
#endif
	}
}

void align_target(void)
{
	if (!get_pattern_visible()) {
		target_lost_c++;
		if (target_lost_c > TARGET_LOST_T) {
			navigation_state = N_SOUND_SEARCH;
			target_lost_c = 0;
		}
	} else {
		target_lost_c = 0;
	}

	int16_t target = 0;
	int16_t current = get_pattern_center() - IMAGE_CENTER;
	int16_t error = target - current;
	float kp = 1;
	int16_t speed = kp * error;
	l_speed = -speed;
	r_speed = speed;
#ifdef DEBUG
	chprintf((BaseSequentialStream *) &SD3, "aligning_target: %5d err %d\n", current, error);
#endif
	if (abs(error) < ALIGN_TOL) {
		l_speed = 0;
		r_speed = 0;
		target_insight_state = TI_APPROACH;
	}
}

void approach_target(void)
{
#ifdef DEBUG
	chprintf((BaseSequentialStream *) &SD3, "approaching target: %5d %5d err %d\n", distances[S_FORWARD_LEFT], distances[S_FORWARD_RIGHT], get_pattern_center() - IMAGE_CENTER);
#endif
	l_speed = APPROACH_TARGET_SPEED;
	r_speed = APPROACH_TARGET_SPEED;

	if (abs(get_pattern_center() - IMAGE_CENTER) > TI_ALIGN_TOL) {
		target_insight_state = TI_ALIGN;
	}

	if (distances[S_FORWARD_LEFT] < DISTANCE_TO_WALL || distances[S_FORWARD_RIGHT] < DISTANCE_TO_WALL) {
		l_speed = 0;
		r_speed = 0;
		navigation_state = N_ARRIVED;
	}

}

void paused(void)
{
	left_motor_set_speed(0);
	right_motor_set_speed(0);
#ifdef DEBUG
	chprintf((BaseSequentialStream *) &SD3, "paused\n");
#endif
}

void execute(void)
{
	left_motor_set_speed(l_speed);
	right_motor_set_speed(r_speed);
}

void arrived(void)
{
#ifdef DEBUG
	chprintf((BaseSequentialStream *) &SD3, "ARRIVED!!!!\n");
#endif
}

static THD_WORKING_AREA(waNavigator, 512);
static THD_FUNCTION(Navigator, arg)
{

	chRegSetThreadName("Navigator");
	(void) arg;

	systime_t time;

	distances = get_distances_ptr();

	while (1) {
		time = chVTGetSystemTime();

		if (get_pd_state() == PD_PICKED_UP) {
			paused();
			set_body_led(0);
			continue;
		} else {
			set_body_led(1);
		}

		if (get_pattern_visible() && navigation_state != N_TARGET_INSIGHT) {
			target_spotted_c++;
			if (target_spotted_c > TARGET_SPOTTED_T) {
#ifdef DEBUG
				chprintf((BaseSequentialStream *) &SD3, "visible target: size: %d, dir: %d\n", get_pattern_width(), get_pattern_center() - IMAGE_CENTER);
#endif
				navigation_state = N_TARGET_INSIGHT;
				target_insight_state = TI_ALIGN;
			}
		} else {
			target_spotted_c = 0;
		}
		//navigation_state=N_DEBUG;
		switch (navigation_state) {
			case N_SOUND_SEARCH:
				clear_leds();
				set_rgb_led(LED2, COLOR_DDORAN);
				set_rgb_led(LED4, COLOR_DDORAN);
				set_rgb_led(LED6, COLOR_DDORAN);
				set_rgb_led(LED8, COLOR_DDORAN);
				sound_search();
				break;
			case N_MOVE_FORWARD:
				clear_leds();
				set_rgb_led(LED2, COLOR_TURKOISE);
				set_rgb_led(LED4, COLOR_TURKOISE);
				set_rgb_led(LED6, COLOR_TURKOISE);
				set_rgb_led(LED8, COLOR_TURKOISE);
				move_forward();
				break;
			case N_FOLLOW_WALL:
				switch (follow_wall_state) {
					case FW_ALIGN:
						set_rgb_led(LED2, COLOR_BORDO);
						set_rgb_led(LED4, COLOR_BORDO);
						set_rgb_led(LED6, COLOR_BORDO);
						set_rgb_led(LED8, COLOR_BORDO);
						align_wall();
						break;
					case FW_ADJUST:
						set_rgb_led(LED2, COLOR_ORANG);
						set_rgb_led(LED4, COLOR_ORANG);
						set_rgb_led(LED6, COLOR_ORANG);
						set_rgb_led(LED8, COLOR_ORANG);
						adjust_wall();
						break;
					case FW_ROTATE_P:
						set_rgb_led(LED2, COLOR_YELLO);
						set_rgb_led(LED4, COLOR_YELLO);
						set_rgb_led(LED6, COLOR_YELLO);
						set_rgb_led(LED8, COLOR_YELLO);
						rotate_90p();
						break;
					case FW_ROTATE_S:
						set_rgb_led(LED2, COLOR_DYELO);
						set_rgb_led(LED4, COLOR_DYELO);
						set_rgb_led(LED6, COLOR_DYELO);
						set_rgb_led(LED8, COLOR_DYELO);
						rotate_90s();
						break;
					case FW_FOLLOW:
						if (wall_side == S_LEFT) {
							set_rgb_led(LED2, COLOR_YELLO);
							set_rgb_led(LED4, COLOR_YELLO);
							set_rgb_led(LED6, COLOR_DYELO);
							set_rgb_led(LED8, COLOR_DYELO);
						} else {
							set_rgb_led(LED2, COLOR_DYELO);
							set_rgb_led(LED4, COLOR_DYELO);
							set_rgb_led(LED6, COLOR_YELLO);
							set_rgb_led(LED8, COLOR_YELLO);
						}
						follow_wall();
						break;
					case FW_TURN_AROUND:
						set_rgb_led(LED2, COLOR_DORANG);
						set_rgb_led(LED4, COLOR_DORANG);
						set_rgb_led(LED6, COLOR_DORANG);
						set_rgb_led(LED8, COLOR_DORANG);
						turn_around();
						break;
				}
				break;
			case N_TARGET_INSIGHT:
				switch (target_insight_state) {
					case TI_ALIGN:
						set_rgb_led(LED2, COLOR_PINK);
						set_rgb_led(LED4, COLOR_PINK);
						set_rgb_led(LED6, COLOR_PINK);
						set_rgb_led(LED8, COLOR_PINK);
						align_target();
						break;
					case TI_APPROACH:
						set_rgb_led(LED2, COLOR_VIOLET);
						set_rgb_led(LED4, COLOR_VIOLET);
						set_rgb_led(LED6, COLOR_VIOLET);
						set_rgb_led(LED8, COLOR_VIOLET);
						approach_target();
						break;

				}
				break;
			case N_ARRIVED:
				set_rgb_led(LED2, COLOR_BLUE);
				set_rgb_led(LED4, COLOR_BLUE);
				set_rgb_led(LED6, COLOR_BLUE);
				set_rgb_led(LED8, COLOR_BLUE);
				arrived();
				break;
			case N_PAUSED:
				set_rgb_led(LED2, COLOR_BLACK);
				set_rgb_led(LED4, COLOR_BLACK);
				set_rgb_led(LED6, COLOR_BLACK);
				set_rgb_led(LED8, COLOR_BLACK);
				paused();
				break;
			case N_DEBUG:
				break;
		}

		execute();

		chThdSleepUntilWindowed(time, time + MS2ST(PERIOD_MS));
	}
}

void navigation_start(void)
{
	navigation_state = N_SOUND_SEARCH;

	chThdCreateStatic(waNavigator, sizeof(waNavigator), NORMALPRIO, Navigator, NULL);

}

