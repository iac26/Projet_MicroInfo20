#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <arm_math.h>
#include <sensors/imu.h>

#include <pickup_detector.h>


#define PERIOD_MS	100

#define MAX_SAMPLES	5

#define FILTER		2
#define PICKUP_THRESH	5000
#define REST_THRESH	100

#define INF_INT16 	32767
#define MAX(a,b)	((a)>(b)?(a):(b))

static int16_t values_x[MAX_SAMPLES];
static int16_t values_y[MAX_SAMPLES];
static int16_t values_z[MAX_SAMPLES];

static int16_t x_avg;
static int16_t x_max;
static int16_t x_min;

static int16_t y_avg;
static int16_t y_max;
static int16_t y_min;

static int32_t z_avg;
static int32_t z_max;
static int32_t z_min;

static int16_t x_pk;
static int16_t y_pk;
static int16_t z_pk;

static int16_t sum;

static uint8_t nb_samples;

static PICKUP_DETECTOR_STATE_t pickup_detector_state;

static THD_WORKING_AREA(waPickupDetect, 256);
static THD_FUNCTION(PickupDetect, arg)
{

	chRegSetThreadName("PickupDetect");
	(void) arg;

	systime_t time;

	nb_samples = 0;

	while (1) {
		time = chVTGetSystemTime();
		PROTEC(nb_samples, MAX_SAMPLES, "pickup1");
		values_x[nb_samples] = get_acc_filtered(X_AXIS, FILTER);
		values_y[nb_samples] = get_acc_filtered(Y_AXIS, FILTER);
		values_z[nb_samples] = get_acc_filtered(Z_AXIS, FILTER);

		nb_samples++;

		if (nb_samples >= MAX_SAMPLES) {
			x_max = -INF_INT16;
			y_max = -INF_INT16;
			z_max = -INF_INT16;

			x_min = INF_INT16;
			y_min = INF_INT16;
			z_min = INF_INT16;

			x_avg = 0;
			y_avg = 0;
			z_avg = 0;

			for (uint8_t i = 0; i < MAX_SAMPLES; i++) {
				if (values_x[i] > x_max) {
					x_max = values_x[i];
				}
				if (values_x[i] < x_min) {
					x_min = values_x[i];
				}
				x_avg += values_x[i];

				if (values_y[i] > y_max) {
					y_max = values_y[i];
				}
				if (values_y[i] < y_min) {
					y_min = values_y[i];
				}
				y_avg += values_y[i];

				if (values_z[i] > z_max) {
					z_max = values_z[i];
				}
				if (values_z[i] < z_min) {
					z_min = values_z[i];
				}
				z_avg += values_z[i];
			}
			x_avg /= nb_samples;
			y_avg /= nb_samples;
			z_avg /= nb_samples;
			nb_samples = 0;

			x_pk = MAX(abs(x_avg - x_min), abs(x_avg - x_max));
			y_pk = MAX(abs(y_avg - y_min), abs(y_avg - y_max));
			z_pk = MAX(abs(z_avg - z_min), abs(z_avg - z_max));

			sum = x_pk + y_pk + z_pk;

			//chprintf((BaseSequentialStream *) &SD3, "PD: (%d) | (%d) | (%d) | %d\n", x_pk, y_pk, z_pk, sum);

			if (pickup_detector_state == PD_PICKED_UP) {
				if (x_pk < REST_THRESH && y_pk < REST_THRESH && z_pk < REST_THRESH) {
					pickup_detector_state = PD_RESTING;
				}
			} else {
				if (x_pk > PICKUP_THRESH || y_pk > PICKUP_THRESH || z_pk > PICKUP_THRESH) {
					pickup_detector_state = PD_PICKED_UP;
				}
			}
		}
	}

	chThdSleepUntilWindowed(time, time + MS2ST(PERIOD_MS));

}

PICKUP_DETECTOR_STATE_t get_pd_state(void)
{
	return pickup_detector_state;
}

void pickup_detector_start(void)
{

	calibrate_acc();
	pickup_detector_state = PD_RESTING;

	chThdCreateStatic(waPickupDetect, sizeof(waPickupDetect), NORMALPRIO, PickupDetect, NULL);
}
