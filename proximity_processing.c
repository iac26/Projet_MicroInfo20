#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <sensors/proximity.h>

#include <proximity_processing.h>

#define PERIOD_MS 100

//#define DEBUG

#define WEIRD_THRESH	2000

#define DATA2MM(val) (1.0/sqrt(val)*438.85)

static uint16_t distances[PROXIMITY_NB_CHANNELS]; //en mm
static uint16_t valeurs[PROXIMITY_NB_CHANNELS];

/*
 * On aurait pu se passer de ce thread et juste retourner les valeurs modifiées des capteurs de distances.
 * Mais on rejette parfois une mesure et donc on souhaiterais garder la mesure précédente pour avoir une
 * mesure disponible en cas de lecture juste après un rejet.
 */

static THD_WORKING_AREA(waProximityProcessor, 256);
static THD_FUNCTION(ProximityProcessor, arg)
{

	chRegSetThreadName("ProximityProcessor");
	(void) arg;

	systime_t time;

	while (1) {
		time = chVTGetSystemTime();
#ifdef DEBUG
//		chprintf((BaseSequentialStream *) &SD3, "time %d\n", ST2MS(time));
#endif
		for (uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
			if (abs(get_calibrated_prox(i)) < WEIRD_THRESH) {
				PROTEC(i, PROXIMITY_NB_CHANNELS, "prox1");
				valeurs[i] = get_calibrated_prox(i);
				distances[i] = DATA2MM(valeurs[i]);
			}
#ifdef DEBUG
			//chprintf((BaseSequentialStream *) &SD3, "ir %d: prox_c: %d dist(mm): %d\n", i, valeurs[i], distances[i]);
#endif
		}
		//chprintf((BaseSequentialStream *) &SD3, "%6.d,%6.d,%6.d,%6.d,%6.d,%6.d,%6.d,%6.d\n",valeurs[0],valeurs[1],valeurs[2],valeurs[3],valeurs[4],valeurs[5],valeurs[6],valeurs[7]);
#ifdef DEBUG
//		chprintf((BaseSequentialStream *) &SD3, "\n\n\n");
#endif

		chThdSleepUntilWindowed(time, time + MS2ST(PERIOD_MS));
	}

	//function
}

uint16_t * get_values_ptr(void)
{
	return valeurs;
}

uint16_t * get_distances_ptr(void)
{
	return distances;
}

void proximity_processing_start(void)
{
	calibrate_ir();
	chThdCreateStatic(waProximityProcessor, sizeof(waProximityProcessor),
	NORMALPRIO, ProximityProcessor, NULL);

}

