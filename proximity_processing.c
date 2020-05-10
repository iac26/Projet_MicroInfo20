/*
 * Audio processing module
 * Author: Timon Binder
 */

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

static uint16_t lut_in_val[] = {12,15,17,19,22,27,32,40,50,60,70,90,110,140,190,255,390,660,1200,2500};
static uint16_t lut_out_val[] = {100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5};


/*
 * @brief		lookup table to find distance values by linear
 * 			interpolation we do not used it as the square root
 * 			is quite fast on the cortex m4 with fpu
 *
 * @param val		value to convert
 *
 * @return		converted value
 */
uint16_t distance_lut(uint16_t val)
{
	if(val < lut_in_val[0]) {
		return 200;
	}
	for(uint8_t i = 1; i < sizeof(lut_in_val)/sizeof(uint16_t); i++) {
		if(val < lut_in_val[i]){
			uint16_t a = val-lut_in_val[i-1];
			uint16_t b = lut_in_val[i]-val;
			return (a*lut_out_val[i]+b*lut_out_val[i-1])/(a+b);
		}
	}
	return 0;
}

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
			//on rejette les mesures avec des valeurs trop grandes  qui semblent etre des erreurs.
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

