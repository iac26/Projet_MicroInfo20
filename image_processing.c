#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <image_processing.h>
#include <camera/dcmi_camera.h>
#include <camera/po8030.h>

//#define SEND_IMAGES

#define IMAGE_WIDTH 	(PO8030_MAX_WIDTH) //160 when subsamples
#define IMAGE_HEIGHT 	160 //40 when subsampled
#define WIDTH		(IMAGE_WIDTH/4)
#define HEIGHT		(IMAGE_HEIGHT/4)
#define CORNER_X	(PO8030_MAX_WIDTH/2 - IMAGE_WIDTH/2)
#define CORNER_Y	20
#define IX(x, y) 	((x) + WIDTH * (y))	//to access pixels stored line by line

//macros pour traitement d'image

#define EDGE_TOL  	60
#define	PATTERN_TOL 	4
#define	NB_TOL  	2

#define MAX_EDGES	20

#define MAX_SUB		16
#define START_SUB	2
#define SUB_FACTOR	2
#define EXPLOR_STEP	4
#define FAILS_T		4

//we can use uint8 bause those are positive value of max 40
static uint8_t rise_dist[MAX_EDGES];
static uint8_t fall_dist[MAX_EDGES];
static uint8_t r2f_dist[MAX_EDGES];
static uint8_t f2r_dist[MAX_EDGES];

static uint8_t nb_rise;
static uint8_t nb_fall;
static uint8_t nb_r2f;
static uint8_t nb_f2r;

static uint8_t last_fall;
static uint8_t last_rise;

static int16_t fall_moy;
static int16_t rise_moy;
static int16_t r2f_moy;
static int16_t f2r_moy;


static uint8_t *img_buff_ptr;

static uint8_t subdivision;
static int16_t prev_larg;
static int16_t larg;
static int16_t col;
static int16_t found;
static uint8_t fails;
static int16_t first_col;
static int16_t last_col;
static uint16_t pattern_width;
static uint8_t pattern_visible;
static uint16_t pattern_center;

static uint16_t frames_processed;

static BSEMAPHORE_DECL(image_ready_sem, TRUE);

uint8_t analyse_col(uint8_t* image, uint16_t x)
{
	nb_rise = 0;
	nb_fall = 0;
	nb_r2f = 0;
	nb_f2r = 0;
	last_fall = 0;
	last_rise = 0;
	volatile uint16_t y = 0;
	while (y < (HEIGHT - 1)) {
		volatile int16_t px1 = image[IX(x, y)];
		volatile int16_t px2 = image[IX(x, y+1)];
		if ((px1 - px2) < -EDGE_TOL) {
			//chprintf((BaseSequentialStream *) &SD3, "%d: found_rise %d: %d\n", x, y,  px1-px2);
			if (nb_rise == 0) {
				last_rise = y;
				nb_rise += 1;
				if (nb_fall != 0) {
					f2r_dist[nb_f2r] = last_rise - last_fall;
					nb_f2r += 1;
				}
			} else {
				rise_dist[nb_rise - 1] = y - last_rise;
				nb_rise += 1;
				last_rise = y;
				if (nb_fall != 0) {
					f2r_dist[nb_f2r] = last_rise - last_fall;
					nb_f2r += 1;
				}

			}
		}
		if ((px1 - px2) > EDGE_TOL) {
			//chprintf((BaseSequentialStream *) &SD3, "%d found_fall %d: %d\n", x, y, px1-px2);
			if (nb_fall == 0) {
				last_fall = y;
				nb_fall += 1;
				if (nb_rise != 0) {
					r2f_dist[nb_r2f] = last_fall - last_rise;
					nb_r2f += 1;
				}
			} else {
				fall_dist[nb_fall - 1] = y - last_fall;
				nb_fall += 1;
				last_fall = y;
				if (nb_rise != 0) {
					r2f_dist[nb_r2f] = last_fall - last_rise;
					nb_r2f += 1;
				}
			}
		}
		if (nb_fall > MAX_EDGES || nb_rise > MAX_EDGES || nb_r2f >= MAX_EDGES || nb_f2r >= MAX_EDGES) {
			break;
		}
		y += 1;
	}

	fall_moy = 0;
	rise_moy = 0;
	r2f_moy = 0;
	f2r_moy = 0;

	if (nb_rise > 1 && nb_fall > 1 && nb_r2f > 0 && nb_f2r > 0) {
		uint8_t i;
		for (i = 0; i < nb_fall - 1; i++) {
			fall_moy += fall_dist[i];
		}
		fall_moy /= (nb_fall - 1);

		for (i = 0; i < nb_rise - 1; i++) {
			rise_moy += rise_dist[i];
		}
		rise_moy /= (nb_rise - 1);

		for (i = 0; i < nb_r2f; i++) {
			r2f_moy += r2f_dist[i];
		}
		r2f_moy /= nb_r2f;

		for (i = 0; i < nb_f2r; i++) {
			f2r_moy += f2r_dist[i];
		}
		f2r_moy /= nb_f2r;

		if (abs(fall_moy - rise_moy) < PATTERN_TOL && abs(r2f_moy / 2 - f2r_moy) < PATTERN_TOL && nb_rise > NB_TOL) {
			//chprintf((BaseSequentialStream *) &SD3, "%d: valid_col\n", x);
			return 1;
		}
	}
	return 0;

}

void search_pattern(uint8_t * image)
{
	//chprintf((BaseSequentialStream *) &SD3, "new_search\n");
	subdivision = START_SUB;
	larg = WIDTH;
	found = 0;
	while (subdivision <= MAX_SUB && !found) {
		prev_larg = larg;
		larg = WIDTH / subdivision;
		col = larg;
		while (col < WIDTH && !found) {
			if ((col % prev_larg) != 0) {
				if (analyse_col(image, col)) {
					found = col;
				}
			}
			col += larg;
		}
		subdivision *= SUB_FACTOR;
	}

	if (found) {
		last_col = found;
		first_col = found;

		col = found + EXPLOR_STEP;
		fails = 0;
		while (col < WIDTH) {
			if (analyse_col(image, col)) {
				last_col = col;
				fails = 0;
			} else {
				fails += 1;
				if (fails > FAILS_T) {
					break;
				}
			}
			col += EXPLOR_STEP;
		}

		col = found - EXPLOR_STEP;
		fails = 0;
		while (col >= 0) {
			if (analyse_col(image, col)) {
				first_col = col;
				fails = 0;
			} else {
				fails += 1;
				if (fails > FAILS_T) {
					break;
				}
			}
			col -= EXPLOR_STEP;
		}
		pattern_center = (first_col + last_col) / 2;
		pattern_width = last_col - first_col;
		pattern_visible = 1;
	} else {
		pattern_visible = 0;
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg)
{

	chRegSetThreadName("CaptureImage");
	(void) arg;

	po8030_advanced_config(FORMAT_YYYY, CORNER_X, CORNER_Y, IMAGE_WIDTH, IMAGE_HEIGHT, SUBSAMPLING_X4, SUBSAMPLING_X4);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	po8030_set_awb(1);

	while (1) {
		//starts a capture
		dcmi_capture_start();

		//waits for the capture to be done
		wait_image_ready();

		//signals an image has been captured
		chBSemSignal(&image_ready_sem);

	}
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg)
{

	chRegSetThreadName("ProcessImage");
	(void) arg;

	frames_processed = 0;

	while (1) {
		//waits until an image has been captured
		chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();
		//chprintf((BaseSequentialStream *) &SD3, "before_IMSEARCH\n");
		search_pattern(img_buff_ptr);
		//chprintf((BaseSequentialStream *) &SD3, "after_IMSEARCH\n");
		frames_processed += 1;

		//chprintf((BaseSequentialStream *) &SD3, "%d: v: %3d c:%3d w:%3d\n", frames_processed, pattern_visible, pattern_center, pattern_width);

#ifdef SEND_IMAGES
		uint16_t size = WIDTH*HEIGHT; //corrected size for subsampling
		chSequentialStreamWrite((BaseSequentialStream *) &SD3, (uint8_t*)"START", 5);
		chSequentialStreamWrite((BaseSequentialStream *) &SD3, (uint8_t*)&size, sizeof(uint16_t));
		chSequentialStreamWrite((BaseSequentialStream *) &SD3, (uint8_t*)&pattern_visible, sizeof(uint8_t));
		chSequentialStreamWrite((BaseSequentialStream *) &SD3, (uint8_t*)&pattern_center, sizeof(uint16_t));
		chSequentialStreamWrite((BaseSequentialStream *) &SD3, (uint8_t*)&pattern_width, sizeof(uint16_t));
		chSequentialStreamWrite((BaseSequentialStream *) &SD3, (uint8_t*)img_buff_ptr, sizeof(uint8_t) * size);
#endif

	}
}

uint8_t get_pattern_visible(void)
{
	return pattern_visible;
}

uint16_t get_pattern_center(void)
{
	return pattern_center;
}

uint16_t get_pattern_width(void)
{
	return pattern_width;
}

void image_processing_start(void)
{

	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);

}

