/*
 * Audio processing module
 * Author: Timon Binder
 */

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <arm_const_structs.h>

//#define SEND_AUDIO

//Macros
#define NB_SAMPLES 	50
#define NO_PHASE_T	5

#define PHASE_MAX 	3
#define CONTROL_THRESH	5
#define FREQ_THRESH	5
#define DET_THRESH	15000

//utilities to access specific data
#define RE(i)		(2*(i))
#define IM(i) 		(2*(i)+1)
#define MIC(i, m)	(4*(i)+(m))
#define RAD2DEG(a)	(180/M_PI*(a))


static uint8_t finding_dir;
static int16_t directions[NB_SAMPLES];
static uint8_t refined_valid;
static uint8_t new_refined;
static uint8_t dir_i;
static uint16_t no_phase_c;
static int16_t refined_angle;


//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];


static float phase_dif_lr;
static float phase_dif_fb;
static uint8_t valid_phase;
static uint16_t freq_i;
static int16_t angle;

/*
 *	Wrapper to call a very optimized fft function provided by ARM
 *	which uses a lot of tricks to optimize the computations
 */
void doFFT_optimized(uint16_t size, float* complex_buffer)
{
	if (size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);

}

/*
 * @brief		utility function to keep angles between -pi and pi
 */
float modulo_cercle(float angle)
{
	while (angle > M_PI) {
		angle -= 2 * M_PI;
	}
	while (angle <= -M_PI) {
		angle += 2 * M_PI;
	}
	return angle;
}

/*
 * @brief		compute the sound direction using the the argument of the complex FFTs of the four microphones
 */
void detect_phase(void)
{
	//detect max index
	volatile uint16_t ix = 0;
	volatile float max = 0;
	for (uint16_t i = 0; i < FFT_SIZE / 2; i++) {
		if (micBack_output[i] > max) {
			ix = i;
			max = micBack_output[i];
		}

	}
	if (max > DET_THRESH) {
		//phases
		float phase_l = atan2(micLeft_cmplx_input[IM(ix)], micLeft_cmplx_input[RE(ix)]);
		float phase_r = atan2(micRight_cmplx_input[IM(ix)], micRight_cmplx_input[RE(ix)]);
		float phase_f = atan2(micFront_cmplx_input[IM(ix)], micFront_cmplx_input[RE(ix)]);
		float phase_b = atan2(micBack_cmplx_input[IM(ix)], micBack_cmplx_input[RE(ix)]);
		//phases diff
		phase_dif_lr = modulo_cercle(phase_l - phase_r);
		phase_dif_fb = modulo_cercle(phase_f - phase_b);
		//angle
		angle = RAD2DEG(atan2(phase_dif_lr, phase_dif_fb));

		valid_phase = 1;
		freq_i = ix;
	} else {
		valid_phase = 0;
	}

}

/*
 * @brief		refine the sound direction measurement by computing the median value over NB_SAMPLES samples
 */
void refine_dir(void)
{
	if (dir_i == 0) {
		directions[dir_i] = angle;
		dir_i++;
	} else if (dir_i >= NB_SAMPLES) {
		//the middle of the sorted table is the median
		refined_angle = (directions[NB_SAMPLES/2] + directions[NB_SAMPLES/2-1])/2;
		refined_valid = 1;
		new_refined = 1;
		dir_i = 0;
	} else {
		//fill a sorted table with measurements
		for (uint16_t i = 0; i < dir_i; i++) {
			if (angle < directions[i]) {
				for (uint16_t j = dir_i; j > i; j--) {
					directions[j] = directions[j - 1];
				}
				directions[i] = angle;
				dir_i++;
				return;
			}
		}
		directions[dir_i] = angle;
		dir_i++;
	}
}

uint8_t get_refined_valid(void) {
	return refined_valid;
}

uint8_t get_new_refined(void) {
	return new_refined;
}

int16_t get_refined_angle(void)
{
	new_refined = 0;
	return refined_angle;
}

int16_t get_sound_angle(void)
{
	return angle;
}

uint16_t get_sound_freq(void)
{
	return 15.139 * freq_i + 5.4228;
}

uint8_t get_sound_valid(void)
{
	return valid_phase;
}

void audio_processing_start(void)
{
	mic_start(&processAudioData);
	finding_dir = 0;
	valid_phase = 0;
}

/*
 *	Callback called when the demodulation of the four microphones is done.
 *	We get 160 samples per mic every 10ms (16kHz)
 *
 *	params :
 *	int16_t *data		Buffer containing 4 times 160 samples. the samples are sorted by micro
 *				so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
 *	uint16_t num_samples	Tells how many data we get in total (should always be 640)
 */
void processAudioData(int16_t *data, uint16_t num_samples)
{

	/*
	 *
	 *	We get 160 samples per mic every 10ms
	 *	So we fill the samples buffers to reach
	 *	1024 samples, then we compute the FFTs.
	 *
	 */
	static volatile uint16_t fft_index = 0;
	static volatile uint16_t data_index = 0;
	//volatile uint16_t n_s_copy = num_samples;

	//we fill our tables
	while (1) {
		PROTEC(RE(fft_index), 2*FFT_SIZE, "audioRE");
		PROTEC(IM(fft_index), 2*FFT_SIZE, "audioIM");
		micRight_cmplx_input[RE(fft_index)] = data[MIC(data_index, MIC_RIGHT)];
		micLeft_cmplx_input[RE(fft_index)] = data[MIC(data_index, MIC_LEFT)];
		micBack_cmplx_input[RE(fft_index)] = data[MIC(data_index, MIC_BACK)];
		micFront_cmplx_input[RE(fft_index)] = data[MIC(data_index, MIC_FRONT)];

		micLeft_cmplx_input[IM(fft_index)] = 0;
		micRight_cmplx_input[IM(fft_index)] = 0;
		micFront_cmplx_input[IM(fft_index)] = 0;
		micBack_cmplx_input[IM(fft_index)] = 0;

		fft_index++;
		data_index++;
		volatile uint8_t need_to_break = 0;
		if (fft_index >= FFT_SIZE) {
			fft_index = 0;
			need_to_break = 1;
		}
		if (4 * data_index >= num_samples) {
			//all the data was put in the tables->return
			data_index = 0;
			return;
		}
		if (need_to_break) {
			//the tables are full->break and compute FFTs
			break;
		}

	}

	//compute FFT and magnitude
	doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
	doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
	doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
	doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

	arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
	arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
	arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
	arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

	//find sound direction
	detect_phase();
	if (valid_phase) {
		refine_dir();
		no_phase_c = 0;
	} else {
		no_phase_c++;
		if (no_phase_c > NO_PHASE_T) {
			no_phase_c = 0;
			dir_i = 0;
			refined_valid = 0;
			new_refined = 0;
		}
	}

#ifdef SEND_AUDIO
	if (valid_phase) {
		//chprintf((BaseSequentialStream *) &SD3, "LR: %4f, FB %4f, F:%03d\n", phase_dif_lr, phase_dif_fb, freq_i);
		chSequentialStreamWrite((BaseSequentialStream * ) &SD3, (uint8_t* )"START", 5);
		chSequentialStreamWrite((BaseSequentialStream * ) &SD3, (uint8_t* ) &phase_dif_lr, sizeof(float));
		chSequentialStreamWrite((BaseSequentialStream * ) &SD3, (uint8_t* ) &phase_dif_fb, sizeof(float));
		chSequentialStreamWrite((BaseSequentialStream * ) &SD3, (uint8_t* ) &angle, sizeof(int16_t));
	}
#endif

}

float* get_audio_buffer_ptr(BUFFER_NAME_t name)
{
	if (name == LEFT_CMPLX_INPUT) {
		return micLeft_cmplx_input;
	} else if (name == RIGHT_CMPLX_INPUT) {
		return micRight_cmplx_input;
	} else if (name == FRONT_CMPLX_INPUT) {
		return micFront_cmplx_input;
	} else if (name == BACK_CMPLX_INPUT) {
		return micBack_cmplx_input;
	} else if (name == LEFT_OUTPUT) {
		return micLeft_output;
	} else if (name == RIGHT_OUTPUT) {
		return micRight_output;
	} else if (name == FRONT_OUTPUT) {
		return micFront_output;
	} else if (name == BACK_OUTPUT) {
		return micBack_output;
	} else {
		return NULL;
	}
}
