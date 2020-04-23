#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//MAcros
#define NB_SAMPLES 	5
#define PHASE_MAX 	3
#define CONTROL_THRESH	5
#define DET_THRESH	30000
#define RE(i)		(2*i)
#define IM(i) 		(2*i+1)
#define MIC(i, m)	(4*i+m)

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

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

static float micLeft_raw[FFT_SIZE];
static float micRight_raw[FFT_SIZE];
static float micFront_raw[FFT_SIZE];
static float micBack_raw[FFT_SIZE];

static float phase_dif;
static uint8_t valid_phase;

static float clean_phase;
static uint8_t phase_ready;
static uint16_t phase_count;
static uint16_t phase_control;
static float phase_samples[NB_SAMPLES];


float modulo_cercle(float angle) {

	while (angle > M_PI) {
		angle -= 2*M_PI;
	}
	while (angle <= -M_PI) {
		angle += 2*M_PI;
	}
	return angle;
}


void detect_phase(void)
{
	//detect max index
	uint16_t ix = 0;
	float max = 0;
	for (uint16_t i = 0; i < FFT_SIZE; i++) {
		if (micLeft_output[i] > max) {
			ix = i;
			max = micLeft_output[i];
		}
	}
	if (max > DET_THRESH) {
		float phase_1 = atan2(micLeft_cmplx_input[IM(ix)], micLeft_cmplx_input[RE(ix)]);
		float phase_2 = atan2(micRight_cmplx_input[IM(ix)], micRight_cmplx_input[RE(ix)]);
		phase_dif = modulo_cercle(phase_1 - phase_2);

		valid_phase = 1;
	} else {
		valid_phase = 0;
	}


}

float get_clean_phase(void) {
	return clean_phase;
}

uint8_t is_phase_ready(void) {
	return phase_ready;
}

float* get_phases(void) {
	return phase_samples;
}

int cleanup_phase(void) {
	phase_control++;
	if (phase_dif < PHASE_MAX && phase_dif > -PHASE_MAX && valid_phase) {
		phase_samples[phase_count] = phase_dif;
		phase_count++;
		phase_control = 0;
	}
	if (phase_count == NB_SAMPLES) {
		float sum = 0;
		for (uint16_t i = 0; i < NB_SAMPLES; i++) {
			sum += phase_samples[i];
		}
		phase_dif = sum / NB_SAMPLES;
		phase_count = 0;
		phase_ready = 1;
		clean_phase = phase_dif;
		return 1;
	} else {
		return 0;
	}
	if (phase_control > CONTROL_THRESH) {
		phase_ready = 0;
	}

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
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/
	static volatile uint16_t fft_index = 0;
	static volatile uint16_t data_index = 0;

	while (1){
		micRight_cmplx_input[RE(fft_index)] = data[MIC(data_index, MIC_RIGHT)];
		micLeft_cmplx_input[RE(fft_index)] = data[MIC(data_index, MIC_LEFT)];
		micBack_cmplx_input[RE(fft_index)] = data[MIC(data_index, MIC_BACK)];
		micFront_cmplx_input[RE(fft_index)] = data[MIC(data_index, MIC_FRONT)];

		micRight_raw[fft_index] = data[4*data_index + MIC_RIGHT];
		micLeft_raw[fft_index] = data[4*data_index + MIC_LEFT];
		micBack_raw[fft_index] = data[4*data_index + MIC_BACK];
		micFront_raw[fft_index] = data[4*data_index + MIC_FRONT];

		micLeft_cmplx_input[IM(fft_index)] = 0;
		micRight_cmplx_input[IM(fft_index)] = 0;
		micFront_cmplx_input[IM(fft_index)] = 0;
		micBack_cmplx_input[IM(fft_index)] = 0;

		fft_index++;
		data_index++;
		if(4*data_index >= num_samples) {
			data_index = 0;
			return;
		}
		if(fft_index >= FFT_SIZE) {
			fft_index = 0;
			break;
		}
	}
	doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
	doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
	doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
	doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

	arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
	arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
	arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
	arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);


	detect_phase();
	if (cleanup_phase()) {
		chBSemSignal(&sendToComputer_sem);
	}

}

float get_phase(void)
{
	return phase_dif;
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else if (name == LEFT_RAW){
		return micLeft_raw;
	}
	else if (name == RIGHT_RAW){
		return micRight_raw;
	}
	else if (name == FRONT_RAW){
		return micFront_raw;
	}
	else if (name == BACK_RAW){
		return micBack_raw;
	}
	else{
		return NULL;
	}
}
