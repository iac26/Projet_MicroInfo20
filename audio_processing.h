#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT,
} BUFFER_NAME_t;

/*
 * @brief		Callback invoked by the microphone module when an audio ample is ready
 *
 * @param data		pointer to the start of the data samples
 * @param num_samples	number of samples in the data array
 */
void processAudioData(int16_t *data, uint16_t num_samples);

/*
 * @brief		returns the state of validity of the refined sound direction data
 *
 * @return		1: valid, 0: not valid
 */
uint8_t get_refined_valid(void);
/*
 * @brief		returns if a new measurement is avaiable since the last call to get_refined_angle
 *
 * @return		1: the data is new, 0: the data is not new
 */
uint8_t get_new_refined(void);

/*
 * @brief		returns the angle of the sound refined by median value of NB_SAMPLES measurements
 *
 * @return		angle of the sound between -180 and 180
 */
int16_t get_refined_angle(void);

/*
 * @brief		returns the raw angle of the sound
 *
 * @return		angle between -180 and 180
 */
int16_t get_sound_angle(void);

/*
 * @brief		returns the frequency of the sound from which the angle was calculated
 *
 * @return		frequency in Hz
 */
uint16_t get_sound_freq(void);

/*
 * @brief		returns the state of validity of the raw angle measurement
 *
 * @return		1: valid, 0: not valid
 */
uint8_t get_sound_valid(void);

/*
 * @brief		returns the pointer to the desired audio buffer
 *
 * @param name		name of the desired audio buffer see: BUFFER_NAME_t
 *
 * @return		pointer to the desired buffer
 */
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

/*
 * @brief		start the audio processing module
 */
void audio_processing_start(void);

#endif /* AUDIO_PROCESSING_H */
