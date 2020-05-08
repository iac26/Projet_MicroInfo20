#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H



/*
 * @brief		returns the state of visibility of the searched pattern
 *
 * @return		1: pattern visible, 0: pattern not visible
 */
uint8_t get_pattern_visible(void);

/*
 * @brief		returns the center of the detected pattern
 *
 * @return		center of the pattern in pixels from the side of the sensor
 */
uint16_t get_pattern_center(void);

/*
 * @brief		returns the center of image
 *
 * @return		center of the image in pixels
 */
uint16_t get_image_center(void);

/*
 * @brief		returns the width of the detected pattern
 *
 * @return		width of the pattern in pixels
 */
uint16_t get_pattern_width(void);

/*
 * @brief		start the image processing module
 */
void image_processing_start(void);

#endif
