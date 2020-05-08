#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <arm_math.h>
#include <spi_comm.h>
#include <camera/dcmi_camera.h>
#include <camera/po8030.h>
#include <motors.h>
#include <leds.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>

#include <audio_processing.h>
#include <proximity_processing.h>
#include <pickup_detector.h>
#include <image_processing.h>
#include <navigation.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
 * @brief		initialise the serial communication with the computer
 */
static void serial_start(void)
{
	static SerialConfig ser_cfg = { 115200, 0, 0, 0, };

	sdStart(&SD3, &ser_cfg); // UART3.
}

//debug tools

/*
 * @brief		main function
 *
 */
int main(void)
{

	halInit();
	chSysInit();
	mpu_init();

	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//periferals init
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();
	dcmi_start();
	po8030_start();
	motors_init();
	proximity_start();
	imu_start();
	spi_comm_start();
	serial_start();
	audio_processing_start();

	chprintf((BaseSequentialStream *) &SD3, "STARTUP\n");

	//modules init
	proximity_processing_start();

	pickup_detector_start();

	image_processing_start();

	navigation_start();

	//infinite loop
	while (1) {

		chThdSleepMilliseconds(1000);
	}

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}
