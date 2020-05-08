#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#include <leds.h>



//debug macro to search for memory leaks
#define PROTEC(a, b, c)	if(!((a) < (b))) while(1) set_front_led(1)
//#define PROTEC(a, b, c)	if(!((a) < (b))) chSysHalt(c)


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
