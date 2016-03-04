#ifndef __LINUX_PL_SENSOR_H
#define __LINUX_PL_SENSOR_H

/* Hooks in APDS9930 */

/**
 * TODO: detect ps sensor value when power button preseed to avoid accidental power_on while in pocket
 */
//int power_key_check_in_pocket(void);
int pocket_detection_check(void);

#endif
