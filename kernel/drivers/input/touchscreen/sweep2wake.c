/*
 * drivers/input/touchscreen/sweep2wake.c
 *
 *
 * Copyright (c) 2012, Dennis Rassmann <showp1984@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * History:
 *	Added sysfs adjustments for different sweep styles
 * 		by paul reioux (aka Faux123) <reioux@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/input/sweep2wake.h>
#include <linux/proxy_sensor.h>
#include <linux/ktime.h>
#include <asm-generic/cputime.h>

/* Tuneables */
#define DEBUG                   0
#define DEFAULT_S2W_Y_LIMIT             950
#define DEFAULT_S2W_X_MAX               530
#define DEFAULT_S2W_X_B1                180
#define DEFAULT_S2W_X_B2                360
#define DEFAULT_S2W_X_FINAL             50
#define DEFAULT_S2W_PWRKEY_DUR          60

/* external function from the ts driver */
bool is_single_touch(void);

/* Resources */
int pocket_detect = 1;
int menu2wake = 2;
int sweep2wake = 2;
int doubletap2wake = 1;
int is_dt2w = 0;
int s2w_st_flag = 0;
bool scr_suspended = false;
static struct input_dev * sweep2wake_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);

// 20150623 jwchen119 - Add new parameter
static int touch_nr = 0, x_pre = 0, y_pre = 0, x_pre2 = 0, y_pre2 =0;
static cputime64_t tap_time_pre = 0, tap_time_pre2 = 0;

static int s2w_start_posn = DEFAULT_S2W_X_B1;
static int s2w_mid_posn = DEFAULT_S2W_X_B2;
static int s2w_end_posn = (DEFAULT_S2W_X_MAX - DEFAULT_S2W_X_FINAL);
// 20150621 jwchen119 - num = 1010 is in kmesg for menu key y coords
static int s2w_height_adjust = 0;
static int s2w_threshold = DEFAULT_S2W_X_FINAL;

static int s2w_swap_coord = 0;

int tripon = 0;
int tripoff = 0;
unsigned long triptime = 0;
#define S2W_TIMEOUT 75
// 20150621 jwchen119 - Due to new template, time format is different
#define DT2W_TIMEOUT_MAX 600
#define DT2W_TIMEOUT_MIN 80
#define DT2W_DELTA 80
// 20150624 jwchen119 - Add M2W timeout
#define M2W_TIMEOUT_MAX 250
#define M2W_TIMEOUT_MIN 20

//#ifdef CONFIG_CMDLINE_OPTIONS
/* Read cmdline for s2w */
static int __init read_s2w_cmdline(char *s2w)
{
	if (strcmp(s2w, "3") == 0) {
//		printk(KERN_INFO "[cmdline_s2w]: Sweep2Sleep enabled. | s2w='%s'", s2w);
		sweep2wake = 3;
	} else if (strcmp(s2w, "2") == 0) {
//		printk(KERN_INFO "[cmdline_s2w]: Sweep2Wake/Sweep2Sleep enabled. | s2w='%s'", s2w);
		sweep2wake = 2;
	} else if (strcmp(s2w, "1") == 0) {
//		printk(KERN_INFO "[cmdline_s2w]: Sweep2Wake enabled. | s2w='%s'", s2w);
		sweep2wake = 1;
	} else if (strcmp(s2w, "0") == 0) {
//		printk(KERN_INFO "[cmdline_s2w]: Sweep2Wake disabled. | s2w='%s'", s2w);
		sweep2wake = 0;
	}else {
//		printk(KERN_INFO "[cmdline_s2w]: No valid input found. Sweep2Wake disabled. | s2w='%s'", s2w);
		sweep2wake = 0;
	}
	return 1;
}
__setup("s2w=", read_s2w_cmdline);
//#endif

/* PowerKey setter */
void sweep2wake_setdev(struct input_dev * input_device) {
	sweep2wake_pwrdev = input_device;
// printk("[SWEEP2WAKE]: power button cached\n");
	return;
}
EXPORT_SYMBOL(sweep2wake_setdev);

static void reset_sweep2wake(void)
{
printk("[SWEEP2WAKE]: ressetting in s2w\n");
//	pr_info("[sweep2wake]: line : %d | func : %s\n", __LINE__, __func__);
	//reset sweep2wake
	tripoff = 0;
	tripon = 0;
	triptime = 0;

	//reset doubletap2wake and menu2wake
	touch_nr = 0;
	tap_time_pre = 0;
	x_pre = 0;
	y_pre = 0;
	tap_time_pre2 = 0;
	x_pre2 = 0;
	y_pre2 = 0;
	is_dt2w = 0;
    return;
}

/* PowerKey work func */
static void sweep2wake_presspwr(struct work_struct * sweep2wake_presspwr_work) {
	if (!mutex_trylock(&pwrkeyworklock))
		return;
//	printk("[SWEEP2WAKE]: pressing power\n");
	reset_sweep2wake();
	input_event(sweep2wake_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(sweep2wake_pwrdev, EV_SYN, 0, 0);
	msleep(DEFAULT_S2W_PWRKEY_DUR);
	input_event(sweep2wake_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(sweep2wake_pwrdev, EV_SYN, 0, 0);
	msleep(DEFAULT_S2W_PWRKEY_DUR);
	mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(sweep2wake_presspwr_work, sweep2wake_presspwr);

/* PowerKey trigger */
void sweep2wake_pwrtrigger(void) {
	int pocket_mode = 1;
	if (scr_suspended == true && pocket_detect == 1) {
		pocket_mode = pocket_detection_check();
//		printk("[SWEEP2WAKE]: inside pocket mode check \n");
	}
	printk("[SWEEP2WAKE]: pocket mode %d\n" ,pocket_mode);
	if (pocket_mode == 1 || pocket_detect == 0) {
//		printk("[SWEEP2WAKE]: power triggered\n");
		schedule_work(&sweep2wake_presspwr_work);
	}
	return;
}

bool is_single_touch(void)
{
	/*int i = 0, cnt = 0;
	for( i= 0; i<MAX_NUM_FINGER; i++ ) {
		if ((!fingerInfo[i].status) ||
				(fingerInfo[i].status == TOUCH_EVENT_RELEASE))
		continue;
		else cnt++;
	}*/
printk("[SWEEP2WAKE]: inside single touch\n");
	if (s2w_st_flag == 1)
	return true;
	else
	return false;
}

/* Sweep2wake main function */
void detect_sweep2wake(int sweep_coord, int sweep_height, unsigned long time, int i)
{
	int swap_temp1, swap_temp2;

        //int prev_coord = 0, next_coord = 0;
        bool single_touch = is_single_touch();
#if DEBUG
        pr_info("[sweep2wake]: sweep_coord,sweep_height(%4d,%4d) single:%s\n",
                sweep_coord, sweep_height, (single_touch) ? "true" : "false");
#endif

	if (s2w_swap_coord == 1) {
		//swap the coordinate system
        pr_info("line : %d | func : %s\n", __LINE__, __func__);
		swap_temp1 = sweep_coord;
		swap_temp2 = sweep_height;

		sweep_height = swap_temp1;
		sweep_coord = swap_temp2;
	}
    //left->right
	if ((single_touch) && (scr_suspended == true) && (sweep2wake > 0) && (sweep_height > s2w_height_adjust)) {
//		printk("[sweep2wake]:left to right x,y(%d,%d) jiffies:%lu\n", sweep_coord, sweep_height, time);
	if (sweep_coord < 50) {
		tripon = 1;
		triptime = time;
	} else if (tripon == 1 && (sweep_coord > 50 && sweep_coord < s2w_start_posn) && time - triptime < 25) {
		tripon = 2;
	} else if (tripon == 2 && (sweep_coord > s2w_start_posn && sweep_coord < s2w_mid_posn) && time - triptime < 50) {
		tripon = 3;
	} else if (tripon == 3 && (sweep_coord > s2w_mid_posn && sweep_coord < s2w_end_posn) && time - triptime < S2W_TIMEOUT) {
//		printk(KERN_INFO "[sweep2wake]: ON");
		sweep2wake_pwrtrigger();
	}
    // 20150618 jwchen119 - Lower the sleep sensitive
    //right->left
	} else if ((scr_suspended == false) && (sweep2wake > 1) && sweep_height > s2w_height_adjust) {
//		printk("[sweep2wake]:right to left x,y(%d,%d) jiffies:%lu\n", sweep_coord, sweep_height, time);
	if (sweep_coord >= s2w_end_posn) {
		tripoff = 1;
		triptime = time;
	} else if (tripoff == 1 && (sweep_coord < s2w_end_posn && sweep_coord > s2w_mid_posn) && time - triptime < 15) {
		tripoff = 2;
	} else if (tripoff == 2 && (sweep_coord < s2w_mid_posn && sweep_coord > s2w_start_posn) && time - triptime < 25) {
		tripoff = 3;
	} else if (tripoff == 3 && (sweep_coord < s2w_start_posn && sweep_coord < 50) && (time - triptime < 40)) {
//		printk(KERN_INFO "[sweep2wake]: OFF");
		sweep2wake_pwrtrigger();
	}
     }
}

// 20150621 jwchen119 - New function to detect the tap
static void new_touch(int x, int y) {
	tap_time_pre = ktime_to_ms(ktime_get());
	x_pre = x;
	y_pre = y;
	touch_nr++;
}

static void new_touch2(int x, int y) {
	tap_time_pre2 = ktime_to_ms(ktime_get());
	x_pre2 = x;
	y_pre2 = y;
	touch_nr++;
}

void doubletap2wake_func(int x, int y, unsigned long time)
{
// 20150621 jwchen119 - Redesign M2W/M2S template and fix sometimes triple touch to wake
// 20150623 jwchen119 - Fix random wake/sleep when rapidly tap on only two of the menu key
	if (scr_suspended == true && doubletap2wake > 0) {
		if (y > s2w_height_adjust && y <= 1010) {
			if (touch_nr == 0) {
				new_touch(x, y);
			} else if (touch_nr == 1) {
				if ((abs(x-x_pre) < DT2W_DELTA)
					&& (abs(y-y_pre) < DT2W_DELTA)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre) > DT2W_TIMEOUT_MIN)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre) < DT2W_TIMEOUT_MAX))
					{
					touch_nr++;
					is_dt2w = 1;
				} else if ((menu2wake > 0)) {
					if (((x-x_pre) == 210)
						&& (y == 1010)
						&& (y_pre == 1010)
						&& ((ktime_to_ms(ktime_get())-tap_time_pre) > M2W_TIMEOUT_MIN)
						&& ((ktime_to_ms(ktime_get())-tap_time_pre) < M2W_TIMEOUT_MAX))
						{
						new_touch2(x, y);
					} else {
						reset_sweep2wake();
						new_touch(x, y);
					}
				} else {
					reset_sweep2wake();
					new_touch(x, y);
				}
			} else if (touch_nr == 2) {
				if (((x-x_pre2) == 210)
					&& (y == 1010)
					&& (y_pre2 == 1010)
					&& (menu2wake > 0)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre2) > M2W_TIMEOUT_MIN)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre2) < M2W_TIMEOUT_MAX))
					{
					touch_nr++;
				} else {
					reset_sweep2wake();
					new_touch(x, y);
				}
			} else {
				reset_sweep2wake();
				new_touch(x, y);
			}
			if ((touch_nr > 1)) {
				if ((is_dt2w == 1) && (touch_nr < 3)) {
//					printk("[DT2W]: OFF->ON\n");
					sweep2wake_pwrtrigger();
				} else if ((touch_nr > 2) && (menu2wake > 0)) {
//					printk("[M2W]: OFF->ON\n");
					sweep2wake_pwrtrigger();
				}
			}
		}
	// Enabled only menu2wake function
	} else if (scr_suspended == true && doubletap2wake == 0 && menu2wake > 0) {
		if (y > s2w_height_adjust && y <= 1010) {
			if (touch_nr == 0) {
				new_touch(x, y);
			} else if (touch_nr == 1) {
				if (((x-x_pre) == 210)
					&& (y == 1010)
					&& (y_pre == 1010)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre) > M2W_TIMEOUT_MIN)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre) < M2W_TIMEOUT_MAX))
					{
					new_touch2(x, y);
				} else {
					reset_sweep2wake();
					new_touch(x, y);
				}
			} else if (touch_nr == 2) {
				if (((x-x_pre2) == 210)
					&& (y == 1010)
					&& (y_pre2 == 1010)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre2) > M2W_TIMEOUT_MIN)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre2) < M2W_TIMEOUT_MAX))
					{
					touch_nr++;
				} else {
					reset_sweep2wake();
					new_touch(x, y);
				}
			} else {
				reset_sweep2wake();
				new_touch(x, y);
			}
			if ((touch_nr > 2)) {
//				printk("[M2W]: OFF->ON\n");
				sweep2wake_pwrtrigger();
			}
		}
	// Menu2Sleep function
	} else if (scr_suspended == false && menu2wake > 1) {
		if (y > s2w_height_adjust && y <= 1010) {
			if (touch_nr == 0) {
				new_touch(x, y);
			} else if (touch_nr == 1) {
				if (((x-x_pre) == -210)
					&& (y == 1010)
					&& (y_pre == 1010)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre) > M2W_TIMEOUT_MIN)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre) < M2W_TIMEOUT_MAX))
					{
					new_touch2(x, y);
				} else {
					reset_sweep2wake();
					new_touch(x, y);
				}
			} else if (touch_nr == 2) {
				if (((x-x_pre2) == -210)
					&& (y == 1010)
					&& (y_pre2 == 1010)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre2) > M2W_TIMEOUT_MIN)
					&& ((ktime_to_ms(ktime_get())-tap_time_pre2) < M2W_TIMEOUT_MAX))
					{
					touch_nr++;
				} else {
					reset_sweep2wake();
					new_touch(x, y);
				}
			} else {
				reset_sweep2wake();
				new_touch(x, y);
			}
			if ((touch_nr > 2)) {
//				printk("[M2W]: ON->OFF\n");
				sweep2wake_pwrtrigger();
			}
		}
	}
        return;
}

/********************* SYSFS INTERFACE ***********************/
static ssize_t s2w_start_posn_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", s2w_start_posn);
}

static ssize_t s2w_start_posn_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		s2w_start_posn = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static ssize_t s2w_mid_posn_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", s2w_mid_posn);
}

static ssize_t s2w_mid_posn_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		s2w_mid_posn = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static ssize_t s2w_end_posn_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", s2w_end_posn);
}

static ssize_t s2w_end_posn_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		s2w_end_posn = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static ssize_t s2w_height_adjust_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", s2w_height_adjust);
}

static ssize_t s2w_height_adjust_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		s2w_height_adjust = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static ssize_t s2w_threshold_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", s2w_threshold);
}

static ssize_t s2w_threshold_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		s2w_threshold = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static ssize_t s2w_swap_coord_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", s2w_swap_coord);
}

static ssize_t s2w_swap_coord_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		s2w_swap_coord = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static ssize_t sweep2wake_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", sweep2wake);
}

static ssize_t sweep2wake_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		sweep2wake = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static ssize_t pocket_detect_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", pocket_detect);
}

static ssize_t pocket_detect_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		pocket_detect = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static ssize_t menu2wake_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", menu2wake);
}

static ssize_t menu2wake_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		menu2wake = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static ssize_t doubletap2wake_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", doubletap2wake);
}

static ssize_t doubletap2wake_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;
	if(sscanf(buf, "%i\n", &data) == 1)
		doubletap2wake = data;
	else
		pr_info("%s: unknown input!\n", __FUNCTION__);
	return count;
}

static struct kobj_attribute s2w_start_posn_attribute =
	__ATTR(s2w_start_posn,
		0666,
		s2w_start_posn_show,
		s2w_start_posn_store);

static struct kobj_attribute s2w_mid_posn_attribute =
	__ATTR(s2w_mid_posn,
		0666,
		s2w_mid_posn_show,
		s2w_mid_posn_store);

static struct kobj_attribute s2w_end_posn_attribute =
	__ATTR(s2w_end_posn,
		0666,
		s2w_end_posn_show,
		s2w_end_posn_store);

static struct kobj_attribute s2w_height_adjust_attribute =
	__ATTR(s2w_height_adjust,
		0666,
		s2w_height_adjust_show,
		s2w_height_adjust_store);

static struct kobj_attribute s2w_threshold_attribute =
	__ATTR(s2w_threshold,
		0666,
		s2w_threshold_show,
		s2w_threshold_store);

static struct kobj_attribute s2w_swap_coord_attribute =
	__ATTR(s2w_swap_coord,
		0666,
		s2w_swap_coord_show,
		s2w_swap_coord_store);

static struct kobj_attribute sweep2wake_attribute =
	__ATTR(sweep2wake,
		0666,
		sweep2wake_show,
		sweep2wake_store);

static struct kobj_attribute pocket_detect_attribute =
	__ATTR(pocket_detect,
		0666,
		pocket_detect_show,
		pocket_detect_store);

static struct kobj_attribute menu2wake_attribute =
	__ATTR(menu2wake,
		0666,
		menu2wake_show,
		menu2wake_store);

static struct kobj_attribute doubletap2wake_attribute =
	__ATTR(doubletap2wake,
		0666,
		doubletap2wake_show,
		doubletap2wake_store);

static struct attribute *s2w_parameters_attrs[] =
	{
		&s2w_start_posn_attribute.attr,
		&s2w_mid_posn_attribute.attr,
		&s2w_end_posn_attribute.attr,
		&s2w_threshold_attribute.attr,
		&s2w_swap_coord_attribute.attr,
		&sweep2wake_attribute.attr,
		&pocket_detect_attribute.attr,
		&menu2wake_attribute.attr,
		&doubletap2wake_attribute.attr,
		&s2w_height_adjust_attribute.attr,
		NULL,
	};

static struct attribute_group s2w_parameters_attr_group =
	{
		.attrs = s2w_parameters_attrs,
	};

static struct kobject *s2w_parameters_kobj;

/*
 * INIT / EXIT stuff below here
 */

static int __init sweep2wake_init(void)
{
	int sysfs_result;

	s2w_parameters_kobj = kobject_create_and_add("android_touch", NULL);
	if (!s2w_parameters_kobj) {
		pr_err("%s kobject create failed!\n", __FUNCTION__);
		return -ENOMEM;
        }

	sysfs_result = sysfs_create_group(s2w_parameters_kobj, &s2w_parameters_attr_group);

        if (sysfs_result) {
		pr_info("%s sysfs create failed!\n", __FUNCTION__);
		kobject_put(s2w_parameters_kobj);
	}

	pr_info("[sweep2wake]: %s done\n", __func__);
	return sysfs_result;
}

static void __exit sweep2wake_exit(void)
{
	return;
}

module_init(sweep2wake_init);
module_exit(sweep2wake_exit);

MODULE_DESCRIPTION("Sweep2wake");
MODULE_LICENSE("GPLv2");
