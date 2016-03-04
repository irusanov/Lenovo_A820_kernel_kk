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
#include <linux/alsps_sensor.h>

/* Tuneables */
#define DEBUG                   		0
#define DEFAULT_S2W_Y_LIMIT             950
#define DEFAULT_S2W_X_MAX               530
#define DEFAULT_S2W_X_B1                50
#define DEFAULT_S2W_X_B2                150
#define DEFAULT_S2W_X_FINAL             50
#define DEFAULT_S2W_PWRKEY_DUR          60

/* external function from the ts driver */
bool is_single_touch(void);

/* Resources */
int pocket_detect = 1;
int sweep2wake = 0;
int s2w_st_flag = 0;
int doubletap2wake = 0;
int dt2w_switch_temp = 1;
int dt2w_changed = 0;
bool scr_suspended = false, exec_count = true;
bool scr_on_touch = false, barrier[2] = {false, false};
static struct input_dev * sweep2wake_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);

static int s2w_start_posn = DEFAULT_S2W_X_B1;
static int s2w_mid_posn = DEFAULT_S2W_X_B2;
static int s2w_end_posn = (DEFAULT_S2W_X_MAX - DEFAULT_S2W_X_FINAL);
static int s2w_height_adjust = 854;
static int s2w_threshold = DEFAULT_S2W_X_FINAL;
//static int s2w_max_posn = DEFAULT_S2W_X_MAX;

static int s2w_swap_coord = 0;

int tripon = 0;
int tripoff = 0;
unsigned long triptime = 0;
unsigned long initial_time = 0;
unsigned long dt2w_time[2] = {0, 0};
unsigned int dt2w_x[2] = {0, 0};
unsigned int dt2w_y[2] = {0, 0};
int status[2] = {0,0};
#define S2W_TIMEOUT 75
#define DT2W_TIMEOUT_MAX 120
#define DT2W_TIMEOUT_MIN 12
#define DT2W_DELTA 75

//#ifdef CONFIG_CMDLINE_OPTIONS
/* Read cmdline for s2w */
static int __init read_s2w_cmdline(char *s2w)
{
	if (strcmp(s2w, "2") == 0) {
		printk(KERN_INFO "[cmdline_s2w]: Sweep2Wake/Sweep2Sleep enabled. | s2w='%s'", s2w);
		sweep2wake = 2;
	} else if (strcmp(s2w, "1") == 0) {
		printk(KERN_INFO "[cmdline_s2w]: Sweep2Wake enabled. | s2w='%s'", s2w);
		sweep2wake = 1;
	} else if (strcmp(s2w, "0") == 0) {
		printk(KERN_INFO "[cmdline_s2w]: Sweep2Wake disabled. | s2w='%s'", s2w);
		sweep2wake = 0;
	}else {
		printk(KERN_INFO "[cmdline_s2w]: No valid input found. Sweep2Wake disabled. | s2w='%s'", s2w);
		sweep2wake = 0;
	}
	return 1;
}
__setup("s2w=", read_s2w_cmdline);
//#endif

/* PowerKey setter */
void sweep2wake_setdev(struct input_dev * input_device) {
	sweep2wake_pwrdev = input_device;
printk("[SWEEP2WAKE]: power button cached\n");
	return;
}
EXPORT_SYMBOL(sweep2wake_setdev);

static void reset_sweep2wake(void)
{
printk("[SWEEP2WAKE]: ressetting in s2w\n");
    pr_info("[sweep2wake]: line : %d | func : %s\n", __LINE__, __func__);
    //reset sweep2wake
    tripoff = 0;
    tripon = 0;
    triptime = 0;

	//reset doubletap2wake
	dt2w_time[0] = 0;
	dt2w_x[0] = 0;
	dt2w_y[0] = 0;
	dt2w_time[1] = 0;
	dt2w_x[1] = 0;
	dt2w_y[1] = 0;
	initial_time = 0;
    return;
}

/* PowerKey work func */
static void sweep2wake_presspwr(struct work_struct * sweep2wake_presspwr_work) {
	if (!mutex_trylock(&pwrkeyworklock))
		return;
	
	printk("[SWEEP2WAKE]: pressing power\n");
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
	if (pocket_detection_check() == 1 && pocket_detect == 1) {
		printk("[SWEEP2WAKE]: power triggered\n");
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
    if ((scr_suspended == true) && (sweep2wake > 0) && sweep_height > s2w_height_adjust) {
        printk("[sweep2wake]:left to right x,y(%d,%d) jiffies:%lu\n", sweep_coord, sweep_height, time);
        if (sweep_coord < 100) {
        tripon = 1;
        triptime = time;
        } else if (tripon == 1 && sweep_coord > 100 && time - triptime < 25) {
            tripon = 2;
        } else if (tripon == 2 && sweep_coord > 200 && time - triptime < 50) {
            tripon = 3;
        } else if (tripon == 3 && sweep_coord > 300 && time - triptime < S2W_TIMEOUT) {
            printk(KERN_INFO "[sweep2wake]: ON");
            sweep2wake_pwrtrigger();
        }
    //right->left
    } else if ((scr_suspended == false) && (sweep2wake > 1) && sweep_height > s2w_height_adjust) {
        printk("[sweep2wake]:right to left x,y(%d,%d) jiffies:%lu\n", sweep_coord, sweep_height, time);
        if (sweep_coord > 400) {
            tripoff = 1;
            triptime = time;
        } else if (tripoff == 1 && sweep_coord < 400 && time - triptime < 25) {
            tripoff = 2;
        } else if (tripoff == 2 && sweep_coord < 300 && time - triptime < 50) {
            tripoff = 3;
        } else if (tripoff == 3 && sweep_coord < 200 && (time - triptime < S2W_TIMEOUT)) {
            printk(KERN_INFO "[sweep2wake]: OFF");
            sweep2wake_pwrtrigger();
        }
    }
}

void doubletap2wake_func(int x, int y, unsigned long time)
{

	int delta_x = 0;
	int delta_y = 0;

	printk("[dt2wake]: x,y(%d,%d) jiffies:%lu\n", x, y, time);

        dt2w_time[1] = dt2w_time[0];
        dt2w_time[0] = time;

	if (!initial_time)
		initial_time = time;	

	if (time - initial_time > 800)
		reset_sweep2wake();
printk("[SWEEP2WAKE]: d2w reset\n");
	
	if ((dt2w_time[0] - dt2w_time[1]) < 10)
		return;
printk("[SWEEP2WAKE]: checking d2w\n");
	dt2w_x[1] = dt2w_x[0];
    dt2w_x[0] = x;
	dt2w_y[1] = dt2w_y[0];
    dt2w_y[0] = y;

	delta_x = (dt2w_x[0]-dt2w_x[1]);
	delta_y = (dt2w_y[0]-dt2w_y[1]);

        if (scr_suspended && doubletap2wake > 0) {
printk("[SWEEP2WAKE]: y = %d, timedelta = %lu, deltax= %d, deltay = %d\n",y,(dt2w_time[0] - initial_time),delta_x,delta_y);
		if (y > 50 && y < 1200
			&& ((dt2w_time[0] - initial_time) > DT2W_TIMEOUT_MIN)
			&& ((dt2w_time[0] - initial_time) < DT2W_TIMEOUT_MAX)
			&& (abs(delta_x) < DT2W_DELTA)
			&& (abs(delta_y) < DT2W_DELTA)
			) {
                printk("[DT2W]: OFF->ON\n");
                sweep2wake_pwrtrigger();
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

