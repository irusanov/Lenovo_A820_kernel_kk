#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

/*********************************************************************
 * VARIABLES
 ********************************************************************/

// PROTON: GPU frequency control in MHz
int proton_gpu_frequency = 286;
module_param(proton_gpu_frequency, int, 0664);
MODULE_PARM_DESC(proton_gpu_frequency, "Sets the GPU frequency");
EXPORT_SYMBOL(proton_gpu_frequency);

// PROTON: GPU voltage control in mV
int proton_gpu_voltage = 1050;
module_param(proton_gpu_voltage, int, 0664);
MODULE_PARM_DESC(proton_gpu_voltage, "Sets the GPU voltage");
EXPORT_SYMBOL(proton_gpu_voltage);

// PROTON: GPU DVFS (1: enable, 0: disable)
int proton_gpu_dvfs = 0;
module_param(proton_gpu_dvfs, int, 0664);
MODULE_PARM_DESC(proton_gpu_dvfs, "Enables or disabled GPU DVFS *Dynamic voltage and frequency)");
EXPORT_SYMBOL(proton_gpu_dvfs);

/*********************************************************************
 * FUNCTION DEFINITIONS
 ********************************************************************/

static int __init proton_control_init(void)
{
    return 0;
}

arch_initcall(proton_control_init);

MODULE_AUTHOR("Ivan Rusanov, ivan.b.rusanov@gmail.com");
MODULE_DESCRIPTION("PROTON GPU Init Driver");
MODULE_LICENSE("GPL");
