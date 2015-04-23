
#include "mtk_sysfreq.h"
#include "mach/mt_gpufreq.h"
#include "mach/mt_clkmgr.h"
#include <mach/sync_write.h>

static bool mt_keep_freq_non_od_set = false;
extern int proton_gpu_frequency;
extern int proton_gpu_voltage;

#define MTK_GPU_DVFS 0

#if MTK_GPU_DVFS
static struct mt_gpufreq_info freqs_special_vrf18_2[] = {
    {GPU_DVFS_F3, 40, 100, GPU_POWER_VRF18_1_05V, 100},
    {GPU_DVFS_F5, 0,  40, GPU_POWER_VRF18_1_05V,  80},
};
static struct mt_gpufreq_info freqs_special2_vrf18_2[] = {
    {GPU_DVFS_F2, 60, 100, GPU_POWER_VRF18_1_05V, 100},
    {GPU_DVFS_F3, 30,  60, GPU_POWER_VRF18_1_05V,  90},
    {GPU_DVFS_F5, 0,  30, GPU_POWER_VRF18_1_05V,  75},
};
static struct mt_gpufreq_info freqs_special3_vrf18_2[] = {
    {GPU_DVFS_F1, 60, 100, GPU_POWER_VRF18_1_05V, 100},
    {GPU_DVFS_F3, 30,  60, GPU_POWER_VRF18_1_05V,  90},
    {GPU_DVFS_F5, 0,  30, GPU_POWER_VRF18_1_05V,  75},
};
#endif

void MtkInitSetFreqTbl(unsigned int tbltype)
{
    switch (tbltype)
    {
#if MTK_GPU_DVFS
    case TBLTYPE1:
//        printk("[GPU DVFS] register vrf18_2 special table ...\n");
        mt_gpufreq_register(freqs_special_vrf18_2, 2);
        break;
    case TBLTYPE2:
//        printk("[GPU DVFS] register vrf18_2 special2 table ...\n");
        mt_gpufreq_register(freqs_special2_vrf18_2, 3);
        break;
    case TBLTYPE3:
//        printk("[GPU DVFS] register vrf18_2 special3 table ...\n");
        mt_gpufreq_register(freqs_special3_vrf18_2, 3);
        break;
#endif
    default:
    case TBLTYPE0:
        mt_gpufreq_non_register();
        break;
    }
}


PVRSRV_ERROR MTKSetFreqInfo(unsigned int freq, unsigned int tbltype)
{
	
	static int voltage;
	static unsigned int pll;
	
	// Select GPU frequency (in MHz)
	switch (proton_gpu_frequency)
	{
		case 476:
    freq = GPU_DVFS_F1;
    tbltype = TBLTYPE3;
    		break;
    	case 403:
    freq = GPU_DVFS_F2;
    tbltype = TBLTYPE2;
    		break;
    	case 357:
    freq = GPU_DVFS_F3;
    tbltype = TBLTYPE1;
    		break;
    	case 312:
    freq = GPU_DVFS_F4;
    tbltype = TBLTYPE0;
    		break;
    	default:
    	case 286:
    freq = GPU_DVFS_F5;
    tbltype = TBLTYPE0;
    		break;
    	case 268:
    freq = GPU_DVFS_F7;
    tbltype = TBLTYPE0;
	}
	
	// Select GPU voltage (in mV)
	switch (proton_gpu_voltage)
	{
		case 1200:
			voltage = GPU_POWER_VRF18_1_20V;
			break;
		case 1175:
			voltage = GPU_POWER_VRF18_1_175V;
			break;
		case 1150:
			voltage = GPU_POWER_VRF18_1_15V;
			break;
		case 1125:
			voltage = GPU_POWER_VRF18_1_125V;
			break;
		case 1100:
			voltage = GPU_POWER_VRF18_1_10V;
			break;
		case 1075:
			voltage = GPU_POWER_VRF18_1_075V;
			break;
		default:
		case 1050:
			voltage = GPU_POWER_VRF18_1_05V;
			break;
	}


//#if defined(MTK_FREQ_OD_INIT)
    if (freq > GPU_DVFS_F7)
    {
        mt_gpufreq_set_initial(freq, GPU_POWER_VRF18_1_05V);
        mt65xx_reg_sync_writel((readl(CLK_CFG_8)&0xffcffff)|0x30000, CLK_CFG_8);
        
        switch (freq)
        {
        	case GPU_DVFS_F1: // 476MHz
        		pll = GPU_MMPLL_D3;
        		break;
        	case GPU_DVFS_F2: // 403MHz
        		pll = GPU_SYSPLL_D2;
        		break;
        	case GPU_DVFS_F3: // 357MHz
        		pll = GPU_MMPLL_D4;
        		break;
        	case GPU_DVFS_F4: // 312MHz
        		pll = GPU_UNIVPLL1_D2;
        		break;
        	default: // set in the default parameter initialization anyway
        	case GPU_DVFS_F5: // 286MHz
        		pll = GPU_MMPLL_D5;
        		break;
        }
		
		mt_gpufreq_keep_frequency_non_OD_init(pll, voltage);
    }
    else
//#endif
    {
        mt_gpufreq_set_initial(freq, GPU_POWER_VRF18_1_05V);
        mt_gpufreq_keep_frequency_non_OD_init(GPU_KEEP_FREQ_NON_OD_BYPASS, GPU_KEEP_VOLT_NON_OD_BYPASS);
    }
//        mt_gpufreq_keep_frequency_non_OD_init(GPU_KEEP_FREQ_NON_OD_BYPASS, GPU_KEEP_VOLT_NON_OD_BYPASS);

#if MTK_GPU_DVFS
    MtkInitSetFreqTbl(tbltype);
#else
	mt_gpufreq_non_register();
#endif

    return PVRSRV_OK;
}

void MtkSetKeepFreq(void)
{
    if (mt_gpufreq_keep_frequency_non_OD_get())
    {
        if (mt_keep_freq_non_od_set==false)
        {
            mt_gpufreq_keep_frequency_non_OD_set(1);
            mt_keep_freq_non_od_set=true;
        }
    }
    else
    {
        if (mt_keep_freq_non_od_set==true)
        {
            mt_gpufreq_keep_frequency_non_OD_set(0);
            mt_keep_freq_non_od_set=false;
        }
    }
}

