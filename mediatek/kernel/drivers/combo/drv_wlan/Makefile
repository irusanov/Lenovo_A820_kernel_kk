ifeq ($(MTK_WLAN_SUPPORT), yes)

SUPPORT_MT6628 := no

$(warning check_mt6628)
SUPPORT_MT6628 := $(if $(filter $(MTK_WLAN_CHIP), MT6628),yes,no)
$(warning check_mt6628_end)

$(warning $(SUPPORT_MT6628));

ifeq ($(SUPPORT_MT6628), yes)
    obj-$(CONFIG_MTK_COMBO_WIFI) += mt6628/
    $(warning include mt6628)
endif    

endif


