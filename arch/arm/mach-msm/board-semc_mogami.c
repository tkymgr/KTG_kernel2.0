/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Adapted for SEMC 2011 devices by Vassilis Tsogkas (tsogkas@ceid.upatras.gr)
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/msm_ssbi.h>
#include <linux/mfd/pmic8058.h>
#include <linux/leds.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/smsc911x.h>
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#include <linux/leds-pmic8058.h>
#include <linux/input/cy8c_ts.h>
#include <linux/msm_adc.h>
#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
#include <linux/uio_driver.h>
#include <linux/i2c/sii9024.h>
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */
#include <linux/dma-mapping.h>
#ifdef CONFIG_CHARGER_BQ24160
#include <linux/i2c/bq24160_charger.h>
#endif
#include <mach/semc_charger_usb.h>
#ifdef CONFIG_SEMC_CHARGER_CRADLE_ARCH
#include <mach/semc_charger_cradle.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_battery.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/semc_rpc_server_handset.h>
#include <mach/semc_battery_data.h>
#include <mach/msm_tsif.h>

#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include "devices.h"
#include "timer.h"
#include <mach/socinfo.h>
#include "board-semc_mogami-keypad.h"
#include "board-semc_mogami-gpio.h"
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#endif
#include <linux/regulator/consumer.h>
#include "pm.h"
#include "spm.h"
#include "acpuclock.h"
#include "pm-boot.h"
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/vreg.h>

#include <mach/sdio_al.h>
#include "smd_private.h"

/* Platform-specific regulator name mappings according to conf. spec. */
#define VREG_L8	"gp7"	/* BMA150, AK8975B, LCD, Touch, HDMI */
#define VREG_L10	"gp4"	/* BMA150, AK8975B */
#define VREG_L15	"gp6"	/* LCD */
#define VREG_L20	"gp13"	/* Touch */

#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300_SPI
#include <linux/spi/cy8ctma300_touch.h>
#endif
#include <linux/leds-as3676_semc.h>
#include "board-semc_mogami-leds.h"
#include "board-semc_mogami-touch.h"
#include <linux/i2c/bq24185_charger.h>
#include <linux/i2c/bq27520_battery.h>
#ifdef CONFIG_INPUT_BMA150
#include <linux/bma150.h>
#endif
#ifdef CONFIG_INPUT_BMA150_NG
#include <linux/bma150_ng.h>
#endif
#ifdef CONFIG_INPUT_BMA250
#include <linux/bma250.h>
#endif
#ifdef CONFIG_INPUT_APDS9702
#include <linux/apds9702.h>
#endif
#include <linux/i2c/akm8975.h>
#ifdef CONFIG_TOUCHSCREEN_CYTTSP_CORE
#include <linux/cyttsp.h>
#endif
#include <mach/mddi_novatek_fwvga.h>
#if defined(CONFIG_FB_MSM_MDDI_SONY_HVGA_LCD)
#include <linux/mddi_sony_s6d05a1_hvga.h>
#endif
#if defined(CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD)
#include <linux/mddi_hitachi_r61529_hvga.h>
#endif
#if defined(CONFIG_FB_MSM_MDDI_SII_HVGA_LCD)
#include <linux/mddi_sii_r61529_hvga.h>
#endif
#if defined(CONFIG_FB_MSM_MDDI_AUO_HVGA_LCD)
#include <linux/mddi_auo_s6d05a1_hvga.h>
#endif
#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
#include <mach/simple_remote_msm7x30_pf.h>
#endif
#ifdef CONFIG_FPC_CONNECTOR_TEST
#include <linux/fpc_connector_test.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_CLEARPAD
#include <linux/clearpad.h>
#endif
#ifdef CONFIG_SEMC_MOGAMI_FELICA_SUPPORT
#include <mach/semc_mogami_felica.h>
#endif
#include <linux/battery_chargalg.h>

#define BQ24185_GPIO_IRQ		(31)
#define CYPRESS_TOUCH_GPIO_RESET	(40)
#define CYPRESS_TOUCH_GPIO_IRQ		(42)
#ifdef CONFIG_TOUCHSCREEN_CLEARPAD
#define SYNAPTICS_TOUCH_GPIO_IRQ	(42)
#endif
#define CYPRESS_TOUCH_GPIO_SPI_CS	(46)
#ifdef CONFIG_INPUT_BMA150
#define BMA150_GPIO			(51)
#endif
#ifdef CONFIG_INPUT_BMA150_NG
#define BMA150_GPIO			(51)
#endif
#ifdef CONFIG_INPUT_BMA250
#define BMA250_GPIO			(51)
#endif
#define SPEAKER_POWERAMP_GPIO		(82)
#define AKM8975_GPIO			(92)
#define NOVATEK_GPIO_RESET		(157)

#if defined(CONFIG_FB_MSM_MDDI_SONY_HVGA_LCD) || \
	defined(CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD) || \
	defined(CONFIG_FB_MSM_MDDI_SII_HVGA_LCD) || \
	defined(CONFIG_FB_MSM_MDDI_AUO_HVGA_LCD)
#define GPIO_MSM_MDDI_XRES		(157)
#endif
#include "board-semc_mogami-regulator.h"

#define MSM_PMEM_SF_SIZE	0x1E00000
#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
#define MSM_FB_SIZE             0x530000
#else
#define MSM_FB_SIZE		0x500000
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */
#define MSM_GPU_PHYS_SIZE       SZ_4M
#define MSM_PMEM_CAMERA_SIZE    0x2F00000
#define MSM_PMEM_ADSP_SIZE      0x1300000
#define PMEM_KERNEL_EBI0_SIZE   0x600000

#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900
#define PMIC_GPIO_SD_DET	22
#define PMIC_GPIO_SDC4_PWR_EN_N 24  /* PMIC GPIO Number 25 */

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#define PM8058_MPP_BASE			   PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)	   (pm_gpio + PM8058_MPP_BASE)


#define GPIO_BQ27520_SOC_INT 20
#define LIPO_BAT_MAX_VOLTAGE 4200
#define LIPO_BAT_MIN_VOLTAGE 3000
#define FULLY_CHARGED_AND_RECHARGE_CAP 95

#if defined(CONFIG_LM3560) || defined(CONFIG_LM3561)
#include <linux/lm356x.h>
#define LM356X_HW_RESET_GPIO 2
#endif
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm72k_otg.h>
#endif

#ifdef CONFIG_SEMC_ONESEG_TUNER_PM
#include <linux/oneseg_tunerpm.h>
#define D_ONESEG_DEVICE_PORT_RESET	38 /* tuner HW reset */
#define D_ONESEG_DEVICE_PORT_POWER	39 /* tuner power suply reset */
#endif /* CONFIG_SEMC_ONESEG_TUNER_PM */

#ifdef CONFIG_SEMC_MOGAMI_IRDA
#include <mach/semc_msm_irda.h>
#define PM_GPIO_IRDA_M_RX   36
#define PM_GPIO_IRDA_M_TX   35
#define PM_GPIO_IRDA_RX1    32
#define PM_GPIO_IRDA_RX2    33
#define PM_GPIO_IRDA_RX3    34
#define PM_GPIO_IRDA_TX1    20
#define PM_GPIO_IRDA_TX2    21
#define PM_GPIO_IRDA_TX3    22
#endif

#define DDR0_BANK_BASE PHYS_OFFSET
#define DDR0_BANK_SIZE 0x03C00000
#define DDR1_BANK_BASE 0x07000000
#define DDR1_BANK_SIZE 0x09000000
#define DDR2_BANK_BASE 0x40000000
#define DDR2_BANK_SIZE 0x10000000

/* Platform specific HW-ID GPIO mask */
static const u8 hw_id_gpios[] = {150, 149, 148, 43};

static unsigned int phys_add = DDR2_BANK_BASE;
unsigned long ebi1_phys_offset = DDR2_BANK_BASE;
EXPORT_SYMBOL(ebi1_phys_offset);

extern void msm_init_pmic_vibrator(void);

extern int mogami_wifi_power(int on);

static int vreg_helper_on(const char *pzName, unsigned mv)
{
	struct vreg *reg = NULL;
	int rc = 0;

	reg = vreg_get(NULL, pzName);
	if (IS_ERR(reg)) {
		printk(KERN_ERR "Unable to resolve VREG name \"%s\"\n", pzName);
		return rc;
	}

	if (mv != (unsigned int)-1)
		rc = vreg_set_level(reg, mv);

	if (rc) {
		printk(KERN_ERR "Unable to set vreg \"%s\" level\n", pzName);
		return rc;
	}

	rc = vreg_enable(reg);
	if (rc) {
		printk(KERN_ERR "Unable to enable vreg \"%s\" level\n", pzName);
		return rc;
	}

	printk(KERN_INFO "Enabled VREG \"%s\" at %u mV\n", pzName, mv);
	return rc;
}

static void vreg_helper_off(const char *pzName)
{
	struct vreg *reg = NULL;
	int rc;

	reg = vreg_get(NULL, pzName);
	if (IS_ERR(reg)) {
		printk(KERN_ERR "Unable to resolve VREG name \"%s\"\n", pzName);
		return;
	}

	rc = vreg_disable(reg);
	if (rc) {
		printk(KERN_ERR "Unable to disable vreg \"%s\" level\n",
			pzName);
		return;
	}

	printk(KERN_INFO "Disabled VREG \"%s\"\n", pzName);
}

static ssize_t hw_id_get_mask(struct class *class, struct class_attribute *attr, char *buf)
{

	char hwid;
	unsigned int i;
	unsigned cfg;
	int rc;
	for (hwid = i = 0; i < ARRAY_SIZE(hw_id_gpios); i++) {
		cfg = GPIO_CFG(hw_id_gpios[i], 0, GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
		rc = gpio_tlmm_config(cfg, GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: Enabling of GPIO failed. "
				"gpio_tlmm_config(%#x, enable)=%d\n",
				__func__, cfg, rc);
			return rc;
		}
		hwid |= (gpio_get_value(hw_id_gpios[i]) & 1) << i;
		rc = gpio_tlmm_config(cfg, GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_INFO
				"%s: Disabling of GPIO failed. "
				"The got GPIO value is valid. "
				"gpio_tlmm_config(%#x, disable)=%d\n",
				__func__, cfg, rc);
		}
	}
	printk(KERN_INFO "Board Mogami HW ID: 0x%02x\n", hwid);
	return sprintf(buf, "0x%02x\n", hwid);
}

static CLASS_ATTR(hwid, 0444, hw_id_get_mask, NULL);
static struct class hwid_class = {.name	= "hwid",};
static void __init hw_id_class_init(void)
{
	int error;
	error = class_register(&hwid_class);
	if (error) {
		printk(KERN_ERR "%s: class_register failed\n", __func__);
		return;
	}
	error = class_create_file(&hwid_class, &class_attr_hwid);
	if (error) {
		printk(KERN_ERR "%s: class_create_file failed\n",
		__func__);
		class_unregister(&hwid_class);
	}
}

#ifdef CONFIG_FPC_CONNECTOR_TEST
extern struct fpc_connections_set fpc_connections_set;

static int fpc_pin_read(int pin)
{
	int val;
	int rc = gpio_tlmm_config(GPIO_CFG(pin, 0, GPIO_CFG_INPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if (rc) {
		pr_err("%s: failed to set gpio input\n", __func__);
		return rc;
	}
	udelay(20);
	val = gpio_get_value(pin);
	rc = gpio_tlmm_config(GPIO_CFG(pin, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (!rc) {
		udelay(20);
		gpio_set_value(pin, 0);
	} else {
		pr_err("%s: failed to set gpio output\n", __func__);
	}
	return val;
}

struct fpc_test_platform_data fpc_test_platform_data = {
	.fpc = &fpc_connections_set,
	.fpc_pin_read = fpc_pin_read,
};

struct platform_device fpc_test_device = {
	.name	= FPC_TEST_DRV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &fpc_test_platform_data,
	},
};
#endif

#ifdef CONFIG_MOGAMI_SLIDER

static const struct gpio_event_direct_entry slider_mogami_gpio_map[] = {
	{180, SW_LID},
};

static struct gpio_event_input_info slider_gpio_info = {
	.info.func = gpio_event_input_func,
	.flags = 0, /* GPIO event active low*/
	.type = EV_SW,
	.keymap = slider_mogami_gpio_map,
	.keymap_size = ARRAY_SIZE(slider_mogami_gpio_map),
};

static struct gpio_event_info *slider_info[] = {
	&slider_gpio_info.info,
};

static struct gpio_event_platform_data slider_data = {
	.name		= "slider-mogami",
	.info		= slider_info,
	.info_count	= ARRAY_SIZE(slider_info),
};

struct platform_device slider_device_mogami = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &slider_data,
	},
};

#endif /* CONFIG_MOGAMI_SLIDER */

static struct input_dev *input_dev_pwr_key = NULL;
static void msm_pmic_pwr_key_rpc_callback(uint32_t key, uint32_t event)
{
	if (!input_dev_pwr_key)
		return;
	switch (key) {
	case HS_PWR_K:
		key = KEY_POWER;
		break;
	case HS_END_K:
		key = KEY_END;
		break;
	default:
		return;
	}
	input_report_key(input_dev_pwr_key, key, event != HS_REL_K);
	input_sync(input_dev_pwr_key);
}

static int __init msm_pmic_pwr_key_init(void)
{
	input_dev_pwr_key = input_allocate_device();
	if (!input_dev_pwr_key) {
		printk(KERN_ERR "%s: Error, unable to alloc pwr key device\n",
			__func__);
		return -1;
	}
	input_dev_pwr_key->name = "msm_pmic_pwr_key";
	input_dev_pwr_key->phys = "semc_rpc_server_handset";
	input_set_capability(input_dev_pwr_key, EV_KEY, KEY_POWER);
	input_set_capability(input_dev_pwr_key, EV_KEY, KEY_END);
	if (input_register_device(input_dev_pwr_key)) {
		printk(KERN_ERR "%s: Error, unable to reg pwr key device\n",
			__func__);
		input_free_device(input_dev_pwr_key);
		return -1;
	}
	return 0;
}
module_init(msm_pmic_pwr_key_init);

/*
 * Add callbacks here. Every defined callback will receive
 * all events. The types are defined in the file
 * semc_rpc_server_handset.h
 */

static handset_cb_array_t semc_rpc_hs_callbacks = {
	&msm_pmic_pwr_key_rpc_callback,
#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
	&simple_remote_pf_button_handler,
#endif
};

static struct semc_handset_data semc_rpc_hs_data = {
	.callbacks = semc_rpc_hs_callbacks,
	.num_callbacks = ARRAY_SIZE(semc_rpc_hs_callbacks),
};

static struct platform_device semc_rpc_handset_device = {
	.name = SEMC_HANDSET_DRIVER_NAME,
	.id = -1,
	.dev = {
		.platform_data = &semc_rpc_hs_data,
	},
};

#ifdef CONFIG_SEMC_MOGAMI_IRDA
static struct msm_gpio irda_uart[] = {
	{ GPIO_CFG(85, 3, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), "UART2DM_Rx" },
	{ GPIO_CFG(87, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), "UART2DM_Tx" },
};
struct pm8058_gpio pm_irda_m_tx = {
	.direction      = PM_GPIO_DIR_IN,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_S3,
	.out_strength   = PM_GPIO_STRENGTH_NO,
	.function       = PM_GPIO_FUNC_NORMAL,
};
struct pm8058_gpio pm_irda_tx = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L2,
	.out_strength   = PM_GPIO_STRENGTH_MED,
	.function       = PM_GPIO_FUNC_2,
};
struct pm8058_gpio pm_irda_m_rx = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_S3,
	.out_strength   = PM_GPIO_STRENGTH_MED,
	.function       = PM_GPIO_FUNC_2,
};
struct pm8058_gpio pm_irda_rx = {
	.direction      = PM_GPIO_DIR_IN,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L2,
	.out_strength   = PM_GPIO_STRENGTH_NO,
	.function       = PM_GPIO_FUNC_NORMAL,
};
struct irda_pm_gpio_config {
	int                gpio;
	struct pm8058_gpio *param;
};
static struct irda_pm_gpio_config irda_pm_gpio[] = {
	{PM_GPIO_IRDA_TX1, &pm_irda_tx},
	{PM_GPIO_IRDA_TX3, &pm_irda_tx},
	{PM_GPIO_IRDA_RX1, &pm_irda_rx},
	{PM_GPIO_IRDA_RX3, &pm_irda_rx},
	{PM_GPIO_IRDA_M_TX, &pm_irda_m_tx},
	{PM_GPIO_IRDA_M_RX, &pm_irda_m_rx},
};

#define MSM_UART2DM_PHYS      0xA3200000
#define PMIC_GPIO_IRDA        38
struct pm8058_gpio g39irda_hig = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 1,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L7,
	.out_strength   = PM_GPIO_STRENGTH_MED,
	.function       = PM_GPIO_FUNC_NORMAL,
};
struct pm8058_gpio g39irda_low = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 0,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_L7,
	.out_strength   = PM_GPIO_STRENGTH_MED,
	.function       = PM_GPIO_FUNC_NORMAL,
};
static int semc_mogami_irda_init(void);
struct irda_platform_data irda_data = {
	.gpio_pow       = PMIC_GPIO_IRDA,
	.gpio_pwcfg_low = &g39irda_low,
	.gpio_pwcfg_hig = &g39irda_hig,
	.paddr_uartdm   = MSM_UART2DM_PHYS,
	.irq_uartdm     = INT_UART2DM_IRQ,
	.chan_uartdm_tx = DMOV_HSUART2_TX_CHAN,
	.crci_uartdm_tx = DMOV_HSUART2_TX_CRCI,
	.chan_uartdm_rx = DMOV_HSUART2_RX_CHAN,
	.crci_uartdm_rx = DMOV_HSUART2_RX_CRCI,
	.clk_str        = "uartdm_clk",
	.clk_dev        = &msm_device_uart_dm2.dev,
	.gpio_init      = semc_mogami_irda_init,
};
static struct platform_device irda_mogami_device = {
	.name   = "semc-msm-irda",
	.id = -1,
	.dev = {
		.platform_data = &irda_data,
	},
};

static int semc_mogami_irda_init(void)
{
	unsigned int ret;
	int i, len;

	ret = 0;

	/* Configure PM8058 GPIO*/
	len = sizeof(irda_pm_gpio)/sizeof(struct irda_pm_gpio_config);
	for (i = 0; i < len; i++) {
		ret = pm8058_gpio_config(irda_pm_gpio[i].gpio,
						irda_pm_gpio[i].param);
		if (ret) {
			pr_err("%s PM_GPIO_IRDA[%d] config failed\n",
				 __func__, i);
			return ret;
		}
	}

	/* Configure MSM UART2DM GPIO*/
	ret = msm_gpios_request_enable(irda_uart, ARRAY_SIZE(irda_uart));
	if (ret) {
		pr_err("%s enable uart2dm gpios failed\n", __func__);
		return ret;
	}

	return 0;
}
#endif

struct pm8xxx_gpio_init_info {
	unsigned			gpio;
	struct pm_gpio		config;
};

static int pm8058_gpios_init(void)
{
	int rc;

	struct pm8xxx_gpio_init_info sdcc_det = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET - 1),
		{
			.direction = PM_GPIO_DIR_IN,
			.pull = PM_GPIO_PULL_NO,
			.vin_sel = 2,
			.function = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol = 0,
		},
	};

	struct pm8xxx_gpio_init_info bq24185_irq = {
		PM8058_GPIO_PM_TO_SYS(BQ24185_GPIO_IRQ - 1),
		{
			.direction	= PM_GPIO_DIR_IN,
			.pull = PM_GPIO_PULL_NO,
			.vin_sel = PM8058_GPIO_VIN_S3,
			.function = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol = 0,
		},
	};

	rc = pm8xxx_gpio_config(sdcc_det.gpio, &sdcc_det.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_SD_DET config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(bq24185_irq.gpio, &bq24185_irq.config);
	if (rc) {
		pr_err("%s BQ24185_GPIO_IRQ config failed with %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

/* Regulator API support */

#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
static struct platform_device msm_proccomm_regulator_dev = {
	.name = PROCCOMM_REGULATOR_DEV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data = &mogami_proccomm_regulator_data
	}
};
#endif

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata = {
	.irq_base		= PMIC8058_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(PMIC_GPIO_INT),
	.irq_trigger_flag       = IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata = {
	.gpio_base		= PM8058_GPIO_PM_TO_SYS(0),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata = {
	.mpp_base	= PM8058_MPP_PM_TO_SYS(0),
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
};

static struct msm_ssbi_platform_data msm7x30_ssbi_pm8058_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
	.slave	= {
		.name			= "pm8058-core",
		.platform_data		= &pm8058_7x30_data,
	},
};

static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RST */
#if !defined(CONFIG_SEMC_CAMERA_MODULE)
#if !defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
#ifndef CONFIG_TIMPANI_CODEC
	GPIO_CFG(1,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VCM */
#endif
	GPIO_CFG(2,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
#endif
#endif
#if defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
#endif
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
#if defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CAM_VGA_RST_N */
#endif
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RST */
#if !defined(CONFIG_SEMC_CAMERA_MODULE)
#if !defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
#ifndef CONFIG_TIMPANI_CODEC
	GPIO_CFG(1,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* VCM */
#endif
	GPIO_CFG(2,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
#endif
#endif
#if defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
#endif
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), /* MCLK */
#if defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* CAM_VGA_RST_N */
#endif
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			       __func__, table[n], rc);
			break;
		}
	}
}

static int config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
			  ARRAY_SIZE(camera_on_gpio_table));
	return 0;
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
			  ARRAY_SIZE(camera_off_gpio_table));
}

struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.flags  = IORESOURCE_DMA,
	}
};

struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
#if defined(CONFIG_SEMC_CAMERA_MODULE) || defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	.ioclk.mclk_clk_rate = 8000000,
	.ioclk.vfe_clk_rate  = 192000000,
#else
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 147456000,
#endif
};

#if defined(CONFIG_SEMC_CAMERA_MODULE) || defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
static struct msm_camera_sensor_flash_data flash_none = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = NULL
 };
#endif

#ifdef CONFIG_SEMC_CAMERA_MODULE
static struct msm_camera_sensor_info msm_camera_sensor_semc_camera_data = {
	.sensor_name      = "semc_camera",
	.sensor_reset     = 0,
	.sub_sensor_reset = 31,
	.sensor_pwd       = 0,
	.vcm_pwd          = 0,
	.vcm_enable       = 0,
	.mclk             = 15,
	.flash_type       = MSM_CAMERA_FLASH_NONE,
	.pdata            = &msm_camera_device_data,
	.resource         = msm_camera_resources,
	.num_resources    = ARRAY_SIZE(msm_camera_resources),
	.flash_data       = &flash_none,
	.csi_if           = 1, /* mipi interface direct */
	.csi_params       = {
		.data_format    = CSI_10BIT,
		.lane_cnt       = 2,
		.lane_assign    = 0xe4,
		.settle_cnt     = 25,
		.dpcm_scheme    = 0
	},
	.vcam_io       = {
		.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "lvsw1"
	},
	.vcam_sd       = {
		.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "gp15"
	},
	.vcam_sa       = {
		.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "gp2"
	},
	.vcam_af       = {
		.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "gp10"
	},
};

static struct platform_device msm_camera_sensor_semc_camera = {
	.name = "msm_camera_semc_camera",
	.dev  = {
		.platform_data = &msm_camera_sensor_semc_camera_data,
	},
};
#endif

#ifdef CONFIG_SEMC_SUB_CAMERA_MODULE
static struct msm_camera_sensor_info msm_camera_sensor_semc_sub_camera_data = {
	.sensor_name      = "semc_sub_camera",
	.sensor_reset     = 0,
	.sub_sensor_reset = 31,
	.sensor_pwd       = 0,
	.vcm_pwd          = 0,
	.vcm_enable       = 0,
	.mclk             = 15,
	.flash_type       = MSM_CAMERA_FLASH_NONE,
	.pdata            = &msm_camera_device_data,
	.resource         = msm_camera_resources,
	.num_resources    = ARRAY_SIZE(msm_camera_resources),
	.flash_data       = &flash_none,
	.csi_if           = 0, /* parallel interface */
	.csi_params       = {
		.data_format    = 0,
		.lane_cnt       = 0,
		.lane_assign    = 0,
		.settle_cnt     = 0,
		.dpcm_scheme    = 0
	},
	.vcam_io       = {
		.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "lvsw1"
	},
	.vcam_sd       = {
		.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "gp15"
	},
	.vcam_sa       = {
		.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "gp2"
	},
	.vcam_af       = {
		.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "gp10"
	},
};

static struct platform_device msm_camera_sensor_semc_sub_camera = {
	.name = "msm_camera_semc_sub_camera",
	.dev  = {
		.platform_data = &msm_camera_sensor_semc_sub_camera_data,
	},
};
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#if defined (CONFIG_MSM_VPE) || defined(CONFIG_SEMC_VPE1)
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};
#endif
#ifdef CONFIG_MSM_VPE
static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif
#if defined(CONFIG_SEMC_VPE1)
static struct platform_device semc_vpe1_device = {
	.name		= "semc_vpe1",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(msm_vpe_resources),
	.resource	= msm_vpe_resources,
};
#endif

#ifdef CONFIG_MSM7KV2_AUDIO
static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(SPEAKER_POWERAMP_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static uint32_t HAC_amp_gpio_config =
   GPIO_CFG(109, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");
	rc = gpio_tlmm_config(audio_pamp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, audio_pamp_gpio_config, rc);
	}

	/* Enabling HAC amplifier */
	rc = gpio_tlmm_config(HAC_amp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, HAC_amp_gpio_config, rc);
	}


	return rc;
}
#endif

void msm_snddev_tx_route_config(void)
{
}

void msm_snddev_tx_route_deconfig(void)
{
}

void msm_hac_amp_on(void)
{
}

void msm_hac_amp_off(void)
{
}

void msm_snddev_poweramp_on(void)
{
	gpio_set_value(SPEAKER_POWERAMP_GPIO, 1);  /* enable speaker poweramp */
	pr_info("%s: power on amplifier\n", __func__);
}

void msm_snddev_poweramp_off(void)
{
	gpio_set_value(SPEAKER_POWERAMP_GPIO, 0);  /* disable speaker poweramp */
	pr_info("%s: power off amplifier\n", __func__);
}

static struct regulator_bulk_data snddev_regs[] = {
	{ .supply = "gp4", .min_uV = 2600000, .max_uV = 2600000 },
	{ .supply = "ncp", .min_uV = 1800000, .max_uV = 1800000 },
};

static int __init snddev_hsed_voltage_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n",
				__func__, rc);
		goto regs_free;
	}

	return 0;

regs_free:
	regulator_bulk_free(ARRAY_SIZE(snddev_regs), snddev_regs);
out:
	return rc;
}


void msm_snddev_hsed_voltage_on(void)
{
	int rc = regulator_bulk_enable(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc)
		pr_err("%s: could not enable regulators: %d\n", __func__, rc);
}

void msm_snddev_hsed_voltage_off(void)
{
	int rc = regulator_bulk_disable(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not disable regulators: %d\n", __func__, rc);
	}
}

#ifdef CONFIG_MSM7KV2_AUDIO
static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}
#endif

static struct msm_gpio mi2s_clk_gpios[] = {
	{ GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_SCLK"},
	{ GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_WS"},
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_MCLK_A"},
};

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD0_A"},
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
	{ GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

	rc = msm_gpios_request_enable(mi2s_clk_gpios,
			ARRAY_SIZE(mi2s_clk_gpios));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_enable(mi2s_tx_data_lines_gpios, 1);
		msm_gpios_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask &&
		      (i < ARRAY_SIZE(mi2s_rx_data_lines_gpios))) {
			if (sd_line_mask & 0x1) {
				msm_gpios_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				msm_gpios_free(
					mi2s_rx_data_lines_gpios + i , 1);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_enable(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	msm_gpios_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}


static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	pm8058_7x30_data.keypad_pdata = mogami_keypad_data();

	return 0;
}

#define TIMPANI_RESET_GPIO	1

struct bahama_config_register{
	u8 reg;
	u8 value;
	u8 mask;
};

enum version{
	VER_1_0,
	VER_2_0,
	VER_UNSUPPORTED = 0xFF
};

static struct regulator *vreg_marimba_1;
static struct regulator *vreg_marimba_2;
static struct regulator *vreg_bahama;

static u8 read_bahama_ver(void)
{
	int rc;
	struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };
	u8 bahama_version;

	rc = marimba_read_bit_mask(&config, 0x00,  &bahama_version, 1, 0x1F);
	if (rc < 0) {
		printk(KERN_ERR
			 "%s: version read failed: %d\n",
			__func__, rc);
			return rc;
	} else {
		printk(KERN_INFO
		"%s: version read got: 0x%x\n",
		__func__, bahama_version);
	}

	switch (bahama_version) {
	case 0x08: /* varient of bahama v1 */
	case 0x10:
	case 0x00:
		return VER_1_0;
	case 0x09: /* variant of bahama v2 */
		return VER_2_0;
	default:
		return VER_UNSUPPORTED;
	}
}

static unsigned int msm_bahama_core_config(int type)
{
	int rc = 0;

	if (type == BAHAMA_ID) {

		int i;
		struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };

		const struct bahama_config_register v20_init[] = {
			/* reg, value, mask */
			{ 0xF4, 0x84, 0xFF }, /* AREG */
			{ 0xF0, 0x04, 0xFF } /* DREG */
		};

		if (read_bahama_ver() == VER_2_0) {
			for (i = 0; i < ARRAY_SIZE(v20_init); i++) {
				u8 value = v20_init[i].value;
				rc = marimba_write_bit_mask(&config,
					v20_init[i].reg,
					&value,
					sizeof(v20_init[i].value),
					v20_init[i].mask);
				if (rc < 0) {
					printk(KERN_ERR
						"%s: reg %d write failed: %d\n",
						__func__, v20_init[i].reg, rc);
					return rc;
				}
				printk(KERN_INFO "%s: reg 0x%02x value 0x%02x"
					" mask 0x%02x\n",
					__func__, v20_init[i].reg,
					v20_init[i].value, v20_init[i].mask);
			}
		}
	}
	printk(KERN_INFO "core type: %d\n", type);

	return rc;
}

static unsigned int msm_bahama_setup_power(void)
{
	int rc = regulator_enable(vreg_bahama);

	if (rc)
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);

	return rc;
};

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;

	if (value != BAHAMA_ID) {
		rc = regulator_disable(vreg_bahama);

		if (rc)
			pr_err("%s: regulator_disable failed (%d)\n",
					__func__, rc);
	}

	return rc;
};

static struct msm_gpio marimba_svlte_config_clock[] = {
	{ GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"MARIMBA_SVLTE_CLOCK_ENABLE" },
};

static unsigned int msm_marimba_gpio_config_svlte(int gpio_cfg_marimba)
{
	if (machine_is_msm8x55_svlte_surf() ||
		machine_is_msm8x55_svlte_ffa()) {
		if (gpio_cfg_marimba)
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 1);
		else
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 0);
	}

	return 0;
};

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = regulator_enable(vreg_marimba_1);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto out;
	}

	rc = regulator_enable(vreg_marimba_2);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto disable_marimba_1;
	}

	return 0;

disable_marimba_1:
	regulator_disable(vreg_marimba_1);
out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = regulator_disable(vreg_marimba_2);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);

	rc = regulator_disable(vreg_marimba_1);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);
};

static int bahama_present(void)
{
	int id;
	switch (id = adie_get_detected_connectivity_type()) {
	case BAHAMA_ID:
		return 1;

	case MARIMBA_ID:
		return 0;

	case TIMPANI_ID:
	default:
	printk(KERN_ERR "%s: unexpected adie connectivity type: %d\n",
			__func__, id);
	return -ENODEV;
	}
}

struct regulator *fm_regulator;
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc, voltage;
	uint32_t irqcfg;
	const char *id = "FMPW";

	int bahama_not_marimba = bahama_present();

	if (bahama_not_marimba < 0) {
		pr_warn("%s: bahama_present: %d\n",
				__func__, bahama_not_marimba);
		rc = -ENODEV;
		goto out;
	}
	if (bahama_not_marimba) {
		fm_regulator = regulator_get(NULL, "s3");
		voltage = 1800000;
	} else {
		fm_regulator = regulator_get(NULL, "s2");
		voltage = 1300000;
	}

	if (IS_ERR(fm_regulator)) {
		rc = PTR_ERR(fm_regulator);
		pr_err("%s: regulator_get failed (%d)\n", __func__, rc);
		goto out;
	}

	rc = regulator_set_voltage(fm_regulator, voltage, voltage);

	if (rc) {
		pr_err("%s: regulator_set_voltage failed (%d)\n", __func__, rc);
		goto regulator_free;
	}

	rc = regulator_enable(fm_regulator);

	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto regulator_free;
	}

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO, PMAPP_CLOCK_VOTE_ON);

	if (rc < 0) {
		pr_err("%s: clock vote failed (%d)\n", __func__, rc);
		goto regulator_disable;
	}

	/*Request the Clock Using GPIO34/AP2MDM_MRMBCK_EN in case
	of svlte*/
	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = marimba_gpio_config(1);
		if (rc < 0) {
			pr_err("%s: clock enable for svlte : %d\n",
					__func__, rc);
			goto clock_devote;
		}
	}
	irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA);
	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n", __func__, irqcfg, rc);
		rc = -EIO;
		goto gpio_deconfig;

	}
	return 0;

gpio_deconfig:
	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa())
		marimba_gpio_config(0);
clock_devote:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO, PMAPP_CLOCK_VOTE_OFF);
regulator_disable:
	regulator_disable(fm_regulator);
regulator_free:
	regulator_put(fm_regulator);
	fm_regulator = NULL;
out:
	return rc;
};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";
	uint32_t irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA);

	int bahama_not_marimba = bahama_present();
	if (bahama_not_marimba == -1) {
		pr_warn("%s: bahama_present: %d\n",
				__func__, bahama_not_marimba);
		return;
	}

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n", __func__, irqcfg, rc);
	}
	if (!IS_ERR_OR_NULL(fm_regulator)) {
		rc = regulator_disable(fm_regulator);

		if (rc)
			pr_err("%s: return val: %d\n", __func__, rc);

		regulator_put(fm_regulator);
		fm_regulator = NULL;
	}
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0)
		pr_err("%s: clock_vote return val: %d\n", __func__, rc);

	/*Disable the Clock Using GPIO34/AP2MDM_MRMBCK_EN in case
	of svlte*/
	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = marimba_gpio_config(0);
		if (rc < 0)
			pr_err("%s: clock disable for svlte : %d\n",
					__func__, rc);
	}
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup =  fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(147),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
	.is_fm_soc_i2s_master = false,
	.config_i2s_gpio = NULL,
};


/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

#define BAHAMA_SLAVE_ID_FM_ADDR         0x2A
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B

static struct regulator_bulk_data codec_regs[] = {
	{ .supply = "s4", .min_uV = 2200000, .max_uV = 2200000 },
};

static int __init msm_marimba_codec_init(void)
{
	int rc = regulator_bulk_get(NULL, ARRAY_SIZE(codec_regs), codec_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(codec_regs), codec_regs);
	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(codec_regs), codec_regs);
out:
	return rc;
}

static int msm_marimba_codec_power(int vreg_on)
{
	int rc = vreg_on ?
		regulator_bulk_enable(ARRAY_SIZE(codec_regs), codec_regs) :
		regulator_bulk_disable(ARRAY_SIZE(codec_regs), codec_regs);

	if (rc) {
		pr_err("%s: could not %sable regulators: %d",
				__func__, vreg_on ? "en" : "dis", rc);
		return rc;
	}

	return 0;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_MARIMBA_CODEC
	.snddev_profile_init = msm_snddev_init,
#endif
};

static struct marimba_platform_data marimba_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_CDC]      = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_FM]        = BAHAMA_SLAVE_ID_FM_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	.bahama_setup = msm_bahama_setup_power,
	.bahama_shutdown = msm_bahama_shutdown_power,
	.marimba_gpio_config = msm_marimba_gpio_config_svlte,
	.bahama_core_config = msm_bahama_core_config,
	.fm = &marimba_fm_pdata,
	.codec = &mariba_codec_pdata,
	.tsadc_ssbi_adap = MARIMBA_SSBI_ADAP,
};

static void __init msm7x30_init_marimba(void)
{
	int rc;

	struct regulator_bulk_data regs[] = {
		{ .supply = "s3",   .min_uV = 1800000, .max_uV = 1800000 },
		{ .supply = "gp16", .min_uV = 1200000, .max_uV = 1200000 },
		{ .supply = "usb2", .min_uV = 1800000, .max_uV = 1800000 },
	};

	rc = msm_marimba_codec_init();

	if (rc) {
		pr_err("%s: msm_marimba_codec_init failed (%d)\n",
				__func__, rc);
		return;
	}

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		return;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		regulator_bulk_free(ARRAY_SIZE(regs), regs);
		return;
	}

	vreg_marimba_1 = regs[0].consumer;
	vreg_marimba_2 = regs[1].consumer;
	vreg_bahama    = regs[2].consumer;
}

static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_MODE_LP) |
	(1 << MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1), 	/* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),	/* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),	/* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),	/* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11),	/* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3),	/* WAV */
		DEC_INSTANCE(4, 3),	/* ADPCM */
		DEC_INSTANCE(4, 2),	/* MP3 */
		DEC_INSTANCE(0, 0),	/* Real Audio */
		DEC_INSTANCE(4, 2),	/* WMA */
		DEC_INSTANCE(3, 2),	/* AAC */
		DEC_INSTANCE(0, 0),	/* Reserved */
		DEC_INSTANCE(0, 0),	/* MIDI */
		DEC_INSTANCE(4, 3),	/* YADPCM */
		DEC_INSTANCE(4, 3),	/* QCELP */
		DEC_INSTANCE(4, 3),	/* AMRNB */
		DEC_INSTANCE(1, 1),	/* AMRWB/WB+ */
		DEC_INSTANCE(4, 3),	/* EVRC */
		DEC_INSTANCE(1, 1),	/* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3),	/* WAV */
		DEC_INSTANCE(4, 3),	/* ADPCM */
		DEC_INSTANCE(4, 3),	/* MP3 */
		DEC_INSTANCE(0, 0),	/* Real Audio */
		DEC_INSTANCE(4, 3),	/* WMA */
		DEC_INSTANCE(4, 3),	/* AAC */
		DEC_INSTANCE(0, 0),	/* Reserved */
		DEC_INSTANCE(0, 0),	/* MIDI */
		DEC_INSTANCE(4, 3),	/* YADPCM */
		DEC_INSTANCE(4, 3),	/* QCELP */
		DEC_INSTANCE(4, 3),	/* AMRNB */
		DEC_INSTANCE(2, 3),	/* AMRWB/WB+ */
		DEC_INSTANCE(4, 3),	/* EVRC */
		DEC_INSTANCE(1, 2),	/* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) /
				    ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev = {
		.platform_data = &msm_device_adspdec_database,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name = "android_usb",
	.id = -1,
	.dev = {
		.platform_data = &android_usb_pdata,
	},
};

static int novatek_reset(void)
{
	msleep(10);
	gpio_set_value(NOVATEK_GPIO_RESET, 1);
	msleep(5); /* hw spec says: 2 ms */
	gpio_set_value(NOVATEK_GPIO_RESET, 0);
	msleep(5); /* hw spec says: 2 ms */
	gpio_set_value(NOVATEK_GPIO_RESET, 1);
	msleep(30); /* hw spec says: 20 ms */
	return 0;
}

static struct novatek_fwvga_platform_data novatek_platform_data = {
	.power = NULL,
	.reset = novatek_reset,
};

static struct platform_device novatek_device = {
	.name	= MDDI_NOVATEK_FWVGA_NAME,
	.id	= -1,
	.dev	= {
		.platform_data = &novatek_platform_data,
	}
};

static const struct panel_id *novatek_panels[] = {
#ifdef CONFIG_MDDI_NOVATEK_PANEL_SHARP_LS040T8LX01
	&novatek_panel_id_sharp_ls040t8lx01_rev_c,
	&novatek_panel_id_sharp_ls040t8lx01_rev_d,
#endif
#ifdef CONFIG_MDDI_NOVATEK_PANEL_SHARP_LS042T3LX
	&novatek_panel_id_sharp_ls042t3lx_type1,
	&novatek_panel_id_sharp_ls042t3lx,
#endif
#ifdef CONFIG_MDDI_NOVATEK_PANEL_SONY_ACX424AKM
	&novatek_panel_id_sony_acx424akm_type1,
	&novatek_panel_id_sony_acx424akm,
#endif
#ifdef CONFIG_MDDI_NOVATEK_PANEL_SONY_ACX427AK
	&novatek_panel_id_sony_acx427ak,
#endif
#ifdef CONFIG_MDDI_NOVATEK_PANEL_SONY_ACX424AK
	&novatek_panel_id_sony_acx424ak,
#endif
#ifdef CONFIG_MDDI_NOVATEK_PANEL_HITACHI_DX09D09VM
	&novatek_panel_id_hitachi_dx09d09vm_type1,
	&novatek_panel_id_hitachi_dx09d09vm,
#endif
#ifdef CONFIG_MDDI_NOVATEK_PANEL_SHARP_LS033T3LX01
	&novatek_panel_id_sharp_ls033t3lx01,
#endif
#ifdef CONFIG_MDDI_NOVATEK_PANEL_TMD_LT033MDV1000
	&novatek_panel_id_tmd_lt033mdv1000,
#endif
	NULL,
};

struct novatek_i2c_pdata novatek_i2c_pdata = {
	.panels = novatek_panels,
};

#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
static struct msm_gpio sii9024_gpio_config_data_enable[] = {
	{ GPIO_CFG(90, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		"HDMI_INT" },
	{ GPIO_CFG(93, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"HDMI_5V_EN" },
	{ GPIO_CFG(102, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"HDMI_PWR_EN" },
	{ GPIO_CFG(105, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"HDMI_RESET_N" },

	{ GPIO_CFG(124, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
		"DTV_PCLK" },
	{ GPIO_CFG(125, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_EN" },
	{ GPIO_CFG(126, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_VSYNC" },
	{ GPIO_CFG(127, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_HSYNC" },

	{ GPIO_CFG(128, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA0" },
	{ GPIO_CFG(129, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA1" },
	{ GPIO_CFG(130, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA2" },
	{ GPIO_CFG(131, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA3" },
	{ GPIO_CFG(132, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA4" },
	{ GPIO_CFG(160, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA5" },
	{ GPIO_CFG(161, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA6" },
	{ GPIO_CFG(162, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA7" },
	{ GPIO_CFG(163, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA8" },
	{ GPIO_CFG(164, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA9" },
	{ GPIO_CFG(165, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA10" },
	{ GPIO_CFG(166, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA11" },
	{ GPIO_CFG(167, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA12" },
	{ GPIO_CFG(168, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA13" },
	{ GPIO_CFG(169, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA14" },
	{ GPIO_CFG(170, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA15" },
	{ GPIO_CFG(171, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA16" },
	{ GPIO_CFG(172, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA17" },
	{ GPIO_CFG(173, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA18" },
	{ GPIO_CFG(174, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA19" },
	{ GPIO_CFG(175, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA20" },
	{ GPIO_CFG(176, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA21" },
	{ GPIO_CFG(177, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA22" },
	{ GPIO_CFG(178, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"DTV_DATA23" },
};

static struct msm_gpio sii9024_gpio_config_data_disable[] = {
	{ GPIO_CFG(90, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"HDMI_INT" },
	{ GPIO_CFG(93, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"HDMI_5V_EN" },
	{ GPIO_CFG(102, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"HDMI_PWR_EN" },
	{ GPIO_CFG(105, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"HDMI_RESET_N" },

	{ GPIO_CFG(124, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_PCLK" },
	{ GPIO_CFG(125, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_EN" },
	{ GPIO_CFG(126, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_VSYNC" },
	{ GPIO_CFG(127, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_HSYNC" },

	{ GPIO_CFG(128, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA0" },
	{ GPIO_CFG(129, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA1" },
	{ GPIO_CFG(130, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA2" },
	{ GPIO_CFG(131, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA3" },
	{ GPIO_CFG(132, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA4" },
	{ GPIO_CFG(160, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA5" },
	{ GPIO_CFG(161, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA6" },
	{ GPIO_CFG(162, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA7" },
	{ GPIO_CFG(163, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA8" },
	{ GPIO_CFG(164, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA9" },
	{ GPIO_CFG(165, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA10" },
	{ GPIO_CFG(166, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA11" },
	{ GPIO_CFG(167, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA12" },
	{ GPIO_CFG(168, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA13" },
	{ GPIO_CFG(169, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA14" },
	{ GPIO_CFG(170, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA15" },
	{ GPIO_CFG(171, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA16" },
	{ GPIO_CFG(172, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA17" },
	{ GPIO_CFG(173, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA18" },
	{ GPIO_CFG(174, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA19" },
	{ GPIO_CFG(175, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA20" },
	{ GPIO_CFG(176, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA21" },
	{ GPIO_CFG(177, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA22" },
	{ GPIO_CFG(178, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		"DTV_DATA23" },
};

static int hdmi_sii_panel_power(int on)
{
	int flag_on = !!on;
	static int dtv_power_save_on;
	struct vreg *vreg_ldo23;
	int rc;
	if (dtv_power_save_on == flag_on)
		return 0;

	dtv_power_save_on = flag_on;

	if (on) {
		rc = msm_gpios_request_enable(sii9024_gpio_config_data_enable,
				ARRAY_SIZE(sii9024_gpio_config_data_enable));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio config failed: %d\n",
				__func__, rc);
			return rc;
		}
	} else {
		rc = msm_gpios_enable(sii9024_gpio_config_data_disable,
				ARRAY_SIZE(sii9024_gpio_config_data_disable));
		msm_gpios_free(sii9024_gpio_config_data_disable,
				ARRAY_SIZE(sii9024_gpio_config_data_disable));
	}

	/*  -- LDO23 for HDMI */
	vreg_ldo23 = vreg_get(NULL, "gp5");

	if (IS_ERR(vreg_ldo23)) {
		printk(KERN_ERR "%s:  vreg23 get failed (%ld)\n",
			__func__, PTR_ERR(vreg_ldo23));
		return rc;
	}

	rc = vreg_set_level(vreg_ldo23, 1200);
	if (rc) {
		printk(KERN_ERR "%s: vreg LDO23 set level failed (%d)\n",
			__func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo23);
	else
		rc = vreg_disable(vreg_ldo23);

	if (rc) {
		printk(KERN_ERR "%s: LDO23 vreg enable failed (%d)\n",
			__func__, rc);
		return rc;
	}

	mdelay(5);		/* ensure power is stable */

	return rc;
}

static struct platform_device hdmi_sii9024a_panel_device = {
	.name   = "sii9024a",
	.id     = 2,
};

static struct sii9024_platform_data sii9024_platform_data = {
	.setchippower        = hdmi_sii_panel_power,
	/* set panel_info */
	.xres               = 1280,
	.yres               = 720,
	.type               = 7, /* DTV_PANEL */
	.pdest              = 1, /* DISPLAY_2 */
	.wait_cycle         = 0,
	.bpp                = 24,
	.fb_num             = 1,
	.clk_rate           = 74250000,
	.lcdc_h_back_porch  = 220,
	.lcdc_h_front_porch = 110,
	.lcdc_h_pulse_width = 40,
	.lcdc_v_back_porch  = 20,
	.lcdc_v_front_porch = 5,
	.lcdc_v_pulse_width = 5,
	.lcdc_border_clr    = 0, /* blk */
	.lcdc_underflow_clr = 0xff, /* blue */
	.lcdc_hsync_skew    = 0,
};
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */

#if defined(CONFIG_FB_MSM_MDDI_SONY_HVGA_LCD) || \
	defined(CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD) || \
	defined(CONFIG_FB_MSM_MDDI_SII_HVGA_LCD) || \
	defined(CONFIG_FB_MSM_MDDI_AUO_HVGA_LCD)
/*  Generic LCD Regulators On function for SEMC mogami displays */
static void semc_mogami_lcd_regulators_on(void)
{
	vreg_helper_on("gp7",1800);  /* L8 */
	vreg_helper_on("gp6",2850);  /* L15 */
}

/* Generic Power On function for SEMC mogami displays */
static void semc_mogami_lcd_power_on(u8 delay1, u8 delay2, u8 delay3)
{
	gpio_tlmm_config(GPIO_CFG(GPIO_MSM_MDDI_XRES,
			0,
			GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA),
			GPIO_CFG_ENABLE );
	gpio_set_value(GPIO_MSM_MDDI_XRES,0);
	semc_mogami_lcd_regulators_on();
	mdelay(delay1);
	gpio_set_value(GPIO_MSM_MDDI_XRES,0);
	mdelay(delay2);
	gpio_set_value(GPIO_MSM_MDDI_XRES,1);
	mdelay(delay3);
}
#endif  /* (CONFIG_FB_MSM_MDDI_SONY_HVGA_LCD) ||
	(CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD) ||
	(CONFIG_FB_MSM_MDDI_SII_HVGA_LCD) ||
	(CONFIG_FB_MSM_MDDI_AUO_HVGA_LCD)*/

#if defined(CONFIG_FB_MSM_MDDI_SONY_HVGA_LCD)
/* Display resolutin */
#define SONY_HVGA_PANEL_XRES 320
#define SONY_HVGA_PANEL_YRES 480

static void sony_hvga_lcd_power_on(void)
{
	semc_mogami_lcd_regulators_on();
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	msleep(11);           /* spec > 10 ms*/
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	msleep(2);           /* spec > 1 ms*/
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	msleep(21); /* spec > 20 ms */
}

static void sony_hvga_lcd_power_off(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	msleep(121); /* spec > 120ms */
	vreg_helper_off("gp7");  /* L8 */
	vreg_helper_off("gp6");  /* L15 */
}

static void sony_hvga_lcd_exit_deep_standby(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	msleep(4);   /* spec: > 3 ms */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	msleep(11);  /* spec: > 10 ms */
}

static struct sony_hvga_platform_data sony_hvga_panel_ext = {
	.power_on = sony_hvga_lcd_power_on,
	.power_off = sony_hvga_lcd_power_off,
	.exit_deep_standby = sony_hvga_lcd_exit_deep_standby,
};

static struct platform_device mddi_sony_hvga_display_device = {
	.name = "mddi_sony_s6d05a1_hvga",
	.id = -1,
	.dev = {
		.platform_data = &sony_hvga_panel_ext,
	}
};
#endif  /* (CONFIG_FB_MSM_MDDI_SONY_HVGA_LCD) */

#if defined(CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD)
/* Display resolution */
#define HITACHI_HVGA_PANEL_XRES 320
#define HITACHI_HVGA_PANEL_YRES 480

static void hitachi_hvga_lcd_power_on(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	semc_mogami_lcd_regulators_on();
	msleep(1);           /* spec > 310us*/
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	msleep(11); /* spec > 10 */
}

static void hitachi_hvga_lcd_power_off(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	msleep(121); /* spec > 120ms */
	vreg_helper_off("gp7");  /* L8 */
	vreg_helper_off("gp6");  /* L15 */
}

static void hitachi_hvga_lcd_exit_deep_standby(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	msleep(2);   /* spec: > 1ms */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	msleep(6);  /* spec: > 5 ms */
}

static struct msm_fb_panel_data hitachi_hvga_panel_data = {
	.panel_info = {
		.xres = HITACHI_HVGA_PANEL_XRES,
		.yres = HITACHI_HVGA_PANEL_YRES,
		.pdest = DISPLAY_1,
		.type = MDDI_PANEL,
		.wait_cycle = 0,
		.bpp = 24,
		.clk_rate = 192000000,
		.clk_min =  190000000,
		.clk_max =  200000000,
		.fb_num = 2,
		.bl_max = 4,
		.bl_min = 1,
		.width = 42,
		.height = 63,
	},
};

static struct hitachi_hvga_platform_data hitachi_hvga_panel_ext = {
	.power_on = hitachi_hvga_lcd_power_on,
	.power_off = hitachi_hvga_lcd_power_off,
	.exit_deep_standby = hitachi_hvga_lcd_exit_deep_standby,
	.dbc_on = 1,
	.dbc_mode = DBC_MODE_VIDEO,
	.panel_data = &hitachi_hvga_panel_data,
};

static struct platform_device mddi_hitachi_hvga_display_device = {
	.name = MDDI_HITACH_R61529_HVGA_NAME,
	.id = -1,
	.dev = {
		.platform_data = &hitachi_hvga_panel_ext,
	}
};
#endif   /* CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD  */

#if defined(CONFIG_FB_MSM_MDDI_SII_HVGA_LCD)
/* Display resolution */
#define SII_HVGA_PANEL_XRES 320
#define SII_HVGA_PANEL_YRES 480

static void sii_hvga_lcd_power_on(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	semc_mogami_lcd_regulators_on();
	msleep(1);           /* spec > 310us*/
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	msleep(11); /* spec > 10 */
}

static void sii_hvga_lcd_power_off(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	msleep(121); /* spec > 120ms */
	vreg_helper_off("gp7");  /* L8 */
	vreg_helper_off("gp6");  /* L15 */
}

static void sii_hvga_lcd_exit_deep_standby(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	msleep(2);   /* spec: > 1ms */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	msleep(6);  /* spec: > 5 ms */
}

static struct msm_fb_panel_data sii_hvga_panel_data = {
	.panel_info = {
		.xres = SII_HVGA_PANEL_XRES,
		.yres = SII_HVGA_PANEL_YRES,
		.pdest = DISPLAY_1,
		.type = MDDI_PANEL,
		.wait_cycle = 0,
		.bpp = 24,
		.clk_rate = 192000000,
		.clk_min =  190000000,
		.clk_max =  200000000,
		.fb_num = 2,
		.bl_max = 4,
		.bl_min = 1,
		.width = 42,
		.height = 63,
	},
};

static struct sii_hvga_platform_data sii_hvga_panel_ext = {
	.power_on = sii_hvga_lcd_power_on,
	.power_off = sii_hvga_lcd_power_off,
	.exit_deep_standby = sii_hvga_lcd_exit_deep_standby,
	.dbc_on = 1,
	.dbc_mode = DBC_MODE_VIDEO,
	.panel_data = &sii_hvga_panel_data,
};

static struct platform_device mddi_sii_hvga_display_device = {
	.name = MDDI_SII_R61529_HVGA_NAME,
	.id = -1,
	.dev = {
		.platform_data = &sii_hvga_panel_ext,
	}
};
#endif   /* CONFIG_FB_MSM_MDDI_SII_HVGA_LCD  */

#if defined(CONFIG_FB_MSM_MDDI_AUO_HVGA_LCD)
/* Display resolution */
#define AUO_HVGA_PANEL_XRES 320
#define AUO_HVGA_PANEL_YRES 480

static void auo_hvga_lcd_power_on(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	semc_mogami_lcd_regulators_on();
	msleep(2);           /* spec > 1 ms*/
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	msleep(51); /* spec > 50 ms */
}

static void auo_hvga_lcd_power_off(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	msleep(121); /* spec > 120ms */
	vreg_helper_off("gp7");  /* L8 */
	vreg_helper_off("gp6");  /* L15 */
}

static void auo_hvga_lcd_exit_deep_standby(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	msleep(2);   /* spec: > 1ms */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	msleep(51);  /* spec: > 50 ms */
}

static struct msm_fb_panel_data auo_hvga_panel_data = {
	.panel_info = {
		.xres = AUO_HVGA_PANEL_XRES,
		.yres = AUO_HVGA_PANEL_YRES,
		.pdest = DISPLAY_1,
		.type = MDDI_PANEL,
		.wait_cycle = 0,
		.bpp = 24,
		.clk_rate = 192000000,
		.clk_min =  190000000,
		.clk_max =  200000000,
		.fb_num = 2,
		.bl_max = 4,
		.bl_min = 1,
		.width = 42,
		.height = 63,
	},
};

static struct auo_hvga_platform_data auo_hvga_panel_ext = {
	.power_on = auo_hvga_lcd_power_on,
	.power_off = auo_hvga_lcd_power_off,
	.exit_deep_standby = auo_hvga_lcd_exit_deep_standby,
	.dbc_on = 1,
	.dbc_mode = DBC_MODE_VIDEO,
	.panel_data = &auo_hvga_panel_data,
};

static struct platform_device mddi_auo_hvga_display_device = {
	.name = MDDI_AUO_S6D05A1_HVGA_NAME,
	.id = -1,
	.dev = {
		.platform_data = &auo_hvga_panel_ext,
	}
};
#endif   /* CONFIG_FB_MSM_MDDI_AUO_HVGA_LCD  */

#if defined(CONFIG_TOUCHSCREEN_CY8CTMA300_SPI) || \
	defined(CONFIG_TOUCHSCREEN_CYTTSP_SPI)
struct msm_gpio ttsp_gpio_cfg_data[] = {
	{ GPIO_CFG(42, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		"ttsp_irq" },
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300_SPI
static int cypress_touch_gpio_init(void);
static int cypress_touch_spi_cs_set(bool val);

static struct cypress_touch_platform_data cypress_touch_data = {
	.x_max = CONFIG_CY8CTMA300_SPI_MAX_X,
	.y_max = CONFIG_CY8CTMA300_SPI_MAX_Y,
	.z_max = CONFIG_CY8CTMA300_SPI_MAX_Z,
	.width_major = CONFIG_CY8CTMA300_SPI_WIDTH_MAJOR,
	.gpio_init = cypress_touch_gpio_init,
	.gpio_irq_pin = CYPRESS_TOUCH_GPIO_IRQ,
	.gpio_reset_pin = CYPRESS_TOUCH_GPIO_RESET,
	.spi_cs_set = cypress_touch_spi_cs_set,
};

static int cypress_touch_gpio_init(void)
{
	int rc;

	msleep(10);

	rc = msm_gpios_enable(ttsp_gpio_cfg_data,
				ARRAY_SIZE(ttsp_gpio_cfg_data));
	if (rc)
		return rc;

	gpio_set_value(CYPRESS_TOUCH_GPIO_RESET, 1);
	return 0;
}

static int cypress_touch_spi_cs_set(bool val)
{
	int rc = 0;
	int cfg;

	if (val) {
		gpio_set_value(CYPRESS_TOUCH_GPIO_SPI_CS, 1);
		cfg = GPIO_CFG(CYPRESS_TOUCH_GPIO_SPI_CS, 1, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
		rc = gpio_tlmm_config(cfg, GPIO_CFG_ENABLE);
		if (rc)
			pr_err("%s: Enabling of GPIO failed. "
				"gpio_tlmm_config(%#x, enable)=%d\n",
				__func__, cfg, rc);
	} else {
		cfg = GPIO_CFG(CYPRESS_TOUCH_GPIO_SPI_CS, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
		rc = gpio_tlmm_config(cfg, GPIO_CFG_ENABLE);
		if (rc)
			pr_err("%s: Enabling of GPIO failed. "
				"gpio_tlmm_config(%#x, enable)=%d\n",
				__func__, cfg, rc);
		gpio_set_value(CYPRESS_TOUCH_GPIO_SPI_CS, 0);
	}
	return rc;
}
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA300_SPI */

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_SPI
int cyttsp_xres(void)
{
	int polarity;
	int rc;

	rc = gpio_direction_input(CYPRESS_TOUCH_GPIO_RESET);
	if (rc) {
		printk(KERN_ERR "%s: failed to set direction input, %d\n",
		       __func__, rc);
		return -EIO;
	}
	polarity = gpio_get_value(CYPRESS_TOUCH_GPIO_RESET) & 0x01;
	printk(KERN_INFO "%s: %d\n", __func__, polarity);
	rc = gpio_direction_output(CYPRESS_TOUCH_GPIO_RESET, polarity ^ 1);
	if (rc) {
		printk(KERN_ERR "%s: failed to set direction output, %d\n",
		       __func__, rc);
		return -EIO;
	}
	msleep(1);
	gpio_set_value(CYPRESS_TOUCH_GPIO_RESET, polarity);
	return 0;
}

int cyttsp_init(int on)
{
	int rc = -1;
	if (on) {
		if (gpio_request(CYPRESS_TOUCH_GPIO_IRQ, "ttsp_irq"))
			goto ttsp_irq_err;
		if (gpio_request(CYPRESS_TOUCH_GPIO_RESET, "ttsp_reset"))
			goto ttsp_reset_err;

		rc = msm_gpios_enable(ttsp_gpio_cfg_data,
					ARRAY_SIZE(ttsp_gpio_cfg_data));
		if (rc)
			goto ttsp_gpio_cfg_err;
		return 0;
	} else {
		rc = 0;
	}
ttsp_gpio_cfg_err:
	gpio_free(CYPRESS_TOUCH_GPIO_RESET);
ttsp_reset_err:
	gpio_free(CYPRESS_TOUCH_GPIO_IRQ);
ttsp_irq_err:
	return rc;
}

int cyttsp_wakeup(void)
{
	int ret;

	ret = gpio_direction_output(CYPRESS_TOUCH_GPIO_IRQ, 0);
	if (ret) {
		printk(KERN_ERR "%s: Failed to request gpio_direction_output\n",
		__func__);
                return ret;
	}
	msleep(50);
	gpio_set_value(CYPRESS_TOUCH_GPIO_IRQ, 0);
	msleep(1);
	gpio_set_value(CYPRESS_TOUCH_GPIO_IRQ, 1);
	udelay(100);
	gpio_set_value(CYPRESS_TOUCH_GPIO_IRQ, 0);
	msleep(1);
	gpio_set_value(CYPRESS_TOUCH_GPIO_IRQ, 1);
	printk(KERN_INFO "%s: wakeup\n", __func__);
	ret = gpio_direction_input(CYPRESS_TOUCH_GPIO_IRQ);
	if (ret) {
		printk(KERN_ERR "%s: Failed to request gpio_direction_input\n",
		__func__);
		return ret;
	}
	msleep(50);
	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_KEY
#define TT_KEY_BACK_FLAG	0x01
#define TT_KEY_MENU_FLAG	0x02
#define TT_KEY_HOME_FLAG	0x04

static struct input_dev *input_dev_cyttsp_key;

static int __init cyttsp_key_init(void)
{
	input_dev_cyttsp_key = input_allocate_device();
	if (!input_dev_cyttsp_key) {
		pr_err("%s: Error, unable to alloc cyttsp key device\n", __func__);
		return -ENOMEM;
	}
	input_dev_cyttsp_key->name = "cyttsp_key";
	input_dev_cyttsp_key->phys = "/sys/bus/spi/devices/spi0.0/";
	input_set_capability(input_dev_cyttsp_key, EV_KEY, KEY_MENU);
	input_set_capability(input_dev_cyttsp_key, EV_KEY, KEY_BACK);
	input_set_capability(input_dev_cyttsp_key, EV_KEY, KEY_HOME);
	if (input_register_device(input_dev_cyttsp_key)) {
		pr_err("%s: Error, unable to reg cyttsp key device\n", __func__);
		input_free_device(input_dev_cyttsp_key);
		return -ENODEV;
	}
	return 0;
}
module_init(cyttsp_key_init);

int cyttsp_key_rpc_callback(u8 data[], int size)
{
	static u8 last;
	u8 toggled = last ^ data[0];

	if (toggled & TT_KEY_MENU_FLAG)
		input_report_key(input_dev_cyttsp_key, KEY_MENU,
			!!(*data & TT_KEY_MENU_FLAG));

	if (toggled & TT_KEY_BACK_FLAG)
		input_report_key(input_dev_cyttsp_key, KEY_BACK,
			!!(*data & TT_KEY_BACK_FLAG));

	if (toggled & TT_KEY_HOME_FLAG)
		input_report_key(input_dev_cyttsp_key, KEY_HOME,
			!!(*data & TT_KEY_HOME_FLAG));

	input_sync(input_dev_cyttsp_key);
	last = data[0];
	return 0;
}
#endif /* CONFIG_TOUCHSCREEN_CYTTSP_KEY */

#endif /* CONFIG_TOUCHSCREEN_CYTTSP_SPI */

#ifdef CONFIG_TOUCHSCREEN_CLEARPAD
static struct msm_gpio clearpad_gpio_config_data[] = {
	{ GPIO_CFG(SYNAPTICS_TOUCH_GPIO_IRQ, 0, GPIO_CFG_INPUT,
		   GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "clearpad3000_irq" },
};

static int clearpad_gpio_configure(int enable)
{
	int rc = 0;

	if (enable)
		rc = msm_gpios_request_enable(clearpad_gpio_config_data,
				ARRAY_SIZE(clearpad_gpio_config_data));
	else
		msm_gpios_free(clearpad_gpio_config_data,
				ARRAY_SIZE(clearpad_gpio_config_data));
	return rc;
}

static struct synaptics_button synaptics_menu_key = {
	.type = EV_KEY,
	.code = KEY_MENU,
};

static struct synaptics_button synaptics_back_key = {
	.type = EV_KEY,
	.code = KEY_BACK,
};

static struct synaptics_funcarea clearpad_funcarea_array[] = {
	{ 0, 0, 479, 853, SYN_FUNCAREA_POINTER, NULL },
	{ 0, 854, 479, 863, SYN_FUNCAREA_BOTTOM_EDGE, NULL},
	{ 0, 884, 159, 921, SYN_FUNCAREA_BUTTON, &synaptics_back_key },
	{ 0, 864, 179, 921, SYN_FUNCAREA_BTN_INBOUND, &synaptics_back_key },
	{ 320, 884, 479, 921, SYN_FUNCAREA_BUTTON, &synaptics_menu_key },
	{ 300, 864, 479, 921, SYN_FUNCAREA_BTN_INBOUND, &synaptics_menu_key },
	{ .func = SYN_FUNCAREA_END }
};

static void clearpad_vreg_off(void)
{
	int i;

	vreg_helper_off(VREG_L20);
	for (i = 0; i < 500; i++)
		udelay(1000);
}

static struct clearpad_platform_data clearpad_platform_data = {
	.irq = MSM_GPIO_TO_INT(SYNAPTICS_TOUCH_GPIO_IRQ),
	.funcarea = clearpad_funcarea_array,
	.gpio_configure = clearpad_gpio_configure,
	.vreg_off = clearpad_vreg_off,
};
#endif

/* Driver(s) to be notified upon change in battery data */
static char *semc_bdata_supplied_to[] = {
	BQ27520_NAME,
	BATTERY_CHARGALG_NAME,
};

static struct semc_battery_platform_data semc_battery_platform_data = {
	.supplied_to = semc_bdata_supplied_to,
	.num_supplicants = ARRAY_SIZE(semc_bdata_supplied_to),
#ifndef CONFIG_BATTERY_BQ27520
	.use_fuelgauge = 1,
#endif
};

static struct platform_device bdata_driver = {
	.name = SEMC_BDATA_NAME,
	.id = -1,
	.dev = {
		.platform_data = &semc_battery_platform_data,
	},
};

static int bq27520_gpio_configure(int enable)
{
	int rc = 0;

	if (!!enable) {
		rc = gpio_request(GPIO_BQ27520_SOC_INT, "bq27520");
		if (rc)
			pr_err("%s: gpio_requeset failed, "
					"rc=%d\n", __func__, rc);
	} else {
		gpio_free(GPIO_BQ27520_SOC_INT);
	}
	return rc;
}

static char *bq27520_supplied_to[] = {
	BATTERY_CHARGALG_NAME,
};

static struct bq27520_block_table bq27520_block_table[BQ27520_BTBL_MAX] = {
	{0x61, 0x00}, {0x3E, 0x24}, {0x3F, 0x00}, {0x42, 0x00},
	{0x43, 0x46}, {0x44, 0x00}, {0x45, 0x19}, {0x46, 0x00},
	{0x47, 0x64}, {0x48, 0x28}, {0x4B, 0xFF}, {0x4C, 0x5F},
	{0x60, 0xF4}
};

struct bq27520_platform_data bq27520_platform_data = {
	.name = BQ27520_NAME,
	.supplied_to = bq27520_supplied_to,
	.num_supplicants = ARRAY_SIZE(bq27520_supplied_to),
	.lipo_bat_max_volt = LIPO_BAT_MAX_VOLTAGE,
	.lipo_bat_min_volt = LIPO_BAT_MIN_VOLTAGE,
	.battery_dev_name = SEMC_BDATA_NAME,
	.gpio_configure = bq27520_gpio_configure,
	.polling_lower_capacity = FULLY_CHARGED_AND_RECHARGE_CAP,
	.polling_upper_capacity = 100,
	.udatap = bq27520_block_table,
#ifdef CONFIG_BATTERY_CHARGALG
	.disable_algorithm = battery_chargalg_disable,
#endif
#ifdef CONFIG_SEMC_CHARGER_CRADLE_ARCH
	.get_ac_online_status = semc_charger_get_ac_online_status,
#endif
};

/* Driver(s) to be notified upon change in charging */
static char *bq24185_supplied_to[] = {
	BATTERY_CHARGALG_NAME,
	SEMC_BDATA_NAME,
};

static int bq24185_gpio_configure(int enable)
{
	int rc = 0;

	if (!!enable) {
		rc = gpio_request(BQ24185_GPIO_IRQ, "bq24185");
		if (rc)
			pr_err("%s: gpio_requeset failed, "
					"rc=%d\n", __func__, rc);
	} else {
		gpio_free(BQ24185_GPIO_IRQ);
	}
	return rc;
}

struct bq24185_platform_data bq24185_platform_data = {
	.name = BQ24185_NAME,
	.supplied_to = bq24185_supplied_to,
	.num_supplicants = ARRAY_SIZE(bq24185_supplied_to),
	.support_boot_charging = 1,
	.rsens = BQ24185_RSENS_REF,
	/* Maximum battery regulation voltage = 4200mV */
	.mbrv = BQ24185_MBRV_MV_4200,
	/* Maximum charger current sense voltage = 71.4mV */
	.mccsv = BQ24185_MCCSV_MV_6p8 | BQ24185_MCCSV_MV_27p2 |
		BQ24185_MCCSV_MV_37p4,
	.notify_vbus_drop = msm_otg_notify_vbus_drop,
	.gpio_configure = bq24185_gpio_configure,
	.vindpm_usb_compliant = VINDPM_4550MV,
	.vindpm_non_compliant = VINDPM_4390MV,
};

static char *battery_chargalg_supplied_to[] = {
	SEMC_BDATA_NAME,
};

static struct battery_chargalg_platform_data battery_chargalg_platform_data = {
	.name = BATTERY_CHARGALG_NAME,
	.supplied_to = battery_chargalg_supplied_to,
	.num_supplicants = ARRAY_SIZE(battery_chargalg_supplied_to),
	.ext_eoc_recharge_enable = 1,
	.temp_hysteresis_design = 3,
	.ddata = &device_data,
	.batt_volt_psy_name = BQ27520_NAME,
	.batt_curr_psy_name = BQ27520_NAME,
#ifdef CONFIG_CHARGER_BQ24185
	.turn_on_charger = bq24185_turn_on_charger,
	.turn_off_charger = bq24185_turn_off_charger,
	.set_charger_voltage = bq24185_set_charger_voltage,
	.set_charger_current = bq24185_set_charger_current,
	.set_input_current_limit = bq24185_set_input_current_limit,
	.set_charging_status = bq24185_set_ext_charging_status,
//	.get_supply_current_limit = NULL,
	.get_restrict_ctl = NULL,
	.get_restricted_setting = NULL,
	.setup_exchanged_power_supply = NULL,
	.charge_set_current_1 = 350,
	.charge_set_current_2 = 550,
	.charge_set_current_3 = 750,
	.overvoltage_max_design = 4225,
#endif
#ifdef CONFIG_SEMC_CHARGER_USB_ARCH
	.get_supply_current_limit = semc_charger_usb_current_ma,
#endif
#ifdef CONFIG_SEMC_CHARGER_CRADLE_ARCH
	.get_ac_online_status = semc_charger_get_ac_online_status,
	.set_input_current_limit_dual = bq24160_set_input_current_limit_dual,
	.set_input_voltage_dpm_usb = bq24160_set_input_voltage_dpm_usb,
	.set_input_voltage_dpm_cradle = bq24160_set_input_voltage_dpm_in,
	.get_supply_current_limit_cradle = semc_charger_cradle_current_ma,
#endif
	.allow_dynamic_charge_current_ctrl = 1,
	.average_current_min_limit = -1,
	.average_current_max_limit = 250,
};

static struct platform_device battery_chargalg_platform_device = {
	.name = BATTERY_CHARGALG_NAME,
	.id = -1,
	.dev = {
		.platform_data = &battery_chargalg_platform_data,
	},
};

#if defined(CONFIG_LM3560) || defined(CONFIG_LM3561)
int lm356x_request_gpio_pins(void)
{
	int result;

	result = gpio_request(LM356X_HW_RESET_GPIO, "LM356X hw reset");
	if (result)
		return result;

	gpio_set_value(LM356X_HW_RESET_GPIO, 1);

	udelay(20);
	return result;
}

int lm356x_release_gpio_pins(void)
{

	gpio_set_value(LM356X_HW_RESET_GPIO, 0);
	gpio_free(LM356X_HW_RESET_GPIO);

	return 0;
}
#endif

#ifdef CONFIG_LM3560
static struct lm356x_platform_data lm3560_platform_data = {
	.hw_enable              = lm356x_request_gpio_pins,
	.hw_disable             = lm356x_release_gpio_pins,
	.led_nums		= 2,
	.strobe_trigger		= LM356X_STROBE_TRIGGER_EDGE,
	.privacy_terminate	= LM356X_PRIVACY_MODE_TURN_BACK,
	.privacy_led_nums	= 1,
	.privacy_blink_period	= 0, /* No bliking */
	.current_limit		= 2300000, /* uA */
	.flash_sync		= LM356X_SYNC_OFF,
	.strobe_polarity	= LM356X_STROBE_POLARITY_HIGH,
	.ledintc_pin_setting	= LM356X_LEDINTC_NTC_THERMISTOR_INPUT,
	.tx1_polarity		= LM356X_TX1_POLARITY_HIGH,
	.tx2_polarity		= LM356X_TX2_POLARITY_HIGH,
	.hw_torch_mode		= LM356X_HW_TORCH_MODE_DISABLE,
};
#endif
#ifdef CONFIG_LM3561
static struct lm356x_platform_data lm3561_platform_data = {
	.hw_enable              = lm356x_request_gpio_pins,
	.hw_disable             = lm356x_release_gpio_pins,
	.led_nums		= 1,
	.strobe_trigger		= LM356X_STROBE_TRIGGER_EDGE,
	.privacy_terminate	= LM356X_PRIVACY_MODE_TURN_BACK,
	.privacy_led_nums	= 0,
	.privacy_blink_period	= 0, /* No bliking */
	.current_limit		= 1000, /* uA
				   selectable value are 1500mA or 1000mA.
				   if set other value,
				   it assume current limit is 1000mA.
				*/
	.flash_sync		= LM356X_SYNC_OFF,
	.strobe_polarity	= LM356X_STROBE_POLARITY_HIGH,
	.ledintc_pin_setting	= LM356X_LEDINTC_NTC_THERMISTOR_INPUT,
	.tx1_polarity		= LM356X_TX1_POLARITY_HIGH,
	.tx2_polarity		= LM356X_TX2_POLARITY_HIGH,
	.hw_torch_mode		= LM356X_HW_TORCH_MODE_DISABLE,
};
#endif

#ifdef CONFIG_INPUT_BMA150
static int bma150_gpio_setup(struct device *dev)
{
	return gpio_request(BMA150_GPIO, "bma150_irq");
}

static void bma150_gpio_teardown(struct device *dev)
{
	gpio_free(BMA150_GPIO);
}

static struct bma150_platform_data bma150_platform_data = {
	.setup    = bma150_gpio_setup,
	.teardown = bma150_gpio_teardown,
};
#endif

#ifdef CONFIG_INPUT_BMA150_NG
static int bma150_gpio_setup(bool request)
{
	if (request)
		return gpio_request(BMA150_GPIO, "bma150_irq");
	else
		gpio_free(BMA150_GPIO);
	return 0;
}

struct bma150_platform_data bma150_ng_platform_data = {
	.gpio_setup = bma150_gpio_setup,
};
#endif

#ifdef CONFIG_INPUT_BMA250
static int bma250_gpio_setup(struct device *dev)
{
	return 0;
}

static void bma250_gpio_teardown(struct device *dev)
{
	return;
}

static void bma250_hw_config(int enable)
{
	return;
}

static void bma250_power_mode(int enable)
{
	return;
}

static struct registers bma250_reg_setup = {
	.range                = BMA250_RANGE_2G,
	.bw_sel               = BMA250_BW_250HZ,
};

static struct bma250_platform_data bma250_platform_data = {
	.setup                = bma250_gpio_setup,
	.teardown             = bma250_gpio_teardown,
	.hw_config            = bma250_hw_config,
	.power_mode           = bma250_power_mode,
	.reg                  = &bma250_reg_setup,
	.bypass_state         = mpu3050_bypassmode,
	.read_axis_data       = bma250_read_axis_from_mpu3050,
	.check_sleep_status   = check_bma250_sleep_state,
	.vote_sleep_status    = vote_bma250_sleep_state,
	.rate                 = BMA250_DEFAULT_RATE,
};
#endif

#ifdef CONFIG_INPUT_APDS9702

#define APDS9702_DOUT_GPIO   88
#define APDS9702_VDD_VOLTAGE 2900
#define APDS9702_WAIT_TIME   5000

static int apds9702_gpio_setup(int request)
{
	if (request)
		return gpio_request(APDS9702_DOUT_GPIO, "apds9702_dout");
	gpio_free(APDS9702_DOUT_GPIO);
	return 0;
}

static void apds9702_hw_config(int enable)
{
	enable = !!enable;
	if (enable)
		vreg_helper_on("wlan", APDS9702_VDD_VOLTAGE);
	else
		vreg_helper_off("wlan");
	usleep(APDS9702_WAIT_TIME);
}

static void apds9702_power_mode(int enable)
{
	return;
}

static struct apds9702_platform_data apds9702_pdata = {
	.gpio_dout      = APDS9702_DOUT_GPIO,
	.is_irq_wakeup  = 1,
	.hw_config      = apds9702_hw_config,
	.power_mode     = apds9702_power_mode,
	.gpio_setup     = apds9702_gpio_setup,
	.ctl_reg = {
		.trg   = 1,
		.pwr   = 1,
		.burst = 7,
		.frq   = 3,
		.dur   = 2,
		.th    = 15,
		.rfilt = 0,
	},
	.phys_dev_path = "/sys/devices/i2c-0/0-0054",
};
#endif

static struct msm_gpio akm8975_gpio_config_data[] = {
	{ GPIO_CFG(AKM8975_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,
		GPIO_CFG_2MA), "akm8975_drdy_irq" },
};

static int akm8975_gpio_setup(void)
{
	return msm_gpios_request_enable(akm8975_gpio_config_data,
			ARRAY_SIZE(akm8975_gpio_config_data));
}

static void akm8975_gpio_shutdown(void)
{
	msm_gpios_disable_free(akm8975_gpio_config_data,
		ARRAY_SIZE(akm8975_gpio_config_data));

}

static struct akm8975_platform_data akm8975_platform_data = {
	.setup = akm8975_gpio_setup,
	.shutdown = akm8975_gpio_shutdown,
};

static struct i2c_board_info msm_i2c_board_info[] = {
#ifdef CONFIG_TOUCHSCREEN_CLEARPAD_I2C
	{
		I2C_BOARD_INFO(CLEARPADI2C_NAME, 0x58 >> 1),
		.platform_data = &clearpad_platform_data,
	},
#endif
	{
		I2C_BOARD_INFO("as3676", 0x80 >> 1),
		.platform_data = &as3676_platform_data,
	},
	{
		I2C_BOARD_INFO(BQ27520_NAME, 0xAA >> 1),
		.irq = MSM_GPIO_TO_INT(GPIO_BQ27520_SOC_INT),
		.platform_data = &bq27520_platform_data,
		.type = BQ27520_NAME,
	},
	{
		I2C_BOARD_INFO(BQ24185_NAME, 0xd6 >> 1),
		.platform_data = &bq24185_platform_data,
		.type = BQ24185_NAME,
		.irq = MSM_GPIO_TO_INT(BQ24185_GPIO_IRQ),
	},
#ifdef CONFIG_INPUT_BMA150
	{ /* TODO: Remove? Added due to wrong bus connection on Anzu SP1. */
		I2C_BOARD_INFO("bma150", 0x70 >> 1),
		.irq = MSM_GPIO_TO_INT(BMA150_GPIO),
		.platform_data = &bma150_platform_data,
		.type = "bma150"
	},
#endif
#ifdef CONFIG_INPUT_BMA150_NG
	{
		I2C_BOARD_INFO("bma150", 0x70 >> 1),
		.irq = MSM_GPIO_TO_INT(BMA150_GPIO),
		.platform_data = &bma150_ng_platform_data,
		.type = "bma150"
	},
#endif
#ifdef CONFIG_INPUT_BMA250
	{
		I2C_BOARD_INFO("bma250", 0x18),
		.irq = MSM_GPIO_TO_INT(BMA250_GPIO),
		.platform_data = &bma250_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_BMP180
	{
		I2C_BOARD_INFO("bmp180", 0x77)
	},
#endif
#ifdef CONFIG_INPUT_APDS9702
	{
		I2C_BOARD_INFO(APDS9702_NAME, 0xA8 >> 1),
		.platform_data = &apds9702_pdata,
		.type = APDS9702_NAME,
	},
#endif
#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
	{
		I2C_BOARD_INFO("sii9024a", 0x76 >> 1),
		.platform_data = &sii9024_platform_data,
		.type = "sii9024a"
	},
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x18 >> 1),
		.irq = MSM_GPIO_TO_INT(AKM8975_GPIO),
		.platform_data = &akm8975_platform_data,
	},
#ifdef CONFIG_LM3560
	{
		I2C_BOARD_INFO("lm3560", 0xA6 >> 1),
		.platform_data = &lm3560_platform_data,
	},
#endif
#ifdef CONFIG_LM3561
	{
		I2C_BOARD_INFO("lm3561", 0xA6 >> 1),
		.platform_data = &lm3561_platform_data,
	},
#endif
};

static struct i2c_board_info mogami_qup_i2c_devices[] __initdata = {
	{
		I2C_BOARD_INFO(MDDI_NOVATEK_I2C_NAME, 0x98 >> 1),
		.type = MDDI_NOVATEK_I2C_NAME,
		.platform_data = &novatek_i2c_pdata,
	},
#ifdef CONFIG_INPUT_BMA150
	{
		I2C_BOARD_INFO("bma150", 0x70 >> 1),
		.irq = MSM_GPIO_TO_INT(BMA150_GPIO),
		.platform_data = &bma150_platform_data,
		.type = "bma150"
	},
#endif
#ifdef CONFIG_INPUT_BMA150_NG
	{
		I2C_BOARD_INFO("bma150", 0x70 >> 1),
		.irq = MSM_GPIO_TO_INT(BMA150_GPIO),
		.platform_data = &bma150_ng_platform_data,
		.type = "bma150"
	},
#endif
#ifdef CONFIG_SEMC_CAMERA_MODULE
	{
		I2C_BOARD_INFO("semc_camera", 0x1A),
		.type = "semc_camera"
	},
#endif
#ifdef CONFIG_SEMC_SUB_CAMERA_MODULE
	{
		I2C_BOARD_INFO("semc_sub_camera", 0x3D),
		.type = "semc_sub_camera"
	},
#endif
};

static struct spi_board_info spi_board_info[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300_SPI
	{
		.modalias       = "cypress_touchscreen",
		.mode           = SPI_MODE_0,
		.platform_data  = &cypress_touch_data,
		.bus_num        = 0,
		.chip_select    = 0,
		.max_speed_hz   = 1 * 1000 * 1000,
		.irq		= MSM_GPIO_TO_INT(CYPRESS_TOUCH_GPIO_IRQ),
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_CYTTSP_SPI
        {
                .modalias       = CY_SPI_NAME,
                .mode           = SPI_MODE_0,
                .irq            = MSM_GPIO_TO_INT(CYPRESS_TOUCH_GPIO_IRQ),
                .platform_data  = &cyttsp_data,
                .bus_num        = 0,
                .chip_select    = 0,
                .max_speed_hz   = 1 *  1000 * 1000,
        },
#endif /* CONFIG_TOUCHSCREEN_CYTTSP_SPI */
};

static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 0,
		.latency = 500,
		.residency = 6000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

u32 msm7x30_power_collapse_latency(enum msm_pm_sleep_mode mode)
{
	switch (mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency;
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency;
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
	default:
	return 0;
	}
}

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_VIRT,
	.v_addr = (uint32_t *)PAGE_OFFSET,
};

static struct resource qsd_spi_resources[] = {
	{
		.name = "spi_irq_in",
		.start = INT_SPI_INPUT,
		.end = INT_SPI_INPUT,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "spi_irq_out",
		.start = INT_SPI_OUTPUT,
		.end = INT_SPI_OUTPUT,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "spi_irq_err",
		.start = INT_SPI_ERROR,
		.end = INT_SPI_ERROR,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "spi_base",
		.start = 0xA8000000,
		.end = 0xA8000000 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "spidm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.name = "spidm_crci",
		.flags = IORESOURCE_DMA,
	},
};

#define AMDH0_BASE_PHYS		0xAC200000
#define ADMH0_GP_CTL		(ct_adm_base + 0x3D8)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_adm_base = 0;
	u32 spi_mux = 0;
	int ret = 0;

	ct_adm_base = ioremap(AMDH0_BASE_PHYS, PAGE_SIZE);
	if (!ct_adm_base) {
		pr_err("%s: Could not remap %x\n", __func__, AMDH0_BASE_PHYS);
		return -ENOMEM;
	}

	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;

	qsd_spi_resources[4].start = DMOV_USB_CHAN;
	qsd_spi_resources[4].end = DMOV_TSIF_CHAN;

	switch (spi_mux) {
	case (1):
		qsd_spi_resources[5].start = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[5].start = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[5].start = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -ENOENT;
	}

	iounmap(ct_adm_base);

	return ret;
}

static struct platform_device qsd_device_spi = {
	.name = "spi_qsd",
	.id = 0,
	.num_resources = ARRAY_SIZE(qsd_spi_resources),
	.resource = qsd_spi_resources,
};

static int msm_qsd_spi_gpio_config(void)
{
	return msm_gpios_request_enable(qsd_spi_gpio_config_data,
					qsd_spi_gpio_config_data_size);
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
			       qsd_spi_gpio_config_data_size);
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.gpio_config = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
#ifdef CONFIG_CHARGER_BQ24185
	if (on)
		(void)bq24185_set_opa_mode(CHARGER_BOOST_MODE);
	else
		(void)bq24185_set_opa_mode(CHARGER_CHARGER_MODE);
#endif
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
	.vbus_power = msm_hsusb_vbus_power,
	.power_budget = 300,
};
#endif

#ifdef CONFIG_SEMC_CHARGER_USB_ARCH
static char *semc_chg_usb_supplied_to[] = {
	BATTERY_CHARGALG_NAME,
	BQ27520_NAME,
};
#endif

static int hs_drv_ampl_ratio[] = {
	HS_DRV_AMPLITUDE_DEFAULT,
	HS_DRV_AMPLITUDE_ZERO_PERCENT,
	HS_DRV_AMPLITUDE_25_PERCENTI,
	HS_DRV_AMPLITUDE_5_PERCENT,
	HS_DRV_AMPLITUDE_75_PERCENT,
};
#ifdef CONFIG_USB_MSM_OTG_72K
static struct regulator *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400000;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075000;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = regulator_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		regulator_set_voltage(vreg_3p3, def_vol, def_vol);
	} else
		regulator_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return regulator_enable(vreg_3p3);
	else
		return regulator_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return regulator_set_voltage(vreg_3p3, mV*1000, mV*1000);
}
#endif

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init);
#endif
static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect = hsusb_rpc_connect,

#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
#else
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
#ifdef CONFIG_SEMC_CHARGER_USB_ARCH
	.chg_vbus_draw		 = semc_charger_usb_vbus_draw,
	.chg_connected		 = semc_charger_usb_connected,
	.chg_init		 = semc_charger_usb_init,
#else
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
#endif
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_set_voltage	 = msm_hsusb_ldo_set_voltage,
//	.bam_disable		 = 1,
#ifdef CONFIG_SEMC_CHARGER_CRADLE_ARCH
	.is_cradle_connected	 = semc_charger_cradle_is_connected,
#endif
#if defined(CONFIG_CHARGER_BQ24185)
	.chg_is_initialized	 = bq24185_charger_initialized,
#endif
//	.pmic_vbus_notif_init	 = msm_hsusb_pmic_vbus_notif_init,
	.phy_can_powercollapse	 = 1,
	.chg_drawable_ida	 	= USB_IDCHG_MAX,
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
#endif
#ifndef CONFIG_USB_EHCI_MSM_72K
typedef void (*notify_vbus_state) (int);
notify_vbus_state notify_vbus_state_func_ptr;
int vbus_on_irq;
static irqreturn_t pmic_vbus_on_irq(int irq, void *data)
{
	pr_info("%s: vbus notification from pmic\n", __func__);

	(*notify_vbus_state_func_ptr) (1);

	return IRQ_HANDLED;
}
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		if (!callback)
			return -ENODEV;

		notify_vbus_state_func_ptr = callback;
		vbus_on_irq = platform_get_irq_byname(&msm_device_otg,
			"vbus_on");
		if (vbus_on_irq <= 0) {
			pr_err("%s: unable to get vbus on irq\n", __func__);
			return -ENODEV;
		}

		ret = request_any_context_irq(vbus_on_irq, pmic_vbus_on_irq,
			IRQF_TRIGGER_RISING, "msm_otg_vbus_on", NULL);
		if (ret < 0) {
			pr_info("%s: request_irq for vbus_on"
				"interrupt failed\n", __func__);
			return ret;
		}
		msm_otg_pdata.pmic_vbus_irq = vbus_on_irq;
		return 0;
	} else {
		free_irq(vbus_on_irq, 0);
		notify_vbus_state_func_ptr = NULL;
		return 0;
	}
}
#endif

#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
static struct resource sii_uio_resources[] = {
	[0] = {
		.start  = MSM_GPIO_TO_INT(90),
		.end    = MSM_GPIO_TO_INT(90),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct uio_info sii_uio_pdata = {
	.name	=  "sii9024a_uio",
	.version = "0.001",
	.mem = {
		{
			.memtype = UIO_MEM_NONE,
			.size    = 0
		}
	},
	.irq       = MSM_GPIO_TO_INT(90),
	.irq_flags = IRQF_TRIGGER_LOW,
};

static struct platform_device sii_uio_dev = {
	.name           = "uio_pdrv_genirq",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(sii_uio_resources),
	.resource       = sii_uio_resources,
	.dev            = {
		.platform_data = &sii_uio_pdata,
	},
};
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {
		.platform_data = &android_pmem_pdata,
	},
};

#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
static struct lcdc_platform_data dtv_pdata = {
};
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
//	.rx_to_inject = 0xFD,
};

static struct resource msm_fb_resources[] = {
	{
		.flags = IORESOURCE_DMA,
	},
};

#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
static int msm_fb_detect_panel(const char *name)
{
	if (!strcmp(name, "sii9024a")) {
		printk(KERN_ERR
			"[HDMI] msm_fb_detect_panel() : name(%s)\n", name);
		return 0;
	}
	return -ENODEV;
}
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */

static struct msm_fb_platform_data msm_fb_pdata = {
#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
	.detect_client = msm_fb_detect_panel,
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name = "msm_fb",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_fb_resources),
	.resource = msm_fb_resources,
	.dev = {
		.platform_data = &msm_fb_pdata,
	},
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI0,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name = "pmem_camera",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI0,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = {.platform_data = &android_pmem_camera_pdata},
};

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	//.mddi_power_save = display_common_power,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

int mdp_core_clk_rate_table[] = {
	122880000,
	122880000,
	192000000,
	192000000,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_core_clk_rate = 192000000,
	//.mdp_core_clk_table = mdp_core_clk_rate_table,
	//.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
	//.mdp_rev = MDP_REV_40,
	//.mem_hid = MEMTYPE_EBI0,
};

#ifdef CONFIG_SEMC_ONESEG_TUNER_PM
struct oneseg_tunerpm_platform_data oneseg_tunerpm_data = {
	.gpio_rst = D_ONESEG_DEVICE_PORT_RESET,
	.gpio_pwr = D_ONESEG_DEVICE_PORT_POWER,
};


struct platform_device oneseg_tunerpm_device = {
	.name = D_ONESEG_TUNERPM_DRIVER_NAME,
	.id = 0,
	.dev  = {
		.platform_data = &oneseg_tunerpm_data,
	},
};
#endif

#ifdef CONFIG_BT
static uint32_t bt_config_on_gpios[] = {
	GPIO_CFG(134, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
	GPIO_CFG(135, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(136, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(137, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
	GPIO_CFG(103, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t bt_config_off_gpios[] = {
	GPIO_CFG(134, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(135, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(136, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(137, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(103, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static int bluetooth_power(int on)
{
	if (on) {
		config_gpio_table(bt_config_on_gpios,
				  ARRAY_SIZE(bt_config_on_gpios));
		gpio_set_value(103, 1);
	} else {
		gpio_set_value(103, 0);
		config_gpio_table(bt_config_off_gpios,
				  ARRAY_SIZE(bt_config_off_gpios));
	}
	return 0;
}

static struct platform_device mogami_device_rfkill = {
	.name = "mogami-rfkill",
	.dev.platform_data = &bluetooth_power,
};
#endif

#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
#define PLUG_DET_ENA_PIN 80
#define PLUG_DET_READ_PIN 26
#define MODE_SWITCH_PIN -1

int simple_remote_pf_initialize_gpio(struct simple_remote_platform_data *data)
{
	int err = 0;
	int i;

	if (!data || -1 == data->headset_detect_enable_pin) {
		printk(KERN_ERR
		       "*** %s - Error: Invalid inparameter (GPIO Pins)."
		       " Aborting!\n", __func__);
		return -EIO;
	}

	err = gpio_request(data->headset_detect_enable_pin,
			   "Simple_remote_plug_detect_enable");
	if (err) {
		printk(KERN_CRIT "%s: Error %d - Request hs_detect_enable pin",
		       __func__, err);
		goto out;
	}

	err = gpio_direction_output(data->headset_detect_enable_pin, 1);
	if (err) {
		printk(KERN_CRIT "%s: Error %d - Set hs_detect_enable pin"
		       " as output high\n", __func__, err);
		goto out_hs_det_enable;
	}

	err = gpio_request(data->headset_detect_read_pin,
			   "Simple_remote_plug_detect_read");
	if (err) {
		printk(KERN_CRIT "%s - Error %d - Request hs-detect_read pin",
		       __func__, err);
		goto out_hs_det_enable;
	}

	err = gpio_direction_input(data->headset_detect_read_pin);
	if (err) {
		printk(KERN_CRIT "%s - Error %d - Set hs-detect pin as input\n",
		       __func__, err);
		goto out_hs_det_read;
	}

	if (0 < data->headset_mode_switch_pin) {
		err = gpio_request(data->headset_mode_switch_pin,
				   "Simple_remote_headset_mode_switch");
		if (err) {
			printk(KERN_CRIT
			       "%s - Error %d - Request hs-mode_switch pin",
			       __func__, err);
			goto out_hs_det_read;
		}

		err = gpio_direction_output(data->headset_mode_switch_pin, 0);
		if (err) {
			printk(KERN_CRIT
			       "%s - Error %d - Set hs-mode_switch pin as "
			       "input\n", __func__, err);
			goto out_hs_mode_switch;
		}
	}

	for (i = 0; i < data->num_regs; i++) {
		data->regs[i].reg = vreg_get(NULL, data->regs[i].name);
		if (IS_ERR(data->regs[i].reg)) {
			printk(KERN_ERR "%s - Failed to find regulator %s\n",
			       __func__, data->regs[i].name);
			err = PTR_ERR(data->regs[i].reg);
			if (0 <= data->headset_mode_switch_pin)
				goto out_hs_mode_switch;
			else
				goto out_hs_det_read;
		}
	}

	return err;

out_hs_mode_switch:
	gpio_free(data->headset_mode_switch_pin);

out_hs_det_read:
	gpio_free(data->headset_detect_read_pin);

out_hs_det_enable:
	gpio_free(data->headset_detect_enable_pin);
out:
	return err;
}

void simple_remote_pf_deinitialize_gpio(
	struct simple_remote_platform_data *data)
{
	gpio_free(data->headset_detect_read_pin);
	gpio_free(data->headset_detect_enable_pin);
}

static struct simple_remote_platform_regulators regs[] =  {
	{
		.name = "ncp",
	},
	{
		.name = "s3",
	},
	{
		.name = "s2",
	},

};

static struct simple_remote_platform_data simple_remote_pf_data = {
	.headset_detect_enable_pin = PLUG_DET_ENA_PIN,
	.headset_detect_read_pin = PLUG_DET_READ_PIN,
	.headset_mode_switch_pin = MODE_SWITCH_PIN,
	.initialize = &simple_remote_pf_initialize_gpio,
	.deinitialize = &simple_remote_pf_deinitialize_gpio,

	.regs = regs,
	.num_regs = ARRAY_SIZE(regs),

	.controller = PM_HSED_CONTROLLER_1,

#ifdef CONFIG_SIMPLE_REMOTE_INVERT_PLUG_DETECTION_STATE
	.invert_plug_det = 1,
#else
	.invert_plug_det = 0,
#endif
};

static struct platform_device simple_remote_pf_device = {
	.name = SIMPLE_REMOTE_PF_NAME,
	.dev = {
		.platform_data = &simple_remote_pf_data,
	},
};
#endif

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
	msm_fb_register_device("dtv", &dtv_pdata);
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define MSM_RAM_CONSOLE_START   (0x50000000 - MSM_RAM_CONSOLE_SIZE)
#define MSM_RAM_CONSOLE_SIZE    (128 * SZ_1K)

static struct resource ram_console_resources[] = {
	[0] = {
		.start  = 0,//MSM_RAM_CONSOLE_START,
		.end    = 0,//MSM_RAM_CONSOLE_START+MSM_RAM_CONSOLE_SIZE-1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resources),
	.resource       = ram_console_resources,
};
#endif

#ifdef CONFIG_MSM_SDIO_AL
static struct msm_gpio mdm2ap_status = {
	GPIO_CFG(77, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"mdm2ap_status"
};

static int configure_mdm2ap_status(int on)
{
	if (on)
		return msm_gpios_request_enable(&mdm2ap_status, 1);
	else {
		msm_gpios_disable_free(&mdm2ap_status, 1);
		return 0;
	}
}

static int get_mdm2ap_status(void)
{
	return gpio_get_value(GPIO_PIN(mdm2ap_status.gpio_cfg));
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.allow_sdioc_version_major_2 = 1,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0003,
};

struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};
#endif /* CONFIG_MSM_SDIO_AL */

#ifdef CONFIG_PMIC_TIME
static struct platform_device pmic_time_device = {
	.name = "pmic_time",
};
#endif

static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
	&msm_proccomm_regulator_dev,
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#if defined (CONFIG_SND_MSM_MVS_DAI_SOC)
	&asoc_msm_mvs,
	&asoc_mvs_dai0,
	&asoc_mvs_dai1,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
	&qsd_device_spi,
	&msm_device_ssbi_pmic1,
	&msm_device_ssbi7,
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
#ifdef CONFIG_FB_MSM_HDMI_SII9024A_PANEL
	&hdmi_sii9024a_panel_device,
	&sii_uio_dev,
#endif /* CONFIG_FB_MSM_HDMI_SII9024A_PANEL */
	&android_pmem_adsp_device,
	&android_pmem_camera_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&msm_device_uart_dm1,
	&semc_rpc_handset_device,
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
	&msm_device_adspdec,
	&qup_device_i2c,
	&msm_kgsl_3d0,
	&msm_kgsl_2d0,
#ifdef CONFIG_SEMC_CAMERA_MODULE
	&msm_camera_sensor_semc_camera,
#endif
#ifdef CONFIG_SEMC_SUB_CAMERA_MODULE
	&msm_camera_sensor_semc_sub_camera,
#endif
	&msm_device_uart3,
	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_SEMC_VPE1)
	&semc_vpe1_device,
#endif

#ifdef CONFIG_MOGAMI_SLIDER
	&slider_device_mogami,
#endif
#ifdef CONFIG_BT
	&mogami_device_rfkill,
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif
	&bdata_driver,
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
	/*      &msm_batt_device, */
	/*      &msm_adc_device, */
	/*      &msm_ebi0_thermal, */
	/*      &msm_ebi1_thermal, */
#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
	&simple_remote_pf_device,
#endif
	&novatek_device,
	&battery_chargalg_platform_device,
#if defined(CONFIG_FB_MSM_MDDI_SONY_HVGA_LCD)
	&mddi_sony_hvga_display_device,
#endif
#if defined(CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD)
	&mddi_hitachi_hvga_display_device,
#endif
#if defined(CONFIG_FB_MSM_MDDI_SII_HVGA_LCD)
	&mddi_sii_hvga_display_device,
#endif
#if defined(CONFIG_FB_MSM_MDDI_AUO_HVGA_LCD)
	&mddi_auo_hvga_display_device,
#endif
#ifdef CONFIG_PMIC_TIME
	&pmic_time_device,
#endif /* CONFIG_PMIC_TIME */
#ifdef CONFIG_FPC_CONNECTOR_TEST
	&fpc_test_device,
#endif
#ifdef CONFIG_MSM_SDIO_AL
	&msm_device_sdio_al,
#endif
#ifdef CONFIG_SEMC_ONESEG_TUNER_PM
	&oneseg_tunerpm_device,
#endif
#ifdef CONFIG_SEMC_MOGAMI_FELICA_SUPPORT
	&semc_mogami_felica_device,
#endif
#ifdef CONFIG_SEMC_MOGAMI_IRDA
	&irda_mogami_device,
#endif
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id * 2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id * 2];
	msm_gpios_enable(msm_i2c_table, 2);
}

static struct regulator *qup_vreg;
static void qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
#if !defined(CONFIG_SEMC_CAMERA_MODULE) && \
	!defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	if (qup_vreg) {
		int rc = vreg_set_level(qup_vreg, 1800);
		if (rc) {
			pr_err("%s: vreg LVS1 set level failed (%d)\n",
			       __func__, rc);
		}
		rc = vreg_enable(qup_vreg);
		if (rc) {
			pr_err("%s: vreg_enable() = %d \n",
				__func__, rc);
		}
	}
#endif
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, msm_i2c_gpios_hw_size))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex = 0,
	.rsl_id = "D:I2C02000022",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
#if defined(CONFIG_SEMC_CAMERA_MODULE) || defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	.clk_freq = 100000,
#else
	.clk_freq = 384000,
#endif
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, qup_i2c_gpios_hw_size))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	qup_vreg = regulator_get(&qup_device_i2c.dev, "lvsw1");
	if (IS_ERR(qup_vreg)) {
		dev_err(&qup_device_i2c.dev,
			"%s: regulator_get failed: %ld\n",
			__func__, PTR_ERR(qup_vreg));
	}
}

static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
};

static struct msm_gpio sdc3_cfg_on_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc3_cfg_off_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), 	"sdc3_dat_0"},
};

#ifdef CONFIG_MMC_MSM_SDC4_LOW_DRIVE_STRENGTH
static struct msm_gpio sdc4_cfg_on_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), 	"sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), 	"sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), 	"sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), 	"sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), 	"sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), 	"sdc4_dat_0"},
};
#else
static struct msm_gpio sdc4_cfg_on_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	"sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	"sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	"sdc4_dat_0"},
};
#endif

#ifdef CONFIG_PMIC_GPIO_25
static struct msm_gpio sdc4_cfg_off_data[] = {
	{GPIO_CFG(58, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc4_clk"},
	{GPIO_CFG(59, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc4_cmd"},
	{GPIO_CFG(60, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc4_dat_0"},
};
#else
/* uSD is never turned off. Same configuration is used for ON and OFF */
static struct msm_gpio sdc4_cfg_off_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	"sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	"sdc4_dat_0"},
};
#endif

static struct sdcc_gpio sdcc_cfg_on_data[] = {
	{
	 .cfg_data = NULL,
	 .size = 0,
	},
	{
	 .cfg_data = NULL,
	 .size = 0,
	},
	{
	 .cfg_data = sdc3_cfg_on_data,
	 .size = ARRAY_SIZE(sdc3_cfg_on_data),
	},
	{
	 .cfg_data = sdc4_cfg_on_data,
	 .size = ARRAY_SIZE(sdc4_cfg_on_data),
	},
};

static struct sdcc_gpio sdcc_cfg_off_data[] = {
	{
	 .cfg_data = NULL,
	 .size = 0,
	},
	{
	 .cfg_data = NULL,
	 .size = 0,
	},
	{
	 .cfg_data = sdc3_cfg_off_data,
	 .size = ARRAY_SIZE(sdc3_cfg_off_data),
	},
	{
	 .cfg_data = sdc4_cfg_off_data,
	 .size = ARRAY_SIZE(sdc4_cfg_off_data),
	},
};

static unsigned long gpio_sts;
static unsigned wifi_init_gpio_en[] = {
	GPIO_CFG(57, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* WLAN EN */
};

static void wlan_init_seq(void)
{
	int rc;
	rc = gpio_tlmm_config(wifi_init_gpio_en[0], GPIO_CFG_ENABLE);

	/* If we fail here print error and continue, this will result in */
	/* higher power consumption but if gpio_tlmm_config() really fails */
	/* than we have far bigger issues as this is the base call for */
	/* config of gpio's */
	if (rc)
		printk(KERN_ERR
		       "%s: gpio_tlmm_config(%#x)=%d\n",
		       __func__, wifi_init_gpio_en[0], rc);

	/* Set device in low VIO-leakage state according to spec */
	/* This is done by toggle WLAN_EN OFF/ON/OFF (pulse width > 10ms) */
	gpio_set_value(57, 0);
	mdelay(1);
	gpio_set_value(57, 1);
	mdelay(12);
	gpio_set_value(57, 0);
}

static int cdcc_pm_gpio_config(unsigned int enable)
{
	int rc = 0;
#ifdef CONFIG_PMIC_GPIO_25
	/*Set PMIC_GPIO_25 to Out, no_pull*/
	struct pm8xxx_gpio_init_info pmic_gpio_25 = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
		{
			.direction = PM_GPIO_DIR_OUT,
			.pull = PM_GPIO_PULL_NO,
			.vin_sel = PM8058_GPIO_VIN_L5,
			.out_strength = PM_GPIO_STRENGTH_HIGH,
			.function = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol = 0,
			.output_value   = 0,
		},
	};
	if (enable)
		pr_info("%s: uSD power on\n", __func__);
	else
		pr_info("%s: uSD power off\n", __func__);
	/* Set output_value to low if enabling power and
	 * high if disabling power
	 */
	pmic_gpio_25.config.output_value = !enable;
	rc = pm8xxx_gpio_config(pmic_gpio_25.gpio, &pmic_gpio_25.config);
	if (rc)
		pr_err("%s: Config PMIC_GPIO_SDC4_PWR_EN_N failed\n", __func__);
#endif
	return rc;
}

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	if ((dev_id > 0) &&
	    (dev_id <= ARRAY_SIZE(sdcc_cfg_off_data))) {
		curr = enable ? &sdcc_cfg_on_data[dev_id - 1] :
				&sdcc_cfg_off_data[dev_id - 1];
	} else {
		pr_err("%s: Incorrect device id %d\n", __func__, dev_id);
		goto out;
	}

	if (!curr->cfg_data)
		goto out;

	if (!(test_bit(dev_id, &gpio_sts) ^ enable))
		goto out;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc) {
			/* Restore gpio_sts to power off */
			clear_bit(dev_id, &gpio_sts);
			pr_err("%s: Failed to turn on GPIOs for slot %d\n",
			       __func__, dev_id);
		}
		/* uSD power on */
		if ((dev_id == 4) && !rc) {
			if (cdcc_pm_gpio_config(enable)) {
				/* Restore gpio_sts to power off */
				clear_bit(dev_id, &gpio_sts);
				msm_gpios_free(curr->cfg_data, curr->size);
			}
		}
	} else {
		clear_bit(dev_id, &gpio_sts);
		/* uSD power off */
		if (dev_id == 4) {
			if (cdcc_pm_gpio_config(enable)) {
				/* Restore gpio_sts to power on */
				set_bit(dev_id, &gpio_sts);
				msm_gpios_free(curr->cfg_data, curr->size);
				goto out;
			}
		}
		rc = msm_gpios_enable(curr->cfg_data, curr->size);
		if (rc) {
			/* Restore gpio_sts to power on */
			set_bit(dev_id, &gpio_sts);
			pr_err("%s: Failed to turn off GPIOs for slot %d\n",
			       __func__, dev_id);
		}
		msm_gpios_free(curr->cfg_data, curr->size);
#ifdef CONFIG_PMIC_GPIO_25
		if (dev_id == 4) {
			/*
			 * 200 milliseconds delay should be sufficient to allow
			 * microSD reaches zero voltage when uSD is power off.
			 */
			msleep(200);
		}
#endif

	}
out:
	return 0;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	return msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
}

static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
	return (unsigned int)
	    !gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS
				    (PMIC_GPIO_SD_DET - 1));
}

/* Wifi chip power control */
static uint32_t wifi_setup_power(struct device *dv, unsigned int vdd)
{
	uint32_t ret = msm_sdcc_setup_power(dv, vdd);
	if (vdd)
		mogami_wifi_power(1);
	else
		mogami_wifi_power(0);
	return ret;
}

static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd = wifi_setup_power,
	.mmc_bus_width = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
	.msmsdcc_fmin = 144000,
	.msmsdcc_fmid = 24576000,
	.msmsdcc_fmax = 49152000,
	.nonremovable = 1,
};

static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd = msm_sdcc_setup_power,
	.mmc_bus_width = MMC_CAP_4_BIT_DATA,
	.status = msm7x30_sdcc_slot_status,
	.status_irq = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, PMIC_GPIO_SD_DET - 1),
	.irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.msmsdcc_fmin = 144000,
	.msmsdcc_fmid = 24576000,
	.msmsdcc_fmax = 49152000,
	.nonremovable = 0,
};

static void __init msm7x30_init_mmc(void)
{
	msm_add_sdcc(3, &msm7x30_sdc3_data);

	msm_add_sdcc(4, &msm7x30_sdc4_data);
}

static struct msm_gpio uart3_config_data[] = {
	{ GPIO_CFG(53, 1, GPIO_CFG_INPUT,   GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "UART3_Rx"},
	{ GPIO_CFG(54, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "UART3_Tx"},
};

static void msm7x30_init_uart3(void)
{
	msm_gpios_request_enable(uart3_config_data,
				 ARRAY_SIZE(uart3_config_data));

}

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(37, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_pclk = "iface_clk",
	.tsif_ref_clk = "ref_clk",
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

/*
 * Temporary place for hardware initialization until the devices in question
 * gets proper drivers
 */
static void __init mogami_temp_fixups(void)
{
	vreg_helper_off("gp3");	/* L0 */
	vreg_helper_off("gp5");	/* L23 */

	gpio_set_value(46, 1);	/* SPI_CS0_N */
	gpio_set_value(134, 1);	/* UART1DM_RFR_N */
	gpio_set_value(137, 1);	/* UART1DM_TXD */
}

static void __init shared_vreg_on(void)
{
	vreg_helper_on(VREG_L20, 3050);
	vreg_helper_on(VREG_L10, 2600);
	vreg_helper_on(VREG_L15, 2900);
	vreg_helper_on(VREG_L8, 1800);
}

static void __init msm7x30_init(void)
{
	unsigned smem_size;
//	uint32_t soc_version = 0;

//	soc_version = socinfo_get_version();
	wlan_init_seq();
	msm_clock_init(&msm7x30_clock_init_data);
	mogami_temp_fixups();

	msm7x30_init_uart3();

	msm_spm_init(&msm_spm_data, 1);
	acpuclk_init(&acpuclk_7x30_soc_data);

//	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
//			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
//		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
//		msm_otg_pdata.ldo_set_voltage = 0;
//	}
	semc_chg_usb_set_supplicants(semc_chg_usb_supplied_to,
				  ARRAY_SIZE(semc_chg_usb_supplied_to));
	if (0 <= CONFIG_USB_HS_DRV_AMPLITUDE ||
		CONFIG_USB_HS_DRV_AMPLITUDE < ARRAY_SIZE(hs_drv_ampl_ratio))
		msm_otg_pdata.drv_ampl =
			hs_drv_ampl_ratio[CONFIG_USB_HS_DRV_AMPLITUDE];
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_otg_pdata.swfi_latency =
	    msm_pm_data
	    [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;

	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(136);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
	buses_init();
#ifdef CONFIG_MSM_SSBI
	msm_device_ssbi_pmic1.dev.platform_data =
				&msm7x30_ssbi_pm8058_pdata;
#endif
	platform_add_devices(msm_footswitch_devices,
			     msm_num_footswitch_devices);
	platform_add_devices(devices, ARRAY_SIZE(devices));

	msm_add_host(0, &msm_usb_host_pdata);

	msm7x30_init_mmc();
	msm_qsd_spi_init();

#ifdef CONFIG_BT
	bluetooth_power(0);
#endif
#ifdef CONFIG_BOSCH_BMA150
	sensors_ldo_init();
#endif
	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
	msm7x30_init_marimba();

#ifdef CONFIG_MSM7KV2_AUDIO
	snddev_poweramp_gpio_init();
	snddev_hsed_voltage_init();
	aux_pcm_gpio_init();
#endif
	hw_id_class_init();
	shared_vreg_on();
#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300_SPI
	cypress_touch_gpio_init();
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA300_SPI */
	msm_init_pmic_vibrator();

	i2c_register_board_info(0, msm_i2c_board_info,
				ARRAY_SIZE(msm_i2c_board_info));

	i2c_register_board_info(2, msm_marimba_board_info,
				ARRAY_SIZE(msm_marimba_board_info));

	i2c_register_board_info(4 /* QUP ID */, mogami_qup_i2c_devices,
			ARRAY_SIZE(mogami_qup_i2c_devices));

	spi_register_board_info(spi_board_info,
		ARRAY_SIZE(spi_board_info));

	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;

	pm8058_gpios_init();

#if defined(CONFIG_FB_MSM_MDDI_SONY_HVGA_LCD) || \
	defined(CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD) || \
	defined(CONFIG_FB_MSM_MDDI_SII_HVGA_LCD)
	semc_mogami_lcd_power_on(11, 2, 21);
#endif
#if defined(CONFIG_FB_MSM_MDDI_AUO_HVGA_LCD)
	semc_mogami_lcd_power_on(2, 21, 51);
#endif

	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_camera_size = MSM_PMEM_CAMERA_SIZE;
static int __init pmem_camera_size_setup(char *p)
{
	pmem_camera_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_camera_size", pmem_camera_size_setup);

static unsigned pmem_kernel_ebi0_size = PMEM_KERNEL_EBI0_SIZE;
static int __init pmem_kernel_ebi0_size_setup(char *p)
{
	pmem_kernel_ebi0_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi0_size", pmem_kernel_ebi0_size_setup);

static struct memtype_reserve msm7x30_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_camera_pdata.size = pmem_camera_size;
	android_pmem_pdata.size = pmem_sf_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x30_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_camera_pdata);
	reserve_memory_for(&android_pmem_pdata);
	msm7x30_reserve_table[MEMTYPE_EBI0].size += pmem_kernel_ebi0_size;
#endif
}

static void __init msm7x30_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x30_paddr_to_memtype(unsigned int paddr)
{
	if (paddr < phys_add)
		return MEMTYPE_EBI0;
	if (paddr >= phys_add && paddr < 0x80000000)
		return MEMTYPE_EBI1;
	return MEMTYPE_NONE;
}

static struct reserve_info msm7x30_reserve_info __initdata = {
	.memtype_reserve_table = msm7x30_reserve_table,
	.calculate_reserve_sizes = msm7x30_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x30_paddr_to_memtype,
};

static void __init msm7x30_reserve(void)
{
	reserve_info = &msm7x30_reserve_info;
	msm_reserve();
}

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	/* RAM Console can't use alloc_bootmem(), since that zeroes the
	 * region */
	size = MSM_RAM_CONSOLE_SIZE;
	ram_console_resources[0].start = msm_fb_resources[0].end+1;
	ram_console_resources[0].end = ram_console_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at (%lx physical) for ram console\n",
		size, (unsigned long)ram_console_resources[0].start);
	/* We still have to reserve it, though */
	reserve_bootmem(ram_console_resources[0].start,size,0);
#endif
}

static void __init msm7x30_map_io(void)
{
	msm_shared_ram_phys = 0x00100000;
	msm_map_msm7x30_io();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
}

static void __init msm7x30_init_early(void)
{
	msm7x30_allocate_memory_regions();
}

static void __init msm7x30_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 3;
	mi->bank[0].start = DDR0_BANK_BASE;
	mi->bank[0].size = DDR0_BANK_SIZE;
	mi->bank[1].start = DDR1_BANK_BASE;
	mi->bank[1].size = DDR1_BANK_SIZE;
	mi->bank[2].start = DDR2_BANK_BASE;
	mi->bank[2].size = DDR2_BANK_SIZE;
}

MACHINE_START(SEMC_MOGAMI, "mogami")
	.boot_params = PLAT_PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
	.fixup = msm7x30_fixup,
MACHINE_END
