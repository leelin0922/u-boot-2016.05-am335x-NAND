/*
 * board.c
 * 
 * Board functions for TI AM335X based boards
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <libfdt.h>
#include <spl.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <miiphy.h>
#include <cpsw.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <environment.h>
#include <watchdog.h>
#include "board.h"
#include <common.h>
#include <autoboot.h>
#include "../../../drivers/lcd/rasterDisplay.h"
#define BITMAP_LOGO
#ifdef BITMAP_LOGO
#include "../../../drivers/lcd/image.h"
#endif
DECLARE_GLOBAL_DATA_PTR;

/* GPIO that controls power to DDR on EVM-SK */
#define GPIO_TO_PIN(bank, gpio)        (32 * (bank) + (gpio))
#define GPIO_DDR_VTT_EN		7
#define DIP_S1			44
#define logo_high	(180)
#define logo_width	(300)

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;
char membuf[1280*800*4+32]={0};
static int progress_position=(480*3/4);

#if 0
static int baltos_set_console(void)
{
	int val, i, dips = 0;
	char buf[7];

	for (i = 0; i < 4; i++) {
		sprintf(buf, "dip_s%d", i + 1);

		if (gpio_request(DIP_S1 + i, buf)) {
			printf("failed to export GPIO %d\n", DIP_S1 + i);
			return 0;
		}

		if (gpio_direction_input(DIP_S1 + i)) {
			printf("failed to set GPIO %d direction\n", DIP_S1 + i);
			return 0;
		}

		val = gpio_get_value(DIP_S1 + i);
		dips |= val << i;
	}

	printf("DIPs: 0x%1x\n", (~dips) & 0xf);

	if ((dips & 0xf) == 0xe)
		setenv("console", "ttyUSB0,115200n8");

	return 0;
}
#endif

static int read_eeprom(BSP_VS_HWPARAM *header)
{
	i2c_set_bus_num(0);

	/* Check if baseboard eeprom is available */
	if (i2c_probe(CONFIG_SYS_I2C_EEPROM_ADDR)) {
		puts("Could not probe the EEPROM; something fundamentally "
			"wrong on the I2C bus.\n");
		return -ENODEV;
	}

	/* read the eeprom using i2c */
	if (i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0, 1, (uchar *)header,
		     sizeof(BSP_VS_HWPARAM))) {
		puts("Could not read the EEPROM; something fundamentally"
			" wrong on the I2C bus.\n");
		return -EIO;
	}

	if (header->Magic != 0xDEADBEEF) {

		printf("Incorrect magic number (0x%x) in EEPROM\n",
				header->Magic);

		/* fill default values */
		header->SystemId = 211;
		header->MAC1[0] = 0x00;
		header->MAC1[1] = 0x00;
		header->MAC1[2] = 0x00;
		header->MAC1[3] = 0x00;
		header->MAC1[4] = 0x00;
		header->MAC1[5] = 0x01;

		header->MAC2[0] = 0x00;
		header->MAC2[1] = 0x00;
		header->MAC2[2] = 0x00;
		header->MAC2[3] = 0x00;
		header->MAC2[4] = 0x00;
		header->MAC2[5] = 0x02;

		header->MAC3[0] = 0x00;
		header->MAC3[1] = 0x00;
		header->MAC3[2] = 0x00;
		header->MAC3[3] = 0x00;
		header->MAC3[4] = 0x00;
		header->MAC3[5] = 0x03;
	}

	return 0;
}

#if defined(CONFIG_SPL_BUILD) || defined(CONFIG_NOR_BOOT)

static const struct ddr_data ddr3_baltos_data = {
	.datardsratio0 = MT41K256M16HA125E_RD_DQS,
	.datawdsratio0 = MT41K256M16HA125E_WR_DQS,
	.datafwsratio0 = MT41K256M16HA125E_PHY_FIFO_WE,
	.datawrsratio0 = MT41K256M16HA125E_PHY_WR_DATA,
};

static const struct cmd_control ddr3_baltos_cmd_ctrl_data = {
	.cmd0csratio = MT41K256M16HA125E_RATIO,
	.cmd0iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

	.cmd1csratio = MT41K256M16HA125E_RATIO,
	.cmd1iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

	.cmd2csratio = MT41K256M16HA125E_RATIO,
	.cmd2iclkout = MT41K256M16HA125E_INVERT_CLKOUT,
};

static struct emif_regs ddr3_baltos_emif_reg_data = {
	.sdram_config = MT41K256M16HA125E_EMIF_SDCFG,
	.ref_ctrl = MT41K256M16HA125E_EMIF_SDREF,
	.sdram_tim1 = MT41K256M16HA125E_EMIF_TIM1,
	.sdram_tim2 = MT41K256M16HA125E_EMIF_TIM2,
	.sdram_tim3 = MT41K256M16HA125E_EMIF_TIM3,
	.zq_config = MT41K256M16HA125E_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41K256M16HA125E_EMIF_READ_LATENCY,
};

static const struct ddr_data ddr3_sbc7109_data = {
	.datardsratio0 = MT41K256M8DA125_RD_DQS,
    .datawdsratio0 = MT41K256M8DA125_WR_DQS,
    .datafwsratio0 = MT41K256M8DA125_PHY_FIFO_WE,
    .datawrsratio0 = MT41K256M8DA125_PHY_WR_DATA,
};

static const struct cmd_control ddr3_sbc7109_cmd_ctrl_data = {
    .cmd0csratio = MT41K256M8DA125_RATIO,
    .cmd0iclkout = MT41K256M8DA125_INVERT_CLKOUT,

    .cmd1csratio = MT41K256M8DA125_RATIO,
    .cmd1iclkout = MT41K256M8DA125_INVERT_CLKOUT,

    .cmd2csratio = MT41K256M8DA125_RATIO,
    .cmd2iclkout = MT41K256M8DA125_INVERT_CLKOUT,
};

static struct emif_regs ddr3_sbc7109_emif_reg_data = {
	.sdram_config = MT41K256M8DA125_EMIF_SDCFG,
	.ref_ctrl = MT41K256M8DA125_EMIF_SDREF,
    .sdram_tim1 = MT41K256M8DA125_EMIF_TIM1,
    .sdram_tim2 = MT41K256M8DA125_EMIF_TIM2,
    .sdram_tim3 = MT41K256M8DA125_EMIF_TIM3,
    .zq_config = MT41K256M8DA125_ZQ_CFG,
    .emif_ddr_phy_ctlr_1 = MT41K256M8DA125_EMIF_READ_LATENCY,
};

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	return (serial_tstc() && serial_getc() == 'c');
}
#endif

#define OSC	(V_OSCK/1000000)
const struct dpll_params dpll_ddr = {
		266, OSC-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_evm_sk = {
		303, OSC-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_baltos = {
		400, OSC-1, 1, -1, -1, -1, -1};

void am33xx_spl_board_init(void)
{
	int mpu_vdd;
	int sil_rev;

	/* Get the frequency */
	dpll_mpu_opp100.m = am335x_get_efuse_mpu_max_freq(cdev);

	/*
	 * The GP EVM, IDK and EVM SK use a TPS65910 PMIC.  For all
	 * MPU frequencies we support we use a CORE voltage of
	 * 1.1375V.  For MPU voltage we need to switch based on
	 * the frequency we are running at.
	 */

	printf("I2C speed: %d Hz\n", CONFIG_SYS_OMAP24_I2C_SPEED);

	if (i2c_probe(TPS65910_CTRL_I2C_ADDR)) {
		puts("i2c: cannot access TPS65910\n");
		return;
	}

	/*
	 * Depending on MPU clock and PG we will need a different
	 * VDD to drive at that speed.
	 */
	sil_rev = readl(&cdev->deviceid) >> 28;
	mpu_vdd = am335x_get_tps65910_mpu_vdd(sil_rev,
					      dpll_mpu_opp100.m);

	/* Tell the TPS65910 to use i2c */
	tps65910_set_i2c_control();

	/* First update MPU voltage. */
	if (tps65910_voltage_update(MPU, mpu_vdd))
		return;

	/* Second, update the CORE voltage. */
	if (tps65910_voltage_update(CORE, TPS65910_OP_REG_SEL_1_1_3))
		return;

	/* Set CORE Frequencies to OPP100 */
	do_setup_dpll(&dpll_core_regs, &dpll_core_opp100);

	/* Set MPU Frequency to what we detected now that voltages are set */
	do_setup_dpll(&dpll_mpu_regs, &dpll_mpu_opp100);
}

const struct dpll_params *get_dpll_ddr_params(void)
{
	enable_i2c0_pin_mux();
    i2c_set_bus_num(0);
    gpio_direction_output(GPIO_TO_PIN(0,22),1); //COM0_MODE_0=1
    gpio_direction_output(GPIO_TO_PIN(0,23),0); //COM0_MODE_1=0
    gpio_direction_output(GPIO_TO_PIN(0,19),0); //COM0_TERM=0
    gpio_direction_output(GPIO_TO_PIN(0,12),1); //LVDS_BLKT_ON=1
    gpio_direction_output(GPIO_TO_PIN(0, 7),0); //LVDS_BRIGHTNESS=1

    return &dpll_ddr_baltos;
}

void set_uart_mux_conf(void)
{
	enable_uart0_pin_mux();
}

void set_mux_conf_regs(void)
{
	enable_board_pin_mux();
}

const struct ctrl_ioregs ioregs_baltos = {
	.cm0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.cm1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.cm2ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.dt0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.dt1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
};

const struct ctrl_ioregs ioregs_sbc7109 = {
    .cm0ioctl       = MT41K256M8DA125_IOCTRL_VALUE,
    .cm1ioctl       = MT41K256M8DA125_IOCTRL_VALUE,
    .cm2ioctl       = MT41K256M8DA125_IOCTRL_VALUE,
    .dt0ioctl       = MT41K256M8DA125_IOCTRL_VALUE,
    .dt1ioctl       = MT41K256M8DA125_IOCTRL_VALUE,
};


void sdram_init(void)
{
    if (1)
        config_ddr(400, &ioregs_sbc7109,
                   &ddr3_sbc7109_data,
                   &ddr3_sbc7109_cmd_ctrl_data,
                   &ddr3_sbc7109_emif_reg_data, 0);
    else
        config_ddr(400, &ioregs_baltos,
                   &ddr3_baltos_data,
                   &ddr3_baltos_cmd_ctrl_data,
                   &ddr3_baltos_emif_reg_data, 0);
}
#endif

#if !defined(CONFIG_SPL_BUILD)
void lcd_gpio_setup(int gpio, char *name, int val)
{
	int ret;
	ret = gpio_request(gpio, name);
	if (ret < 0) {
		printf("%s: Unable to request %s\n", __func__, name);
		return;
	}
	ret = gpio_direction_output(gpio, 0);
	if (ret < 0) {
		printf("%s: Unable to set %s as output\n", __func__, name);
		goto err_free_gpio;
	}
	gpio_set_value(gpio, val);
	return;
err_free_gpio:
	gpio_free(gpio);
	return;
}

void lcdbacklight_off(int gpio)
{
	gpio_set_value(gpio,0);
	//Lcd_off();
	return;
}

void lcdbacklight_on(int gpio)
{
	gpio_set_value(gpio,1); //write 0 to turn on
	//Lcd_on();
	return;
}

void board_lcd_reset(int gpio)
{
	gpio_set_value(gpio,0);
	Lcd_reset();
	return;
}
#endif

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
	//puts("eeprom_i2c_init\n");
	eeprom_i2c_init();

#if defined(CONFIG_HW_WATCHDOG)
	hw_watchdog_init();
#endif

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
	gpmc_init();
#endif
	return 0;
}

void lcd_progress_complete(void)
{
	int i=0,j=0,k=0,offset=0;
	int screen_width=eeprom_i2c_get_width();
	int screen_height=eeprom_i2c_get_height();

	offset=progress_position*screen_width*4+(screen_width-logo_width)/2*4+32;
	for(i=0;i<5;i++)
	{
		for(j=0;j<logo_width;j++)
		{
			k=offset+screen_width*4*i+4*j;
			membuf[k+1]=0x00;
		}
	}	
	progress_position +=10;
}

void lcd_progress_index(int percent)
{
	int i=0,j=0,k=0,offset=0;
	int screen_width=eeprom_i2c_get_width();
	int screen_height=eeprom_i2c_get_height();
	int currentstep=logo_width*percent/100;

	offset=progress_position*screen_width*4+(screen_width-logo_width)/2*4+32;
	for(i=0;i<5;i++)
	{
		for(j=0;j<logo_width;j++)
		{
			//k=offset+800*4*i+4*j;
			k=offset+screen_width*4*i+4*j;
			if(j<currentstep)
			{
				membuf[k+0]=0xff;
				membuf[k+1]=0xff;
				//membuf[k+2]=0xff;
			}
			else
			{
				//memset(membuf+k,0x00,(400-j)*4);
				memset(membuf+k,0x00,(logo_width-j)*4);
				break;
			}
		}
	}	
}

void lcd_buffer_init(void)
{
	int size;
	unsigned int i=0,j=0,k=0,offset=0;
	int screen_width=eeprom_i2c_get_width();
	int screen_height=eeprom_i2c_get_height();

	//puts("lcd_buffer_init\n");
#if !defined(CONFIG_SPL_BUILD)
	lcd_gpio_setup(20, "lcdvcc", 1);
	lcd_gpio_setup(12, "lcdpwm", 1);
#endif
	size = screen_width*screen_height*4+32;
	memset(membuf, 0, size);
#ifndef BITMAP_LOGO
	membuf[1]=0x40;
	offset=(screen_height-logo_high)/2*screen_width*4+(screen_width-logo_width)/2*4+32;
	for(i=0;i<logo_high;i++)
	{
		for(j=0;j<logo_width;j++)
		{
			k=offset+screen_width*4*i+4*j;
			if(i<(logo_high/2))
			{
				if(j<(logo_width/2))
				{
					membuf[k+2]=0xff;
				}
				else
				{
					membuf[k+1]=0xff;
				}
			}
			else
			{
				if(j<(logo_width/2))
				{
					membuf[k+0]=0xff;
				}
				else
				{
					membuf[k+0]=0xff;
					membuf[k+1]=0xff;
				}
			}
		}
	}
#else
	memcpy(membuf,image,32);
	offset=(screen_height-logo_high)/2*screen_width*4+(screen_width-logo_width)/2*4+32;
	j=8;
	k=logo_width*4;
	for(i=0;i<logo_high;i++)
	{
		memcpy(membuf+offset+i*screen_width*4,image+j+i*logo_width,k);
	}
#endif

#if !defined(CONFIG_SPL_BUILD)
	//puts("Lcd_Init\n");
	Lcd_Init((unsigned int)membuf,(unsigned int)size);
	lcd_gpio_setup(7, "lcdbacklight", 1);
#endif
	progress_position=screen_height*3/4;
	//printf("screen_width= %d, screen_height= %d, progress_position=%d\n", screen_width, screen_height, progress_position);
}

int ft_board_setup(void *blob, bd_t *bd)
{
	int node, ret;
	unsigned char mac_addr[6];
	BSP_VS_HWPARAM header;

	/* get production data */
	if (read_eeprom(&header))
		return 0;

	/* setup MAC1 */
	mac_addr[0] = header.MAC1[0];
	mac_addr[1] = header.MAC1[1];
	mac_addr[2] = header.MAC1[2];
	mac_addr[3] = header.MAC1[3];
	mac_addr[4] = header.MAC1[4];
	mac_addr[5] = header.MAC1[5];


	node = fdt_path_offset(blob, "/ocp/ethernet/slave@4a100200");
	if (node < 0) {
		printf("no /soc/fman/ethernet path offset\n");
		return -ENODEV;
	}

	ret = fdt_setprop(blob, node, "mac-address", &mac_addr, 6);
	if (ret) {
		printf("error setting local-mac-address property\n");
		return -ENODEV;
	}

	/* setup MAC2 */
	mac_addr[0] = header.MAC2[0];
	mac_addr[1] = header.MAC2[1];
	mac_addr[2] = header.MAC2[2];
	mac_addr[3] = header.MAC2[3];
	mac_addr[4] = header.MAC2[4];
	mac_addr[5] = header.MAC2[5];

	node = fdt_path_offset(blob, "/ocp/ethernet/slave@4a100300");
	if (node < 0) {
		printf("no /soc/fman/ethernet path offset\n");
		return -ENODEV;
	}

	ret = fdt_setprop(blob, node, "mac-address", &mac_addr, 6);
	if (ret) {
		printf("error setting local-mac-address property\n");
		return -ENODEV;
	}

	printf("\nFDT was successfully setup\n");

	return 0;
}

#if 0
static struct module_pin_mux dip_pin_mux[] = {
	{OFFSET(gpmc_ad12), (MODE(7) | RXACTIVE )},	/* GPIO1_12 */
	{OFFSET(gpmc_ad13), (MODE(7)  | RXACTIVE )},	/* GPIO1_13 */
	{OFFSET(gpmc_ad14), (MODE(7)  | RXACTIVE )},	/* GPIO1_14 */
	{OFFSET(gpmc_ad15), (MODE(7)  | RXACTIVE )},	/* GPIO1_15 */
	{-1},
};
#endif

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
	//puts("board_late_init\n");
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	//BSP_VS_HWPARAM header;
	//char model[4];

	/* get production data */
	/*
	if (read_eeprom(&header)) {
		strcpy(model, "211");
	} else {
		sprintf(model, "%d", header.SystemId);
		if (header.SystemId == 215) {
			configure_module_pin_mux(dip_pin_mux);
			baltos_set_console();
		}
	}
	setenv("board_name", model);
	*/
#endif

	return 0;
}
#endif

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
static void cpsw_control(int enabled)
{
	/* VTP can be added here */

	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 0,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 7,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 2,
	.slave_data		= cpsw_slaves,
	.active_slave		= 1,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};
#endif

#if ((defined(CONFIG_SPL_ETH_SUPPORT) || defined(CONFIG_SPL_USBETH_SUPPORT)) \
		&& defined(CONFIG_SPL_BUILD)) || \
	((defined(CONFIG_DRIVER_TI_CPSW) || \
	  defined(CONFIG_USB_ETHER) && defined(CONFIG_USB_MUSB_GADGET)) && \
	 !defined(CONFIG_SPL_BUILD))
int board_eth_init(bd_t *bis)
{
	int rv, n = 0;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;
	__maybe_unused struct am335x_baseboard_id header;

	/*
	 * Note here that we're using CPSW1 since that has a 1Gbit PHY while
	 * CSPW0 has a 100Mbit PHY.
	 *
	 * On product, CPSW1 maps to port labeled WAN.
	 */

	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid1l);
	mac_hi = readl(&cdev->macid1h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");

		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

#ifdef CONFIG_DRIVER_TI_CPSW
	writel((GMII1_SEL_RMII | GMII2_SEL_RGMII | RGMII2_IDMODE), &cdev->miisel);
	cpsw_slaves[1].phy_if = PHY_INTERFACE_MODE_RGMII;
	rv = cpsw_register(&cpsw_data);
	if (rv < 0)
		printf("Error %d registering CPSW switch\n", rv);
	else
		n += rv;
#endif

	/*
	 *
	 * CPSW RGMII Internal Delay Mode is not supported in all PVT
	 * operating points.  So we must set the TX clock delay feature
	 * in the AR8051 PHY.  Since we only support a single ethernet
	 * device in U-Boot, we only do this for the first instance.
	 */
#define AR8051_PHY_DEBUG_ADDR_REG	0x1d
#define AR8051_PHY_DEBUG_DATA_REG	0x1e
#define AR8051_DEBUG_RGMII_CLK_DLY_REG	0x5
#define AR8051_RGMII_TX_CLK_DLY		0x100
	const char *devname;
	devname = miiphy_get_current_dev();

	miiphy_write(devname, 0x7, AR8051_PHY_DEBUG_ADDR_REG,
			AR8051_DEBUG_RGMII_CLK_DLY_REG);
	miiphy_write(devname, 0x7, AR8051_PHY_DEBUG_DATA_REG,
			AR8051_RGMII_TX_CLK_DLY);
#endif

#if defined(CONFIG_USB_ETHER) && \
    (!defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_USBETH_SUPPORT))
    if (is_valid_ethaddr(mac_addr))
        eth_setenv_enetaddr("usbnet_devaddr", mac_addr);

    rv = usb_eth_initialize(bis);
    if (rv < 0)
        printf("Error %d registering USB_ETHER\n", rv);
    else
        n += rv;
#endif

	return n;
}
#endif


#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
	//puts("misc_init_r\n");
#ifdef AUTO_UPDATESYS
    if(*((int *)0x80000000) == 8)
    {
		Load_config_from_mmc();
		lcd_buffer_init();
        run_command("run auto_update_nand", 0);
        while(1)
            udelay(1000);
    }
	else
	{
		//eeprom_i2c_init();
		Load_config_from_mmc();
		lcd_buffer_init();
		set_7109_env();
	}
#endif

    if(*((int *)0x80000000) == 8)
        setenv("bootdev", "MMC");
    else if(*((int *)0x80000000) == 5)
        setenv("bootdev", "NAND");
    else
        printf("Boot device don't exist\n");

    return 0;
}
#endif
