/*
 * (C) Copyright 2012 Michal Simek <monstr@monstr.eu>
 * (C) Copyright 2012 David Andrey <david.andrey@netmodule.com>
 *
 * based on Xilinx zynq_common/board.c
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <netdev.h>
#include <zynqpl.h>
#include <asm/arch/hardware.h>
#include <asm/arch/sys_proto.h>

#include <nand.h>
#include <i2c.h>
#include <phy.h>

#define BOOT_MODE_REG     (ZYNQ_SYS_CTRL_BASEADDR + 0x25C)
#define BOOT_MODES_MASK    0x0000000F
#define QSPI_MODE         (0x00000001)            /**< QSPI */
#define NOR_FLASH_MODE    (0x00000002)            /**< NOR  */
#define NAND_FLASH_MODE   (0x00000004)            /**< NAND */
#define SD_MODE           (0x00000005)            /**< Secure Digital card */
#define JTAG_MODE         (0x00000000)            /**< JTAG */

extern int zynq_nand_init(struct nand_chip *nand_chip, int devnum);

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_FPGA
static Xilinx_desc fpga;

/* It can be done differently */
static Xilinx_desc fpga010 = XILINX_XC7Z010_DESC(0x10);
static Xilinx_desc fpga020 = XILINX_XC7Z020_DESC(0x20);
static Xilinx_desc fpga030 = XILINX_XC7Z030_DESC(0x30);
static Xilinx_desc fpga045 = XILINX_XC7Z045_DESC(0x45);
static Xilinx_desc fpga100 = XILINX_XC7Z100_DESC(0x100);
#endif

#define __REG(x)        (*((volatile u32 *)(x)))

int board_init(void)
{
#ifdef CONFIG_FPGA
	u32 idcode;

	idcode = zynq_slcr_get_idcode();

	switch (idcode) {
	case XILINX_ZYNQ_7010:
		fpga = fpga010;
		break;
	case XILINX_ZYNQ_7020:
		fpga = fpga020;
		break;
	case XILINX_ZYNQ_7030:
		fpga = fpga030;
		break;
	case XILINX_ZYNQ_7045:
		fpga = fpga045;
		break;
	case XILINX_ZYNQ_7100:
		fpga = fpga100;
		break;
	}
#endif

	/* temporary hack to clear pending irqs before Linux as it
	 * will hang Linux
	 */
	writel(0x26d, 0xe0001014);

	/* temporary hack to take USB out of reset til the is fixed
	 * in Linux
	 */
	writel(0x80, 0xe000a204);
	writel(0x80, 0xe000a208);
	writel(0x80, 0xe000a040);
	writel(0x00, 0xe000a040);
	writel(0x80, 0xe000a040);
//	icache_enable();

#ifdef CONFIG_FPGA
	fpga_init();
	fpga_add(fpga_xilinx, &fpga);
#endif

	return 0;
}

int board_late_init (void)
{
    unsigned int fpgaVersion;
    unsigned int phyControl;

    puts("FPGA:  ");
    fpgaVersion = __REG(CONFIG_FPGA_VERSION_REG);
    printf("v%d.%d\n", (fpgaVersion & CONFIG_FPGA_VERSION_MAJOR_MASK) >> CONFIG_FPGA_VERSION_MAJOR_FSB,
                       (fpgaVersion & CONFIG_FPGA_VERSION_MINOR_MASK) >> CONFIG_FPGA_VERSION_MINOR_FSB);

#ifdef CONFIG_ZYNQ_GEM0
    /* Release Phy reset for eth0 */
    phyControl = __REG(CONFIG_FPGA_PHYCTRL_REG);
    __REG(CONFIG_FPGA_PHYCTRL_REG) = phyControl & ~CONFIG_FPGA_PHYCTRL_MODPHY_MASK;

    udelay(100);
#endif
#ifdef CONFIG_ZYNQ_GEM1
    /* Release Phy reset for eth1 */
    phyControl = __REG(CONFIG_FPGA_PHYCTRL_REG);
    __REG(CONFIG_FPGA_PHYCTRL_REG) = phyControl & ~CONFIG_FPGA_PHYCTRL_PHYD_MASK;
    udelay(100);
#endif
	return 0;
}

/**
 * Set pin muxing for NAND access
 */
static void set_mio_mux_nand( void ){
#define NANDMUX0 0x0000001610
#define NANDMUX  0x0000000610

	writel(0xDF0D, &slcr_base->slcr_unlock);	//unlock slcr

	/* Define MuxIO for NAND */
	/* Caution: overwrite some QSPI muxing !!! */
	writel(NANDMUX0, &slcr_base->mio_pin[0]);	/* Pin 0, NAND Flash Chip Select */
	writel(NANDMUX,  &slcr_base->mio_pin[2]);	/* Pin 2, NAND Flash ALEn */
	writel(NANDMUX,  &slcr_base->mio_pin[3]);	/* Pin 3, NAND WE_B */
	writel(NANDMUX,  &slcr_base->mio_pin[4]);	/* Pin 4, NAND Flash IO Bit 2 */
	writel(NANDMUX,  &slcr_base->mio_pin[5]);	/* Pin 5, NAND Flash IO Bit 0 */
	writel(NANDMUX,  &slcr_base->mio_pin[6]);	/* Pin 6, NAND Flash IO Bit 1 */
	writel(NANDMUX,  &slcr_base->mio_pin[7]);	/* Pin 7, NAND Flash CLE_B */
	writel(NANDMUX,  &slcr_base->mio_pin[8]);	/* Pin 8, NAND Flash RD_B */
	writel(NANDMUX,  &slcr_base->mio_pin[9]);	/* Pin 9, NAND Flash IO Bit 4 */
	writel(NANDMUX,  &slcr_base->mio_pin[10]);	/* Pin 10, NAND Flash IO Bit 5 */
	writel(NANDMUX,  &slcr_base->mio_pin[11]);	/* Pin 11, NAND Flash IO Bit 6 */
	writel(NANDMUX,  &slcr_base->mio_pin[12]);	/* Pin 12, NAND Flash IO Bit 7 */
	writel(NANDMUX,  &slcr_base->mio_pin[13]);	/* Pin 13, NAND Flash IO Bit 3 */
	writel(NANDMUX,  &slcr_base->mio_pin[14]);	/* Pin 14, NAND Flash Busy */

	writel(0x767B, &slcr_base->slcr_lock);		//lock slcr
}


/**
 * Set the pin muxing for QSPI NOR access
 */
static void set_mio_mux_qspi( void ){
#define QSPIMUX 0x0000000602

	writel(0xDF0D, &slcr_base->slcr_unlock);		//unlock slcr

	/* Define MuxIO for QSPI */
	/* Caution: overwrite some NAND muxing !!! */
	writel(0x00001600, &slcr_base->mio_pin[0]);		/* Pin 0, Level 3 Mux */
	writel(0x00001602, &slcr_base->mio_pin[1]);		/* Pin 1, Quad SPI 0 Chip Select */
	writel(QSPIMUX,    &slcr_base->mio_pin[2]);		/* Pin 2, Quad SPI 0 IO Bit 0 */
	writel(QSPIMUX,    &slcr_base->mio_pin[3]);		/* Pin 3, Quad SPI 0 IO Bit 1 */
	writel(QSPIMUX,    &slcr_base->mio_pin[4]);		/* Pin 4, Quad SPI 0 IO Bit 2 */
	writel(QSPIMUX,    &slcr_base->mio_pin[5]);		/* Pin 5, Quad SPI 0 IO Bit 3 */
	writel(QSPIMUX,    &slcr_base->mio_pin[6]);		/* Pin 6, Quad SPI 0 Clock */
	writel(QSPIMUX,    &slcr_base->mio_pin[7]);		/* Pin 7, Reserved*/
	writel(QSPIMUX,    &slcr_base->mio_pin[8]);		/* Pin 8, Quad SPI Feedback Clock */
	writel(0x00001600, &slcr_base->mio_pin[9]);		/* Pin 9, Level mux -> disable */
	writel(0x00001600, &slcr_base->mio_pin[10]);	/* Pin 10, Level mux -> disable */
	writel(0x00001600, &slcr_base->mio_pin[11]);	/* Pin 11, Level mux -> disable */
	writel(0x00001600, &slcr_base->mio_pin[12]);	/* Pin 12, Level mux -> disable */
	writel(0x00001600, &slcr_base->mio_pin[13]);	/* Pin 13, Level mux -> disable */
	writel(0x00001600, &slcr_base->mio_pin[14]);	/* Pin 14, Level mux -> disable */

	writel(0x767B, &slcr_base->slcr_lock);			//lock slcr
}

static int zx3_current_storage = ZX3_NONE;

void zx3_set_storage (int store) {
	debug("zx3_set_storage: from %u to %u\n", zx3_current_storage, store);

	if (store == zx3_current_storage)
		return;

	switch (store)
	{
		case ZX3_NAND:
			set_mio_mux_nand ();
			zx3_current_storage = ZX3_NAND;
			break;
		case ZX3_QSPI:
			set_mio_mux_qspi();
			zx3_current_storage = ZX3_QSPI;
			break;
		default:
			zx3_current_storage = ZX3_NONE;
			break;
	}
}

#ifdef CONFIG_CMD_NET
int board_phy_config(struct phy_device *phydev)
{
	static int do_once_gem0 = 0;
	static int do_once_gem1 = 0;

	/* first interface, on module */
	if (phydev->dev->iobase == ZYNQ_GEM_BASEADDR0) {
	    if (do_once_gem0 == 0) {
	        /* Giga skew value */
	        if (phydev->phy_id == 0x00221611) { /* KSZ9021, used on first board series */
	            printf("board phy config: KSZ9021 @ %u\n", phydev->addr);
	            phy_write(phydev, phydev->addr, 0xB, 0x8104); // RGMII clock and control pad skew (reg 260)
	            phy_write(phydev, phydev->addr, 0xC, 0xF0F0);
	            phy_write(phydev, phydev->addr, 0xB, 0x8105); // RGMII RX pad skew (reg 261)
	            phy_write(phydev, phydev->addr, 0xC, 0x0);
	            do_once_gem0 = 1;
	        }
	        else if ((phydev->phy_id & 0x00fffff0) == 0x00221620 ) { /* KSZ9031, last four bits are revision number -> ignore */
	            printf("board phy config: KSZ9031 @ %u\n", phydev->addr);
	            phy_write(phydev, phydev->addr, 0xD, 0x0002);
	            phy_write(phydev, phydev->addr, 0xE, 0x0008); // Reg 0x8
	            phy_write(phydev, phydev->addr, 0xD, 0x4002);
	            phy_write(phydev, phydev->addr, 0xE, 0x03FF); //3FF = max RXC and TXC delay
	            do_once_gem0 = 1;
	        }
	        else {
	            printf ("board phy config: unsupported PHY Model, ID:0x%08X\n", phydev->phy_id);
	        }
	    }
	}
	/* second interface, on board */
	else if (phydev->dev->iobase == ZYNQ_GEM_BASEADDR1) {
	    if (do_once_gem1 == 0) {
	        printf("board phy config:  MV88E1118 @ %u\n", phydev->addr);
	        phy_write(phydev, phydev->addr, 0x16, 0x0002); /* Page 2 */
	        phy_write(phydev, phydev->addr, 0x15, 0x3010); /* GigaB */
	        phy_write(phydev, phydev->addr, 0x16, 0x0003); /* Page 3 */
	        phy_write(phydev, phydev->addr, 0x10, 0x1240); /* Led settings */
	        phy_write(phydev, phydev->addr, 0x11, 0x4425);
	        do_once_gem1 = 1;
	    }

	}

	/* call the standard PHY configuration as well */
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

/**
 * Read the MAC address in the module EEPROM
 */
int zx3_read_mac_address(uint8_t *buf, uint8_t index)
{
	/* we have only two interfaces */
	if (index > 1)
	    return 0;

	/* Read MAC address. */
	if (i2c_read(ZX3_SHA_EEPROM_ADDR, ZX3_SHA_EEPROM_MAC_OFFSET,
					ZX3_SHA_EEPROM_ADDR_LEN, (uint8_t *) &buf[0], 6) == 0) {

		/* only the MAC 0 is stored, but the second is reserved, so add index value */
		buf[5] += index;

		/* Check that MAC address is valid. */
		if (is_valid_ether_addr(buf)) {
			return 1; /* Found */
		}
		else {
			printf ("Invalid MAC stored in EEPROM for interface %d\n", index);
			return 0;
		}
	}
	else {
		printf("Read from Module EEPROM @ 0x%02x failed\n",
				ZX3_SHA_EEPROM_ADDR);
		return 0;
	}

}

int board_eth_init(bd_t *bis)
{
	u32 ret = 0;
	u_int8_t mac[6];

#ifdef CONFIG_ZYNQ_GEM0
	if (zx3_read_mac_address (mac, 0)) {
		int err;

		/* set address env if not already set */
		err = eth_setenv_enetaddr("ethaddr", mac);
		if (err)
			printf("Failed to set MAC address 0 from EEPROM to env.\n");
	}
#endif
#ifdef CONFIG_ZYNQ_GEM1
    if (zx3_read_mac_address (mac, 1)) {
        int err;

        /* set address env if not already set */
        err = eth_setenv_enetaddr("eth1addr", mac);
        if (err)
            printf("Failed to set MAC address 1 from EEPROM to env.\n");
    }
#endif


#ifdef CONFIG_XILINX_AXIEMAC
	ret |= xilinx_axiemac_initialize(bis, XILINX_AXIEMAC_BASEADDR,
						XILINX_AXIDMA_BASEADDR);
#endif
#ifdef CONFIG_XILINX_EMACLITE
	u32 txpp = 0;
	u32 rxpp = 0;
# ifdef CONFIG_XILINX_EMACLITE_TX_PING_PONG
	txpp = 1;
# endif
# ifdef CONFIG_XILINX_EMACLITE_RX_PING_PONG
	rxpp = 1;
# endif
	ret |= xilinx_emaclite_initialize(bis, XILINX_EMACLITE_BASEADDR,
			txpp, rxpp);
#endif

#if defined(CONFIG_ZYNQ_GEM)
# if defined(CONFIG_ZYNQ_GEM0)
	ret |= zynq_gem_initialize(bis, ZYNQ_GEM_BASEADDR0,
						CONFIG_ZYNQ_GEM_PHY_ADDR0, 0);
# endif
# if defined(CONFIG_ZYNQ_GEM1)
	ret |= zynq_gem_initialize(bis, ZYNQ_GEM_BASEADDR1,
						CONFIG_ZYNQ_GEM_PHY_ADDRD, 0);
# endif
#endif

	return ret;
}
#endif

#ifdef CONFIG_CMD_MMC
int board_mmc_init(bd_t *bd)
{
    int ret = 0;

# if defined(CONFIG_ZYNQ_SDHCI0)
    ret = zynq_sdhci_init(ZYNQ_SDHCI_BASEADDR0);
# endif
# if defined(CONFIG_ZYNQ_SDHCI1)
    ret |= zynq_sdhci_init(ZYNQ_SDHCI_BASEADDR1);
# endif
    return ret;
}
#endif

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)CONFIG_SYS_SDRAM_BASE,
							CONFIG_SYS_SDRAM_SIZE);
	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size = CONFIG_SYS_SDRAM_SIZE;

	return 0;
}
