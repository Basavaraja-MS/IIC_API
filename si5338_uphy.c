/*Initialize SI5338 with I2C interface and initialize UPHY TC
 * Hardware specific code
 *TODO: 1. Move magic numbers into names 
 *	2. Avoid Loops and enhance debug support 
 *
 * **/




#include <linux/kernel.h>
#include <linux/printk.h>

#include <platform/hardware.h>
#include <platform/lcd.h>

#include <platform/iic_api.h>
#include <platform/si5338_uphy.h>

#define printf printk

#define csp_out32(Addr, Value) \
  (*(volatile unsigned int  *)((Addr)) = (Value))

#define csp_read32(addr) \
   *((volatile unsigned int *)(addr))



#define IIC_BASE_ADDRESS 	0xFD070000
#define IIC_SLAVE_ADDRESS 	0x70
#define APB2JTAG_BASE  		0xFD400000
#define APB2GPIO_BASE  		0xFD030000
#define AXI_SW_BASE    		0xFD080000
#define AXI_LED_BASE   		(AXI_SW_BASE + 0x8)

#define XPAR_PCIE_MGMT_APB_BASEADDR  0xFB000000

#define PCIE_CORE_CONFIG_SPACE_BASE  (XPAR_PCIE_MGMT_APB_BASEADDR + 0x000000)
#define PCIE_CORE_LOCAL_MGMT_BASE    (XPAR_PCIE_MGMT_APB_BASEADDR + 0x100000)
#define PCIE_CORE_AXI_CONFIG_BASE    (XPAR_PCIE_MGMT_APB_BASEADDR + 0x400000)

int udelay(unsigned int val){
        volatile unsigned int i;
        for (i = 0; i < val*10; i++ );
}

void reset_assert_uphy (void)
{
	//tc rst =0 apb/tap rst =0 phyreset =0 pipe rst = 0
	csp_out32 ((APB2GPIO_BASE + (0x30<<2)), 0x0);   // TOP_CHIP_RST_B
	csp_out32 ((APB2GPIO_BASE + (0x34<<2)), 0x0);   // UPHY_APB_PRESET_N
	csp_out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x0);   // UPHY_TAP_TRST_N
	csp_out32 ((APB2GPIO_BASE + (0x38<<2)), 0x0);   // UPHY_PHY_RESET_N
	csp_out32 ((APB2GPIO_BASE + (0x40<<2)), 0x0);   // UPHY_PIPE_RESET_N
}

void reset_assert_pcie (void)
{
	csp_out32 ((APB2GPIO_BASE + (0x55<<2)), 0x0);   // SOFT_PIPE_RESET_N
}

void reset_deassert_pcie (void)
{
	csp_out32 ((APB2GPIO_BASE + (0x55<<2)), 0x1);   // SOFT_PIPE_RESET_N
}

void link_training_disable (void)
{
	csp_out32 ((APB2GPIO_BASE + (0x43<<2)), 0x0);   // LINK_TRAINING_ENABLE
}

void link_training_enable (void)
{
	csp_out32 ((APB2GPIO_BASE + (0x43<<2)), 0x1);   // LINK_TRAINING_ENABLE
}


void uphy_bringup(void)
{
	unsigned int read_val=0;
	
	// When CRTL in Endpoint
	
#ifdef DEBUG_PRINT_ENABLE
	PRINT("UPHY Init Start\n\r");
#endif
	
	//tc rst =1 apb/tap rst =1 phyreset =1 pipe rst = 1
	csp_out32 ((APB2GPIO_BASE + (0x30<<2)), 0x1);   // TOP_CHIP_RST_B
	csp_out32 ((APB2GPIO_BASE + (0x34<<2)), 0x1);   // UPHY_APB_PRESET_N
	csp_out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x1);   // UPHY_TAP_TRST_N
	csp_out32 ((APB2GPIO_BASE + (0x38<<2)), 0x1);   // UPHY_PHY_RESET_N
	csp_out32 ((APB2GPIO_BASE + (0x40<<2)), 0x1);   // UPHY_PIPE_RESET_N
	
	udelay (0x200);
	
	//tc rst =0 apb/tap rst =0 phyreset =0 pipe rst = 0
	csp_out32 ((APB2GPIO_BASE + (0x30<<2)), 0x0);   // TOP_CHIP_RST_B
	csp_out32 ((APB2GPIO_BASE + (0x34<<2)), 0x0);   // UPHY_APB_PRESET_N
	csp_out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x0);   // UPHY_TAP_TRST_N
	csp_out32 ((APB2GPIO_BASE + (0x38<<2)), 0x0);   // UPHY_PHY_RESET_N
	csp_out32 ((APB2GPIO_BASE + (0x40<<2)), 0x0);   // UPHY_PIPE_RESET_N
	udelay (0x200);
	
	csp_out32 ((APB2GPIO_BASE + (0x31<<2)), 0x0);   // TOP_CHIPMODE for PCIe
	udelay (0x500);
	csp_out32 ((APB2GPIO_BASE + (0x32<<2)), 0x2);   // TOP_SEL_TAP 1x for TC
	udelay (0x500);
	
	//pipe rst = 1
	csp_out32 ((APB2GPIO_BASE + (0x40<<2)), 0x1);   // UPHY_PIPE_RESET_N
	udelay (0x500);
	
	csp_out32 ((APB2GPIO_BASE + (0x3B<<2)), 0x1);   // 32bit_sel
	csp_out32 ((APB2GPIO_BASE + (0x1C<<2)), 0x1);   // PIPE_L01_TX_ELEC_IDLE
	csp_out32 ((APB2GPIO_BASE + (0x29<<2)), 0x1);   // PIPE_L00_TX_ELEC_IDLE
	csp_out32 ((APB2GPIO_BASE + (0x2F<<2)), 0x2);   // Powerdown
	csp_out32 ((APB2GPIO_BASE + (0x3A<<2)), 0x1);   // UPHY_PHY_TX_CMN_MODE_EN
	csp_out32 ((APB2GPIO_BASE + (0x39<<2)), 0x1);   // UPHY_PHY_RX_ELEC_IDLE_DET_EN
	
	csp_out32 ((APB2GPIO_BASE + (0x20<<2)), 0x2);   // full rt clock 125mhz
	
	//apb rst =1
	csp_out32 ((APB2GPIO_BASE + (0x34<<2)), 0x1);   // UPHY_APB_PRESET_N
	udelay (0x50);
	
	//tc rst =1
	csp_out32 ((APB2GPIO_BASE + (0x30<<2)), 0x1);   // TOP_CHIP_RST_B
	udelay (0x50);
	
	//tap rst = 1
	csp_out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x1);   // UPHY_TAP_TRST_N
	udelay (0x50);
	
	csp_out32 ((APB2JTAG_BASE + (0x0024<<4)), 0x3);   //uphy_t28_comp__TC_UPHY_CTRL_REG_15
	udelay (0x500);
	
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x0024<<4)));
	//xil_printf(" uphy_t28_comp__TC_UPHY_CTRL_REG_15 = 0x%X \r\n", read_val);
	
	//tap rst = 0
	csp_out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x0);   // UPHY_TAP_TRST_N
	udelay (0x50);
	
	//phy reset =0
	csp_out32 ((APB2GPIO_BASE + (0x38<<2)), 0x0);   // UPHY_PHY_RESET_N
	udelay (0x500);
	
	csp_out32 ((APB2GPIO_BASE + (0x32<<2)), 0x0);   // TOP_SEL_TAP 0x for IP
	udelay (0x500);
	
	//tap rst = 1
	csp_out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x1);   // UPHY_TAP_TRST_N
	udelay (0x50);
	
	//phy reset =1
	csp_out32 ((APB2GPIO_BASE + (0x38<<2)), 0x1);   // UPHY_PHY_RESET_N
	udelay (0x500);
	
	
	csp_out32 ((APB2JTAG_BASE + (0xC800<<4)), 0x3800);   //PHY_PMA_CMN_CTRL1  differential clock selection
	udelay (0x500);
	
	//csp_out32 ((APB2JTAG_BASE + (0x0022<<4)), 0x0040);   //CMN_SSM_BIAS_TMR only for Gen2
	//udelay (0x10000);
	
	
	read_val = 0;
	while (!(read_val & 0x0001) )
	{
	read_val = Xil_In32 ((APB2JTAG_BASE + (0xC800<<4)));  //wait for PHY_PMA_CMN_CTRL1[0] === 1
	//xil_printf(" PHY_PMA_CMN_CTRL1 = 0x%X \r\n", read_val);
	}
	udelay (0x50);
	
	csp_out32 ((APB2JTAG_BASE + (0xcc10<<4)), 0x0020);   //for lane 0    PHY_PMA_ISO_XCVR_CTRL
	udelay (0x50);
	csp_out32 ((APB2JTAG_BASE + (0xcc50<<4)), 0x0020);   //for lane 1    PHY_PMA_ISO_XCVR_CTRL
	udelay (0x50);
	
	csp_out32 ((APB2JTAG_BASE + (0x8004<<4)), 0x1010);   //RX_PSC_A4
	udelay (0x50);
	csp_out32 ((APB2JTAG_BASE + (0x8080<<4)), 0x2AB3);   //RX_CDRLF_CNFG
	udelay (0x50);
	csp_out32 ((APB2JTAG_BASE + (0x819D<<4)), 0x8014);   //RX_REE_PEAK_COVRD - 0x8014
	udelay (0x50);
	csp_out32 ((APB2JTAG_BASE + (0x81BB<<4)), 0x4080);   //RX_REE_CTRL_DATA_MASK
	udelay (0x50);
	
#ifdef DEBUG_PRINT_ENABLE
	//Reading all the register
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x8004<<4)));
	xil_printf(" RX_PSC_A4 = 0x%X \r\n", read_val);
	
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x8080<<4)));
	xil_printf(" RX_CDRLF_CNFG = 0x%X \r\n", read_val);
	
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x819D<<4)));
	xil_printf(" RX_REE_PEAK_COVRD = 0x%X \r\n", read_val);
	
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x81BB<<4)));
	xil_printf(" RX_REE_CTRL_DATA_MASK = 0x%X \r\n", read_val);
	
	PRINT("\nUPHY Init Done\n\r");
#endif
	
	//csp_out32 ((APB2GPIO_BASE + (0x43<<2)), 0x1);   // LINK_TRAINING_ENABLE
	//csp_out32 ((APB2GPIO_BASE + (0x55<<2)), 0x1);   // SOFT_PIPE_RESET_N
	
	//csp_out32 ((APB2GPIO_BASE + (0x1C<<2)), 0x0);   // PIPE_L01_TX_ELEC_IDLE
	//csp_out32 ((APB2GPIO_BASE + (0x29<<2)), 0x0);   // PIPE_L00_TX_ELEC_IDLE
	
	//When ctrl in RC
	//csp_out32 ((APB2GPIO_BASE + (0x56<<2)), 0x1);   // perst_o
	//csp_out32 ((APB2GPIO_BASE + (0x57<<2)), 0x1);   // RP_EP_MODE_SEL
	
	//csp_out32 ((APB2GPIO_BASE + (0x43<<2)), 0x1);   // LINK_TRAINING_ENABLE
	//csp_out32 ((APB2GPIO_BASE + (0x55<<2)), 0x1);   // SOFT_PIPE_RESET_N
	
	udelay (0x50);
	

}

void pcie_mode_root_complex (void)
{
	csp_out32 ((APB2GPIO_BASE + (0x57<<2)), 0x1);
}

void pcie_mode_end_point (void)
{
	csp_out32 ((APB2GPIO_BASE + (0x57<<2)), 0x0);
}

void perstn_assert (void)
{
	csp_out32 ((APB2GPIO_BASE + (0x56<<2)), 0x0);
}

void perstn_deassert (void)
{
	csp_out32 ((APB2GPIO_BASE + (0x56<<2)), 0x1);
}

void wait_perstn_deassert (void)
{
	unsigned int read_val=0;
	
	while (!(read_val & 0x0001) )
	{
		read_val = Xil_In32 ((APB2GPIO_BASE + (0x0A<<2)));  //wait for perst_i === 1
	}
}

void wait_perstn_assert (void)
{
	unsigned int read_val=0;
	
	while ((read_val & 0x0001) )
	{
		read_val = Xil_In32 ((APB2GPIO_BASE + (0x0A<<2)));  //wait for perst_i === 1
	}
}

void wait_link_training_done (void)
{
	unsigned int read_val=0;
	
	while (!(read_val & 0x0001) )
	{
		read_val = Xil_In32 (PCIE_CORE_LOCAL_MGMT_BASE + 0x00);
		udelay (0x50);
	}
	//csp_out32 ((AXI_LED_BASE), 0x05);
}

	
struct iic_data{
	uint8_t addr;
	uint8_t data;
};	 			
		
struct iic_data si_iic_data[] =  {
	{0xE6,0x1F},
	{0xF1,0x85},
	{0xFF,0x00},
	
	{0x0,0x00},
	{0x1,0x00},
	{0x2,0x00},
	{0x3,0x00},
	{0x4,0x00},
	{0x5,0x00},
	{0x6,0x04},
	{0x7,0x00},
	{0x8,0x70},
	{0x9,0x0F},
	{0xA,0x00},
	{0xB,0x00},
	{0xC,0x00},
	{0xD,0x00},
	{0xE,0x00},
	{0xF,0x00},
	{0x10,0x00},
	{0x11,0x00},
	{0x12,0x00},
	{0x13,0x00},
	{0x14,0x00},
	{0x15,0x00},
	{0x16,0x00},
	{0x17,0x00},
	{0x18,0x00},
	{0x19,0x00},
	{0x1A,0x00},
	{0x1B,0x70},
	{0x1C,0x03},
	{0x1D,0x62},
	{0x1E,0xA2},
	{0x1F,0x02},
	{0x20,0x02},
	{0x21,0x02},
	{0x22,0x02},
	{0x23,0xAA},
	{0x24,0x07},
	{0x25,0x07},
	{0x26,0x07},
	{0x27,0x07},
	{0x28,0xE7},
	{0x29,0x1C},
	{0x2A,0x27},
	{0x2B,0x00},
	{0x2C,0x00},
	{0x2D,0x00},
	{0x2E,0x00},
	{0x2F,0x14},
	{0x30,0x35},
	{0x31,0x00},
	{0x32,0x03},
	{0x33,0x07},
	{0x34,0x10},
	{0x35,0x00},
	{0x36,0x0B},
	{0x37,0x00},
	{0x38,0x00},
	{0x39,0x00},
	{0x3A,0x00},
	{0x3B,0x01},
	{0x3C,0x00},
	{0x3D,0x00},
	{0x3E,0x00},
	{0x3F,0x10},
	{0x40,0x00},
	{0x41,0x0B},
	{0x42,0x00},
	{0x43,0x00},
	{0x44,0x00},
	{0x45,0x00},
	{0x46,0x01},
	{0x47,0x00},
	{0x48,0x00},
	{0x49,0x00},
	{0x4A,0x10},
	{0x4B,0x00},
	{0x4C,0x0B},
	{0x4D,0x00},
	{0x4E,0x00},
	{0x4F,0x00},
	{0x50,0x00},
	{0x51,0x01},
	{0x52,0x00},
	{0x53,0x00},
	{0x54,0x00},
	{0x55,0x10},
	{0x56,0x00},
	{0x57,0x0B},
	{0x58,0x00},
	{0x59,0x00},
	{0x5A,0x00},
	{0x5B,0x00},
	{0x5C,0x01},
	{0x5D,0x00},
	{0x5E,0x00},
	{0x5F,0x00},
	{0x60,0x10},
	{0x61,0x00},
	{0x62,0x32},
	{0x63,0x00},
	{0x64,0x00},
	{0x65,0x00},
	{0x66,0x00},
	{0x67,0x01},
	{0x68,0x00},
	{0x69,0x00},
	{0x6A,0x80},
	{0x6B,0x00},
	{0x6C,0x00},
	{0x6D,0x00},
	{0x6E,0x40},
	{0x6F,0x00},
	{0x70,0x00},
	{0x71,0x00},
	{0x72,0x40},
	{0x73,0x00},
	{0x74,0x80},
	{0x75,0x00},
	{0x76,0x40},
	{0x77,0x00},
	{0x78,0x00},
	{0x79,0x00},
	{0x7A,0x40},
	{0x7B,0x00},
	{0x7C,0x00},
	{0x7D,0x00},
	{0x7E,0x00},
	{0x7F,0x00},
	{0x80,0x00},
	{0x81,0x00},
	{0x82,0x00},
	{0x83,0x00},
	{0x84,0x00},
	{0x85,0x00},
	{0x86,0x00},
	{0x87,0x00},
	{0x88,0x00},
	{0x89,0x00},
	{0x8A,0x00},
	{0x8B,0x00},
	{0x8C,0x00},
	{0x8D,0x00},
	{0x8E,0x00},
	{0x8F,0x00},
	{0x90,0x00},
	{0x91,0x00},
	{0x92,0xFF},
	{0x93,0x00},
	{0x94,0x00},
	{0x95,0x00},
	{0x96,0x00},
	{0x97,0x00},
	{0x98,0x00},
	{0x99,0x00},
	{0x9A,0x00},
	{0x9B,0x00},
	{0x9C,0x00},
	{0x9D,0x00},
	{0x9E,0x00},
	{0x9F,0x00},
	{0xA0,0x00},
	{0xA1,0x00},
	{0xA2,0x00},
	{0xA3,0x00},
	{0xA4,0x00},
	{0xA5,0x00},
	{0xA6,0x00},
	{0xA7,0x00},
	{0xA8,0x00},
	{0xA9,0x00},
	{0xAA,0x00},
	{0xAB,0x00},
	{0xAC,0x00},
	{0xAD,0x00},
	{0xAE,0x00},
	{0xAF,0x00},
	{0xB0,0x00},
	{0xB1,0x00},
	{0xB2,0x00},
	{0xB3,0x00},
	{0xB4,0x00},
	{0xB5,0x00},
	{0xB6,0x00},
	{0xB7,0x00},
	{0xB8,0x00},
	{0xB9,0x00},
	{0xBA,0x00},
	{0xBB,0x00},
	{0xBC,0x00},
	{0xBD,0x00},
	{0xBE,0x00},
	{0xBF,0x00},
	{0xC0,0x00},
	{0xC1,0x00},
	{0xC2,0x00},
	{0xC3,0x00},
	{0xC4,0x00},
	{0xC5,0x00},
	{0xC6,0x00},
	{0xC7,0x00},
	{0xC8,0x00},
	{0xC9,0x00},
	{0xCA,0x00},
	{0xCB,0x00},
	{0xCC,0x00},
	{0xCD,0x00},
	{0xCE,0x00},
	{0xCF,0x00},
	{0xD0,0x00},
	{0xD1,0x00},
	{0xD2,0x00},
	{0xD3,0x00},
	{0xD4,0x00},
	{0xD5,0x00},
	{0xD6,0x00},
	{0xD7,0x00},
	{0xD8,0x00},
	{0xD9,0x00},
	{0xDA,0x00},
	{0xDB,0x00},
	{0xDC,0x00},
	{0xDD,0x0D},
	{0xDE,0x00},
	{0xDF,0x00},
	{0xE0,0xF4},
	{0xE1,0xF0},
	{0xE2,0x00},
	{0xE3,0x00},
	{0xE4,0x00},
	{0xE5,0x00},
	{0xE6,0x00},
	{0xE7,0x00},
	{0xE8,0x00},
	{0xE9,0x00},
	{0xEA,0x00},
	{0xEB,0x00},
	{0xEC,0x00},
	{0xED,0x00},
	{0xEE,0x14},
	{0xEF,0x00},
	{0xF0,0x00},
	{0xF2,0x00},
	{0xF3,0xF0},
	{0xF4,0x00},
	{0xF5,0x00},
	{0xF7,0x00},
	{0xF8,0x00},
	{0xF9,0xA8},
	{0xFA,0x00},
	{0xFB,0x84},
	{0xFC,0x00},
	{0xFD,0x00},
	{0xFE,0x00},
	{0xFF,0xFF},
	
	{0x0,0x00},
	{0x1,0x00},
	{0x2,0x00},
	{0x3,0x00},
	{0x4,0x00},
	{0x5,0x00},
	{0x6,0x00},
	{0x7,0x00},
	{0x8,0x00},
	{0x9,0x00},
	{0xA,0x00},
	{0xB,0x00},
	{0xC,0x00},
	{0xD,0x00},
	{0xE,0x00},
	{0xF,0x00},
	{0x10,0x00},
	{0x11,0x01},
	{0x12,0x00},
	{0x13,0x00},
	{0x14,0x90},
	{0x15,0x31},
	{0x16,0x00},
	{0x17,0x00},
	{0x18,0x01},
	{0x19,0x00},
	{0x1A,0x00},
	{0x1B,0x00},
	{0x1C,0x00},
	{0x1D,0x00},
	{0x1E,0x00},
	{0x1F,0x00},
	{0x20,0x00},
	{0x21,0x01},
	{0x22,0x00},
	{0x23,0x00},
	{0x24,0x90},
	{0x25,0x31},
	{0x26,0x00},
	{0x27,0x00},
	{0x28,0x01},
	{0x29,0x00},
	{0x2A,0x00},
	{0x2B,0x00},
	{0x2C,0x00},
	{0x2D,0x00},
	{0x2E,0x00},
	{0x2F,0x00},
	{0x30,0x00},
	{0x31,0x01},
	{0x32,0x00},
	{0x33,0x00},
	{0x34,0x90},
	{0x35,0x31},
	{0x36,0x00},
	{0x37,0x00},
	{0x38,0x01},
	{0x39,0x00},
	{0x3A,0x00},
	{0x3B,0x00},
	{0x3C,0x00},
	{0x3D,0x00},
	{0x3E,0x00},
	{0x3F,0x00},
	{0x40,0x00},
	{0x41,0x01},
	{0x42,0x00},
	{0x43,0x00},
	{0x44,0x90},
	{0x45,0x31},
	{0x46,0x00},
	{0x47,0x00},
	{0x48,0x01},
	{0x49,0x00},
	{0x4A,0x00},
	{0x4B,0x00},
	{0x4C,0x00},
	{0x4D,0x00},
	{0x4E,0x00},
	{0x4F,0x00},
	{0x50,0x00},
	{0x51,0x00},
	{0x52,0x00},
	{0x53,0x00},
	{0x54,0x90},
	{0x55,0x31},
	{0x56,0x00},
	{0x57,0x00},
	{0x58,0x01},
	{0x59,0x00},
	{0x5A,0x00},
	{0x5B,0x00},
	{0x5C,0x00},
	{0x5D,0x00},
	{0x5E,0x00}
};

int hardware_main (void){
	
	uint32_t StatusReg;
	int retVal, i;			
	size_t size;

	pcie_mode_root_complex();
	perstn_assert();
	
	printf("si5338 Initiated \n");

	retVal = XIic_DynInit(IIC_BASE_ADDRESS);
	if (retVal != XST_SUCCESS) {
					printf ("ERROR in dynamic init 0x%x \n", retVal);
					return 0;

	}

	/*
	 * Make sure all the Fifo's are cleared and Bus is Not busy.
	 */
	printf("Status clear taking time\n");
	while (((StatusReg = XIic_ReadReg(IIC_BASE_ADDRESS,
				XIIC_SR_REG_OFFSET)) &
				(XIIC_SR_RX_FIFO_EMPTY_MASK |
				XIIC_SR_TX_FIFO_EMPTY_MASK |
				XIIC_SR_BUS_BUSY_MASK)) !=
				(XIIC_SR_RX_FIFO_EMPTY_MASK |
				XIIC_SR_TX_FIFO_EMPTY_MASK)) {

	}

	printf("Total Size %d\n", size = sizeof(si_iic_data));
	printf("Size of each %d\n", sizeof(si_iic_data[0]));
	
	for (i = 0; i < size/2; i++){
                retVal =  XIic_Send(IIC_BASE_ADDRESS, IIC_SLAVE_ADDRESS,
                               (uint8_t *)&si_iic_data[i], sizeof(si_iic_data[i]), XIIC_STOP);
		if(retVal != sizeof(si_iic_data[i])){
			printf("Err at Addr %d Data %d count %d res %d\n", si_iic_data[i].addr, si_iic_data[i].data, i, retVal);
		}	
	}
	printf("si5338 Init done\n");

	uphy_bringup();
	printf("uphy init done\n");

	perstn_deassert();
	link_training_enable();
	reset_deassert_pcie ();
	wait_link_training_done();
	printf("success in link training\n");
	udelay(0x100);
	return 0;
}
