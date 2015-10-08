/*
 * (C) Copyright Idexx 2012
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <part.h>
#include <fat.h>
#include <mmc.h>
#include <asm/string.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/io.h>
#include <asm/arch/omap3_dss.h>

#define SPLASH_IMG_BASE	0x82000000
#define LCD_FB_BASE	0x83000000

#define LCD_GPIO_RESET	131
#define LCD_GPIO_BL	154
#define LCD_GPIO_EN	155

#define LCD_GPIO_SCL	156
#define LCD_GPIO_SDI	158
#define LCD_GPIO_SDO	159
#define LCD_GPIO_CS	161

enum lcd_panel {
	LCD_PANEL_SEIKO,
	LCD_PANEL_HIMAX
};

static uchar *splash;
static uchar *lcdfb;

static struct panel_config dss_panel;

extern block_dev_desc_t *mmc_get_dev(int dev);

struct omap_video_timings {
	/* Unit: pixels */
	u16 x_res;
	/* Unit: pixels */
	u16 y_res;
	/* Unit: KHz */
	u32 pixel_clock;
	/* Unit: pixel clocks */
	u16 hsw; /* Horizontal synchronization pulse width */
	u16 hfp; /* Horizontal front porch */
	u16 hbp; /* Horizontal back porch */
	/* Unit: line clocks */
	u16 vsw; /* Vertical synchronization pulse width */
	u16 vfp; /* Vertical front porch */
	u16 vbp; /* Vertical back porch */
};

struct lcd_panel_config {
	s8				name[32];
	s32				config;
	s32				acb;
	s8				data_lines;
	struct omap_video_timings	timing;
};

static struct lcd_panel_config lcd_panel_cfg[3] = {
	{ //480 x 800, Seiko 35WVF0HZ2
		.name	= "seiko",
		.config = OMAP_DSS_LCD_TFT | 
			OMAP_DSS_LCD_IVS | 
			OMAP_DSS_LCD_IHS | 
			OMAP_DSS_LCD_IPC | 
			OMAP_DSS_LCD_ONOFF,
		.acb	= 0,
		.data_lines = 24,
		.timing = {
			.x_res		= 480,
			.y_res		= 800,
			.pixel_clock	= 27000,
			.hfp		= 3,
			.hsw		= 2,
			.hbp		= 28,
			.vfp		= 2,
			.vsw		= 2,
			.vbp		= 10,
		},
	},
	{ //480 x 800, Himax HX8369A02
		.name	= "himax",
		.config = OMAP_DSS_LCD_TFT | 
			OMAP_DSS_LCD_IVS | 
			OMAP_DSS_LCD_IHS | 
			OMAP_DSS_LCD_IPC | 
			OMAP_DSS_LCD_ONOFF,
		.acb	= 0,
		.data_lines = 24,
		.timing = {
			.x_res		= 480,
			.y_res		= 800,
			.pixel_clock	= 27000,
			.hfp		= 10,
			.hsw		= 10,
			.hbp		= 10,
			.vfp		= 2,
			.vsw		= 2,
			.vbp		= 2,
		},
	},
	{ }
};

/*
 * Get timing info for DSS
 */
static struct panel_config *get_panel_config(struct lcd_panel_config *panel)
{
	/* Convert from timings into panel_config structure */
	dss_panel.panel_color = 0x0; /* black */
	dss_panel.load_mode = 0x2; /* Frame Mode */
	dss_panel.panel_type = 1; /* Active TFT */

	dss_panel.timing_h = panel->timing.hsw - 1;
	dss_panel.timing_h |= ((panel->timing.hfp - 1)  <<  8);
	dss_panel.timing_h |= ((panel->timing.hbp - 1)  <<  20);
	dss_panel.timing_v = panel->timing.vsw - 1;
	dss_panel.timing_v |= ((panel->timing.vfp - 1)  <<  8);
	dss_panel.timing_v |= ((panel->timing.vbp - 1)   <<  20);
	dss_panel.pol_freq = panel->acb;
	dss_panel.pol_freq |= ((panel->config & 0x3f)  <<  12);
	dss_panel.lcd_size = panel->timing.x_res - 1;
	dss_panel.lcd_size |= (panel->timing.y_res - 1)  <<  16;
	if (panel->data_lines == 24)
		dss_panel.data_lines = LCD_DL_24BIT;
	else
		dss_panel.data_lines = LCD_DL_16BIT;
	dss_panel.pixel_clock = panel->timing.pixel_clock;

	return &dss_panel;	
}

/*
 * LCD and DSS mux config
 */
static void lcd_pinmux_setup(void)
{
	//Initialize the SPI4 interface
	MUX_VAL(CP(MCBSP1_FSX), (IEN  | PTD | DIS | M4)); /*MCSPI4_CS0*/
	MUX_VAL(CP(MCBSP1_CLKR), (IEN  | PTD | DIS | M4)); /*MCSPI4_CLK*/
	MUX_VAL(CP(MCBSP1_DX), 	(IEN  | PTD | DIS | M4)); /*MCSPI4_SIMO*/
	MUX_VAL(CP(MCBSP1_DR), 	(IEN  | PTD | DIS | M4)); /*MCSPI4_SOMI*/

	MUX_VAL(CP(MMC2_CMD), 	(IDIS | PTD | DIS | M4)); /*GPIO_131*/
	MUX_VAL(CP(MCBSP4_DX), 	(IDIS | PTD | DIS | M4)); /*GPIO_154*/
	MUX_VAL(CP(MCBSP4_FSX), (IDIS | PTD | DIS | M4)); /*GPIO_155*/
	MUX_VAL(CP(GPMC_NCS5), 	(IEN  | PTD | DIS | M3)); /*GPT10 backlight*/

	//Setup common pins
	MUX_VAL(CP(DSS_PCLK), 	(IDIS | PTD | DIS | M0)); /*DSS_PCLK*/
	MUX_VAL(CP(DSS_HSYNC), 	(IEN  | PTD | DIS | M0)); /*DSS_HSYNC*/
	MUX_VAL(CP(DSS_VSYNC), 	(IEN  | PTD | DIS | M0)); /*DSS_VSYNC*/
	MUX_VAL(CP(DSS_ACBIAS),	(IDIS | PTD | DIS | M0)); /*DSS_ACBIAS*/
	MUX_VAL(CP(DSS_DATA18),	(IDIS | PTD | DIS | M3)); /*DSS_DATA0*/
	MUX_VAL(CP(DSS_DATA19),	(IDIS | PTD | DIS | M3)); /*DSS_DATA1*/
	MUX_VAL(CP(DSS_DATA20),	(IDIS | PTD | DIS | M3)); /*DSS_DATA2*/
	MUX_VAL(CP(DSS_DATA21),	(IDIS | PTD | DIS | M3)); /*DSS_DATA3*/
	MUX_VAL(CP(DSS_DATA22),	(IDIS | PTD | DIS | M3)); /*DSS_DATA4*/
	MUX_VAL(CP(DSS_DATA23),	(IDIS | PTD | DIS | M3)); /*DSS_DATA5*/
	MUX_VAL(CP(DSS_DATA6), 	(IDIS | PTD | DIS | M0)); /*DSS_DATA6*/
	MUX_VAL(CP(DSS_DATA7), 	(IDIS | PTD | DIS | M0)); /*DSS_DATA7*/
	MUX_VAL(CP(DSS_DATA8), 	(IDIS | PTD | DIS | M0)); /*DSS_DATA8*/
	MUX_VAL(CP(DSS_DATA9), 	(IDIS | PTD | DIS | M0)); /*DSS_DATA9*/
	MUX_VAL(CP(DSS_DATA10),	(IDIS | PTD | DIS | M0)); /*DSS_DATA10*/
	MUX_VAL(CP(DSS_DATA11),	(IDIS | PTD | DIS | M0)); /*DSS_DATA11*/
	MUX_VAL(CP(DSS_DATA12),	(IDIS | PTD | DIS | M0)); /*DSS_DATA12*/
	MUX_VAL(CP(DSS_DATA13),	(IDIS | PTD | DIS | M0)); /*DSS_DATA13*/
	MUX_VAL(CP(DSS_DATA14),	(IDIS | PTD | DIS | M0)); /*DSS_DATA14*/
	MUX_VAL(CP(DSS_DATA15),	(IDIS | PTD | DIS | M0)); /*DSS_DATA15*/
	MUX_VAL(CP(DSS_DATA16),	(IDIS | PTD | DIS | M0)); /*DSS_DATA16*/
	MUX_VAL(CP(DSS_DATA17),	(IDIS | PTD | DIS | M0)); /*DSS_DATA17*/
	MUX_VAL(CP(SYS_BOOT0), 	(IDIS | PTD | DIS | M3)); /*DSS_DATA18*/
	MUX_VAL(CP(SYS_BOOT1), 	(IDIS | PTD | DIS | M3)); /*DSS_DATA19*/
	MUX_VAL(CP(SYS_BOOT3), 	(IDIS | PTD | DIS | M3)); /*DSS_DATA20*/
	MUX_VAL(CP(SYS_BOOT4), 	(IDIS | PTD | DIS | M3)); /*DSS_DATA21*/
	MUX_VAL(CP(SYS_BOOT5), 	(IDIS | PTD | DIS | M3)); /*DSS_DATA22*/
	MUX_VAL(CP(SYS_BOOT6), 	(IDIS | PTD | DIS | M3)); /*DSS_DATA23*/

	//Delay 1ms to allow mux config. to initialize
	udelay(1000);
}

/*
 * SPI bus communication
 */
#define SBUS_CLK_DELAY	1

#define sbus_start() 	\
	omap_set_gpio_dataout(LCD_GPIO_CS, 0); \
	udelay(SBUS_CLK_DELAY)

#define sbus_stop()	omap_set_gpio_dataout(LCD_GPIO_CS, 1)

#define sbus_clock()	\
	omap_set_gpio_dataout(LCD_GPIO_SCL, 0);	\
	udelay(SBUS_CLK_DELAY); \
	omap_set_gpio_dataout(LCD_GPIO_SCL, 1);	\
	udelay(SBUS_CLK_DELAY)				

/* Register writes are prefixed with value below - Seiko */
#define LCD_SEIKO_REG_WRITE	0x70
/* Data writes are prefixed with the value below - Seiko */
#define LCD_SEIKO_VAL_WRITE	0x72
/* Register writes are prefixed with value below - Himax */
#define LCD_HIMAX_REG_WRITE	0x00
/* Data writes are prefixed with the value below - Himax */
#define LCD_HIMAX_VAL_WRITE	0x01

/*
 * SPI bus tx bit
 */
static void sbus_tx_bit(u16 sbit)
{
	if (sbit)
		omap_set_gpio_dataout(LCD_GPIO_SDI, 1);
	else
		omap_set_gpio_dataout(LCD_GPIO_SDI, 0);

	//Once complete cycle to tx one bit
	sbus_clock();
}

/*
 * SPI bus tx data
 */
static void sbus_tx_data(u16 data, u8 numbits)
{
	//Send bits of data MSB to LSB
	do {
		sbus_tx_bit((data>>(--numbits)) & 0x01);
	} while (numbits);
}

/*
 * Write SPI register data
 */
static int write_reg(enum lcd_panel panel, u8 reg, u8 num, ...)
{
        u8 i;
	u16 msg;
        int val;
        va_list args;

        //Register index
	msg = panel == LCD_PANEL_SEIKO ? 
			LCD_SEIKO_REG_WRITE << 8 : 
			LCD_HIMAX_REG_WRITE << 8;
	msg |= reg;

        sbus_start();
        sbus_tx_data(msg, panel == LCD_PANEL_SEIKO ? 16 : 9);
	sbus_stop();

        //Register value
        va_start(args, num);
        for (i = 0; i < num; i++) {
        	val = va_arg(args, int);
		msg = panel == LCD_PANEL_SEIKO ? 
				LCD_SEIKO_VAL_WRITE << 8 : 
				LCD_HIMAX_VAL_WRITE << 8;
		msg |= val;

		sbus_start();
      		sbus_tx_data(msg, panel == LCD_PANEL_SEIKO ? 16 : 9);
		sbus_stop();
        }
        va_end(args);

        return 0;
}

/*
 * LCD SPI bus init
 */
static void sbus_init(void)
{
	//Initialize gpios direction
	omap_set_gpio_direction(LCD_GPIO_SCL, 0);
	omap_set_gpio_direction(LCD_GPIO_CS, 0);
	omap_set_gpio_direction(LCD_GPIO_SDI, 0);
	omap_set_gpio_direction(LCD_GPIO_SDO, 1);
	omap_set_gpio_direction(LCD_GPIO_RESET, 0);
	omap_set_gpio_direction(LCD_GPIO_EN, 0);
	omap_set_gpio_direction(LCD_GPIO_BL, 0);

	omap_set_gpio_dataout(LCD_GPIO_RESET, 0);
	omap_set_gpio_dataout(LCD_GPIO_EN, 0);
	omap_set_gpio_dataout(LCD_GPIO_BL, 0);
	udelay(10000);

	//Reset Sequence for LCD
	omap_set_gpio_dataout(LCD_GPIO_RESET, 1);
	udelay(10000);
	omap_set_gpio_dataout(LCD_GPIO_RESET, 0);
	udelay(100000);
	omap_set_gpio_dataout(LCD_GPIO_RESET, 1);
	udelay(100000);
}

/*
 * LCD register init
 */
static void lcd_reg_init(enum lcd_panel panel)
{
	if (panel == LCD_PANEL_SEIKO) {
		write_reg(panel, 0x20, 1, 0x00);
        	write_reg(panel, 0x3A, 1, 0x70);
       		write_reg(panel, 0xB1, 3, 0x06, 0x1E, 0x0C);
       		write_reg(panel, 0xB2, 2, 0x10, 0xC8);
        	write_reg(panel, 0xB3, 1, 0x00);
        	write_reg(panel, 0xB4, 1, 0x04);
        	write_reg(panel, 0xB5, 5, 0x10, 0x20, 0x20, 0x00, 0x00);
        	write_reg(panel, 0xB6, 6, 0x01, 0x18, 0x02, 0x40, 0x10, 0x40);
        	write_reg(panel, 0xC3, 5, 0x03, 0x04, 0x03, 0x03, 0x03);
        	write_reg(panel, 0xC4, 6, 0x12, 0x22, 0x10, 0x0C, 0x03, 0x6C);
        	write_reg(panel, 0xC5, 1, 0x76);
        	write_reg(panel, 0xF9, 1, 0x40);
        	write_reg(panel, 0xC6, 2, 0x23, 0x50);
        	write_reg(panel, 0xD0, 9, 0x00, 0x44, 0x44, 0x16, 
					  0x00, 0x03, 0x61, 0x16, 0x03);
        	write_reg(panel, 0xD1, 9, 0x00, 0x44, 0x44, 0x16, 
					  0x00, 0x03, 0x61, 0x16, 0x03);
        	write_reg(panel, 0xD2, 9, 0x00, 0x44, 0x44, 0x16, 
					  0x00, 0x03, 0x61, 0x16, 0x03);
        	write_reg(panel, 0xD3, 9, 0x00, 0x44, 0x44, 0x16, 
					  0x00, 0x03, 0x61, 0x16, 0x03);
        	write_reg(panel, 0xD4, 9, 0x00, 0x44, 0x44, 0x16, 
					  0x00, 0x03, 0x61, 0x16, 0x03);
        	write_reg(panel, 0xD5, 9, 0x00, 0x44, 0x44, 0x16, 
					  0x00, 0x03, 0x61, 0x16, 0x03);
		udelay(1000);
        	write_reg(panel, 0x29, 1, 0x00);
		write_reg(panel, 0x11, 1, 0x00);
	} else { //Himax
		write_reg(panel, 0xb9,3,0xff,0x83,0x69); 
		//Set Power
        	write_reg(panel, 0xB1,19, 
			     0x85, 0x00, 0x34, 0x07, 0x00, 0x0F, 0x0F, 0x2A,
			     0x32, 0x3F, 0x3F,
			     //update VBIAS
			     0x01, 0x3A, 0x01, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6);
		//Set display to 480x800
		write_reg(panel, 0xB2, 15, 
			     0x00, 0x28, 0x05, 0x05, 0x70, 0x00, 0xFF, 0x00, 
			     0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x01); 
		//Set RGB interface
		write_reg(panel, 0xB3, 1, 0x1); 
		//Set CYC
		write_reg(panel, 0xB4, 5, 0x00, 0x18, 0x80, 0x06, 0x02);  
		//Set VCOM
		write_reg(panel, 0xB6, 2, 0x42, 0x42);  
		//Set GIP
		write_reg(panel, 0xD5, 26, 
			     0x00, 0x04, 0x03, 0x00, 0x01, 0x05, 0x28, 0x70, 
			     0x01, 0x03, 0x00, 0x00, 0x40, 0x06, 0x51, 0x07, 
			     0x00, 0x00, 0x41, 0x06, 0x50, 0x07, 0x07, 0x0F, 
			     0x04, 0x00);  
		//Set gamma
		write_reg(panel, 0xE0, 34, 
			     0x00, 0x13, 0x19, 0x38, 0x3D, 0x3F, 0x28, 0x46, 
			     0x07, 0x0D, 0x0E, 0x12, 0x15, 0x12, 0x14, 0x0F, 
			     0x17, 0x00, 0x13, 0x19, 0x38, 0x3D, 0x3F, 0x28, 
			     0x46, 0x07, 0x0D, 0x0E, 0x12, 0x15, 0x12, 0x14, 
			     0x0F, 0x17); 
		//Set DGC LUT
        	write_reg(panel, 0xC1, 127, 0x01,
			     //R
			     0x04, 0x13, 0x1a, 0x20, 0x27, 0x2c, 0x32, 0x36,
			     0x3f, 0x47, 0x50, 0x59, 0x60, 0x68, 0x71, 0x7B,
			     0x82, 0x89, 0x91, 0x98, 0xA0, 0xA8, 0xB0, 0xB8,
			     0xC1, 0xC9, 0xD0, 0xD7, 0xE0, 0xE7, 0xEF, 0xF7,
			     0xFE, 0xCF, 0x52, 0x34, 0xF8, 0x51, 0xF5, 0x9D,
			     0x75, 0x00,
			     //G
			     0x04, 0x13, 0x1a, 0x20, 0x27, 0x2c, 0x32, 0x36,
			     0x3f, 0x47, 0x50, 0x59, 0x60, 0x68, 0x71, 0x7B,
			     0x82, 0x89, 0x91, 0x98, 0xA0, 0xA8, 0xB0, 0xB8,
			     0xC1, 0xC9, 0xD0, 0xD7, 0xE0, 0xE7, 0xEF, 0xF7,
			     0xFE, 0xCF, 0x52, 0x34, 0xF8, 0x51, 0xF5, 0x9D,
			     0x75, 0x00,
			     //B
			     0x04, 0x13, 0x1a, 0x20, 0x27, 0x2c, 0x32, 0x36,
			     0x3f, 0x47, 0x50, 0x59, 0x60, 0x68, 0x71, 0x7B,
			     0x82, 0x89, 0x91, 0x98, 0xA0, 0xA8, 0xB0, 0xB8,
			     0xC1, 0xC9, 0xD0, 0xD7, 0xE0, 0xE7, 0xEF, 0xF7,
			     0xFE, 0xCF, 0x52, 0x34, 0xF8, 0x51, 0xF5, 0x9D,
			     0x75, 0x00); 
		//Set tear on
		write_reg(panel, 0x35, 1, 0x00);
		//Set pixel format
		write_reg(panel, 0x3A, 1, 0x77);   
		//Exit sleep mode
		write_reg(panel, 0x11, 0);
		udelay(1000);
		//Set display on
		write_reg(panel, 0x29,0);  
		//Set memory start
		write_reg(panel, 0x2C,0); 
		udelay(1000);
	}
}

/*
 * Enable LCD panel and backlight
 */
static void lcd_enable(void)
{
	//Start clocks
	omap3_dss_enable();

	//Delay 300ms to allow the panel to initialize
	udelay(300000);

	//Turn on lcd panel pwr
	omap_set_gpio_dataout(LCD_GPIO_EN, 1);

	//Use gpio_56 as their backlight SOM
	init_gpt_timer(10, 70, 100);

	//Delay 10ms to allow the panel to stabilize
	udelay(10000);

	//Turn on lcd backlight pwr
	omap_set_gpio_dataout(LCD_GPIO_BL, 1);
}

/*
 * Convert given image to 24bpp image on 32-bit
 */
/*
 *==================================*
	----------------------
	BMP Color Table Entry
	----------------------
	Name		Offset

	Blue		0x00
	Green		0x01
	Red		0x02
	Reserved	0x03
 *==================================*
	----------------------
	      BMP Header
	----------------------
	Name		Offset

	Signature	0x00
	File size	0x02
	Reserved	0x06
	Data offset	0x0A
	Size		0x0E
	Width		0x12
	Height		0x16
	Planes		0x1A
	Bit count	0x1C
	Compression	0x1E
	Image size	0x22
	X Pix_per_m	0x26
	Y Pix_per_m	0x2A
	Colors used	0x2E
	Colors important 0x32
 *==================================*
	----------------------
	      BMP Image
	----------------------
	Name 		Offset

	BMP header	0x00
	BMP color table	0x36
 *==================================*
 */
int lcd_display_bitmap(uchar *bmp_image)
{
	uchar *fb;
	uchar *bmap;
	ushort bmp_bpix;
	ushort width, height, line_len;
	ulong image_size;
	ulong w, h, i;
	uchar cmap[1024];

	if (!bmp_image) return -1;

	//FIXME: unable to dereference 'bmp_image' as pointer

	//Is bmp image ?
	if (!(bmp_image[0] == 'B') && 
	    !(bmp_image[1] == 'M')) {
		printf ("No valid bmp image\n");
		return -1;
	}

	//Get image width
	width = le32_to_cpu(bmp_image[0x12] | 
			(bmp_image[0x13] << 8) | 
			(bmp_image[0x14] << 16) | 
			(bmp_image[0x15] << 24));
	//Get image height
	height = le32_to_cpu(bmp_image[0x16] | 
			(bmp_image[0x17] << 8) | 
			(bmp_image[0x18] << 16) | 
			(bmp_image[0x19] << 24));
	//Get bits per pixel
	bmp_bpix = le16_to_cpu(bmp_image[0x1C] | 
			(bmp_image[0x1D] << 8));
	//Get image size
	image_size = le32_to_cpu(bmp_image[0x22] | 
			(bmp_image[0x23] << 8) | 
			(bmp_image[0x24] << 16) | 
			(bmp_image[0x25] << 24));

	//Set line length for 32bpp
	line_len = width << 2;

	switch (bmp_bpix) {
	case 8: //Convert 8bpp to 24bpp on 32-bit
		//Get bmp color table
		bmap = bmp_image + 0x36;

		//Collect image palettes
		memcpy(cmap, bmap, 1024);

		//Get bmp data offset
		bmap = bmp_image + le32_to_cpu(bmp_image[0x0A] | 
			(bmp_image[0x0B] << 8) | 
			(bmp_image[0x0C] << 16) | 
			(bmp_image[0x0D] << 24));

		//Get pixel data
		for (h = 1, i = image_size - 1; h <= height; ++h) {
			fb = lcdfb + line_len * h;
			for (w = 1; w <= width; ++w, --i) {
				u16 index = bmap[i] << 2;
				*(ulong *) (fb -= 4) = *(ulong *) (cmap + index);
			}
		}
		break;
	case 16: //Convert 16bpp to 24bpp on 32-bit
		//Get bmp data offset
		bmap = bmp_image + le32_to_cpu(bmp_image[0x0A] | 
			(bmp_image[0x0B] << 8) | 
			(bmp_image[0x0C] << 16) | 
			(bmp_image[0x0D] << 24));

		//Get pixel data
		for (h = 1, i = image_size - 2; h <= height; ++h) {
			fb = lcdfb + line_len * h;
			for (w = 1; w <= width; ++w, i -= 2) {
				ushort color = *(ushort *) (bmap + i);
				*(ulong *) (fb -= 4) = (((color & 0x1f) << 3) |
						(((color & 0x7e0) >> 3) << 8) | 
						((color & 0xf800) << 8));
			}
		}
		break;
	case 24: //Convert 24bpp to 24bpp on 32-bit
		//Get bmp data offset
		bmap = bmp_image + le32_to_cpu(bmp_image[0x0A] | 
			(bmp_image[0x0B] << 8) | 
			(bmp_image[0x0C] << 16) | 
			(bmp_image[0x0D] << 24));

		//Get pixel data
		for (h = 1, i = image_size - 3; h <= height; ++h) {
			fb = lcdfb + line_len * h;
			for (w = 1; w <= width; ++w, i -= 3) {
				*(ulong *) (fb -= 4) = bmap[i] | 
						(bmap[i + 1] << 8) | 
						(bmap[i + 2] << 16);
			}
		}
		break;
	default:
		printf("Unsupport bmp file\n");
		return -1;
	}

	return 0;
}

/*
 * LCD and DSS init
 */
void lcd_init(omap_boot_device_t boot_device)
{
       	int ret;
	long size;
	enum lcd_panel panel;

	splash = (uchar *) SPLASH_IMG_BASE;
	lcdfb = (uchar *) LCD_FB_BASE;

	/* 
	 * TODO: Get splash image from NAND if boot device is NAND.
	 *	 Decide a way to identify the lcd panel. For sdmmc, 
	 *	 the lcd panel is decided based on splash image name.
	 */

	//Init mmc
       	ret = mmc_init(1);
       	if (ret == 0) {
        	printf("MMC init failed for splash image read\n");
               	return;
       	}

	//Read splash screen
       	ret = fat_register_device(mmc_get_dev(0), 1);
	if (ret == -1) {
		printf("Failed to register FAT for splash image read\n");
		return;
	}
	size = file_fat_read("himax", (void *) splash, 0);
	if (size <= 0) {
		size = file_fat_read("seiko", (void *) splash, 0);
		if (size <= 0) {
			printf("Failed to read splash image\n");
			return;
		} else panel = LCD_PANEL_SEIKO;
	} else panel = LCD_PANEL_HIMAX;

	//Convert to 32-bit bmp
	ret = lcd_display_bitmap(splash);
	if (ret == -1) {
		printf("Failed to convert splash image to 24-bit image\n");
		return;
	}

	//LCD mux setup
	lcd_pinmux_setup();

	//DSS setup
	omap3_dss_panel_config(get_panel_config(&lcd_panel_cfg[panel]));
	omap3_dss_set_fb((ulong) lcdfb);

	//LCD setup
	sbus_init();

	//Enable display
	lcd_enable();

	//Configure LCD
	lcd_reg_init(panel);

	printf("Splash screen loaded\n");
}
