/*
 *  linux/drivers/video/ls2k_fb.c -- Virtual frame buffer device
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>

#include <asm/addrspace.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <ls2k.h>
#include <asm/addrspace.h>
#include "edid.h"

#ifdef LS2K_FB_DEBUG
#define LS2K_DEBUG(frm, arg...)	\
	printk("ls2k_fb: %s %d: "frm, __func__, __LINE__, ##arg);
#else
#define LS2K_DEBUG(frm, arg...)
#endif /* LS2K_FB_DEBUG */

#define ON	1
#define OFF	0

#define CUR_WIDTH_SIZE		32
#define CUR_HEIGHT_SIZE		32
#define DEFAULT_BITS_PER_PIXEL	16

//#define CONFIG_DC_NOCOHERENT
#if defined(CONFIG_DMA_NONCOHERENT) || defined(CONFIG_DC_NOCOHERENT)
#define DEFAULT_CURSOR_MEM	0x900000000ef00000
#define DEFAULT_CURSOR_DMA	0x0ef00000
#define DEFAULT_FB_MEM		0x900000000e800000
#define DEFAULT_PHY_ADDR	0x0e800000
#define DEFAULT_FB_DMA		0x0e800000
#else
#define DEFAULT_CURSOR_MEM	0x980000000ef00000
#define DEFAULT_CURSOR_DMA	0x0ef00000
#define DEFAULT_FB_MEM		0x980000000e800000
#define DEFAULT_PHY_ADDR	0x0e800000
#define DEFAULT_FB_DMA		0x0e800000
#endif

#define LO_OFF	0
#define HI_OFF	8
struct pix_pll {
	unsigned int l2_div;
	unsigned int l1_loopc;
	unsigned int l1_frefc;
};

struct ls2k_fb_par {
	struct platform_device *pdev;
	struct fb_info *fb_info;
	unsigned long reg_base;
	unsigned int irq;
	unsigned int htotal;
	unsigned int vtotal;
	u8 *edid;
};
static char *mode_option = NULL;
static void *cursor_mem;
static unsigned int cursor_size = 0x1000;
static void *videomemory;
static	dma_addr_t dma_A;
static dma_addr_t cursor_dma;

static u_long videomemorysize = 0;
module_param(videomemorysize, ulong, 0);
static int clockpol;
module_param(clockpol, int, 0);
DEFINE_SPINLOCK(fb_lock);
static void config_pll(unsigned long pll_base, struct pix_pll *pll_cfg);
static struct fb_var_screeninfo ls2k_fb_default __initdata = {
	.xres		= 1280,
	.yres		= 1024,
	.xres_virtual	= 1280,
	.yres_virtual	= 1024,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel = DEFAULT_BITS_PER_PIXEL,
	.red		= { 11, 5 ,0},
	.green		= { 5, 6, 0 },
	.blue		= { 0, 5, 0 },
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.pixclock	= 9184,
	.left_margin	= 432,
	.right_margin	= 80,
	.upper_margin	= 36,
	.lower_margin	= 1,
	.hsync_len	= 216,
	.vsync_len	= 3,
	.sync		= 4,
	.vmode =	FB_VMODE_NONINTERLACED,
};
static struct fb_fix_screeninfo ls2k_fb_fix __initdata = {
	.id =		"Virtual FB",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	1,
	.ypanstep =	1,
	.ywrapstep =	1,
	.accel =	FB_ACCEL_NONE,
};

static bool ls2k_fb_enable __initdata = 1;	/* disabled by default */
module_param(ls2k_fb_enable, bool, 0);

/*
 *  Internal routines
 */

static u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;

	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
	return (length);
}

/*
 *  Setting the video mode has been split into two parts.
 *  First part, xxxfb_check_var, must not write anything
 *  to hardware, it should only verify and adjust var.
 *  This means it doesn't alter par but it does use hardware
 *  data from it to check this var.
 */

static int ls2k_fb_check_var(struct fb_var_screeninfo *var,
			 struct fb_info *info)
{
	u_long line_length;

	/*
	 *  FB_VMODE_CONUPDATE and FB_VMODE_SMOOTH_XPAN are equal!
	 *  as FB_VMODE_SMOOTH_XPAN is only used internally
	 */

	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = info->var.xoffset;
		var->yoffset = info->var.yoffset;
	}

	/*
	 *  Some very basic checks
	 */
	if (!var->xres)
		var->xres = 640;
	if (!var->yres)
		var->yres = 480;
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;
	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;
	if (var->bits_per_pixel <= 1)
		var->bits_per_pixel = 1;
	else if (var->bits_per_pixel <= 8)
		var->bits_per_pixel = 8;
	else if (var->bits_per_pixel <= 16)
		var->bits_per_pixel = 16;
	else if (var->bits_per_pixel <= 24)
		var->bits_per_pixel = 24;
	else if (var->bits_per_pixel <= 32)
		var->bits_per_pixel = 32;
	else
		return -EINVAL;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	/*
	 *  Memory limit
	 */
	line_length =
		get_line_length(var->xres_virtual, var->bits_per_pixel);
	if (videomemorysize &&  line_length * var->yres_virtual > videomemorysize)
		return -ENOMEM;

	/*
	 * Now that we checked it we alter var. The reason being is that the video
	 * mode passed in might not work but slight changes to it might make it
	 * work. This way we let the user know what is acceptable.
	 */
	switch (var->bits_per_pixel) {
	case 1:
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 15:		/* RGBA 555 */
		var->red.offset = 10;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 5;
		var->blue.offset = 0;
		var->blue.length = 5;
		break;
	case 16:		/* BGR 565 */
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 24:		/* RGB 888 */
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 32:		/* ARGB 8888 */
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		break;
	}
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;

	return 0;
}

static unsigned int cal_freq(unsigned int pixclock_khz, struct pix_pll * pll_config)
{
	unsigned int pstdiv, loopc, frefc;
	unsigned long a, b, c;
	unsigned long min = 1000;

	for (pstdiv = 1; pstdiv < 64; pstdiv++) {
		a = (unsigned long)pixclock_khz * pstdiv;
		for (frefc = 3; frefc < 6; frefc++) {
			for (loopc = 24; loopc < 161; loopc++) {

				if ((loopc < 12 * frefc) || 
						(loopc > 32 * frefc))
					continue;

				b = 100000L * loopc / frefc;
				c = (a > b) ? (a - b) : (b - a);
				if (c < min) {

					pll_config->l2_div = pstdiv;
					pll_config->l1_loopc = loopc;
					pll_config->l1_frefc = frefc;
					/*printk("found = %d, pix = %d khz; min = %ld; pstdiv = %x;"*/
							/*"loopc = %x; frefc = %x\n", found, pixclock_khz, */
							/*min, pll_config->l2_div, pll_config->l1_loopc, */
							/*pll_config->l1_frefc);*/


					return 1;
				}
			}	

		}
	}

	return 0;
}

static void config_pll(unsigned long pll_base, struct pix_pll *pll_cfg)
{
	unsigned long out;

	out = (1 << 7) | (1L << 42) | (3 << 10) | 
		((unsigned long)(pll_cfg->l1_loopc) << 32) | 
		((unsigned long)(pll_cfg->l1_frefc) << 26); 

	ls2k_writeq(0, pll_base + LO_OFF);
	ls2k_writeq(1 << 19, pll_base + LO_OFF);
	ls2k_writeq(out, pll_base + LO_OFF);
	ls2k_writeq(pll_cfg->l2_div, pll_base + HI_OFF);
	out = (out | (1 << 2));
	ls2k_writeq(out, pll_base + LO_OFF);

	while (!(ls2k_readq(pll_base + LO_OFF) & 0x10000)) ;

	ls2k_writeq((out | 1), pll_base + LO_OFF);
}
static void ls2k_reset_cursor_image(void)
{
	u8 __iomem *addr = (u8 *)DEFAULT_CURSOR_MEM;
	memset(addr, 0, 32*32*4);
}

static int ls2k_init_regs(struct fb_info *info)
{
	unsigned int pix_freq;
	unsigned int depth;
	unsigned int hr, hss, hse, hfl;
	unsigned int vr, vss, vse, vfl;
	int ret;
	struct pix_pll pll_cfg;
	struct fb_var_screeninfo *var = &info->var;
	struct ls2k_fb_par *par = (struct ls2k_fb_par *)info->par;
	unsigned long base = par->reg_base;

	hr	= var->xres;
	hss	= hr + var->right_margin;
	hse	= hss + var->hsync_len;
	hfl	= hse + var->left_margin;

	vr	= var->yres;
	vss	= vr + var->lower_margin;
	vse	= vss + var->vsync_len;
	vfl	= vse + var->upper_margin;

	depth = var->bits_per_pixel;
	pix_freq = PICOS2KHZ(var->pixclock);

	ret = cal_freq(pix_freq, &pll_cfg);
	if (ret) {
		config_pll(LS2K_PIX0_PLL, &pll_cfg);
		config_pll(LS2K_PIX1_PLL, &pll_cfg);
	}


	ls2k_writel(dma_A, base + LS2K_FB_ADDR0_DVO0_REG);
	ls2k_writel(dma_A, base + LS2K_FB_ADDR0_DVO1_REG);
	ls2k_writel(dma_A, base + LS2K_FB_ADDR1_DVO0_REG);
	ls2k_writel(dma_A, base + LS2K_FB_ADDR1_DVO1_REG);
	ls2k_writel(0, base + LS2K_FB_DITCFG_DVO0_REG);
	ls2k_writel(0, base + LS2K_FB_DITCFG_DVO1_REG);
	ls2k_writel(0, base + LS2K_FB_DITTAB_LO_DVO0_REG);
	ls2k_writel(0, base + LS2K_FB_DITTAB_LO_DVO1_REG);
	ls2k_writel(0, base + LS2K_FB_DITTAB_HI_DVO0_REG);
	ls2k_writel(0, base + LS2K_FB_DITTAB_HI_DVO1_REG);
	ls2k_writel(clockpol&1?0x80001111:0x80001311, base + LS2K_FB_PANCFG_DVO0_REG);
	ls2k_writel(clockpol&2?0x80001111:0x80001311, base + LS2K_FB_PANCFG_DVO1_REG);
	ls2k_writel(0x00000000, base + LS2K_FB_PANTIM_DVO0_REG);//mtf add
	ls2k_writel(0x00000000, base + LS2K_FB_PANTIM_DVO1_REG);//mtf add

/* these 4 lines cause out of range, because 
 * the hfl hss vfl vss are different with PMON vgamode cfg.
 * So the refresh freq in kernel and refresh freq in PMON are different.
 * */
	ls2k_writel((hfl << 16) | hr, base + LS2K_FB_HDISPLAY_DVO0_REG);
	ls2k_writel((hfl << 16) | hr, base + LS2K_FB_HDISPLAY_DVO1_REG);
	ls2k_writel(0x40000000 | (hse << 16) | hss, base + LS2K_FB_HSYNC_DVO0_REG);
	ls2k_writel(0x40000000 | (hse << 16) | hss, base + LS2K_FB_HSYNC_DVO1_REG);
	ls2k_writel((vfl << 16) | vr, base + LS2K_FB_VDISPLAY_DVO0_REG);
	ls2k_writel((vfl << 16) | vr, base + LS2K_FB_VDISPLAY_DVO1_REG);
	ls2k_writel(0x40000000 | (vse << 16) | vss, base + LS2K_FB_VSYNC_DVO0_REG);
	ls2k_writel(0x40000000 | (vse << 16) | vss, base + LS2K_FB_VSYNC_DVO1_REG);

	switch (depth) {
	case 32:
	case 24:
		ls2k_writel(0x00100104, base + LS2K_FB_CFG_DVO0_REG);
		ls2k_writel((hr * 4 + 255) & ~255, base + LS2K_FB_STRI_DVO0_REG);
		ls2k_writel(0x00100104, base + LS2K_FB_CFG_DVO1_REG);
		ls2k_writel((hr * 4 + 255) & ~255, base + LS2K_FB_STRI_DVO1_REG);
		break;
	case 16:
		ls2k_writel(0x00100103, base + LS2K_FB_CFG_DVO0_REG);
		ls2k_writel((hr * 2 + 255) & ~255, base + LS2K_FB_STRI_DVO0_REG);
		ls2k_writel(0x00100103, base + LS2K_FB_CFG_DVO1_REG);
		ls2k_writel((hr * 2 + 255) & ~255, base + LS2K_FB_STRI_DVO1_REG);
		break;
	case 15:
		ls2k_writel(0x00100102, base + LS2K_FB_CFG_DVO0_REG);
		ls2k_writel((hr * 2 + 255) & ~255, base + LS2K_FB_STRI_DVO0_REG);
		ls2k_writel(0x00100102, base + LS2K_FB_CFG_DVO1_REG);
		ls2k_writel((hr * 2 + 255) & ~255, base + LS2K_FB_STRI_DVO1_REG);
		break;
	case 12:
		ls2k_writel(0x00100101, base + LS2K_FB_CFG_DVO0_REG);
		ls2k_writel((hr * 2 + 255) & ~255, base + LS2K_FB_STRI_DVO0_REG);
		ls2k_writel(0x00100101, base + LS2K_FB_CFG_DVO1_REG);
		ls2k_writel((hr * 2 + 255) & ~255, base + LS2K_FB_STRI_DVO1_REG);
		break;
	default:
		ls2k_writel(0x00100104, base + LS2K_FB_CFG_DVO0_REG);
		ls2k_writel((hr * 4 + 255) & ~255, base + LS2K_FB_STRI_DVO0_REG);
		ls2k_writel(0x00100104, base + LS2K_FB_CFG_DVO1_REG);
		ls2k_writel((hr * 4 + 255) & ~255, base + LS2K_FB_STRI_DVO1_REG);
		break;
	}

/* cursor */
	/* Select full color ARGB mode */
	ls2k_writel(0x00050202, base + LS2K_FB_CUR_CFG_REG);
	ls2k_writel(cursor_dma, base + LS2K_FB_CUR_ADDR_REG);
	ls2k_writel(0x00060122, base + LS2K_FB_CUR_LOC_ADDR_REG);
	ls2k_writel(0x00eeeeee, base + LS2K_FB_CUR_BACK_REG);
	ls2k_writel(0x00aaaaaa, base + LS2K_FB_CUR_FORE_REG);
	ls2k_reset_cursor_image();
/* enable interupt */
	ls2k_writel(0x280 << 16, base + LS2K_FB_INT_REG);
	return 0;
}

#ifdef LS2K_FB_DEBUG
void show_var(struct fb_var_screeninfo *var)
{
	printk(" xres: %d\n"
		" yres: %d\n",
		var->xres,
		var->yres);
}
#endif

/* This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the
 * change in par. For this driver it doesn't do much.
 */
static int ls2k_fb_set_par(struct fb_info *info)
{
	unsigned long flags;
	info->fix.line_length = get_line_length(info->var.xres_virtual,
						info->var.bits_per_pixel);
#if defined(CONFIG_DMA_NONCOHERENT) || defined(CONFIG_DC_NOCOHERENT)
	*(volatile int *)(int)0xbfe10430 &= ~8; 
#else
	*(volatile int *)(int)0xbfe10430 |= 8; 
#endif
#ifdef LS2K_FB_DEBUG
	show_var(&info->var);
#endif
	spin_lock_irqsave(&fb_lock, flags);

	ls2k_init_regs(info);

	spin_unlock_irqrestore(&fb_lock, flags);
	return 0;
}

/*
 *  Set a single color register. The values supplied are already
 *  rounded down to the hardware's capabilities (according to the
 *  entries in the var structure). Return != 0 for invalid regno.
 */

static int ls2k_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info)
{
	if (regno >= 256)	/* no. of hw registers */
		return 1;
	/*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue =
			(red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Directcolor:
	 *   var->{color}.offset contains start of bitfield
	 *   var->{color}.length contains length of bitfield
	 *   {hardwarespecific} contains width of RAMDAC
	 *   cmap[X] is programmed to (X << red.offset) | (X << green.offset) | (X << blue.offset)
	 *   RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Pseudocolor:
	 *    uses offset = 0 && length = RAMDAC register width.
	 *    var->{color}.offset is 0
	 *    var->{color}.length contains widht of DAC
	 *    cmap is not used
	 *    RAMDAC[X] is programmed to (red, green, blue)
	 * Truecolor:
	 *    does not use DAC. Usually 3 are present.
	 *    var->{color}.offset contains start of bitfield
	 *    var->{color}.length contains length of bitfield
	 *    cmap is programmed to (red << red.offset) | (green << green.offset) |
	 *                      (blue << blue.offset) | (transp << transp.offset)
	 *    RAMDAC does not exist
	 */
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		v = (red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset) |
			(transp << info->var.transp.offset);
		switch (info->var.bits_per_pixel) {
		case 8:
			break;
		case 16:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		return 0;
	}
	return 0;
}

static int ls2k_fb_blank (int blank_mode, struct fb_info *info)
{
	return 0;
}

/*************************************************************
 *                Hardware Cursor Routines                   *
 *************************************************************/

/**
 * ls2k_enable_cursor - show or hide the hardware cursor
 * @mode: show (1) or hide (0)
 *
 * Description:
 * Shows or hides the hardware cursor
 */
static void ls2k_enable_cursor(int mode, unsigned long base)
{
	unsigned int tmp = ls2k_readl(base + LS2K_FB_CUR_CFG_REG);
	tmp &= ~0xff;
	ls2k_writel(mode ? (tmp | 0x02) : (tmp | 0x00),
			base + LS2K_FB_CUR_CFG_REG);
}

static void ls2k_load_cursor_image(int width, int height, u8 *data)
{
	u32 __iomem *addr = (u32 *)DEFAULT_CURSOR_MEM;
	int row, col, i, j, bit = 0;
	col = (width > CUR_HEIGHT_SIZE)? CUR_HEIGHT_SIZE : width;
	row = (height > CUR_WIDTH_SIZE)? CUR_WIDTH_SIZE : height;

	for (i = 0; i < CUR_HEIGHT_SIZE; i++) {
		for (j = 0; j < CUR_WIDTH_SIZE; j++) {
			if (i < height && j < width) {
				bit = data[(i * width + width - j) >> 3] &
					(1 << ((i * width + width - j) & 0x7));
				addr[i * CUR_WIDTH_SIZE + j] =
					bit ? 0xffffffff : 0;
			 } else {
				addr[i * CUR_WIDTH_SIZE + j] = 0x0;
			 }
		}
	}
}


static int ls2k_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	struct ls2k_fb_par *par = (struct ls2k_fb_par *)info->par;
	unsigned long base = par->reg_base;

	if (cursor->image.width > CUR_WIDTH_SIZE ||
			cursor->image.height > CUR_HEIGHT_SIZE)
		return -ENXIO;

	ls2k_enable_cursor(OFF, base);

	if (cursor->set & FB_CUR_SETPOS) {
		u32 tmp;

		tmp = (cursor->image.dx - info->var.xoffset) & 0xffff;
		tmp |= (cursor->image.dy - info->var.yoffset) << 16;
		ls2k_writel(tmp, base + LS2K_FB_CUR_LOC_ADDR_REG);
	}

	if (cursor->set & FB_CUR_SETSIZE)
		ls2k_reset_cursor_image();

	if (cursor->set & FB_CUR_SETHOT) {
		u32 hot = (cursor->hot.x << 16) | (cursor->hot.y << 8);
		u32 con = ls2k_readl(base + LS2K_FB_CUR_CFG_REG) & 0xff;
		ls2k_writel(hot | con, base + LS2K_FB_CUR_CFG_REG);
	}

	if (cursor->set & FB_CUR_SETCMAP)
		;

	if (cursor->set & (FB_CUR_SETSHAPE | FB_CUR_SETIMAGE)) {
		int size = ((cursor->image.width + 7) >> 3) *
			cursor->image.height;
		int i;
		u8 *data = kmalloc(32 * 32 * 4, GFP_ATOMIC);

		if (data == NULL)
			return -ENOMEM;

		switch (cursor->rop) {
		case ROP_XOR:
			for (i = 0; i < size; i++)
				data[i] = cursor->image.data[i] ^ cursor->mask[i];
			break;
		case ROP_COPY:
		default:
			for (i = 0; i < size; i++)
				data[i] = cursor->image.data[i] & cursor->mask[i];
			break;
		}

		ls2k_load_cursor_image(cursor->image.width,
				       cursor->image.height, data);
		kfree(data);
	}

	if (cursor->enable)
		ls2k_enable_cursor(ON, base);

	return 0;
}

struct cursor_req {
	u32 x;
	u32 y;
};

#define CURIOSET_CORLOR		0x4607
#define CURIOSET_POSITION	0x4608
#define CURIOLOAD_ARGB		0x4609
#define CURIOLOAD_IMAGE		0x460A
#define CURIOHIDE_SHOW		0x460B
#define FBEDID_GET		0x860C

static int ls2k_fb_ioctl(struct fb_info *info, unsigned int cmd,
		                        unsigned long arg)
{
	u32 tmp;
	struct cursor_req req;
	struct ls2k_fb_par *par = (struct ls2k_fb_par *)info->par;
	unsigned long base = par->reg_base;
	void __user *argp = (void __user *)arg;
	u8 *cursor_base = (u8 *)DEFAULT_CURSOR_MEM;

	switch (cmd) {
	case CURIOSET_CORLOR:
		break;
	case CURIOSET_POSITION:
		LS2K_DEBUG("CURIOSET_POSITION\n");
		if (copy_from_user(&req, argp, sizeof(struct cursor_req)))
			return -EFAULT;
		tmp = (req.x - info->var.xoffset) & 0xffff;
		tmp |= (req.y - info->var.yoffset) << 16;
		ls2k_writel(tmp, base + LS2K_FB_CUR_LOC_ADDR_REG);
		break;
	case CURIOLOAD_ARGB:
		LS2K_DEBUG("CURIOLOAD_ARGB\n");
		if (copy_from_user(cursor_base, argp, 32 * 32 * 4))
			return -EFAULT;
		break;
	case CURIOHIDE_SHOW:
		LS2K_DEBUG("CURIOHIDE_SHOW:%s\n", arg ? "show" : "hide");
		ls2k_enable_cursor(arg, base);
		break;
	case FBEDID_GET:
		LS2K_DEBUG("FBEDID GET\n");
		if(par->edid != NULL)
		{
			if (copy_to_user(argp, par->edid, EDID_LENGTH))
				return -EFAULT;
		}
		break;
	default:
		return -ENOTTY;

	}

	return 0;
}


/*
 *  Pan or Wrap the Display
 *
 *  This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
 */

static int ls2k_fb_pan_display(struct fb_var_screeninfo *var,
			struct fb_info *info)
{
	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset < 0
			|| var->yoffset >= info->var.yres_virtual
			|| var->xoffset)
			return -EINVAL;
	} else {
		if (var->xoffset + var->xres > info->var.xres_virtual ||
			var->yoffset + var->yres > info->var.yres_virtual)
			return -EINVAL;
	}
	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;
	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;
	return 0;
}

#ifndef MODULE
static int __init ls2k_fb_setup(char *options)
{
	char *this_opt;
	ls2k_fb_enable = 1;

	if (!options || !*options)
		return 1;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)
			continue;
		if (!strncmp(this_opt, "disable", 7))
			ls2k_fb_enable = 0;
		else
			mode_option = this_opt;
	}
	return 1;
}
#endif	/* MODULE */

static const struct i2c_device_id eep_ids[] = {
	{ "dvo0-eeprom-edid", 0 },
	{ "dvo1-eeprom-edid", 1 },
	{ /* END OF LIST */ }
};

static struct i2c_client * dvo_client[2];

MODULE_DEVICE_TABLE(i2c, eep_ids);


static int eep_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	dvo_client[0] = client;
	return 0;
}

static int eep_remove(struct i2c_client *client)
{

	i2c_unregister_device(client);
	return 0;
}

static struct i2c_driver eep_driver = {
	.driver = {
		.name = "eep-edid",
		.owner = THIS_MODULE,
	},
	.probe = eep_probe,
	.remove = eep_remove,
	.id_table = eep_ids,
};

static unsigned char *fb_do_probe_ddc_edid(struct i2c_client *client)
{
	unsigned char start = 0x0;
	unsigned char *buf = kmalloc(EDID_LENGTH, GFP_KERNEL);
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &start,
		}, {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= EDID_LENGTH,
			.buf	= buf,
		}
	};

	if (!buf) {
		dev_warn(&client->adapter->dev, "unable to allocate memory for EDID "
			 "block.\n");
		return NULL;
	}

	if (i2c_transfer(client->adapter, msgs, 2) == 2)
		return buf;

	dev_warn(&client->adapter->dev, "unable to read EDID block.\n");
	kfree(buf);
	return NULL;
}

static unsigned char *ls2k_fb_i2c_connector(struct ls2k_fb_par *fb_par)
{
	int i;
	unsigned char *edid = NULL;

	if (i2c_add_driver(&eep_driver)) {
		pr_err("i2c-%ld No eeprom device register!",eep_driver.id_table->driver_data);
		return NULL;
	}

	for(i = 0; i < 2; i++) {
		if(!dvo_client[i])
			continue;
		edid = fb_do_probe_ddc_edid(dvo_client[i]);

		if (edid) {
			fb_par->edid = edid;
		}
		return edid;
	}

	return NULL;
}

static void ls2k_find_init_mode(struct fb_info *info)
{
        struct fb_videomode mode;
        struct fb_var_screeninfo var;
	unsigned char *edid;
        struct fb_monspecs *specs = &info->monspecs;
	struct ls2k_fb_par *par = info->par;
        int found = 0;

	info->var = ls2k_fb_default;
        INIT_LIST_HEAD(&info->modelist);
        memset(&mode, 0, sizeof(struct fb_videomode));
        memset(&var, 0, sizeof(struct fb_var_screeninfo));
	var.bits_per_pixel = DEFAULT_BITS_PER_PIXEL;

	edid = ls2k_fb_i2c_connector(par);
	if (!edid) {
		goto def;
	}

	fb_edid_to_monspecs(par->edid, specs);

        if (specs->modedb == NULL) {
		printk("ls2h-fb: Unable to get Mode Database\n");
		goto def;
	}

        fb_videomode_to_modelist(specs->modedb, specs->modedb_len,
                                 &info->modelist);
        if (specs->modedb != NULL) {
                const struct fb_videomode *m;
                if (!found) {
                        m = fb_find_best_display(&info->monspecs, &info->modelist);
                        mode = *m;
                        found = 1;
                }

                fb_videomode_to_var(&var, &mode);
        }

        if (mode_option) {
		printk("mode_option: %s\n", mode_option);
                fb_find_mode(&var, info, mode_option, specs->modedb,
                             specs->modedb_len, (found) ? &mode : NULL,
                             info->var.bits_per_pixel);
	}

	fb_destroy_modedb(specs->modedb);
	specs->modedb = NULL;
	info->var = var;
	return;
def:
	info->var = ls2k_fb_default;
        if (mode_option) {
		printk("mode_option: %s\n", mode_option);
                if(fb_find_mode(&var, info, mode_option, specs->modedb,
                             specs->modedb_len, (found) ? &mode : NULL,
                             info->var.bits_per_pixel))
		info->var = var;
	}
	return;
}

/* irq */
static irqreturn_t ls2kfb_irq(int irq, void *dev_id)
{
	unsigned int val, cfg;
	unsigned long flags;
	struct fb_info *info = (struct fb_info *) dev_id;
	struct ls2k_fb_par *par = (struct ls2k_fb_par *)info->par;
	unsigned long base = par->reg_base;

	spin_lock_irqsave(&fb_lock, flags);

	val = ls2k_readl(base + LS2K_FB_INT_REG);
	ls2k_writel(val & (0xffff << 16), base + LS2K_FB_INT_REG);

	cfg = ls2k_readl(base + LS2K_FB_CFG_DVO0_REG);
	cfg = ls2k_readl(base + LS2K_FB_CFG_DVO1_REG);
	/* if underflow, reset VGA */
	if (val & 0x280) {
		ls2k_writel(0, base + LS2K_FB_CFG_DVO0_REG);
		ls2k_writel(cfg, base + LS2K_FB_CFG_DVO0_REG);
		ls2k_writel(0, base + LS2K_FB_CFG_DVO1_REG);
		ls2k_writel(cfg, base + LS2K_FB_CFG_DVO1_REG);
	}

	spin_unlock_irqrestore(&fb_lock, flags);

	return IRQ_HANDLED;
}

static struct fb_ops ls2k_fb_ops = {
	.owner			= THIS_MODULE,
	.fb_check_var		= ls2k_fb_check_var,
	.fb_set_par		= ls2k_fb_set_par,
	.fb_setcolreg		= ls2k_fb_setcolreg,
	.fb_blank		= ls2k_fb_blank,
	.fb_pan_display		= ls2k_fb_pan_display,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_cursor		= ls2k_fb_cursor,
	.fb_ioctl		= ls2k_fb_ioctl,
	.fb_compat_ioctl	= ls2k_fb_ioctl,
};

/*
 *  Initialisation
 */
static int ls2k_fb_probe(struct platform_device *dev)
{
	int irq;
	struct fb_info *info;
	int retval = -ENOMEM;
	struct ls2k_fb_par *par;
	struct resource *r;

	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		dev_err(&dev->dev, "no irq for device\n");
		return -ENOENT;
	}

	info = framebuffer_alloc(sizeof(u32) * 256, &dev->dev);
	if (!info)
		return -ENOMEM;

	info->fix = ls2k_fb_fix;
	info->node = -1;
	info->fbops = &ls2k_fb_ops;
	info->pseudo_palette = info->par;
	info->flags = FBINFO_FLAG_DEFAULT;

	par = kzalloc(sizeof(struct ls2k_fb_par), GFP_KERNEL);
	if (!par) {
		retval = -ENOMEM;
		goto release_info;
	}

	info->par = par;
	par->fb_info = info;
	par->pdev = dev;
	par->irq = irq;
	r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!r) {
		retval = -ENOMEM;
		goto release_par;
	}

	par->reg_base = r->start;
	
	if(!ls2k_fb_enable)
	{
		ls2k_writel(ls2k_readl(par->reg_base  + LS2K_FB_CFG_DVO0_REG)&~0x100, par->reg_base  + LS2K_FB_CFG_DVO0_REG);
		ls2k_writel(ls2k_readl(par->reg_base  + LS2K_FB_CFG_DVO1_REG)&~0x100, par->reg_base  + LS2K_FB_CFG_DVO1_REG);
		ls2k_writel(ls2k_readl(par->reg_base  + LS2K_FB_PANCFG_DVO0_REG)&~0x1, par->reg_base  + LS2K_FB_PANCFG_DVO0_REG);
		ls2k_writel(ls2k_readl(par->reg_base  + LS2K_FB_PANCFG_DVO1_REG)&~0x1, par->reg_base  + LS2K_FB_PANCFG_DVO1_REG);
		return 0;
	}
	
	ls2k_find_init_mode(info);

	if (!videomemorysize) {
		videomemorysize = max(info->var.xres_virtual *
					info->var.yres_virtual *
					info->var.bits_per_pixel / 8, 1600*1280*16/8);
	}

	/*
	 * For real video cards we use ioremap.
	 */
	videomemory = (void *)DEFAULT_FB_MEM;
	dma_A = (dma_addr_t)DEFAULT_FB_DMA;

	pr_info("videomemory=%lx\n",(unsigned long)videomemory);
	pr_info("videomemorysize=%lx\n",videomemorysize);
	pr_info("dma_A=%x\n",(int)dma_A);
	memset(videomemory, 0, videomemorysize);

	cursor_mem = (void *)DEFAULT_CURSOR_MEM;
	cursor_dma = (dma_addr_t)DEFAULT_CURSOR_DMA;
	memset (cursor_mem,0x88FFFF00,cursor_size);

	info->screen_base = (char __iomem *)videomemory;
	info->fix.smem_start = DEFAULT_PHY_ADDR;
	info->fix.smem_len = videomemorysize;

	retval = fb_alloc_cmap(&info->cmap, 32, 0);
	if (retval < 0) goto release_par;

	info->fbops->fb_check_var(&info->var, info);
	par->htotal = info->var.xres;
	par->vtotal = info->var.yres;
	retval = register_framebuffer(info);
	if (retval < 0)
		goto release_map;

	retval = request_irq(irq, ls2kfb_irq, IRQF_DISABLED, dev->name, info);
	if (retval) {
		dev_err(&dev->dev, "cannot get irq %d - err %d\n", irq, retval);
		goto unreg_info;
	}

	platform_set_drvdata(dev, info);

	pr_info("fb%d: Virtual frame buffer device, using %ldK of"
			"video memory\n", info->node, videomemorysize >> 10);

	return 0;
unreg_info:
	unregister_framebuffer(info);
release_map:
	fb_dealloc_cmap(&info->cmap);
release_par:
	kfree(par);
release_info:
	platform_set_drvdata(dev, NULL);
	framebuffer_release(info);
	return retval;
}

static int ls2k_fb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct ls2k_fb_par *par = info->par;
	int irq = par->irq;

	free_irq(irq, info);
	fb_dealloc_cmap(&info->cmap);
	unregister_framebuffer(info);
	platform_set_drvdata(dev, info);
	framebuffer_release(info);
	if (par->edid)
		kfree(par->edid);
	kfree(par);

	return 0;
}

#if    defined(CONFIG_SUSPEND)
/**
 **      ls2k_fb_suspend - Suspend the device.
 **      @dev: platform device
 **      @msg: the suspend event code.
 **
 **      See Documentation/power/devices.txt for more information
 **/
static u32 output_mode;
void console_lock(void);
void console_unlock(void);

static int ls2k_fb_suspend(struct platform_device *dev, pm_message_t msg)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct ls2k_fb_par *par = (struct ls2k_fb_par *)info->par;
	unsigned long base = par->reg_base;

	console_lock();
	fb_set_suspend(info, 1);
	console_unlock();

	output_mode = ls2k_readl(base + LS2K_FB_DVO_OUTPUT_REG);

	return 0;
}

/**
 **      ls2k_fb_resume - Resume the device.
 **      @dev: platform device
 **
 **      See Documentation/power/devices.txt for more information
 **/
static int ls2k_fb_resume(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct ls2k_fb_par *par = (struct ls2k_fb_par *)info->par;
	unsigned long base = par->reg_base;

	ls2k_fb_set_par(info);
	ls2k_writel(output_mode, base + LS2K_FB_DVO_OUTPUT_REG);

	console_lock();
	fb_set_suspend(info, 0);
	console_unlock();

	return 0;
}
#endif

static struct platform_driver ls2k_fb_driver = {
	.probe	= ls2k_fb_probe,
	.remove = ls2k_fb_remove,
#ifdef	CONFIG_SUSPEND
	.suspend = ls2k_fb_suspend,
	.resume	 = ls2k_fb_resume,
#endif
	.driver = {
		.name	= "ls2k-fb",
	},
};



static struct pci_device_id ls2k_fb_devices[] = {
	{PCI_DEVICE(0x14, 0x7a06)},
	{0, 0, 0, 0, 0, 0, 0}
};

/*
 * DC
 */
static struct resource ls2k_dc_resources[] = {
	[0] = {
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ls2k_dc_device = {
	.name           = "ls2k-fb",
	.id             = 0,
	.num_resources	= ARRAY_SIZE(ls2k_dc_resources),
	.resource	= ls2k_dc_resources,
};

static int ls2k_fb_pci_register(struct pci_dev *pdev,
				 const struct pci_device_id *ent)
{
	int ret;
	unsigned char v8;

	pr_debug("ls2k_fb_pci_register BEGIN\n");
	
	/* Enable device in PCI config */
	ret = pci_enable_device(pdev);
	if (ret < 0) {
		printk(KERN_ERR "ls2kfb (%s): Cannot enable PCI device\n",
		       pci_name(pdev));
		goto err_out;
	}

	/* request the mem regions */
	ret = pci_request_region(pdev, 0, "ls2kfb io");
	if (ret < 0) {
		printk( KERN_ERR "ls2kfb (%s): cannot request region 0.\n",
			pci_name(pdev));
		goto err_out;
	}

	ls2k_dc_resources[0].start = pci_resource_start (pdev, 0);
	ls2k_dc_resources[0].end = pci_resource_end(pdev, 0);

	ret = pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &v8); //need api from pci irq 

	if (ret == PCIBIOS_SUCCESSFUL) {

		ls2k_dc_resources[1].start = v8;
		ls2k_dc_resources[1].end = v8;

		platform_device_register(&ls2k_dc_device);
		platform_driver_register(&ls2k_fb_driver);

	}
err_out:
	return ret;
}

static void ls2k_fb_pci_unregister(struct pci_dev *pdev)
{

	platform_driver_unregister(&ls2k_fb_driver);
	pci_release_region(pdev, 0);
}

int ls2k_fb_pci_suspend(struct pci_dev *pdev, pm_message_t mesg)
{
	ls2k_fb_suspend(&ls2k_dc_device, mesg);
	pci_save_state(pdev);
	return 0;
}

int ls2k_fb_pci_resume(struct pci_dev *pdev)
{
	ls2k_fb_resume(&ls2k_dc_device);
	return 0;
}

static struct pci_driver ls2k_fb_pci_driver = {
	.name		= "ls2k-fb",
	.id_table	= ls2k_fb_devices,
	.probe		= ls2k_fb_pci_register,
	.remove		= ls2k_fb_pci_unregister,
#ifdef	CONFIG_SUSPEND
	.suspend = ls2k_fb_pci_suspend,
	.resume	 = ls2k_fb_pci_resume,
#endif
};

static int __init ls2k_fb_init (void)
{
#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("ls2k-fb", &option))
		return -ENODEV;
	ls2k_fb_setup(option);
#endif
	return pci_register_driver (&ls2k_fb_pci_driver);
}


static void __exit ls2k_fb_exit (void)
{
	i2c_del_driver(&eep_driver);
	pci_unregister_driver (&ls2k_fb_pci_driver);
}

module_init(ls2k_fb_init);
module_exit(ls2k_fb_exit);

MODULE_LICENSE("GPL");
