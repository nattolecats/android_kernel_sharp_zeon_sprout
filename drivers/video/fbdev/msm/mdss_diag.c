/* drivers/video/fbdev/msm/mdss_diag.c  (Display Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include "mdss_mdp.h"
#include "mdss_dsi.h"
#include "mdss_fb.h"
#include "mdss_debug.h"
#include "mdss_diag.h"
#include "msm_mdss_context.h"
#if 0
#include "mdss_det.h"
#endif

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define USES_PANEL_SINANJU
#if 0
#if defined(CONFIG_ARCH_DOPPIO)||defined(CONFIG_ARCH_JOSUKE)
#define USES_PANEL_SINANJU
#else /* CONFIG_ARCH_DOPPIO */
#define USES_PANEL_HAYABUSA
#endif /* CONFIG_ARCH_DOPPIO */
#endif

#if defined(USES_PANEL_ANDY)
#define MDSS_DIAG_MIPI_CHECK_ENABLE
#define MIPICHK_AMP
#define MDSS_DIAG_MIPI_CLKCHG_ENABLE
#define MDSS_DIAG_PANEL_GMM_VOLTAGE
#define MDSS_DIAG_PANEL_FLICKER
#define MDSS_DIAG_DEFAULT_PAGE 0x10
#elif defined(USES_PANEL_SINANJU)
#define MDSS_DIAG_PANEL_VIDEO_MODE
#define MDSS_DIAG_MIPI_CHECK_ENABLE
#define MDSS_DIAG_DEFAULT_PAGE -1
#elif defined(USES_PANEL_HAYABUSA)
#define MDSS_DIAG_PANEL_VIDEO_MODE
#define MDSS_DIAG_MIPI_CHECK_ENABLE
#define MDSS_DIAG_PANEL_GMM_VOLTAGE
#define MDSS_DIAG_PANEL_FLICKER
#define MDSS_DIAG_DEFAULT_PAGE 0x10
#else  /* USES_PANEL_XXX */
  #error undefined panel
#endif  /* USES_PANEL_XXX */

#if defined(MDSS_DIAG_PANEL_GMM_VOLTAGE) || defined(MDSS_DIAG_PANEL_FLICKER)
#include <soc/qcom/sh_smem.h>
#include "msm_mdss_context.h"
#endif /* defined(MDSS_DIAG_PANEL_GMM_VOLTAGE) || defined(MDSS_DIAG_PANEL_FLICKER) */

#define MDSS_DSI_DSIPHY_REGULATOR_CTRL_0	(0x00)
#define MDSS_DIAG_WAIT_1FRAME_US		(16666)
#define MDSS_DIAG_WAIT_SLEEPOUT_US		(60 * 1000)

#ifdef MDSS_DIAG_PANEL_FLICKER
#define FLICKER_SET_ALL			(MSMFB_REG_WRITE | MSMFB_SAVE_VALUE | \
					MSMFB_SAVE_VALUE_LOW | MSMFB_RESET_VALUE)
#endif /* MDSS_DIAG_PANEL_FLICKER */

#define MDSS_MIPICHK_AMP_NUM             (8)
#if defined(USES_PANEL_SINANJU)
#define MDSS_MIPICHK_SENSITIV_NUM        (8)
#else /* USES_PANEL_SINANJU */
#define MDSS_MIPICHK_SENSITIV_NUM        (16)
#endif /* USES_PANEL_SINANJU */

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
#define MDSS_DIAG_DSI_MAX_CMDS_CNT       (512)
#define MDSS_DIAG_DSI_ONE_PAYLOAD_LENGTH (4)
#define MDSS_DIAG_DSI_PAYLOADS_LENGHTH   (MDSS_DIAG_DSI_MAX_CMDS_CNT * MDSS_DIAG_DSI_ONE_PAYLOAD_LENGTH)

struct mdss_diag_panel_pad {
	unsigned char page;
	unsigned char addr;
	unsigned char len;
	unsigned char *data;
};

struct mdss_diag_pad_item {
	struct mdss_diag_panel_pad *pad;
	size_t len;
};

#if defined(USES_PANEL_ANDY)
#include "mdss_diag_andy.h"
#elif defined(USES_PANEL_SINANJU)
#include "mdss_diag_sinanju.h"
#elif defined(USES_PANEL_HAYABUSA)
#include "mdss_diag_hayabusa.h"
#endif  /* USES_PANEL_XXX */

/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
int mdss_diag_panel_dcs_write0(struct mdss_panel_data *pdata,
				char addr);
int mdss_diag_panel_dcs_write1(struct mdss_panel_data *pdata,
				char addr,
				char data);
static int mdss_diag_panel_dcs_read(struct mdss_panel_data *pdata, char addr,
				int rlen, char *rbuf);
static int mdss_diag_panel_switch_panel_page(struct mdss_panel_data *pdata,
				short page);

#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
static int mdss_diag_mipi_check_exec(struct msm_fb_data_type *mfd, uint8_t *result, uint8_t frame_cnt, uint8_t amp, uint8_t sensitiv,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl,
			uint8_t *sensitiv_master, uint8_t *sensitiv_slave);
static int mdss_diag_mipi_check_manual(struct msm_fb_data_type *mfd, struct mdp_mipi_check_param *mipi_check_param,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl,
			uint8_t *sensitiv_master, uint8_t *sensitiv_slave);
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave,
			struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_diag_mipi_check_test(struct msm_fb_data_type *mfd, uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static int mdss_diag_mipi_check_test_video(uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static int mdss_diag_mipi_check_test_cmd(struct msm_fb_data_type *mfd, uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static int mdss_diag_read_sensitiv(struct mdss_dsi_ctrl_pdata *ctrl, uint8_t *sensitiv_master, uint8_t *sensitiv_slave);
static int mdss_diag_write_sensitiv(struct mdss_dsi_ctrl_pdata *ctrl, uint8_t *sensitiv_master, uint8_t *sensitiv_slave);
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl);
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */

struct mdss_mdp_ctl* mdss_shdisp_get_mdpctrl(int fbinx);

#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
static int mdss_diag_mipi_clkchg_setparam(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl);
static void mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_panel_data *pdata);
static int __mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl, struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl);
static int mdss_diag_mipi_clkchg_panel_clk_update(struct mdss_panel_data *pdata);
static int __mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata);
static void mdss_diag_mipi_clkchg_param_log(struct mdp_mipi_clkchg_param *mdp_mipi_clkchg_param);
static int mdss_diag_panel_osc_set(struct mdss_panel_data *pdata, int osc);
static int mdss_diag_panel_osc_send_cmd(
				struct mdss_panel_data *pdata, int osc);
static int mdss_wait4ppdone(struct mdss_mdp_ctl *pctl);
#if defined(USES_PANEL_HAYABUSA)
static int  mdss_diag_hayabusa_osc_send_cmd(
				struct mdss_dsi_ctrl_pdata *ctrl, int osc);
#endif  /* USES_PANEL_HAYABUSA */
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

#ifdef MDSS_DIAG_PANEL_FLICKER
static int mdss_diag_set_flicker_if_adjusted(struct mdss_panel_data *pdata);
static int mdss_diag_get_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param);
static int mdss_diag_send_flicker_param(struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_vcom vcom);
static int mdss_diag_set_flicker_ctx(struct mdp_flicker_param flicker_param);
static int mdss_diag_set_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param flicker_param);
#if defined(USES_PANEL_ANDY)
static int mdss_diag_andy_read_flicker(struct mdss_dsi_ctrl_pdata *ctrl, unsigned char out_buf[3]);
static int mdss_diag_andy_get_flicker_low(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param);
static int mdss_diag_andy_get_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param);
static int mdss_diag_andy_send_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_andy_vcom *vcom);
static int mdss_diag_andy_calc_vcom(struct mdss_vcom in, struct mdss_andy_vcom *out, unsigned short vcomoffset);
#elif defined(USES_PANEL_HAYABUSA)
static int mdss_diag_hayabusa_read_flicker(struct mdss_dsi_ctrl_pdata *ctrl, unsigned char out_buf[3]);
static int mdss_diag_hayabusa_get_flicker_low(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param);
static int mdss_diag_hayabusa_get_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param);
static int mdss_diag_hayabusa_send_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_hayabusa_vcom *vcom);
static int mdss_diag_hayabusa_calc_vcom(struct mdss_vcom in, struct mdss_hayabusa_vcom *out, unsigned short vcomoffset);
#elif defined(USES_PANEL_SINANJU)
static int mdss_diag_sinanju_read_flicker(struct mdss_dsi_ctrl_pdata *ctrl, unsigned char out_buf[2]);
static int mdss_diag_sinanju_get_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param);
static int mdss_diag_sinanju_send_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_sinanju_vcom *vcom);
static int mdss_diag_sinanju_calc_vcom(struct mdss_vcom in, struct mdss_sinanju_vcom *out, unsigned short vcomoffset);
#endif  /* USES_PANEL_XXX */
static int mdss_diag_init_flicker_param(struct mdss_flicker_ctx *flicker_ctx);
#endif /* MDSS_DIAG_PANEL_FLICKER */

#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE
static int mdss_diag_panel_check_panel_type(enum panel_type panel_type);
static int mdss_diag_get_wdtype_fromlen(int datalen);
static void mdss_diag_panel_clear_cmds(void);
static bool mdss_diag_panel_add_cmd(char addr, int datalen, char *data);
static int mdss_diag_panel_kickoff_cmds(struct mdss_panel_data *pdata);
static int mdss_diag_panel_kickoff_cmds_sub(struct mdss_panel_data *pdata);
static int mdss_diag_panel_read_paditems(struct mdss_panel_data *pdata,
				size_t paditemlen,
				const struct mdss_diag_pad_item *paditems);
static int mdss_diag_panel_read_paditems_sub(struct mdss_panel_data *pdata,
				size_t paditemlen,
				const struct mdss_diag_pad_item *paditems);
static void mdss_diag_panel_update_gmm_volt_pad(
				union mdp_gmm_volt *gmm_volt);
static int mdss_diag_panel_make_gmm_volt_cmds(void);
static int mdss_diag_panel_make_gmm_volt_cmds_paditems(size_t paditemlen,
				const struct mdss_diag_pad_item *paditems);
static void mdss_diag_panel_copy_gmm_to_paddata(int paddlen,
				char *paddata,
				unsigned short *gmm);
static void mdss_diag_panel_copy_gmm_from_paddata(int len, unsigned short *gmm,
				char *paddata);
static int mdss_diag_panel_read_gmm_volt(struct mdss_panel_data *pdata);
static int mdss_diag_panel_read_gmm_volt_dispoff(struct mdss_panel_data *pdata);
static void mdss_diag_panel_copy_gmm_volt_from_pad(
				struct mdp_gmm_volt_info *gmm_volt_info);
static int mdss_diag_set_panel_typ_volt(struct mdss_panel_data *pdata);
static int mdss_diag_set_panel_typ_gmm(struct mdss_panel_data *pdata);
static int mdss_diag_panel_set_adjust_gmm(struct mdss_panel_data *pdata,
			struct mdp_gmm_volt_info *gmm_volt_info);
static int mdss_diag_panel_set_unadjust_gmm(struct mdss_panel_data *pdata,
			struct mdp_gmm_volt_info *gmm_volt_info);
static int mdss_diag_set_panel_gmm(struct mdss_panel_data *pdata);
static int mdss_diag_init_gmm_param(struct mdss_gmmvolt_ctx *gmmvolt_ctx);
static int mdss_diag_init_adjustable_params(
				struct shdisp_boot_context *shdisp_boot_ctx);
#if defined(USES_PANEL_ANDY)
static void mdss_diag_andy_update_gmm_pad(
				struct andy_gmm_volt *gmm_volt);
static void mdss_diag_andy_update_volt_pad(
				struct andy_gmm_volt *gmm_volt);
static void mdss_diag_andy_copy_gmm_volt_from_pad(
				struct andy_gmm_volt *gmm_volt);
static int mdss_diag_init_andy_gmm_param(
				struct mdss_gmmvolt_ctx *gmmvolt_ctx);
#elif defined(USES_PANEL_SINANJU)
static void mdss_diag_sinanju_copy_gmm_volt_from_pad(
				struct sinanju_gmm_volt *gmm_volt);
static void mdss_diag_sinanju_update_gmm_pad(
				struct sinanju_gmm_volt *gmm_volt);
static void mdss_diag_sinanju_update_volt_pad(
				struct sinanju_gmm_volt *gmm_volt);
static int mdss_diag_init_sinanju_gmm_param(
				struct mdss_gmmvolt_ctx *gmmvolt_ctx);
#elif defined(USES_PANEL_HAYABUSA)
static void mdss_diag_hayabusa_update_gmm_pad(
				struct hayabusa_gmm_volt *gmm_volt);
static void mdss_diag_hayabusa_update_volt_pad(
				struct hayabusa_gmm_volt *gmm_volt);
static void mdss_diag_hayabusa_update_advgmm_pad(
				struct hayabusa_gmm_volt *gmm_volt);
static void mdss_diag_hayabusa_copy_gmm_volt_from_pad(
				struct hayabusa_gmm_volt *gmm_volt);
static int mdss_diag_init_hayabusa_gmm_param(
				struct mdss_gmmvolt_ctx *gmmvolt_ctx);
#endif  /* USES_PANEL_XXX */
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */
static void mdss_diag_init_mutex(void);
#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
static void mdss_diag_mipi_clkchg_lock(struct mdss_mdp_ctl *pctl);
static void mdss_diag_mipi_clkchg_unlock(void);
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

/* ------------------------------------------------------------------------- */
/* EXTERNAL FUNCTION                                                         */
/* ------------------------------------------------------------------------- */
extern struct fb_info *mdss_fb_get_fbinfo(int id);
extern void mdss_dsi_err_intr_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, u32 mask,int enable);
//extern int mdss_fb_pan_display_ex_diag(struct fb_info *info,
//			struct mdp_display_commit *disp_commit);
//extern int mdss_fb_pan_idle_diag(struct msm_fb_data_type *mfd);
#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE
#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
extern void mdss_mdp_video_transfer_ctrl(struct mdss_mdp_ctl *ctl, int onoff);
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */
#else /* MDSS_DIAG_PANEL_GMM_VOLTAGE */
#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
extern void mdss_mdp_video_transfer_ctrl(struct mdss_mdp_ctl *ctl, int onoff);
#endif /* MDSS_DIAG_PANEL_VIDEO_MODE */
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */
#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
extern void mdss_dsi_hs_clk_lane_enable(bool enable);
extern void mdss_dsi_set_phy_regulator_enable(struct mdss_dsi_ctrl_pdata *ctrl);
extern int mdss_mdp_cmd_flush_delayed_off_clk_work(struct mdss_mdp_ctl *pctl);
extern void mdss_dsi_pll_relock(struct mdss_dsi_ctrl_pdata *ctrl);
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */
extern int mdss_mdp_update_panel_timing(struct mdss_dsi_ctrl_pdata *ctrl,
			mdp_mipi_clkchg_panel_t *panel_data);

/* ------------------------------------------------------------------------- */
/* CONTEXT                                                                   */
/* ------------------------------------------------------------------------- */
#if defined(MDSS_DIAG_PANEL_GMM_VOLTAGE)
struct mdss_gmm_kerl_ctx {
	/* gmm adjusted status */
	unsigned char			gmm_adj_status;
	/* gmm/volt/other data(used by post_on_cmd) */
	union mdp_gmm_volt		gmm_volt_adjusted;
	int				cmds_cnt;
	struct dsi_cmd_desc		cmds[MDSS_DIAG_DSI_MAX_CMDS_CNT];
	int				free_payload_pos;
	char				cmds_payloads[MDSS_DIAG_DSI_PAYLOADS_LENGHTH];
};
#endif /* defined(MDSS_DIAG_PANEL_GMM_VOLTAGE) */

struct mdss_diag_context {
#ifdef MDSS_DIAG_PANEL_FLICKER
	struct mdss_flicker_ctx mdss_flicker_ctx;
#endif /* MDSS_DIAG_PANEL_FLICKER */
#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE
	struct mdss_gmm_kerl_ctx mdss_gmm_ctx;
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */
	struct mutex blank_lock;
	struct mutex displaythread_lock;
	struct mutex chkstatus_lock;
};
static struct mdss_diag_context mdss_diag_ctx;

#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
#if defined(USES_PANEL_HAYABUSA)
#define MDSS_DIAG_PANEL_OSC_TYPES (3)
static struct osc_type {
	char rtna;
	char stv_delay;
	char stv_adv;
	char gck_delay;
	char gck_adv;
	char soe_w;
	char oscset1;
	char oscset2;
} const osc_types_value[MDSS_DIAG_PANEL_OSC_TYPES] = {
	//           rtna, stvd, stva, gckd, gcka, soew, osc1, osc2
	/* typeA */{ 0x81, 0x07, 0x1A, 0x07, 0x1A, 0x7F, 0x23, 0xBE},
	/* typeB */{ 0x7B, 0x07, 0x19, 0x07, 0x19, 0x79, 0x22, 0x2E},
	/* typeC */{ 0x75, 0x06, 0x18, 0x06, 0x18, 0x73, 0x20, 0x9E},
};
#else  /* USES_PANEL_HAYABUSA */
#define MDSS_DIAG_PANEL_OSC_TYPES (0)
#endif /* USES_PANEL_HAYABUSA */
#define MDSS_DIAG_PANEL_OSC_DEFAULT (0)
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_init_mutex(void)
{
	mutex_init(&mdss_diag_ctx.blank_lock);
	mutex_init(&mdss_diag_ctx.displaythread_lock);
	mutex_init(&mdss_diag_ctx.chkstatus_lock);
}


/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
void mdss_lock_blank(void)
{
	mutex_lock(&mdss_diag_ctx.blank_lock);
}


/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
void mdss_unlock_blank(void)
{
	mutex_unlock(&mdss_diag_ctx.blank_lock);
}


/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
void mdss_lock_displaythread(void)
{
	mutex_lock(&mdss_diag_ctx.displaythread_lock);
}


/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
void mdss_unlock_displaythread(void)
{
	mutex_unlock(&mdss_diag_ctx.displaythread_lock);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
void mdss_lock_chkstatus(void)
{
	mutex_lock(&mdss_diag_ctx.chkstatus_lock);
}


/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
void mdss_unlock_chkstatus(void)
{
	mutex_unlock(&mdss_diag_ctx.chkstatus_lock);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_panel_dispon(struct mdss_panel_data *pdata)
{
	mdss_diag_panel_dcs_write0(pdata, 0x29);
}
#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_panel_dispoff(struct mdss_panel_data *pdata)
{
	mdss_diag_panel_dcs_write0(pdata, 0x28);
	usleep_range(MDSS_DIAG_WAIT_1FRAME_US, MDSS_DIAG_WAIT_1FRAME_US);
}
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */

#if defined(USES_PANEL_HAYABUSA)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_panel_enter_sleep(struct mdss_panel_data *pdata)
{
	mdss_diag_panel_dcs_write0(pdata, 0x10);
	usleep_range(4 * MDSS_DIAG_WAIT_1FRAME_US, 4 * MDSS_DIAG_WAIT_1FRAME_US);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_panel_exit_sleep(struct mdss_panel_data *pdata)
{
	mdss_diag_panel_dcs_write0(pdata, 0x11);
	usleep_range(MDSS_DIAG_WAIT_SLEEPOUT_US, MDSS_DIAG_WAIT_SLEEPOUT_US);
}
#endif  /* USES_PANEL_HAYABUSA */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_set_adjusted(struct mdss_panel_data *pdata)
{
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;

	if (!mdata) {
		pr_err("%s:failed to get mdata.\n", __func__);
		return -EFAULT;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
#ifdef MDSS_DIAG_PANEL_FLICKER
	ret = mdss_diag_set_flicker_if_adjusted(pdata);
#endif /* MDSS_DIAG_PANEL_FLICKER */
#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE
	ret = mdss_diag_set_panel_gmm(pdata);
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */
#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
	if (ctrl_pdata->panel_mode == DSI_CMD_MODE) {
		if (pdata->panel_info.internal_osc != MDSS_DIAG_PANEL_OSC_DEFAULT) {
			mdss_diag_panel_osc_send_cmd(pdata,
				pdata->panel_info.internal_osc);
		}
	}
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

	if (ctrl_pdata->panel_mode == DSI_VIDEO_MODE) {
		struct mdp_mipi_clkchg_param request_mipiclk;
		struct mdss_mdp_ctl *ctl = NULL;

		ctl = mdss_shdisp_get_mdpctrl(0);
		if (!ctl) {
			pr_err("%s: ctl is NULL.\n", __func__);
			return -ENXIO;
		}

		mutex_lock(&ctl->mipiclk_lock);
		memcpy(&request_mipiclk, &ctl->request_mipiclk,
			sizeof(request_mipiclk));
		mutex_unlock(&ctl->mipiclk_lock);

		if (request_mipiclk.host.clock_rate == 466000000) {
			struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
			ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);
			if (!ctrl_pdata) {
				pr_err("%s: ctrl_pdata is NULL.\n", __func__);
				return -ENXIO;
			}

			mdss_mdp_update_panel_timing(ctrl_pdata, &request_mipiclk.panel);
		}
	}

	mdss_diag_panel_switch_panel_page(pdata, MDSS_DIAG_DEFAULT_PAGE);

	if (ctrl_pdata->panel_mode == DSI_VIDEO_MODE) {
		mdss_diag_panel_dispon(pdata);
	}

	return ret;
}

#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_mipi_check(struct msm_fb_data_type *mfd, struct mdp_mipi_check_param *mipi_check_param, struct mdss_panel_data *pdata)
{
	int ret;
	u32 isr;
	uint8_t amp_data = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct mdss_dsi_ctrl_pdata *sctrl_pdata = NULL;
	#define SENSITIV_DATA_NUM		(2)
	uint8_t sens_data_master[SENSITIV_DATA_NUM];
	uint8_t *sens_pdata_slave = NULL;
	uint8_t sens_data_slave[SENSITIV_DATA_NUM];

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (pdata->next) {
		sctrl_pdata = container_of(pdata->next, struct mdss_dsi_ctrl_pdata, panel_data);
	}

	pr_debug("%s: in master=%p slave=%p\n", __func__, ctrl_pdata, sctrl_pdata);

	if (!ctrl_pdata) {
		pr_err("%s: ctrl_pdata is NULL.\n", __func__);
		return -ENXIO;
	}

	mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
			MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_ON);
	if (sctrl_pdata) {
		mdss_dsi_clk_ctrl(sctrl_pdata, sctrl_pdata->dsi_clk_handle,
				MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_ON);
	}

	mdss_dsi_err_intr_ctrl(ctrl_pdata, DSI_INTR_ERROR_MASK, 0);
	if (sctrl_pdata) {
		mdss_dsi_err_intr_ctrl(sctrl_pdata, DSI_INTR_ERROR_MASK, 0);
	}

	mdss_diag_dsi_cmd_bta_sw_trigger(ctrl_pdata);
	if (sctrl_pdata) {
		mdss_diag_dsi_cmd_bta_sw_trigger(sctrl_pdata);
	}

	amp_data = 0;
	memset(sens_data_master, 0, SENSITIV_DATA_NUM);
	memset(sens_data_slave , 0, SENSITIV_DATA_NUM);
	if (sctrl_pdata) {
		sens_pdata_slave = sens_data_slave;
	}

	mdss_diag_mipi_check_get_param(
			&amp_data,
			sens_data_master,
			sens_pdata_slave,
			ctrl_pdata);

	ret = mdss_diag_mipi_check_manual(
			mfd,
			mipi_check_param,
			ctrl_pdata,
			sctrl_pdata,
			sens_data_master,
			sens_pdata_slave);

	mdss_diag_mipi_check_set_param(
			amp_data,
			sens_data_master,
			sens_pdata_slave,
			ctrl_pdata, sctrl_pdata);

	/* MMSS_DSI_0_INT_CTRL */
	isr = MIPI_INP(ctrl_pdata->ctrl_base + 0x0110);
	MIPI_OUTP(ctrl_pdata->ctrl_base + 0x0110, isr);
	if (sctrl_pdata) {
		isr = MIPI_INP(sctrl_pdata->ctrl_base + 0x0110);
		MIPI_OUTP(sctrl_pdata->ctrl_base + 0x0110, isr);
	}

	mdss_dsi_err_intr_ctrl(ctrl_pdata, DSI_INTR_ERROR_MASK, 1);
	if (sctrl_pdata) {
		mdss_dsi_err_intr_ctrl(sctrl_pdata, DSI_INTR_ERROR_MASK, 1);
	}

	mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
			MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_OFF);
	if (sctrl_pdata) {
		mdss_dsi_clk_ctrl(sctrl_pdata, sctrl_pdata->dsi_clk_handle,
				MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_OFF);
	}

	pr_debug("%s: out\n", __func__);

	return ret;
}
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg(struct msm_fb_data_type *mfd,
		struct mdp_mipi_clkchg_param *mipi_clkchg_param)
{
#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
	int ret = 0;
	struct mdss_mdp_ctl *pctl;
	struct mdss_panel_data *pdata;

	pr_debug("%s: called\n", __func__);
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] mdpctrl is NULL.\n", __func__);
		return -EIO;
	}
	MDSS_XLOG(mipi_clkchg_param->host.clock_rate, XLOG_FUNC_ENTRY);
	pdata = pctl->panel_data;

	mdss_diag_mipi_clkchg_lock(pctl);

	mdss_diag_mipi_clkchg_param_log(mipi_clkchg_param);

	mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata);

	if (mdss_fb_is_power_off(mfd)) {
		ret = mdss_diag_mipi_clkchg_panel_clk_data(pdata);
	} else {
		ret = mdss_diag_panel_osc_set(pdata,
					mipi_clkchg_param->internal_osc);
		if(ret){
			pr_err("%s: failed to mdss_diag_panel_osc_set()"
					"ret=%d\n", __func__, ret);
		}
		ret = mdss_diag_mipi_clkchg_setparam(mipi_clkchg_param, pctl);
	}

	mdss_diag_mipi_clkchg_unlock();

	pr_debug("%s: end ret(%d)\n", __func__, ret);
	MDSS_XLOG(XLOG_FUNC_EXIT);

	return ret;
#else  /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */
	return 0;
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */
}

#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_mipi_check_manual(struct msm_fb_data_type *mfd, struct mdp_mipi_check_param *mipi_check_param,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl,
			uint8_t *sensitiv_master, uint8_t *sensitiv_slave)
{
	int ret = 0;
	uint8_t result[2] = {MDSS_MIPICHK_RESULT_OK, MDSS_MIPICHK_RESULT_OK};
	uint8_t dummy[2] = {MDSS_MIPICHK_RESULT_OK, MDSS_MIPICHK_RESULT_OK};

	pr_debug("%s: in\n", __func__);

	if ((mipi_check_param->amp & ~(MDSS_MIPICHK_AMP_NUM - 1)) != 0) {
		pr_err("%s: out of range. amp=0x%02X\n", __func__, mipi_check_param->amp);
		return -EINVAL;
	}

	if ((mipi_check_param->sensitiv & ~(MDSS_MIPICHK_SENSITIV_NUM - 1)) != 0) {
		pr_err("%s: out of range. sensitiv=0x%02X\n", __func__, mipi_check_param->sensitiv);
		return -EINVAL;
	}

	ret = mdss_diag_mipi_check_exec(mfd, result,
			mipi_check_param->frame_cnt, mipi_check_param->amp, mipi_check_param->sensitiv,
			ctrl, sctrl, sensitiv_master, sensitiv_slave);
	if (ret) {
		result[0] = MDSS_MIPICHK_RESULT_NG;
		result[1] = MDSS_MIPICHK_RESULT_NG;
	}

	if ((result[0] != MDSS_MIPICHK_RESULT_OK) || (result[1] != MDSS_MIPICHK_RESULT_OK)) {
		pr_debug("%s: recovery display.\n", __func__);
		mdss_diag_mipi_check_exec(mfd, dummy,
				1, MDSS_MIPICHK_AMP_NUM - 1, MDSS_MIPICHK_SENSITIV_NUM - 1,
				ctrl, sctrl, sensitiv_master, sensitiv_slave);
	}

	mipi_check_param->result_master = result[0];
	mipi_check_param->result_slave  = result[1];

	pr_debug("%s: out master=%d slave=%d\n", __func__,
				mipi_check_param->result_master, mipi_check_param->result_slave);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_mipi_check_exec(struct msm_fb_data_type *mfd, uint8_t *result, uint8_t frame_cnt, uint8_t amp, uint8_t sensitiv,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl,
			uint8_t *sensitiv_master, uint8_t *sensitiv_slave)
{
	int ret = 0;
	uint8_t set_amp;
	uint8_t set_sensitiv_master[2] = {0, 0};
	uint8_t set_sensitiv_slave[2] = {0, 0};

	static const uint8_t amp_tbl[MDSS_MIPICHK_AMP_NUM] = {
		0x03,
		0x02,
		0x00,
		0x01,
		0x04,
		0x05,
		0x06,
		0x07
	};

	pr_debug("%s: in frame_cnt=0x%02X amp=0x%02X sensitiv=0x%02X\n", __func__, frame_cnt, amp, sensitiv);

	set_amp = (amp_tbl[amp] << 1) | 1;

#if defined(USES_PANEL_HAYABUSA) || defined(USES_PANEL_ANDY)
	set_sensitiv_master[0]  = sensitiv << 4;
	set_sensitiv_master[0] |= *(sensitiv_master) & 0x0F;
	if (sctrl) {
		set_sensitiv_slave[0]   = sensitiv << 4;
		set_sensitiv_slave[0]  |= *(sensitiv_slave) & 0x0F;
	}
#elif defined(USES_PANEL_SINANJU)
	set_sensitiv_master[0]  = *(sensitiv_master);
	set_sensitiv_master[0]  = sensitiv & 0x07;
#endif /* USES_PANEL_SINANJU */

	mdss_diag_mipi_check_set_param(set_amp, set_sensitiv_master, set_sensitiv_slave, ctrl, sctrl);

	ret = mdss_diag_mipi_check_test(mfd, result, frame_cnt, ctrl, sctrl);

	pr_debug("%s: out ret=%d\n", __func__, ret);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	pr_debug("%s: amp=0x%02X sensitiv_master=0x%02X,0x%02X\n", __func__,
			amp, sensitiv_master[0], sensitiv_master[1]);

#if defined(MIPICHK_AMP)
	/* MMSS_DSI_0_PHY_REG_DSIPHY_REGULATOR_CTRL_0 */
	MIPI_OUTP((ctrl->phy_regulator_io.base) + MDSS_DSI_DSIPHY_REGULATOR_CTRL_0, amp);
	if (sctrl) {
		MIPI_OUTP((sctrl->phy_regulator_io.base) + MDSS_DSI_DSIPHY_REGULATOR_CTRL_0, amp);
	}
	wmb();
#endif /* MIPICHK_AMP */

	mdss_diag_write_sensitiv(ctrl, sensitiv_master, sensitiv_slave);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave,
			struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;
#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	struct mdss_mdp_ctl *ctl = NULL;
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */

#if defined(MIPICHK_AMP)
	/* MMSS_DSI_0_PHY_REG_DSIPHY_REGULATOR_CTRL_0 */
	*amp = MIPI_INP((ctrl->phy_regulator_io.base)+ MDSS_DSI_DSIPHY_REGULATOR_CTRL_0);
#endif /* MIPICHK_AMP */

#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	ctl = mdss_shdisp_get_mdpctrl(0);
	mdss_mdp_video_transfer_ctrl(ctl, false);
	ret = mdss_diag_read_sensitiv(ctrl, sensitiv_master, sensitiv_slave);
	mdss_mdp_video_transfer_ctrl(ctl, true);
#else  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	ret = mdss_diag_read_sensitiv(ctrl, sensitiv_master, sensitiv_slave);
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */

	pr_debug("%s: amp=0x%02X sensitiv_master=0x%02X,0x%02X ret=%d\n", __func__, *amp,
			sensitiv_master[0], sensitiv_master[1], ret);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test(struct msm_fb_data_type *mfd, uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	int ret = 0;
	char mode;

	mode = ctrl->panel_mode;

	if (mode == DSI_VIDEO_MODE) {
		ret = mdss_diag_mipi_check_test_video(result, frame_cnt, ctrl, sctrl);
	} else if (mode == DSI_CMD_MODE) {
		ret = mdss_diag_mipi_check_test_cmd(mfd, result, frame_cnt, ctrl, sctrl);
	} else {
		pr_err("%s: invalid panel_mode=%d\n", __func__, mode);
		ret = -EINVAL;
	}

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_video(uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	int ret = 0;
	uint32_t sleep;

	sleep = frame_cnt * MDSS_DIAG_WAIT_1FRAME_US;
	pr_debug("%s: frame_cnt=%d sleep=%d\n", __func__, frame_cnt, sleep);


	pr_debug("%s: sleep start.\n",__func__);
	usleep_range(sleep, sleep);
	pr_debug("%s: sleep finish.\n",__func__);

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(ctrl);
	if (ret) {
		result[0] = MDSS_MIPICHK_RESULT_NG;
	} else {
		result[0] = MDSS_MIPICHK_RESULT_OK;
	}
	if (sctrl) {
		ret = mdss_diag_dsi_cmd_bta_sw_trigger(sctrl);
		if (ret) {
			result[1] = MDSS_MIPICHK_RESULT_NG;
		} else {
			result[1] = MDSS_MIPICHK_RESULT_OK;
		}
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_cmd(struct msm_fb_data_type *mfd, uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	int ret = 0;
	int i;
	struct mdss_overlay_private *mdp5_data = NULL;
	struct mdss_mdp_ctl *ctl;

	pr_debug("%s: in\n", __func__);

	mdp5_data = mfd_to_mdp5_data(mfd);
	if (!mdp5_data)
		return -EINVAL;

	ctl = mdp5_data->ctl;
	if (!ctl)
		return -EINVAL;

	for (i = 0; i < frame_cnt; i++) {
		pr_debug("%s: frame=%d\n", __func__, i);

//		mdss_fb_pan_display_ex_diag(&mfd->msm_fb_backup.info, &mfd->msm_fb_backup.disp_commit);
//		mdss_fb_pan_idle_diag(mfd);
	}

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(ctrl);
	if (ret) {
		result[0] = MDSS_MIPICHK_RESULT_NG;
	} else {
		result[0] = MDSS_MIPICHK_RESULT_OK;
	}

	if (sctrl) {
		ret = mdss_diag_dsi_cmd_bta_sw_trigger(sctrl);
		if (ret) {
			result[1] = MDSS_MIPICHK_RESULT_NG;
		} else {
			result[1] = MDSS_MIPICHK_RESULT_OK;
		}
	}

	pr_debug("%s: out\n", __func__);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_read_sensitiv(struct mdss_dsi_ctrl_pdata *ctrl, uint8_t *sensitiv_master, uint8_t *sensitiv_slave)
{
	int ret = 0;
#if defined(USES_PANEL_ANDY)
	unsigned char sensitiv_addr[1] = {0x45};
	unsigned char r_buf = 0x00;

	ret = mdss_diag_panel_switch_panel_page(
				&(ctrl->panel_data), 0xEE);
	if (ret) {
		pr_err("%s: failed to switch page\n", __func__);
		return ret;
	}

	ret = mdss_diag_panel_dcs_read(&(ctrl->panel_data),
					sensitiv_addr[0], 1, &r_buf);
	if (ret) {
		pr_err("%s: failed to read\n", __func__);
		return ret;
	}
	sensitiv_master[0] = r_buf;
#elif defined(USES_PANEL_HAYABUSA)
	unsigned char sensitiv_addr[2] = {0x7E, 0x97};
	unsigned char r_buf = 0x00;

	ret = mdss_diag_panel_switch_panel_page(
				&(ctrl->panel_data), 0xE0);
	if (ret) {
		pr_err("%s: failed to switch page\n", __func__);
		return ret;
	}

	ret = mdss_diag_panel_dcs_read(&(ctrl->panel_data),
					sensitiv_addr[0], 1, &r_buf);
	if (ret) {
		pr_err("%s: failed to read\n", __func__);
		return ret;
	}
	sensitiv_master[0] = r_buf;

	if (sensitiv_slave != NULL) {
		r_buf = 0;
		ret = mdss_diag_panel_dcs_read(&(ctrl->panel_data),
					 sensitiv_addr[1], 1, &r_buf);
		if (ret) {
			pr_err("%s: failed to read slave\n", __func__);
			return ret;
		}
		sensitiv_slave[0] = r_buf;
	}
#elif defined(USES_PANEL_SINANJU)
	char sensitiv_addr = 0xB6;
	unsigned char r_buf[5];

	memset(r_buf, 0, sizeof(r_buf));
	ret = mdss_diag_panel_dcs_read(&(ctrl->panel_data),
				sensitiv_addr,
				sizeof(r_buf),
				r_buf);

	sensitiv_master[0] = r_buf[4];
#endif /* USES_PANEL_SINANJU */
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_write_sensitiv(struct mdss_dsi_ctrl_pdata *ctrl, uint8_t *sensitiv_master, uint8_t *sensitiv_slave)
{
	int ret = 0;
#if defined(USES_PANEL_ANDY)
	unsigned char sensitiv_addr[1] = {0x45};
	unsigned char w_buf = 0x00;

	ret = mdss_diag_panel_switch_panel_page(
				&(ctrl->panel_data), 0xEE);
	if (ret) {
		pr_err("%s: failed to switch page\n", __func__);
		return ret;
	}

	w_buf = sensitiv_master[0];
	ret = mdss_diag_panel_dcs_write1(&(ctrl->panel_data),
					sensitiv_addr[0], w_buf);
	if (ret) {
		pr_err("%s: failed to write\n", __func__);
		return ret;
	}
#elif defined(USES_PANEL_HAYABUSA)
	unsigned char sensitiv_addr[2] = {0x7E, 0x97};
	unsigned char w_buf = 0x00;

	ret = mdss_diag_panel_switch_panel_page(
				&(ctrl->panel_data), 0xE0);
	if (ret) {
		pr_err("%s: failed to switch page\n", __func__);
		return ret;
	}

	w_buf = sensitiv_master[0];
	ret = mdss_diag_panel_dcs_write1(&(ctrl->panel_data),
					sensitiv_addr[0], w_buf);
	if (ret) {
		pr_err("%s: failed to write master\n", __func__);
		return ret;
	}

	if (sensitiv_slave != NULL) {
		w_buf = sensitiv_slave[0];
		ret = mdss_diag_panel_dcs_write1(&(ctrl->panel_data),
					sensitiv_addr[1], w_buf);
		if (ret) {
			pr_err("%s: failed to write slave\n", __func__);
		}
	}
#elif defined(USES_PANEL_SINANJU)
	struct dcs_cmd_req cmdreq;
	unsigned char sensitiv_addr[1] = {0xB6};
	char sensitiv[4] = {0xF0, sensitiv_addr[0], 0x04, 0x00};
	struct dsi_cmd_desc sensitiv_cmd[] = {
		{{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(sensitiv)}, sensitiv},
	};

	sensitiv[3] = *(sensitiv_master);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = sensitiv_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
#endif /* USES_PANEL_SINANJU */
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;
	u32 status;
	int timeout_us = 35000;

	pr_debug("%s: in\n", __func__);

	if (ctrl == NULL) {
		pr_err("%s: ctrl is NULL.\n", __func__);
		return -EINVAL;
	}

	/* CMD_MODE_BTA_SW_TRIGGER */
	MIPI_OUTP((ctrl->ctrl_base) + 0x098, 0x01);	/* trigger */
	wmb();

	/* Check for CMD_MODE_DMA_BUSY */
	if (readl_poll_timeout(((ctrl->ctrl_base) + 0x0008),
				status, ((status & 0x0010) == 0),
				0, timeout_us)) {
		pr_info("%s: timeout. status=0x%08x\n", __func__, status);
		return -EIO;
	}

	ret = mdss_diag_dsi_ack_err_status(ctrl);

	pr_debug("%s: out status=0x%08x ret=%d\n", __func__, status, ret);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 status;
	unsigned char *base;
	u32 ack = 0x10000000;

	base = ctrl->ctrl_base;

	status = MIPI_INP(base + 0x0068);/* DSI_ACK_ERR_STATUS */
	if (status) {
		MIPI_OUTP(base + 0x0068, status);
		/* Writing of an extra 0 needed to clear error bits */
		MIPI_OUTP(base + 0x0068, 0);

		status &= ~(ack);
		if(status){
			pr_err("%s: status=0x%08x\n", __func__, status);
			return -EIO;
		}
	}

	return 0;
}
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
struct mdss_mdp_ctl* mdss_shdisp_get_mdpctrl(int fbinx)
{
	return ((struct mdss_overlay_private*)((struct msm_fb_data_type*)mdss_fb_get_fbinfo(fbinx)->par)->mdp.private1)->ctl;
}

#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_mipi_clkchg_lock(struct mdss_mdp_ctl *pctl)
{
	mdss_lock_chkstatus();
	mdss_lock_blank();
	mdss_lock_displaythread();
	mdss_wait4ppdone(pctl);
}


/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_mipi_clkchg_unlock(void)
{
	mdss_unlock_displaythread();
	mdss_unlock_blank();
	mdss_unlock_chkstatus();
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_setparam(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);

	if (pctl->is_video_mode) {
	} else {
		mutex_lock(&pctl->rsrc_lock);
	}

	ret |= mdss_diag_mipi_clkchg_host(pctl);

	if (pctl->is_video_mode) {
	} else {
		mutex_unlock(&pctl->rsrc_lock);
	}

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void __mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_panel_data *pdata)
{
	int i;
	struct mdss_panel_info *pinfo = &(pdata->panel_info);

	pr_debug("%s: called panel name = %s\n", __func__, pdata->panel_info.panel_name);

	pinfo->clk_rate = mipi_clkchg_param->host.clock_rate;
	if (pinfo->is_split_display) {
		pinfo->xres = mipi_clkchg_param->host.display_width/2;
	} else {
		pinfo->xres = mipi_clkchg_param->host.display_width;
	}
	pinfo->yres = mipi_clkchg_param->host.display_height;
	pinfo->lcdc.h_pulse_width = mipi_clkchg_param->host.hsync_pulse_width;
	pinfo->lcdc.h_back_porch = mipi_clkchg_param->host.h_back_porch;
	pinfo->lcdc.h_front_porch = mipi_clkchg_param->host.h_front_porch;
	pinfo->lcdc.v_pulse_width = mipi_clkchg_param->host.vsync_pulse_width;
	pinfo->lcdc.v_back_porch = mipi_clkchg_param->host.v_back_porch;
	pinfo->lcdc.v_front_porch = mipi_clkchg_param->host.v_front_porch;
	pinfo->mipi.t_clk_post = mipi_clkchg_param->host.t_clk_post;
	pinfo->mipi.t_clk_pre = mipi_clkchg_param->host.t_clk_pre;
	for ( i=0; i<sizeof(mipi_clkchg_param->host.timing_ctrl); i++ ) {
		pinfo->mipi.dsi_phy_db.timing[i] = mipi_clkchg_param->host.timing_ctrl[i];
	}

	pr_debug("%s: end\n", __func__);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_panel_data *pdata)
{
	pr_debug("%s: called\n", __func__);

	__mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata);
	if (pdata->next) {
		__mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata->next);
	}

	pr_debug("%s: end\n", __func__);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_reconfig(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mipi_panel_info *pinfo;

	if (!pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s, start\n", __func__);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
		panel_data);
	pinfo = &pdata->panel_info.mipi;

	/* reset DSI */
	mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
			  MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_ON);
	mdss_dsi_sw_reset(ctrl_pdata, true);
	mdss_dsi_set_phy_regulator_enable(ctrl_pdata);
	mdss_dsi_ctrl_setup(ctrl_pdata);
	mdss_dsi_controller_cfg(true, pdata);
	mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
			  MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_OFF);
	mdss_dsi_pll_relock(ctrl_pdata);

	pr_debug("%s: end\n", __func__);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int __mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl, struct mdss_panel_data *pdata)
{
	int ret = 0;

	pr_debug("%s: called panel name = %s\n", __func__, pdata->panel_info.panel_name);
	ret = mdss_diag_reconfig(pdata);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_stop_clkln_hs(struct mdss_panel_data *pdata)
{
	int cnt = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	cnt = ctrl_pdata->clk_lane_cnt;
	if (cnt) {
		pr_debug("%s: ctrl_pdata(0x%p), clk_lane_cnt = %d\n", __func__,
				ctrl_pdata, cnt);
		mdss_dsi_hs_clk_lane_enable(false);
	}
	return cnt;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_start_clkln_hs(struct mdss_panel_data *pdata, int cnt)
{
	if (cnt) {
		pr_debug("%s: clk_lane_cnt = %d\n", __func__,  cnt);
		mdss_dsi_hs_clk_lane_enable(true);
	}
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl)
{
	int ret = 0;
	struct mdss_panel_data *pdata = pctl->panel_data;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int mpdata_clk_ln_hs_cnt = 0;

	pr_debug("%s: called\n", __func__);

	// host clock update
	ret |= mdss_diag_mipi_clkchg_panel_clk_update(pdata);
	if (pdata->next) {
		ret |= mdss_diag_mipi_clkchg_panel_clk_update(pdata->next);
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	// backup and stop clnln_hs(m/s)
	mpdata_clk_ln_hs_cnt = mdss_diag_mipi_stop_clkln_hs(pdata);

	// host register update
	ret |= __mdss_diag_mipi_clkchg_host(pctl, pdata);
	if (ret) {
		pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_host err panel name = %s.\n"
			, __func__, pdata->panel_info.panel_name);
	}
	if (pdata->next) {
		ret |= __mdss_diag_mipi_clkchg_host(pctl, pdata->next);
		if (ret) {
			pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_host err panel name = %s.\n"
				, __func__, pdata->next->panel_info.panel_name);
		}
	}

	// restart clnln_hs(if stop by this func.)
	mdss_diag_mipi_start_clkln_hs(pdata, mpdata_clk_ln_hs_cnt);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_panel_clk_update(struct mdss_panel_data *pdata)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);

	ret = __mdss_diag_mipi_clkchg_panel_clk_data(pdata);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int __mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata)
{
	int ret = 0;
	ret = mdss_dsi_clk_refresh(pdata, false);
	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);

	ret = __mdss_diag_mipi_clkchg_panel_clk_data(pdata);
	if (ret) {
		pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_panel_clk_data err panel name = %s.\n"
			, __func__, pdata->panel_info.panel_name);
	}
	if (pdata->next) {
		ret |= __mdss_diag_mipi_clkchg_panel_clk_data(pdata->next);
		if (ret) {
			pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_panel_clk_data err panel name = %s.\n"
				, __func__, pdata->next->panel_info.panel_name);
		}
	}

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_mipi_clkchg_param_log(struct mdp_mipi_clkchg_param *mdp_mipi_clkchg_param)
{
	pr_debug("[%s]param->host.clock_rate         = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.clock_rate          );
	pr_debug("[%s]param->host.display_width      = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.display_width       );
	pr_debug("[%s]param->host.display_height     = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.display_height      );
	pr_debug("[%s]param->host.hsync_pulse_width  = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.hsync_pulse_width   );
	pr_debug("[%s]param->host.h_back_porch       = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.h_back_porch        );
	pr_debug("[%s]param->host.h_front_porch      = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.h_front_porch       );
	pr_debug("[%s]param->host.vsync_pulse_width  = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.vsync_pulse_width   );
	pr_debug("[%s]param->host.v_back_porch       = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.v_back_porch        );
	pr_debug("[%s]param->host.v_front_porch      = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.v_front_porch       );
	pr_debug("[%s]param->host.t_clk_post         = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.t_clk_post          );
	pr_debug("[%s]param->host.t_clk_pre          = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.t_clk_pre           );
	pr_debug("[%s]param->host.timing_ctrl[ 0]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 0]     );
	pr_debug("[%s]param->host.timing_ctrl[ 1]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 1]     );
	pr_debug("[%s]param->host.timing_ctrl[ 2]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 2]     );
	pr_debug("[%s]param->host.timing_ctrl[ 3]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 3]     );
	pr_debug("[%s]param->host.timing_ctrl[ 4]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 4]     );
	pr_debug("[%s]param->host.timing_ctrl[ 5]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 5]     );
	pr_debug("[%s]param->host.timing_ctrl[ 6]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 6]     );
	pr_debug("[%s]param->host.timing_ctrl[ 7]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 7]     );
	pr_debug("[%s]param->host.timing_ctrl[ 8]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 8]     );
	pr_debug("[%s]param->host.timing_ctrl[ 9]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 9]     );
	pr_debug("[%s]param->host.timing_ctrl[10]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[10]     );
	pr_debug("[%s]param->host.timing_ctrl[11]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[11]     );

#if defined(USES_PANEL_HAYABUSA)
	pr_debug("[%s]param->internal_osc            = %10d\n"  , __func__, mdp_mipi_clkchg_param->internal_osc             );
#endif  /* USES_PANEL_HAYABUSA */

	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_wait4ppdone(struct mdss_mdp_ctl *pctl)
{
	int ret = 0;
	struct mdss_mdp_ctl *sctl;
	struct mdss_panel_data *pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;

	pr_debug("%s: in\n", __func__);

	if (pctl->ops.wait_pingpong) {
		pr_debug("%s: wait_pingpong called\n", __func__);
		ret = pctl->ops.wait_pingpong(pctl, NULL);
		if(ret){
			pr_err("%s: failed to wait_pingpong. ret=%d\n", __func__, ret);
		}
	}
	sctl = mdss_mdp_get_split_ctl(pctl);
	if (sctl && sctl->ops.wait_pingpong) {
		ret = sctl->ops.wait_pingpong(sctl, NULL);
		if(ret){
			pr_err("%s: failed to wait_pingpong(split_ctl). ret=%d\n", __func__, ret);
		}
	}

	pdata = pctl->panel_data;
	if (!pdata) {
		pr_err("%s: panel_data is NULL.\n", __func__);
		return 0;
	}

	pinfo = &(pdata->panel_info);
	if (pinfo->type == MIPI_CMD_PANEL) {
		// wait for delayed_off_work(and gate_ctrl_work)
		ret = mdss_mdp_cmd_flush_delayed_off_clk_work(pctl);
	}
	pr_debug("%s: out ret=%d\n", __func__, ret);

	return 0;
}


/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_osc_set(struct mdss_panel_data *pdata, int osc)
{
	int ret = 0;
	struct mdss_panel_info *pinfo = NULL;

	if ((osc >= MDSS_DIAG_PANEL_OSC_TYPES) || (osc < 0)) {
		pr_err("%s: %d isn't supported osc_type\n", __func__, osc);
		return -EINVAL;
	}

	if (!pdata) {
		pr_err("%s: panel_data is NULL.\n", __func__);
		return -ENXIO;
	}

	pinfo = &(pdata->panel_info);
	if (pinfo->internal_osc == osc) {
		pr_debug("%s: req-osctype is same as cur-osctype\n", __func__);
		return 0;
	}
	ret = mdss_diag_panel_osc_send_cmd(pdata, osc);
	if (!ret) {
		// only master param
		pinfo->internal_osc = osc;
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_osc_send_cmd(
				struct mdss_panel_data *pdata, int osc)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: ctrl_pdata is NULL.\n", __func__);
		return -ENXIO;
	}

#if defined(USES_PANEL_HAYABUSA)
	ret = mdss_diag_hayabusa_osc_send_cmd(ctrl_pdata, osc);
#endif  /* USES_PANEL_HAYABUSA */
	return ret;
}


#if defined(USES_PANEL_HAYABUSA)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_hayabusa_osc_send_cmd(
				struct mdss_dsi_ctrl_pdata *ctrl, int osc)
{
	static char osc_cmd_buf[][2] = {
		{ 0xFF, 0x26},
		{ 0x21, 0x3B},
		{ 0x0A, 0x00},
		{ 0xFF, 0x10},
		{ 0xB0, 0x1A},// and 4frame wait
		{ 0xEF, 0x32},
		{ 0xFF, 0xF0},
		// oscset1, 2
		{ 0xFF, 0x10},
		{ 0xEF, 0x33},
		{ 0xFF, 0x24},
		// rtna - soe_w and 1frame wait
		{ 0xFF, 0x26},
		{ 0x21, 0x00},
		{ 0x0A, 0x00},
		{ 0xFF, 0x10},
		{ 0xB0, 0x1A},
	};
	static char oscset1_cmd[]   = { 0x29, 0x00},
		    oscset2_cmd[]   = { 0x28, 0x00},
		    rtna_cmd[]      = { 0x8F, 0x00},
		    stv_delay_cmd[] = { 0x5B, 0x00},
		    stv_adv_cmd[]   = { 0x5C, 0x00},
		    gck_delay_cmd[] = { 0x68, 0x00},
		    gck_adv_cmd[]   = { 0x69, 0x00},
		    soe_w_cmd[]     = { 0x9A, 0x00};

	static struct dsi_cmd_desc osc_cmd[] = {
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[0]},
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[1]},
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[2]},
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0   , 2}, osc_cmd_buf[3]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 17*4, 2}, osc_cmd_buf[4]},
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[5]},
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[6]},

		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, oscset1_cmd    },
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, oscset2_cmd    },

		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[7] },
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[8] },
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[9] },

		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, rtna_cmd       },
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, stv_delay_cmd  },
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, stv_adv_cmd    },
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, gck_delay_cmd  },
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, gck_adv_cmd    },
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 17*1, 2}, soe_w_cmd      },

		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[10]},
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[11]},
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[12]},
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0,    2}, osc_cmd_buf[13]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 0,    2}, osc_cmd_buf[14]},
	};

	int ret = 0;
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: in osc=%d\n", __func__, osc);

	if ((osc >= ARRAY_SIZE(osc_types_value)) || (osc < 0)) {
		pr_err("%s: %d isn't supported osc_type\n", __func__, osc);
		return -EINVAL;
	}

	// update cmds value from types
	oscset1_cmd[1]   = osc_types_value[osc].oscset1;
	oscset2_cmd[1]   = osc_types_value[osc].oscset2;
	rtna_cmd[1]      = osc_types_value[osc].rtna;
	stv_delay_cmd[1] = osc_types_value[osc].stv_delay;
	stv_adv_cmd[1]   = osc_types_value[osc].stv_adv;
	gck_delay_cmd[1] = osc_types_value[osc].gck_delay;
	gck_adv_cmd[1]   = osc_types_value[osc].gck_adv;
	soe_w_cmd[1]     = osc_types_value[osc].soe_w;

	// send command
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = osc_cmd;
	cmdreq.cmds_cnt = ARRAY_SIZE(osc_cmd);
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if (!ret) {
		pr_err("%s: failed to mdss_dsi_cmdlist_put()\n", __func__);
		ret = -EIO;
	} else {
		ret = 0;
	}

	pr_debug("%s: out ret=%d\n", __func__, ret);
	return ret;
}
#endif  /* USES_PANEL_HAYABUSA */
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

#ifdef MDSS_DIAG_PANEL_FLICKER
#if defined(USES_PANEL_ANDY)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_andy_calc_vcom(struct mdss_vcom in,
		struct mdss_andy_vcom *out, unsigned short vcomoffset)
{
	unsigned short tmp;
	unsigned short vcomadj;
	unsigned short vcomdcoff;

	pr_debug("%s: vcom=0x%04x vcom_low=0x%04x vcom_offset=0x%04x\n", __func__,
			in.vcom, in.vcom_low, vcomoffset);

	if (out == NULL) {
		pr_err("%s: <NULL_POINTER> out.\n", __func__);
		return -EPERM;
	}

	vcomadj = in.vcom + vcomoffset;
	if (vcomadj > VCOM_MAX_ANDY) {
		vcomadj = VCOM_MAX_ANDY;
	}
#if (VCOM_MIN > 0)
	if (vcomadj < VCOM_MIN) {
		vcomadj = VCOM_MIN;
	}
#endif
	pr_debug("%s: vcomadj=0x%04x\n", __func__, vcomadj);

	out->vcom1_l = vcomadj & 0xFF;
	out->vcom2_l = out->vcom1_l;
	out->vcom12_h = 0x60;
	if ((vcomadj >> 8) & 0x01) {
		out->vcom12_h |= 0x03;
	}

	pr_debug("%s: VCOM1_L=0x%02x VCOM2_L=0x%02x VCOM12_H=0x%02x\n", __func__,
			out->vcom1_l, out->vcom2_l, out->vcom12_h);

	vcomdcoff = (vcomadj + 1) / 2;

	if (in.vcom_low >= in.vcom) {
		tmp = ((in.vcom_low - in.vcom) & 0x0F);
	} else {
		tmp = (((in.vcom - in.vcom_low - 1) & 0x0F) | 0x10);
	}
	out->lpvcom1 = (tmp & 0x1F);
	out->lpvcom2 = out->lpvcom1;
	if (vcomdcoff & 0x100) {
		out->lpvcom2 |= 0x80;
	}

	out->vcomoff = (unsigned char) (vcomdcoff & 0xFF);

	pr_debug("%s: LPVCOM1=0x%02x LPVCOM2=0x%02x VCOMOFF=0x%02x", __func__,
			out->lpvcom1, out->lpvcom2, out->vcomoff);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_andy_send_flicker(struct mdss_dsi_ctrl_pdata *ctrl,
		struct mdss_andy_vcom *vcom)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	unsigned char addr_value[][2] = {
		{0xFF, 0x01},
		{0x13, 0x00},
		{0x14, 0x00},
		{0x15, 0x00},
		{0x5C, 0x00},
		{0x5D, 0x00},
		{0x5E, 0x00},
	};

	struct dsi_cmd_desc flicker_cmd[] = {
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[0]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[1]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[2]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[3]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[4]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[5]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[6]},
	};

	addr_value[1][1] = vcom->vcom1_l;
	addr_value[2][1] = vcom->vcom2_l;
	addr_value[3][1] = vcom->vcom12_h;
	addr_value[4][1] = vcom->lpvcom1;
	addr_value[5][1] = vcom->lpvcom2;
	addr_value[6][1] = vcom->vcomoff;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = flicker_cmd;
	cmdreq.cmds_cnt = ARRAY_SIZE(flicker_cmd);
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	pr_debug("%s:end ret=%d\n", __func__, ret);
	if (ret > 0) {
		ret = 0;
	}
	return ret;
}
#elif defined(USES_PANEL_HAYABUSA)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_hayabusa_calc_vcom(struct mdss_vcom in,
		struct mdss_hayabusa_vcom *out, unsigned short vcomoffset)
{
	unsigned short tmp;
	unsigned short vcomadj;
	unsigned short vcomdcoff;

	pr_debug("%s: vcom=0x%04x vcom_low=0x%04x vcom_offset=0x%04x\n", __func__,
			in.vcom, in.vcom_low, vcomoffset);

	if (out == NULL) {
		pr_err("%s: <NULL_POINTER> out.\n", __func__);
		return -EPERM;
	}

	vcomadj = in.vcom + vcomoffset;
	if (vcomadj > VCOM_MAX_HAYABUSA) {
		vcomadj = VCOM_MAX_HAYABUSA;
	}
#if (VCOM_MIN > 0)
	if (vcomadj < VCOM_MIN) {
		vcomadj = VCOM_MIN;
	}
#endif
	pr_debug("%s: vcomadj=0x%04x\n", __func__, vcomadj);

	out->vcom1_l = vcomadj & 0xFF;
	out->vcom2_l = out->vcom1_l;
	out->vcom12_h = 0x00;
	if ((vcomadj >> 8) & 0x01) {
		out->vcom12_h |= 0x03;
	}

	pr_debug("%s: VCOM1_L=0x%02x VCOM2_L=0x%02x VCOM12_H=0x%02x\n", __func__,
			out->vcom1_l, out->vcom2_l, out->vcom12_h);

	vcomdcoff = (vcomadj + 1) / 2;

	if (in.vcom_low >= in.vcom) {
		tmp = ((in.vcom_low - in.vcom) & 0x0F);
	} else {
		tmp = (((in.vcom - in.vcom_low - 1) & 0x0F) | 0x10);
	}
	out->lpvcom1 = ((tmp & 0x1F) | 0x60);
	out->lpvcom2 = ((tmp & 0x1F) | 0x40);

	out->vcomoff_l = (unsigned char) (vcomdcoff & 0xFF);

	if (vcomdcoff & 0x100) {
		out->vcomoff_h = (unsigned char) (0xA0);
	} else {
		out->vcomoff_h = (unsigned char) (0x80);
	}

	pr_debug("%s: LPVCOM1=0x%02x LPVCOM2=0x%02x VCOMOFF_L=0x%02x VCOMOFF_H=0x%02x",
			__func__, out->lpvcom1, out->lpvcom2, out->vcomoff_l, out->vcomoff_h);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_hayabusa_send_flicker(struct mdss_dsi_ctrl_pdata *ctrl,
		struct mdss_hayabusa_vcom *vcom)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	unsigned char addr_value[][2] = {
		{0xFF, 0x20},
		{0x13, 0x00},
		{0x14, 0x00},
		{0x15, 0x00},
		{0x5A, 0x00},
		{0x5B, 0x00},
		{0x5C, 0x00},
		{0x5E, 0x00},
	};

	struct dsi_cmd_desc flicker_cmd[] = {
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[0]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[1]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[2]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[3]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[4]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[5]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[6]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, addr_value[7]},
	};

	addr_value[1][1] = vcom->vcom1_l;
	addr_value[2][1] = vcom->vcom2_l;
	addr_value[3][1] = vcom->vcom12_h;
	addr_value[4][1] = vcom->lpvcom1;
	addr_value[5][1] = vcom->lpvcom2;
	addr_value[6][1] = vcom->vcomoff_l;
	addr_value[7][1] = vcom->vcomoff_h;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = flicker_cmd;
	cmdreq.cmds_cnt = ARRAY_SIZE(flicker_cmd);
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	pr_debug("%s:end ret=%d\n", __func__, ret);
	if (ret > 0) {
		ret = 0;
	}
	return ret;
}
#elif defined(USES_PANEL_SINANJU)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_sinanju_calc_vcom(struct mdss_vcom in,
		struct mdss_sinanju_vcom *out, unsigned short vcomoffset)
{
	unsigned short vcomadj;
	unsigned short vcomdcoff;

	pr_debug("%s: vcom=0x%04x vcom_low=0x%04x vcom_offset=0x%04x\n", __func__,
			in.vcom, in.vcom_low, vcomoffset);

	if (out == NULL) {
		pr_err("%s: <NULL_POINTER> out.\n", __func__);
		return -EPERM;
	}

	vcomadj = in.vcom + vcomoffset;
	if (vcomadj > VCOM_MAX_SINANJU) {
		vcomadj = VCOM_MAX_SINANJU;
	}
#if (VCOM_MIN > 0)
	if (vcomadj < VCOM_MIN) {
		vcomadj = VCOM_MIN;
	}
#endif
	pr_debug("%s: vcomadj=0x%04x\n", __func__, vcomadj);

	vcomdcoff = vcomadj / 2;

	out->vcom_fw_h = (vcomadj & 0x0300) >> 8;
	out->vcom_fw_l = vcomadj & 0x00FF;
	out->vcomdcoff_h = (vcomdcoff & 0x0300) >> 8;
	out->vcomdcoff_l = vcomdcoff & 0x00FF;

	pr_debug("%s: VCOM_FW_H=0x%02x\n", __func__, out->vcom_fw_h);
	pr_debug("%s: VCOM_FW_L=0x%02x\n", __func__, out->vcom_fw_l);
	pr_debug("%s: VCOMDCOFF_H=0x%02x\n", __func__, out->vcomdcoff_h);
	pr_debug("%s: VCOMDCOFF_L=0x%02x\n", __func__, out->vcomdcoff_l);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_sinanju_send_flicker(struct mdss_dsi_ctrl_pdata *ctrl,
		struct mdss_sinanju_vcom *vcom)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	char data[] = {0xD5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	struct dsi_cmd_desc cmd = {
		{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(data)},
		data
	};

	data[1] = vcom->vcom_fw_h;
	data[2] = vcom->vcom_fw_l;
	data[5] = vcom->vcomdcoff_h;
	data[6] = vcom->vcomdcoff_l;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	pr_debug("%s:end ret=%d\n", __func__, ret);
	if (ret > 0) {
		ret = 0;
	}
	return ret;
}
#endif  /* USES_PANEL_XXX */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_set_flicker_ctx(struct mdp_flicker_param flicker_param)
{
	if (flicker_param.request & MSMFB_SAVE_VALUE) {
		mdss_diag_ctx.mdss_flicker_ctx.vcom.vcom = flicker_param.vcom;
		mdss_diag_ctx.mdss_flicker_ctx.nvram = 0x9000 | flicker_param.vcom;
	}
	if (flicker_param.request & MSMFB_SAVE_VALUE_LOW) {
		mdss_diag_ctx.mdss_flicker_ctx.vcom.vcom_low = flicker_param.vcom;
	}
	if (flicker_param.request & MSMFB_RESET_VALUE) {
		mdss_diag_ctx.mdss_flicker_ctx.vcom.vcom = 0;
		mdss_diag_ctx.mdss_flicker_ctx.vcom.vcom_low = 0;
		mdss_diag_ctx.mdss_flicker_ctx.nvram = 0;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_send_flicker_param(struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_vcom vcom)
{
	int ret;
#if defined(USES_PANEL_ANDY)
	struct mdss_andy_vcom out;
#elif defined(USES_PANEL_HAYABUSA)
	struct mdss_hayabusa_vcom out;
#elif defined(USES_PANEL_SINANJU)
	struct mdss_sinanju_vcom out;
#endif  /* USES_PANEL_XXX */

	memset(&out, 0, sizeof(out));
#if defined(USES_PANEL_ANDY)
	ret = mdss_diag_andy_calc_vcom(vcom, &out, 0);
#elif defined(USES_PANEL_HAYABUSA)
	ret = mdss_diag_hayabusa_calc_vcom(vcom, &out, 0);
#elif defined(USES_PANEL_SINANJU)
	ret = mdss_diag_sinanju_calc_vcom(vcom, &out, 0);
#endif  /* USES_PANEL_XXX */
	if (ret != 0) {
		pr_err("%s: <RESULT_FAILURE> mdss_diag_hayabusa_calc_vcom.\n", __func__);
		return ret;
	}
#if defined(USES_PANEL_ANDY)
	ret = mdss_diag_andy_send_flicker(ctrl, &out);
#elif defined(USES_PANEL_HAYABUSA)
	ret = mdss_diag_hayabusa_send_flicker(ctrl, &out);
#elif defined(USES_PANEL_SINANJU)
	ret = mdss_diag_sinanju_send_flicker(ctrl, &out);
#endif  /* USES_PANEL_XXX */
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_set_flicker_if_adjusted(struct mdss_panel_data *pdata)
{
	int ret = 0;
	unsigned short nvram = mdss_diag_ctx.mdss_flicker_ctx.nvram;
	struct mdss_dsi_ctrl_pdata *ctrl;

	if (!pdata) {
		pr_err("%s: panel_data is NULL.\n", __func__);
		return -ENXIO;
	}
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl) {
		pr_err("%s: ctrl_pdata is NULL.\n", __func__);
		return -ENXIO;
	}
	if (IS_FLICKER_ADJUSTED(nvram)) {
		ret = mdss_diag_send_flicker_param(ctrl, mdss_diag_ctx.mdss_flicker_ctx.vcom);
		pr_debug("%s: flicker adjusted. vcom=0x%x vcom_low=0x%x\n",
			__func__, mdss_diag_ctx.mdss_flicker_ctx.vcom.vcom,
			mdss_diag_ctx.mdss_flicker_ctx.vcom.vcom_low);
	} else {
		pr_debug("%s:flicker not adjusted\n", __func__);
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_set_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param flicker_param)
{
	int ret = 0;
	struct mdss_vcom vcom;
	vcom.vcom = flicker_param.vcom;
	vcom.vcom_low = flicker_param.vcom;

	if (flicker_param.request & MSMFB_REG_WRITE) {
		ret = mdss_diag_send_flicker_param(ctrl, vcom);
		mdss_diag_panel_switch_panel_page(&(ctrl->panel_data), MDSS_DIAG_DEFAULT_PAGE);
		if (ret != 0) {
			pr_err("%s: <RESULT_FAILURE> mdss_diag_send_flicker_param.\n", __func__);
			return ret;
		}

	}
	mdss_diag_set_flicker_ctx(flicker_param);

	return 0;
}

#if defined(USES_PANEL_ANDY)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_andy_read_flicker(struct mdss_dsi_ctrl_pdata *ctrl, unsigned char out_buf[3])
{
	int ret = 0;
	int i;
	int read_reg_size = 0;
	struct dcs_cmd_req cmdreq;
	char bank[] = {0xFF, 0x01};
	struct dsi_cmd_desc bank_cmd = {
		{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(bank)},
		bank
	};
	unsigned char read_buf[2] = {0x00, 0x00};
	char read_cmd[] = {0x00};
	struct dsi_cmd_desc dcs_read_cmd = {
		{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_cmd)},
		read_cmd
	};
	char read_reg[3] = {0x13, 0x15, 0x5C};

	/* to page 0x01 */
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &bank_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if (ret < 0) {
		pr_err("%s: <RESULT_FAILURE> to page 0x01.\n", __func__);
		return ret;
	}
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
	cmdreq.rlen = sizeof(read_buf);
	cmdreq.rbuf = read_buf;
	cmdreq.cb = NULL; /* call back */

	read_reg_size = sizeof(read_reg);
	for (i = 0; i < read_reg_size; i++) {
		memset(&read_buf, 0, sizeof(read_buf));
		memcpy(&read_cmd, &read_reg[i], sizeof(read_cmd));
		ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
		if (ret < 0) {
			pr_err("%s: <RESULT_FAILURE> mdss_dsi_cmdlist_put addr=0x%02x data=0x%02x\n",
			__func__, read_reg[i], read_buf[0]);
			return ret;
		}
		out_buf[i] = read_buf[0];
	}
	pr_debug("%s: Read. VCOM1_L=0x%02x VCOM12_H=0x%02x LPVCOM1=0x%02x\n",
		__func__, out_buf[0], out_buf[1], out_buf[2]);
	/* to page 0x00 */
	bank[1] = MDSS_DIAG_DEFAULT_PAGE;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &bank_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if (ret < 0) {
		pr_err("%s: <RESULT_FAILURE> to page 0x%02x.\n", __func__, bank[1]);
		return ret;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_andy_get_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param)
{
	int ret = 0;

	unsigned char readbuf[3];
	memset(readbuf, 0x00, sizeof(readbuf));

	ret = mdss_diag_andy_read_flicker(ctrl, readbuf);
	if (ret < 0) {
		pr_err("%s: <RESULT_FAILURE> mdss_diag_andy_read_flicker.\n", __func__);
		return ret;
	}

	flicker_param->vcom = ((readbuf[1] & 0x01) << 8) | readbuf[0];

	pr_debug("%s: out vcom=0x%04X\n", __func__, flicker_param->vcom);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_andy_get_flicker_low(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param)
{
	int ret = 0;

	unsigned char readbuf[3];
	unsigned short tmp_vcom;
	memset(readbuf, 0x00, sizeof(readbuf));

	ret = mdss_diag_andy_read_flicker(ctrl, readbuf);
	if (ret < 0) {
		pr_err("%s: <RESULT_FAILURE> mdss_diag_andy_read_flicker.\n", __func__);
		return ret;
	}

	tmp_vcom = ((readbuf[1] & 0x01) << 8) | readbuf[0];
	if (readbuf[2] & 0x10) {
		flicker_param->vcom = tmp_vcom - (readbuf[2] & 0x0f) - 1;
	} else {
		flicker_param->vcom = tmp_vcom + (readbuf[2] & 0x0f);
	}
	pr_debug("%s: out vcom_low=0x%04X\n", __func__, flicker_param->vcom);
	return ret;
}
#elif defined(USES_PANEL_HAYABUSA)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_hayabusa_read_flicker(struct mdss_dsi_ctrl_pdata *ctrl, unsigned char out_buf[3])
{
	int ret = 0;
	int i;
	int read_reg_size = 0;
	struct dcs_cmd_req cmdreq;
	char bank[] = {0xFF, 0x20};
	struct dsi_cmd_desc bank_cmd = {
		{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(bank)},
		bank
	};
	unsigned char read_buf[2] = {0x00, 0x00};
	char read_cmd[] = {0x00};
	struct dsi_cmd_desc dcs_read_cmd = {
		{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_cmd)},
		read_cmd
	};
	char read_reg[3] = {0x13, 0x15, 0x5A};

	/* to page 0x20 */
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &bank_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if (ret < 0) {
		pr_err("%s: <RESULT_FAILURE> to page 0x20.\n", __func__);
		return ret;
	}
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
	cmdreq.rlen = sizeof(read_buf);
	cmdreq.rbuf = read_buf;
	cmdreq.cb = NULL; /* call back */

	read_reg_size = sizeof(read_reg);
	for (i = 0; i < read_reg_size; i++) {
		memset(&read_buf, 0, sizeof(read_buf));
		memcpy(&read_cmd, &read_reg[i], sizeof(read_cmd));
		ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
		if (ret < 0) {
			pr_err("%s: <RESULT_FAILURE> mdss_dsi_cmdlist_put addr=0x%02x data=0x%02x\n",
			__func__, read_reg[i], read_buf[0]);
			return ret;
		}
		out_buf[i] = read_buf[0];
	}
	pr_debug("%s: Read. VCOM1_L=0x%02x VCOM12_H=0x%02x LPVCOM2=0x%02x\n",
		__func__, out_buf[0], out_buf[1], out_buf[2]);
	/* to page 0x10 */
	bank[1] = MDSS_DIAG_DEFAULT_PAGE;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &bank_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if (ret < 0) {
		pr_err("%s: <RESULT_FAILURE> to page 0x%02x.\n", __func__, bank[1]);
		return ret;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_hayabusa_get_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param)
{
	int ret = 0;
#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	struct mdss_mdp_ctl *ctl = NULL;
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */

	unsigned char readbuf[3];
	memset(readbuf, 0x00, sizeof(readbuf));

#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	ctl = mdss_shdisp_get_mdpctrl(0);
	mdss_mdp_video_transfer_ctrl(ctl, false);
	ret = mdss_diag_hayabusa_read_flicker(ctrl, readbuf);
	mdss_mdp_video_transfer_ctrl(ctl, true);
#else  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	ret = mdss_diag_hayabusa_read_flicker(ctrl, readbuf);
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	if (ret < 0) {
		pr_err("%s: <RESULT_FAILURE> mdss_diag_hayabusa_read_flicker.\n", __func__);
		return ret;
	}

	flicker_param->vcom = ((readbuf[1] & 0x01) << 8) | readbuf[0];

	pr_debug("%s: out vcom=0x%04X\n", __func__, flicker_param->vcom);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_hayabusa_get_flicker_low(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param)
{
	int ret = 0;
#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	struct mdss_mdp_ctl *ctl = NULL;
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */

	unsigned char readbuf[3];
	unsigned short tmp_vcom;
	memset(readbuf, 0x00, sizeof(readbuf));

#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	ctl = mdss_shdisp_get_mdpctrl(0);
	mdss_mdp_video_transfer_ctrl(ctl, false);
	ret = mdss_diag_hayabusa_read_flicker(ctrl, readbuf);
	mdss_mdp_video_transfer_ctrl(ctl, true);
#else  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	ret = mdss_diag_hayabusa_read_flicker(ctrl, readbuf);
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	if (ret < 0) {
		pr_err("%s: <RESULT_FAILURE> mdss_diag_hayabusa_read_flicker.\n", __func__);
		return ret;
	}

	tmp_vcom = ((readbuf[1] & 0x01) << 8) | readbuf[0];
	if (readbuf[2] & 0x10) {
		flicker_param->vcom = tmp_vcom - (readbuf[2] & 0x0f) - 1;
	} else {
		flicker_param->vcom = tmp_vcom + (readbuf[2] & 0x0f);
	}
	pr_debug("%s: out vcom_low=0x%04X\n", __func__, flicker_param->vcom);
	return ret;
}
#elif defined(USES_PANEL_SINANJU)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_sinanju_read_flicker(struct mdss_dsi_ctrl_pdata *ctrl, unsigned char out_buf[2])
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	unsigned char read_buf[10];
	char data[] = {0xD5};
	struct dsi_cmd_desc cmd = {
		{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(data)},
		data
	};

	memset(&read_buf, 0, sizeof(read_buf));
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
	cmdreq.rlen = sizeof(read_buf);
	cmdreq.rbuf = read_buf;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	if (ret == sizeof(read_buf)) {
		out_buf[0] = read_buf[0];
		out_buf[1] = read_buf[1];
		pr_debug("%s: out_buf=0x%02X 0x%02X\n", __func__, out_buf[0], out_buf[1]);
		ret = 0;
	} else {
		pr_err("%s: <RESULT_FAILURE> mdss_diag_sinanju_read_flicker. ret(%d)\n", __func__, ret);
		ret = -EIO;
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_sinanju_get_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param)
{
	int ret = 0;
	unsigned char readbuf[2];
#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	struct mdss_mdp_ctl *ctl = NULL;
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */

	memset(readbuf, 0x00, sizeof(readbuf));

#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	ctl = mdss_shdisp_get_mdpctrl(0);
	mdss_mdp_video_transfer_ctrl(ctl, false);
	ret = mdss_diag_sinanju_read_flicker(ctrl, readbuf);
	mdss_mdp_video_transfer_ctrl(ctl, true);
#else  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	ret = mdss_diag_sinanju_read_flicker(ctrl, readbuf);
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	if (ret < 0) {
		pr_err("%s: <RESULT_FAILURE> mdss_diag_sinanju_get_flicker.\n", __func__);
		return ret;
	}
	flicker_param->vcom = ((readbuf[0] & 0x03) << 8) | readbuf[1];

	pr_debug("%s: out vcom=0x%04X\n", __func__, flicker_param->vcom);
	return ret;
}
#endif  /* USES_PANEL_XXX */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_get_flicker(struct mdss_dsi_ctrl_pdata *ctrl, struct mdp_flicker_param *flicker_param)
{
	int ret = 0;

	switch (flicker_param->request) {
	case MSMFB_GET_VALUE:
#if defined(USES_PANEL_ANDY)
		ret = mdss_diag_andy_get_flicker(ctrl, flicker_param);
#elif defined(USES_PANEL_HAYABUSA)
		ret = mdss_diag_hayabusa_get_flicker(ctrl, flicker_param);
#elif defined(USES_PANEL_SINANJU)
		ret = mdss_diag_sinanju_get_flicker(ctrl, flicker_param);
#endif  /* USES_PANEL_XXX */
		break;
	case MSMFB_GET_VALUE_LOW:
#if defined(USES_PANEL_ANDY)
		ret = mdss_diag_andy_get_flicker_low(ctrl, flicker_param);
#elif defined(USES_PANEL_HAYABUSA)
		ret = mdss_diag_hayabusa_get_flicker_low(ctrl, flicker_param);
#endif  /* USES_PANEL_XXX */
		break;
	default:
		pr_err("%s: request param error.\n", __func__);
		return -EPERM;
	}
	return ret;
}
#endif /* MDSS_DIAG_PANEL_FLICKER */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_set_flicker_param(struct mdss_panel_data *pdata, struct mdp_flicker_param flicker_param)
{
#ifdef MDSS_DIAG_PANEL_FLICKER
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl;
	unsigned short vcom_max;

	pr_debug("%s: in request=0x%04X vcom=0x%04X\n", __func__, flicker_param.request, flicker_param.vcom);

	if (!pdata) {
		pr_err("%s: panel_data is NULL.\n", __func__);
		return -ENXIO;
	}
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl) {
		pr_err("%s: ctrl_pdata is NULL.\n", __func__);
		return -ENXIO;
	}
	if (flicker_param.request | FLICKER_SET_ALL) {
		;
	}else{
		pr_err("%s: request invalid.\n", __func__);
	}

#if defined(USES_PANEL_ANDY)
	vcom_max = VCOM_MAX_ANDY;
#elif defined(USES_PANEL_HAYABUSA)
	vcom_max = VCOM_MAX_HAYABUSA;
#elif defined(USES_PANEL_SINANJU)
	vcom_max = VCOM_MAX_SINANJU;
#endif  /* USES_PANEL_XXX */
	if (flicker_param.vcom > vcom_max) {
		pr_err("%s: over VCOM_MAX(0x%04d)\n", __func__, flicker_param.vcom);
		return -EINVAL;
	}
#if (VCOM_MIN > 0)
	if (flicker_param.vcom < VCOM_MIN) {
		pr_err("%s: under VCOM_MIN(0x%04d)\n", __func__, flicker_param.vcom);
		return -EINVAL;
	}
#endif

	ret = mdss_diag_set_flicker(ctrl, flicker_param);
	if (ret != 0) {
		pr_err("%s: <RESULT_FAILURE> mdss_diag_set_flicker.\n", __func__);
		return ret;
	}
#endif /* MDSS_DIAG_PANEL_FLICKER */
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_get_flicker_param(struct mdss_panel_data *pdata, struct mdp_flicker_param *flicker_param)
{
#ifdef MDSS_DIAG_PANEL_FLICKER
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl;

	if (!pdata) {
		pr_err("%s: panel_data is NULL.\n", __func__);
		return -ENXIO;
	}
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl) {
		pr_err("%s: ctrl_pdata is NULL.\n", __func__);
		return -ENXIO;
	}
	if (flicker_param == NULL) {
		pr_err("%s: <NULL_POINTER> flicker_param.\n", __func__);
		return -EPERM;
	}
	pr_debug("%s: in request=0x%04X vcom=0x%04X\n", __func__, flicker_param->request, flicker_param->vcom);
	ret = mdss_diag_get_flicker(ctrl, flicker_param);
#endif /* MDSS_DIAG_PANEL_FLICKER */
	return 0;
}

#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_check_panel_type(enum panel_type panel_type)
{
	int ret = 0;
	pr_debug("%s: panel_type=%u\n", __func__, panel_type);
	switch (panel_type) {
#if defined(USES_PANEL_ANDY)
	case PANEL_TYPE_ANDY:
#elif defined(USES_PANEL_HAYABUSA)
	case PANEL_TYPE_HAYABUSA:
#elif defined(USES_PANEL_SINANJU)
	case PANEL_TYPE_SINANJU:
#endif  /* USES_PANEL_XXX */
	//case PANEL_TYPE_SAZABI:
		break;
	default:
		pr_err("%s: unsupported panel_type[%d]\n",
			__func__, panel_type);
		ret = -EINVAL;
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_panel_copy_gmm_to_paddata(int paddlen,
				char *paddata,
				unsigned short *gmm)
{
	int paddatapos = 0;
	int gmmpos = 0;
	while(paddatapos < paddlen) {
		paddata[paddatapos++] = ((gmm[gmmpos] >> 8) & 0x0003);
		paddata[paddatapos++] = (gmm[gmmpos++] & 0x00FF);
	}
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_panel_copy_gmm_from_paddata(int len, unsigned short *gmm,
				char *paddata)
{
	int cnt = 0;
	int paddatapos = 0;
	for (cnt = 0; cnt < len; ++cnt) {
		paddatapos = cnt * 2;
		gmm[cnt] =  (paddata[paddatapos] << 8) | (paddata[paddatapos + 1]);
		gmm[cnt] &= 0x03FF;
	}
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_get_wdtype_fromlen(int datalen)
{
	int dtype = DTYPE_DCS_WRITE;
	switch(datalen) {
	case 0:
		dtype = DTYPE_DCS_WRITE;
		break;
	case 1:
		dtype = DTYPE_DCS_WRITE1;
		break;
	default:
		dtype = DTYPE_DCS_LWRITE;
	}
	return dtype;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_panel_clear_cmds(void)
{
	struct mdss_gmm_kerl_ctx *gmm_ctx = &mdss_diag_ctx.mdss_gmm_ctx;
	/* clear dsi cmds structure and payload buffer */
	memset(&gmm_ctx->cmds, 0, sizeof(gmm_ctx->cmds));
	memset(&gmm_ctx->cmds_payloads, 0,
			sizeof(gmm_ctx->cmds_payloads));
	gmm_ctx->cmds_cnt = 0;
	gmm_ctx->free_payload_pos = 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static bool mdss_diag_panel_add_cmd(char addr, int datalen, char *data)
{
	struct dsi_cmd_desc *dsc = NULL;
	char *payload = NULL;
	struct mdss_gmm_kerl_ctx *gmm_ctx = &mdss_diag_ctx.mdss_gmm_ctx;

	pr_debug("%s: addr=0x%02x, datalen=%d, data[0] = 0x%02x\n",
			__func__, addr, datalen, data[0]);

	if (gmm_ctx->cmds_cnt >= MDSS_DIAG_DSI_MAX_CMDS_CNT) {
		pr_err("%s: failed to add: addr(0x%02x), datalen(%d),"
			" data[0](0x%02x) cmdcnt(%d)\n",
			__func__, addr, datalen, data[0],
			gmm_ctx->cmds_cnt);
		return false;
	}

	if (gmm_ctx->free_payload_pos + (datalen + 1)
		>= MDSS_DIAG_DSI_PAYLOADS_LENGHTH) {
		pr_err("%s: failed to add: addr(0x%02x), datalen(%d), "
			"data[0](0x%02x) free_payload_cnt(%d)\n",
			__func__, addr, datalen, data[0],
			MDSS_DIAG_DSI_PAYLOADS_LENGHTH
				 - gmm_ctx->free_payload_pos);
		return false;
	}

	payload = &gmm_ctx->cmds_payloads[gmm_ctx->free_payload_pos];
	*payload = addr;
	memcpy(payload + 1, data, datalen);

	dsc = &gmm_ctx->cmds[gmm_ctx->cmds_cnt];
	memset(dsc, 0, sizeof(*dsc));
	dsc->dchdr.dtype = mdss_diag_get_wdtype_fromlen(datalen);
	dsc->dchdr.last = 0;
	dsc->dchdr.vc = 0;
	dsc->dchdr.ack = 0;
	dsc->dchdr.wait = 0;
	dsc->dchdr.dlen = (datalen + 1);
	dsc->payload = payload;

	gmm_ctx->cmds_cnt++;
	gmm_ctx->free_payload_pos += (datalen + 1); /* addr + data */

	return true;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_kickoff_cmds(struct mdss_panel_data *pdata)
{
	int ret = 0;
#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	struct mdss_mdp_ctl *ctl;

	ctl = mdss_shdisp_get_mdpctrl(0);
	mdss_mdp_video_transfer_ctrl(ctl, false);
	ret = mdss_diag_panel_kickoff_cmds_sub(pdata);
	mdss_mdp_video_transfer_ctrl(ctl, true);
#else  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	ret = mdss_diag_panel_kickoff_cmds_sub(pdata);
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_kickoff_cmds_sub(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_gmm_kerl_ctx *gmm_ctx = &mdss_diag_ctx.mdss_gmm_ctx;

	pr_debug("%s: added_cmds=%d\n", __func__, gmm_ctx->cmds_cnt);
	if ((!gmm_ctx->cmds_cnt)
		|| (gmm_ctx->cmds_cnt > MDSS_DIAG_DSI_MAX_CMDS_CNT))
		return 0;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		return -EPERM;
	}

	gmm_ctx->cmds[gmm_ctx->cmds_cnt-1].dchdr.last = 1;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = gmm_ctx->cmds;
	cmdreq.cmds_cnt = gmm_ctx->cmds_cnt;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.rbuf = NULL;
	cmdreq.cb = NULL;

	ret = mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	if (ret < 0) {
		pr_err("%s: failed to mdss_dsi_cmdlist_put\n",
			__func__);
	}
	pr_debug("%s: out ret=%d\n", __func__, ret);
	return ret;
}
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_panel_dcs_write0(struct mdss_panel_data *pdata, char addr)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc dsc;
	char payload[2];
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		return -EPERM;
	}

	memset(&payload, 0, sizeof(payload));
	payload[0] = addr;
	payload[1] = 0x00;

	memset(&dsc, 0, sizeof(dsc));
	dsc.dchdr.dtype = DTYPE_DCS_WRITE;
	dsc.dchdr.last = 1;
	dsc.dchdr.vc = 0;
	dsc.dchdr.ack = 0;
	dsc.dchdr.wait = 0;
	dsc.dchdr.dlen = 1;
	dsc.payload = payload;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dsc;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_CLK_CTRL | CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.rbuf = NULL;
	cmdreq.cb = NULL;

	ret = mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	if (ret <= 0) {
		pr_err("%s: failed to mdss_dsi_cmdlist_put\n",
			__func__);
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_panel_dcs_write1(struct mdss_panel_data *pdata,
				char addr,
				char data)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc dsc;
	char payload[2];
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (!pdata) {
		return -EPERM;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		return -EPERM;
	}

	memset(&payload, 0, sizeof(payload));
	payload[0] = addr;
	payload[1] = data;

	memset(&dsc, 0, sizeof(dsc));
	dsc.dchdr.dtype = DTYPE_DCS_WRITE1;
	dsc.dchdr.last = 1;
	dsc.dchdr.vc = 0;
	dsc.dchdr.ack = 0;
	dsc.dchdr.wait = 0;
	dsc.dchdr.dlen = 2;
	dsc.payload = payload;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dsc;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_CLK_CTRL | CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.rbuf = NULL;
	cmdreq.cb = NULL;

	ret = mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	if (ret <= 0) {
		pr_err("%s: failed to mdss_dsi_cmdlist_put\n",
			__func__);
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_dcs_read(struct mdss_panel_data *pdata, char addr,
				int rlen, char *rbuf)
{
	int ret;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc dsc;
	char payload;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		return -EPERM;
	}

	payload = addr;

	memset(&dsc, 0, sizeof(dsc));
	dsc.dchdr.dtype = DTYPE_DCS_READ;
	dsc.dchdr.last = 1;
	dsc.dchdr.vc = 0;
	dsc.dchdr.ack = 1;
	dsc.dchdr.wait = 5;
	dsc.dchdr.dlen = 1;
	dsc.payload = &payload;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dsc;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
	cmdreq.rlen = rlen;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = NULL;

	ret = mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
	pr_debug("%s: addr=0x%02x, rbuf[0]=0x%02X\n", __func__, addr, rbuf[0]);
	if (ret != rlen) {
		pr_err("%s: failed to mdss_dsi_cmdlist_put("
			   "addr[0x%02x], rlen[%d] "
			   "rbuf[0x%02X], ret[%d]\n",
			__func__,
			addr, rlen,
			rbuf[0], ret);
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_switch_panel_page(struct mdss_panel_data *pdata,
				short page)
{
	char pageaddr = 0xFF;
	if(page < 0) {
		return 0;
	}
	return mdss_diag_panel_dcs_write1(pdata, pageaddr, page);
}

#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_panel_update_gmm_volt_pad(
				union mdp_gmm_volt *gmm_volt)
{
#if defined(USES_PANEL_ANDY)
	mdss_diag_andy_update_gmm_pad(&gmm_volt->andy);
	mdss_diag_andy_update_volt_pad(&gmm_volt->andy);
#elif defined(USES_PANEL_SINANJU)
	mdss_diag_sinanju_update_gmm_pad(&gmm_volt->sinanju);
	mdss_diag_sinanju_update_volt_pad(&gmm_volt->sinanju);
#elif defined(USES_PANEL_HAYABUSA)
	mdss_diag_hayabusa_update_gmm_pad(&gmm_volt->hayabusa);
	mdss_diag_hayabusa_update_volt_pad(&gmm_volt->hayabusa);
	mdss_diag_hayabusa_update_advgmm_pad(&gmm_volt->hayabusa);
#endif  /* USES_PANEL_XXX */
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_make_gmm_volt_cmds(void)
{
	int ret = 0;
	size_t paditemlen = 0;
	const struct mdss_diag_pad_item *paditems = NULL;
#if defined(USES_PANEL_ANDY)
	paditemlen = ARRAY_SIZE(mdss_diag_andy_gmm_volt_pads);
	paditems = mdss_diag_andy_gmm_volt_pads;
#elif defined(USES_PANEL_SINANJU)
	paditemlen = ARRAY_SIZE(mdss_diag_sinanju_gmm_volt_pads);
	paditems = mdss_diag_sinanju_gmm_volt_pads;
#elif defined(USES_PANEL_HAYABUSA)
	paditemlen = ARRAY_SIZE(mdss_diag_hayabusa_gmm_volt_pads);
	paditems = mdss_diag_hayabusa_gmm_volt_pads;
#endif  /* USES_PANEL_XXX */
	ret = mdss_diag_panel_make_gmm_volt_cmds_paditems(paditemlen, paditems);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_read_paditems(struct mdss_panel_data *pdata,
				size_t paditemlen,
				const struct mdss_diag_pad_item *paditems)
{
	int ret = 0;
	ret = mdss_diag_panel_read_paditems_sub(pdata, paditemlen, paditems);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_read_paditems_sub(struct mdss_panel_data *pdata,
				size_t paditemlen,
				const struct mdss_diag_pad_item *paditems)
{
	int ret = 0;
	size_t paditempos = 0;
	char lastpage = 0;

	pr_debug("%s: in\n", __func__);
	if ((paditemlen == 0) || (!paditems)) {
		pr_err("%s: paditems error(len[%zu], paditems[0x%p])\n",
			__func__, paditemlen, paditems);
		return -EINVAL;
	}

	for (paditempos = 0; paditempos < paditemlen; paditempos++) {
		struct mdss_diag_panel_pad *pad
			 = paditems[paditempos].pad;
		size_t padlen = paditems[paditempos].len;
		size_t padpos = 0;

		for (padpos = 0; padpos < padlen; padpos++) {
			/* send change page cmd, when page changes */
			if (lastpage != pad[padpos].page) {
				lastpage = pad[padpos].page;
				ret = mdss_diag_panel_switch_panel_page(
					pdata, lastpage);
				if (ret) {
					pr_err("%s: failed to change page "
						   "paditempos[%zu], "
						   "padpos[%zu],"
						   "page[0x%02x]\n",
						   __func__,
						   paditempos,
						   padpos,
						   lastpage);
					goto err_end;
				}
			}
			/* clear read buffer */
			memset(pad[padpos].data, 0, 1);

			ret = mdss_diag_panel_dcs_read(pdata,
						pad[padpos].addr,
						pad[padpos].len,
						pad[padpos].data);
			if (ret) {
				pr_err("%s: failed to read page(0x%02x), "
					   "addr(0x%02x)\n", __func__,
					   pad[padpos].page,
					   pad[padpos].addr);
				goto err_end;
			}
		}
	}

	// change page to default
	mdss_diag_panel_switch_panel_page(pdata, MDSS_DIAG_DEFAULT_PAGE);

	ret = 0;

err_end:
	pr_debug("%s: out ret=%d\n", __func__, ret);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_make_gmm_volt_cmds_paditems(size_t paditemlen,
				const struct mdss_diag_pad_item *paditems)
{
	int ret = 0;
	size_t paditempos = 0;
	char lastpage = 0;
	char lastpagepayload[2]    = {0xFF, 0x00};
	bool added_all = true;

	pr_debug("%s: in\n", __func__);
	if ((paditemlen == 0) || (!paditems)) {
		pr_err("%s: paditems error(len[%zu], paditems[0x%p])\n",
			__func__, paditemlen, paditems);
		return -EINVAL;
	}

	for (paditempos = 0; paditempos < paditemlen; paditempos++) {
		struct mdss_diag_panel_pad *pad
			 = paditems[paditempos].pad;
		size_t padlen = paditems[paditempos].len;
		size_t padpos = 0;

		for (padpos = 0; padpos < padlen; padpos++) {
			/* add change page cmd, when page changes */
			if (lastpage != pad[padpos].page) {
				lastpage = pad[padpos].page;
				lastpagepayload[1] = lastpage;
				added_all &= mdss_diag_panel_add_cmd(
					lastpagepayload[0], 1,
					&lastpagepayload[1]);
			}

			/* add cmd (gmm or volt and other) */
			added_all &= mdss_diag_panel_add_cmd(
					pad[padpos].addr, pad[padpos].len,
					pad[padpos].data);
		}
	}

	if (!added_all) {
		pr_err("%s: failed to add dsicmd\n", __func__);
		ret = -EINVAL;
	}
	pr_debug("%s: out ret = %d, added_all=%d\n", __func__, ret, added_all);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_read_gmm_volt(struct mdss_panel_data *pdata)
{
	int ret;
	size_t paditemlen = 0;
	const struct mdss_diag_pad_item *paditems = NULL;
#if defined(USES_PANEL_ANDY)
	paditemlen = ARRAY_SIZE(mdss_diag_andy_gmm_volt_pads);
	paditems = mdss_diag_andy_gmm_volt_pads;
#elif defined(USES_PANEL_SINANJU)
	paditemlen = ARRAY_SIZE(mdss_diag_sinanju_gmm_volt_pads);
	paditems = mdss_diag_sinanju_gmm_volt_pads;
#elif defined(USES_PANEL_HAYABUSA)
	paditemlen = ARRAY_SIZE(mdss_diag_hayabusa_gmm_volt_pads);
	paditems = mdss_diag_hayabusa_gmm_volt_pads;
#endif  /* USES_PANEL_XXX */
	ret = mdss_diag_panel_read_paditems(pdata, paditemlen, paditems);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_panel_read_gmm_volt_dispoff(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_mdp_ctl *ctl;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

#if 0
	mdss_dsi_pre_panel_off(pdata);
#endif

	ctl = mdss_shdisp_get_mdpctrl(0);
	if (ctrl_pdata->panel_mode == DSI_VIDEO_MODE) {
		mdss_mdp_video_transfer_ctrl(ctl, false);
	}

	mdss_diag_panel_switch_panel_page(pdata, MDSS_DIAG_DEFAULT_PAGE);
	mdss_diag_panel_dispoff(pdata);
#if defined(USES_PANEL_HAYABUSA)
	mdss_diag_panel_enter_sleep(pdata);
#endif /* USES_PANEL_HAYABUSA */

	mdss_diag_panel_read_gmm_volt(pdata);

#if defined(USES_PANEL_HAYABUSA)
	mdss_diag_panel_exit_sleep(pdata);
#endif /* USES_PANEL_HAYABUSA */
	mdss_diag_panel_dispon(pdata);

	if (ctrl_pdata->panel_mode == DSI_VIDEO_MODE) {
		mdss_mdp_video_transfer_ctrl(ctl, true);
	}

#if 0
	mdss_det_post_panel_on(pdata);
#endif
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_panel_copy_gmm_volt_from_pad(
				struct mdp_gmm_volt_info *gmm_volt_info)
{
#if defined(USES_PANEL_ANDY)
	mdss_diag_andy_copy_gmm_volt_from_pad(
					&gmm_volt_info->gmm_volt.andy);
#elif defined(USES_PANEL_SINANJU)
	mdss_diag_sinanju_copy_gmm_volt_from_pad(
					&gmm_volt_info->gmm_volt.sinanju);
#elif defined(USES_PANEL_HAYABUSA)
	mdss_diag_hayabusa_copy_gmm_volt_from_pad(
					&gmm_volt_info->gmm_volt.hayabusa);
#endif  /* USES_PANEL_XXX */
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */

static int mdss_diag_set_panel_typ_volt(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	pr_debug("%s: send typ volt\n", __func__);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: ctrl_pdata is NULL.\n", __func__);
		return -ENXIO;
	}

	if (ctrl_pdata->panel_typ_volt_cmds.cmd_cnt == 0) {
		pr_warn("%s: typ_volt_cmds is not present\n", __func__);
		return 0;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = ctrl_pdata->panel_typ_volt_cmds.cmds;
	cmdreq.cmds_cnt = ctrl_pdata->panel_typ_volt_cmds.cmd_cnt;
	cmdreq.flags = CMD_CLK_CTRL | CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.rbuf = NULL;
	cmdreq.cb = NULL;

	ret = mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
	if (ret < 0) {
		pr_err("%s: failed to mdss_dsi_cmdlist_put ret[%d]\n",
			__func__, ret);
	}

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_set_panel_typ_gmm(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	pr_debug("%s: send typ gmm\n", __func__);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: ctrl_pdata is NULL.\n", __func__);
		return -ENXIO;
	}

	if (ctrl_pdata->panel_typ_gmm_cmds.cmd_cnt == 0) {
		pr_warn("%s: typ_gmm_cmds is not present\n", __func__);
		return 0;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = ctrl_pdata->panel_typ_gmm_cmds.cmds;
	cmdreq.cmds_cnt = ctrl_pdata->panel_typ_gmm_cmds.cmd_cnt;
	cmdreq.flags = CMD_CLK_CTRL | CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
	cmdreq.rlen = 0;
	cmdreq.rbuf = NULL;
	cmdreq.cb = NULL;

	ret = mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
	if (ret < 0) {
		pr_err("%s: failed to mdss_dsi_cmdlist_put ret[%d]\n",
			__func__, ret);
	}

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */

static int mdss_diag_set_panel_gmm(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_gmm_kerl_ctx *gmm_ctx = &mdss_diag_ctx.mdss_gmm_ctx;

	if (!pdata) {
		pr_err("%s: panel_data is NULL.\n", __func__);
		return -ENXIO;
	}

	// gmm is adjusted
	if (gmm_ctx->gmm_adj_status == MDSS_GMM_ADJ_STATUS_OK) {
		pr_debug("%s: build and send adjusted gmm\n", __func__);
		/* update pad from adjusted gmm */
		mdss_diag_panel_update_gmm_volt_pad(
			&gmm_ctx->gmm_volt_adjusted);

		/* refresh dsi cmds buffer */
		mdss_diag_panel_clear_cmds();

		/* build dsi cmds from pad(gmm/volt/other) */
		ret = mdss_diag_panel_make_gmm_volt_cmds();
		if (ret) {
			return ret;
		}

		/* send gmm/volt/other cmds to panel H/W */
		ret = mdss_diag_panel_kickoff_cmds_sub(pdata);
	}

	if (ret > 0) {
		ret = 0;
	} else if (ret < 0) {
		pr_err("%s: failed to send gmm adj_staus=0x%02x\n",
			__func__, gmm_ctx->gmm_adj_status);
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */

static int mdss_diag_panel_set_adjust_gmm(struct mdss_panel_data *pdata,
			struct mdp_gmm_volt_info *gmm_volt_info)
{
	int ret = 0;
	struct mdss_gmm_kerl_ctx *gmm_ctx = &mdss_diag_ctx.mdss_gmm_ctx;

	/* convert and store gmm/volt/other to pad */
	mdss_diag_panel_update_gmm_volt_pad(&gmm_volt_info->gmm_volt);

	/* refresh dsi cmds buffer */
	mdss_diag_panel_clear_cmds();

	/* build dsi cmds from pad(gmm/volt/other) */
	ret = mdss_diag_panel_make_gmm_volt_cmds();
	if (ret) {
		return ret;
	}

	/* send gmm/volt/other cmds to panel H/W */
	ret = mdss_diag_panel_kickoff_cmds(pdata);
	if (ret > 0) {
		ret = 0;
	}

	mdss_diag_panel_switch_panel_page(pdata, MDSS_DIAG_DEFAULT_PAGE);

	/* copy adjusted gmm/volt/other for panel on cmds */
	memcpy(&gmm_ctx->gmm_volt_adjusted,
		&gmm_volt_info->gmm_volt,
		sizeof(gmm_ctx->gmm_volt_adjusted));
	gmm_ctx->gmm_adj_status = MDSS_GMM_ADJ_STATUS_OK;

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */

static int mdss_diag_panel_set_unadjust_gmm(struct mdss_panel_data *pdata,
			struct mdp_gmm_volt_info *gmm_volt_info)
{
	int ret = 0;
#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	struct mdss_mdp_ctl *ctl = NULL;

	ctl = mdss_shdisp_get_mdpctrl(0);
	mdss_mdp_video_transfer_ctrl(ctl, false);
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */

	mdss_diag_ctx.mdss_gmm_ctx.gmm_adj_status = MDSS_GMM_ADJ_STATUS_NOT_SET;
	ret = mdss_diag_set_panel_typ_volt(pdata);
	if (ret < 0) {
		goto error;
	}
	ret = mdss_diag_set_panel_typ_gmm(pdata);
	if (ret < 0) {
		goto error;
	}
	ret = 0;
error:
#if defined(MDSS_DIAG_PANEL_VIDEO_MODE)
	mdss_mdp_video_transfer_ctrl(ctl, true);
#endif  /* MDSS_DIAG_PANEL_VIDEO_MODE */
	mdss_diag_panel_switch_panel_page(pdata, MDSS_DIAG_DEFAULT_PAGE);
	return ret;
}

#if defined(USES_PANEL_ANDY)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_andy_copy_gmm_volt_from_pad(
				struct andy_gmm_volt *gmm_volt)
{
	size_t cnt = 0;

	pr_debug("%s: in\n", __func__);

	/* copy gmmR from pad */
	mdss_diag_panel_copy_gmm_from_pad(
			ARRAY_SIZE(gmm_volt->gmmR),
			gmm_volt->gmmR,
			mdss_diag_andy_gmm_R);

	/* copy gmmG from pad */
	mdss_diag_panel_copy_gmm_from_pad(
			ARRAY_SIZE(gmm_volt->gmmG),
			gmm_volt->gmmG,
			mdss_diag_andy_gmm_G);

	/* copy gmmB from pad */
	mdss_diag_panel_copy_gmm_from_pad(
			ARRAY_SIZE(gmm_volt->gmmB),
			gmm_volt->gmmB,
			mdss_diag_andy_gmm_B);

	// copy voltage from pad
	gmm_volt->vgh    = mdss_diag_andy_volt[cnt++].data;
	gmm_volt->vgl    = mdss_diag_andy_volt[cnt++].data;
	gmm_volt->gvddp  = mdss_diag_andy_volt[cnt++].data;
	gmm_volt->gvddn  = mdss_diag_andy_volt[cnt++].data;
	gmm_volt->gvddp2 = mdss_diag_andy_volt[cnt++].data;
	gmm_volt->vgho   = mdss_diag_andy_volt[cnt++].data;
	gmm_volt->vglo   = mdss_diag_andy_volt[cnt++].data;
	gmm_volt->avddr  = mdss_diag_andy_volt[cnt++].data;
	gmm_volt->aveer  = mdss_diag_andy_volt[cnt].data;

	pr_debug("%s: out\n", __func__);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_andy_update_gmm_pad(
				struct andy_gmm_volt *gmm_volt)
{
	pr_debug("%s: in\n", __func__);

	/* update gmmR */
	mdss_diag_panel_copy_gmm_to_pad(
			ARRAY_SIZE(mdss_diag_andy_gmm_R),
			mdss_diag_andy_gmm_R,
			gmm_volt->gmmR);

	/* update gmmG */
	mdss_diag_panel_copy_gmm_to_pad(
			ARRAY_SIZE(mdss_diag_andy_gmm_G),
			mdss_diag_andy_gmm_G,
			gmm_volt->gmmG);

	/* update gmmB */
	mdss_diag_panel_copy_gmm_to_pad(
			ARRAY_SIZE(mdss_diag_andy_gmm_B),
			mdss_diag_andy_gmm_B,
			gmm_volt->gmmB);

	pr_debug("%s: out\n", __func__);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_andy_update_volt_pad(
				struct andy_gmm_volt *gmm_volt)
{
	int cnt = 0;
	pr_debug("%s: in\n", __func__);

	/* update voltage */
	mdss_diag_andy_volt[cnt++].data = gmm_volt->vgh;
	mdss_diag_andy_volt[cnt++].data = gmm_volt->vgl;
	mdss_diag_andy_volt[cnt++].data = gmm_volt->gvddp;
	mdss_diag_andy_volt[cnt++].data = gmm_volt->gvddn;
	mdss_diag_andy_volt[cnt++].data = gmm_volt->gvddp2;
	mdss_diag_andy_volt[cnt++].data = gmm_volt->vgho;
	mdss_diag_andy_volt[cnt++].data = gmm_volt->vglo;
	mdss_diag_andy_volt[cnt++].data = gmm_volt->avddr;
	mdss_diag_andy_volt[cnt].data   = gmm_volt->aveer;

	pr_debug("%s: out\n", __func__);
}

#elif defined(USES_PANEL_SINANJU)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_sinanju_copy_gmm_volt_from_pad(
				struct sinanju_gmm_volt *gmm_volt)
{
	pr_debug("%s: in\n", __func__);

	/* copy gmm from pad */
	mdss_diag_panel_copy_gmm_from_paddata(
			ARRAY_SIZE(gmm_volt->gmm),
			gmm_volt->gmm,
			mdss_diag_sinanju_gmm_data);

	// copy voltage from pad
	gmm_volt->vgh = mdss_diag_sinanju_vol_data_d0[SINANJU_VGH_POS];
	gmm_volt->vgl = mdss_diag_sinanju_vol_data_d0[SINANJU_VGL_POS];
	gmm_volt->vpl = mdss_diag_sinanju_vol_data_d1[SINANJU_VPL_POS];
	gmm_volt->vnl = mdss_diag_sinanju_vol_data_d1[SINANJU_VNL_POS];

	pr_debug("%s: out\n", __func__);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_sinanju_update_gmm_pad(
				struct sinanju_gmm_volt *gmm_volt)
{
	pr_debug("%s: in\n", __func__);

	/* update gmm */
	mdss_diag_panel_copy_gmm_to_paddata(
			ARRAY_SIZE(mdss_diag_sinanju_gmm_data),
			mdss_diag_sinanju_gmm_data,
			gmm_volt->gmm);

	pr_debug("%s: out\n", __func__);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_sinanju_update_volt_pad(
				struct sinanju_gmm_volt *gmm_volt)
{
	pr_debug("%s: in\n", __func__);

	/* update voltage */
	mdss_diag_sinanju_vol_data_d0[SINANJU_VGH_POS] = gmm_volt->vgh;
	mdss_diag_sinanju_vol_data_d0[SINANJU_VGL_POS] = gmm_volt->vgl;
	mdss_diag_sinanju_vol_data_d1[SINANJU_VPL_POS] = gmm_volt->vpl;
	mdss_diag_sinanju_vol_data_d1[SINANJU_VNL_POS] = gmm_volt->vnl;

	pr_debug("%s: out\n", __func__);
}
#elif defined(USES_PANEL_HAYABUSA)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_hayabusa_copy_gmm_volt_from_pad(
				struct hayabusa_gmm_volt *gmm_volt)
{
	size_t cnt = 0, len = 0;

	pr_debug("%s: in\n", __func__);

	/* copy gmmR from pad */
	mdss_diag_panel_copy_gmm_from_paddata(
			ARRAY_SIZE(gmm_volt->gmmR),
			gmm_volt->gmmR,
			mdss_diag_hayabusa_gmm_R_data);

	/* copy gmmG from pad */
	mdss_diag_panel_copy_gmm_from_paddata(
			ARRAY_SIZE(gmm_volt->gmmG),
			gmm_volt->gmmG,
			mdss_diag_hayabusa_gmm_G_data);

	/* copy gmmB from pad */
	mdss_diag_panel_copy_gmm_from_paddata(
			ARRAY_SIZE(gmm_volt->gmmB),
			gmm_volt->gmmB,
			mdss_diag_hayabusa_gmm_B_data);

	// copy voltage from pad
	gmm_volt->vgh    = mdss_diag_hayabusa_volt_data[cnt++];
	gmm_volt->vgl    = mdss_diag_hayabusa_volt_data[cnt++];
	gmm_volt->gvddp  = mdss_diag_hayabusa_volt_data[cnt++];
	gmm_volt->gvddn  = mdss_diag_hayabusa_volt_data[cnt++];
	gmm_volt->gvddp2 = mdss_diag_hayabusa_volt_data[cnt++];
	gmm_volt->vgho   = mdss_diag_hayabusa_volt_data[cnt++];
	gmm_volt->vglo   = mdss_diag_hayabusa_volt_data[cnt++];

	cnt = 0;
	len = ARRAY_SIZE(gmm_volt->adv_gmm);

	/* copy advanced gmm from pad */
	memcpy(gmm_volt->adv_gmm, mdss_diag_hayabusa_advgmm_data, len);

	pr_debug("%s: out\n", __func__);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_hayabusa_update_gmm_pad(
				struct hayabusa_gmm_volt *gmm_volt)
{
	pr_debug("%s: in\n", __func__);

	/* update gmmR */
	mdss_diag_panel_copy_gmm_to_paddata(
			ARRAY_SIZE(mdss_diag_hayabusa_gmm_R_data),
			mdss_diag_hayabusa_gmm_R_data,
			gmm_volt->gmmR);

	/* update gmmG */
	mdss_diag_panel_copy_gmm_to_paddata(
			ARRAY_SIZE(mdss_diag_hayabusa_gmm_G_data),
			mdss_diag_hayabusa_gmm_G_data,
			gmm_volt->gmmG);

	/* update gmmB */
	mdss_diag_panel_copy_gmm_to_paddata(
			ARRAY_SIZE(mdss_diag_hayabusa_gmm_B_data),
			mdss_diag_hayabusa_gmm_B_data,
			gmm_volt->gmmB);

	pr_debug("%s: out\n", __func__);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_hayabusa_update_volt_pad(
				struct hayabusa_gmm_volt *gmm_volt)
{
	int cnt = 0;
	pr_debug("%s: in\n", __func__);

	/* update voltage */
	mdss_diag_hayabusa_volt_data[cnt++] = gmm_volt->vgh;
	mdss_diag_hayabusa_volt_data[cnt++] = gmm_volt->vgl;
	mdss_diag_hayabusa_volt_data[cnt++] = gmm_volt->gvddp;
	mdss_diag_hayabusa_volt_data[cnt++] = gmm_volt->gvddn;
	mdss_diag_hayabusa_volt_data[cnt++] = gmm_volt->gvddp2;
	mdss_diag_hayabusa_volt_data[cnt++] = gmm_volt->vgho;
	mdss_diag_hayabusa_volt_data[cnt]   = gmm_volt->vglo;

	pr_debug("%s: out\n", __func__);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void mdss_diag_hayabusa_update_advgmm_pad(
				struct hayabusa_gmm_volt *gmm_volt)
{
	int len = ARRAY_SIZE(gmm_volt->adv_gmm);
	pr_debug("%s: in\n", __func__);

	/* update advanced gmm */
	memcpy(mdss_diag_hayabusa_advgmm_data, gmm_volt->adv_gmm, len);

	pr_debug("%s: out\n", __func__);
}
#endif  /* USES_PANEL_XXX */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_panel_set_gmm(struct mdss_panel_data *pdata,
			struct mdp_gmm_volt_info *gmm_volt_info)
{
	int ret = 0;
	pr_info("%s: in\n", __func__);

	if ((!pdata) || (!gmm_volt_info)) {
		pr_err("%s: argment err pdata(0x%p), gmm_volt_info(0x%p)\n",
			__func__, pdata, gmm_volt_info);
		return -EINVAL;
	}

	ret = mdss_diag_panel_check_panel_type(
		gmm_volt_info->panel_type);
	if (ret) {
		return ret;
	}

	switch(gmm_volt_info->request) {
	case MSMFB_GMMVOLT_REQ_ADJUST:
		ret = mdss_diag_panel_set_adjust_gmm(
					pdata, gmm_volt_info);
		break;
	case MSMFB_GMMVOLT_REQ_UNADJUST:
		ret = mdss_diag_panel_set_unadjust_gmm(
					pdata, gmm_volt_info);
		break;
	default:
		pr_err("%s: unsupported request(%d)\n",
			__func__, gmm_volt_info->request);
		return -EINVAL;
	}

	pr_info("%s: out ret=%d\n", __func__, ret);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_panel_get_gmm(struct mdss_panel_data *pdata,
			struct mdp_gmm_volt_info *gmm_volt_info)
{
	int ret = 0;
	pr_info("%s: in\n", __func__);

	if ((!pdata) || (!gmm_volt_info)) {
		pr_err("%s: argment err pdata(0x%p), gmm_volt_info(0x%p)\n",
			__func__, pdata, gmm_volt_info);
		return -EINVAL;
	}

	ret = mdss_diag_panel_check_panel_type(
		gmm_volt_info->panel_type);

	if (ret) {
		return ret;
	}

	ret = mdss_diag_panel_read_gmm_volt_dispoff(pdata);

	if (ret) {
		return ret;
	}

	/* copy read to gmm_volt */
	mdss_diag_panel_copy_gmm_volt_from_pad(gmm_volt_info);

	pr_info("%s: out ret=%d\n", __func__, ret);
	return ret;
}
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */

#ifdef MDSS_DIAG_PANEL_FLICKER
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_init_flicker_param(struct mdss_flicker_ctx *flicker_ctx)
{
	int ret = 0;

	if (!IS_FLICKER_ADJUSTED(flicker_ctx->nvram)) {
		pr_debug("%s: flicker is not adjusted\n", __func__);
		return ret;
	}
	mdss_diag_ctx.mdss_flicker_ctx.vcom.vcom = flicker_ctx->vcom.vcom;
	mdss_diag_ctx.mdss_flicker_ctx.vcom.vcom_low = flicker_ctx->vcom.vcom_low;
	mdss_diag_ctx.mdss_flicker_ctx.nvram = flicker_ctx->nvram;
	return ret;
}
#endif /* MDSS_DIAG_PANEL_FLICKER */

#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE
#if defined(USES_PANEL_ANDY)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_init_andy_gmm_param(
					struct mdss_gmmvolt_ctx *gmmvolt_ctx)
{
	int ret = 0;
	struct mdss_gmm_kerl_ctx *gmm_ctx = &mdss_diag_ctx.mdss_gmm_ctx;

	gmm_ctx->gmm_adj_status = gmmvolt_ctx->status;
	memcpy(&gmm_ctx->gmm_volt_adjusted.andy,
		&gmmvolt_ctx->gmm_volt.andy,
		sizeof(gmm_ctx->gmm_volt_adjusted.andy));

	return ret;
}

#elif defined(USES_PANEL_SINANJU)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_init_sinanju_gmm_param(
					struct mdss_gmmvolt_ctx *gmmvolt_ctx)
{
	int ret = 0;
	struct mdss_gmm_kerl_ctx *gmm_ctx = &mdss_diag_ctx.mdss_gmm_ctx;

	gmm_ctx->gmm_adj_status = gmmvolt_ctx->status;
	memcpy(&gmm_ctx->gmm_volt_adjusted.sinanju,
		&gmmvolt_ctx->gmm_volt.sinanju,
		sizeof(gmm_ctx->gmm_volt_adjusted.sinanju));

	return ret;
}
#elif defined(USES_PANEL_HAYABUSA)
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_init_hayabusa_gmm_param(
					struct mdss_gmmvolt_ctx *gmmvolt_ctx)
{
	int ret = 0;
	struct mdss_gmm_kerl_ctx *gmm_ctx = &mdss_diag_ctx.mdss_gmm_ctx;

	gmm_ctx->gmm_adj_status = gmmvolt_ctx->status;
	memcpy(&gmm_ctx->gmm_volt_adjusted.hayabusa,
		&gmmvolt_ctx->gmm_volt.hayabusa,
		sizeof(gmm_ctx->gmm_volt_adjusted.hayabusa));

	return ret;
}
#endif  /* USES_PANEL_XXX */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int mdss_diag_init_gmm_param(struct mdss_gmmvolt_ctx *gmmvolt_ctx)
{
	int ret = 0;

	if (gmmvolt_ctx->status != MDSS_GMM_ADJ_STATUS_OK) {
		pr_debug("%s: gmm is not adjusted\n", __func__);
		return ret;
	}
#if defined(USES_PANEL_ANDY)
	mdss_diag_init_andy_gmm_param(gmmvolt_ctx);
#elif defined(USES_PANEL_SINANJU)
	mdss_diag_init_sinanju_gmm_param(gmmvolt_ctx);
#elif defined(USES_PANEL_HAYABUSA)
	mdss_diag_init_hayabusa_gmm_param(gmmvolt_ctx);
#endif  /* USES_PANEL_HAYABUSA */
	return ret;
}
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
#if defined(MDSS_DIAG_PANEL_GMM_VOLTAGE) || defined(MDSS_DIAG_PANEL_FLICKER)
static int mdss_diag_init_adjustable_params(
				struct shdisp_boot_context *shdisp_boot_ctx)
{
	int ret = 0;
#ifdef MDSS_DIAG_PANEL_FLICKER
	mdss_diag_init_flicker_param(&shdisp_boot_ctx->flicker_ctx);
#endif /* MDSS_DIAG_PANEL_FLICKER */
#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE
	mdss_diag_init_gmm_param(&shdisp_boot_ctx->gmmvolt_ctx);
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */
	return ret;
}
#endif /* defined(MDSS_DIAG_PANEL_GMM_VOLTAGE) || defined(MDSS_DIAG_PANEL_FLICKER) */

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_init(struct msm_fb_data_type *mfd)
{
	int ret = 0;
#if defined(MDSS_DIAG_PANEL_GMM_VOLTAGE) || defined(MDSS_DIAG_PANEL_FLICKER)
	struct shdisp_boot_context *shdisp_boot_ctx = NULL;
	sharp_smem_common_type * smem = NULL;
#endif /* defined(MDSS_DIAG_PANEL_GMM_VOLTAGE) || defined(MDSS_DIAG_PANEL_FLICKER) */

	mdss_diag_init_mutex();

#if defined(MDSS_DIAG_PANEL_GMM_VOLTAGE) || defined(MDSS_DIAG_PANEL_FLICKER)
	smem = sh_smem_get_common_address();

	if (smem == NULL) {
		pr_err("%s: failed to "
			"sh_smem_get_common_address()\n", __func__);
		return ret;
	}

	shdisp_boot_ctx = (struct shdisp_boot_context*)smem->shdisp_data_buf;

	mdss_diag_init_adjustable_params(shdisp_boot_ctx);
#endif /* defined(MDSS_DIAG_PANEL_GMM_VOLTAGE) || defined(MDSS_DIAG_PANEL_FLICKER) */
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int mdss_diag_parse_osc(struct device_node *np)
{
	return 0;
}
