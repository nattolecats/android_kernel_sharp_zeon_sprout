/* drivers/video/fbdev/msm/msm_mdss_context.h  (Display Driver)
 *
 * Copyright (C) 2017 SHARP CORPORATION
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

#ifndef MSM_SMEM_H
#define MSM_SMEM_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* Flicker status and min/max */
#define IS_FLICKER_ADJUSTED(param)    (((param & 0xF000) == 0x9000) ? 1 : 0)
#define VCOM_MIN	(0x0000)
#define VCOM_MAX_ANDY		(0x0190)
#define VCOM_MAX_HAYABUSA	(0x0190)
#define VCOM_MAX_SINANJU	(0x03E8)
#define DEFAULT_VCOM    (0x42)

/* Voltage/Gamma Adjusted status */
#define MDSS_GMM_ADJ_STATUS_OK          (0x96)
#define MDSS_GMM_ADJ_STATUS_NOT_SET     (0x00)

enum {
	MDSS_UPPER_UNIT_IS_NOT_CONNECTED = 0,
	MDSS_UPPER_UNIT_IS_CONNECTED
};

// for context.lcd_switch
enum {
	MDSS_PANEL_ROSETTA = 0,
	MDSS_PANEL_HAYABUSA,
	MDSS_PANEL_SINANJU,
};

enum {
	MDSS_PANEL_DISPONCHK_SUCCESS,
	MDSS_PANEL_DISPONCHK_STATERR,
	MDSS_PANEL_DISPONCHK_READERR
};

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
/* Synchronize for msm_mdp.h */
#ifndef MDSS_ROSETTA_GMM_SIZE

#define MDSS_HAYABUSA_GMM_SIZE (60)
#define MDSS_HAYABUSA_ADV_GMM_SIZE (30)
#define MDSS_ROSETTA_GMM_SIZE (62)
#define MDSS_ROSETTA_ADV_GMM_SIZE (30)
#define MDSS_SINANJU_GMM_SIZE (38)

struct hayabusa_gmm_volt {
	unsigned short gmmR[MDSS_HAYABUSA_GMM_SIZE];
	unsigned short gmmG[MDSS_HAYABUSA_GMM_SIZE];
	unsigned short gmmB[MDSS_HAYABUSA_GMM_SIZE];
	unsigned char vgh;
	unsigned char vgl;
	unsigned char gvddp;
	unsigned char gvddn;
	unsigned char gvddp2;
	unsigned char vgho;
	unsigned char vglo;
	unsigned char adv_gmm[MDSS_HAYABUSA_ADV_GMM_SIZE];
};

struct rosetta_gmm_volt {
	unsigned short gmmR[MDSS_ROSETTA_GMM_SIZE];
	unsigned short gmmG[MDSS_ROSETTA_GMM_SIZE];
	unsigned short gmmB[MDSS_ROSETTA_GMM_SIZE];
	unsigned char vgh;
	unsigned char vgl;
	unsigned char gvddp;
	unsigned char gvddn;
	unsigned char gvddp2;
	unsigned char vgho;
	unsigned char vglo;
	unsigned char adv_gmm[MDSS_ROSETTA_ADV_GMM_SIZE];
};

struct sinanju_gmm_volt {
	unsigned short gmm[MDSS_SINANJU_GMM_SIZE];
	unsigned char vgh;
	unsigned char vgl;
	unsigned char vpl;
	unsigned char vnl;
};

union mdp_gmm_volt {
	struct rosetta_gmm_volt rosetta;
	struct hayabusa_gmm_volt hayabusa;
	struct sinanju_gmm_volt  sinanju;
}__attribute__((aligned(8)));

struct mdp_panel_otp_info {
    unsigned char status;
    signed char a;
    signed char b;
}__attribute__((aligned(8)));

#endif /* MDSS_ROSETTA_GMM_SIZE */

/* msm_mdss contexts */
struct mdss_vcom {
	unsigned short vcom;
	unsigned short vcom_low;
};

struct mdss_flicker_ctx {
	struct mdss_vcom vcom;
	unsigned short   nvram;
}__attribute__((aligned(8)));

struct mdss_gmmvolt_ctx {
	unsigned char       status;
	union mdp_gmm_volt  gmm_volt __attribute__((aligned(8)));
}__attribute__((aligned(8)));

struct shdisp_boot_context {
	unsigned char            lcd_switch;
	unsigned char            panel_connected;
	unsigned char            disp_on_status;
	unsigned char            panel_trim;
	struct mdss_gmmvolt_ctx  gmmvolt_ctx;
	struct mdss_flicker_ctx  flicker_ctx;
};

/* flicker structures */
struct mdss_hayabusa_vcom {
	char vcom1_l;
	char vcom2_l;
	char vcom12_h;
	char lpvcom1;
	char lpvcom2;
	char vcomoff_l;
	char vcomoff_h;
};

struct mdss_rosetta_vcom {
	char vcom1_l;
	char vcom2_l;
	char vcom12_h;
	char lpvcom1;
	char lpvcom2;
	char vcomoff_l;
	char vcomoff_h;
};

struct mdss_sinanju_vcom {
	char vcom_fw_h;
	char vcom_fw_l;
	char vcomdcoff_h;
	char vcomdcoff_l;
};

#endif /* MSM_SMEM_H */
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
