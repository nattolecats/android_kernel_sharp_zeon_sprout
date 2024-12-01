/* drivers/video/fbdev/msm/mdss_diag_sinanju.h  (Display Driver)
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

#ifndef MDSS_DIAG_SINANJU_H
#define MDSS_DIAG_SINANJU_H

#ifdef MDSS_DIAG_PANEL_GMM_VOLTAGE

char mdss_diag_sinanju_gmm_data[MDSS_SINANJU_GMM_SIZE * 2];
// this ary is page, addr, payload(sinanju panel is need page 0x00)
struct mdss_diag_panel_pad mdss_diag_sinanju_gmm[] = {
	{0x00, 0xC7, MDSS_SINANJU_GMM_SIZE * 2, &mdss_diag_sinanju_gmm_data[0]},
};

#define SINANJU_VGH_POS	(1)
#define SINANJU_VGL_POS	(11)
char mdss_diag_sinanju_vol_data_d0[]= {
	0xC1,
	0x28, // VGH
	0x8A,
	0x66,
	0x09,
	0x90,
	0x00,
	0xDB,
	0x0F,
	0xC5,
	0x11,
	0x14, // VGL
	0xC3,
	0xFE,
	0x09,
	0x08,
	0xC7,
	0x0F, // 17
};

#define SINANJU_VPL_POS	(0)
#define SINANJU_VNL_POS	(1)
char mdss_diag_sinanju_vol_data_d1[]= {
	0xDD, // VPLVL(VPL)
	0xDD, // VNLVL(VNL)
	0x33,
	0x33,
	0x07,
	0x07,
	0x3B,
	0x22,
	0x3A,
	0x22,
	0x3A,
	0x05,
	0x33,
	0x73,
	0x07,
	0x33,
	0x33,
	0x13,
	0xD3,
	0xDA,
	0x06,
	0x96,
	0x13,
	0x13,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00, // 28
};

struct mdss_diag_panel_pad mdss_diag_sinanju_volt[] = {
	{0x00, 0xD0, 18, &mdss_diag_sinanju_vol_data_d0[0]},
	{0x00, 0xD1, 29, &mdss_diag_sinanju_vol_data_d1[0]},
};

const struct mdss_diag_pad_item mdss_diag_sinanju_gmm_volt_pads[] = {
	{mdss_diag_sinanju_volt, ARRAY_SIZE(mdss_diag_sinanju_volt)},
	{mdss_diag_sinanju_gmm,  ARRAY_SIZE(mdss_diag_sinanju_gmm)},
};
#endif /* MDSS_DIAG_PANEL_GMM_VOLTAGE */

#endif /* MDSS_DIAG_SINANJU_H */