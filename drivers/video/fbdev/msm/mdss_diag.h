/* drivers/video/fbdev/msm/mdss_diag.h  (Display Driver)
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

#ifndef MDSS_DIAG_H
#define MDSS_DIAG_H

#include "mdss_fb.h"
#include "mdss_panel.h"
#include <linux/msm_mdp.h>

extern int mdss_diag_mipi_check(struct msm_fb_data_type *mfd,
			struct mdp_mipi_check_param * mipi_check_param,
			struct mdss_panel_data *pdata);
extern int mdss_diag_mipi_clkchg(struct msm_fb_data_type *mfd,
		struct mdp_mipi_clkchg_param *mipi_clkchg_param);
extern int mdss_diag_set_adjusted(struct mdss_panel_data *pdata);
extern void mdss_lock_blank(void);
extern void mdss_unlock_blank(void);
extern void mdss_lock_displaythread(void);
extern void mdss_unlock_displaythread(void);
extern void mdss_lock_chkstatus(void);
extern void mdss_unlock_chkstatus(void);
extern int mdss_diag_panel_set_gmm(struct mdss_panel_data *pdata,
			struct mdp_gmm_volt_info *gmm_volt_info);
extern int mdss_diag_panel_get_gmm(struct mdss_panel_data *pdata,
			struct mdp_gmm_volt_info *gmm_volt_info);
extern int mdss_diag_get_flicker_param(struct mdss_panel_data *pdata,
			struct mdp_flicker_param *flicker_param);
extern int mdss_diag_set_flicker_param(struct mdss_panel_data *pdata,
			struct mdp_flicker_param flicker_param);
extern int mdss_diag_parse_osc(struct device_node *np);
extern int mdss_diag_init(struct msm_fb_data_type *mfd);
#endif /* MDSS_DIAG_H */
