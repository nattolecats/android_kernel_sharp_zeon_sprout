/* drivers/video/fbdev/msm/mdss_proc.h  (Display Driver)
 *
 * Copyright (C) 2016 SHARP CORPORATION
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

#ifndef MDSS_PROC_H
#define MDSS_PROC_H
#if defined(CONFIG_ANDROID_ENGINEERING)
extern int mdss_proc_init(struct msm_fb_data_type *mfd);
#else
static int mdss_proc_init(struct msm_fb_data_type *mfd)
{
    return 0;
}
#endif /* CONFIG_DEBUG_FS */
#endif /* MDSS_PROC_H */
