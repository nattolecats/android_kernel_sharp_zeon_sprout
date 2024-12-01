/* drivers/video/fbdev/msm/mdss_proc.c  (Display Driver)
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
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/fb.h>
#include <linux/leds.h>
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#if 0
#include "mdss_diag.h"
#endif
#include "msm_mdss_context.h"

/* ------------------------------------------------------------------------- */
/* DEFINE                                                                    */
/* ------------------------------------------------------------------------- */

#define MDSS_FPS_LED_PANEL_SUPPORT

#define MDSS_DTYPE_DCS_WRITE     (0x05)
#define MDSS_DTYPE_DCS_WRITE1    (0x15)
#define MDSS_DTYPE_DCS_READ      (0x06)
#define MDSS_DTYPE_DCS_LWRITE    (0x39)

#define MDSS_DTYPE_GEN_WRITE     (0x03)
#define MDSS_DTYPE_GEN_WRITE1    (0x13)
#define MDSS_DTYPE_GEN_WRITE2    (0x23)
#define MDSS_DTYPE_GEN_LWRITE    (0x29)
#define MDSS_DTYPE_GEN_READ      (0x04)
#define MDSS_DTYPE_GEN_READ1     (0x14)
#define MDSS_DTYPE_GEN_READ2     (0x24)

#define MDSS_LCDDR_BUF_MAX       (107)
#define PROC_BUF_LENGTH            (4096)
#define PROC_BLOCK_SIZE	(PAGE_SIZE - 1024)

#define MDSS_PROC_DSI_COLLECT_MAX      (512)
#define MDSS_PROC_DSI_PAYLOAD_BUF_LEN  (4 * MDSS_PROC_DSI_COLLECT_MAX)


#define MDSS_DSI_CTRL_DSI_EN             BIT(0)
#define MDSS_DSI_CTRL_VIDEO_MODE_EN      BIT(1)
#define MDSS_DSI_CTRL_CMD_MODE_EN        BIT(2)

enum {
	MDSS_DEBUG_MFR_SET       = 10,
	MDSS_DEBUG_FPS_MFR       = 40,
	MDSS_DEBUG_FLICKER_SET   = 80,
	MDSS_DEBUG_FLICKER_GET   = 81,
	MDSS_DEBUG_DSI_DCS_WRITE = 90,
	MDSS_DEBUG_DSI_DCS_READ  = 91,
	MDSS_DEBUG_DSI_GEN_WRITE = 92,
	MDSS_DEBUG_DSI_GEN_READ  = 93,
};

#define MDSS_LOG_ENABLE

struct mdss_dsi_cmd_desc {
	char dtype;
	short dlen;
	char *payload;
	int  wait;
	unsigned char mode;
};

struct mdss_dsi_cmd_req {
	char dtype;
	unsigned char addr;
	unsigned char size;
	unsigned char mode;
	unsigned char *data;
};

struct mdss_procfs {
	int id;
	int par[4];
};

#ifdef MDSS_FPS_LED_PANEL_SUPPORT

struct shled_tri_led {
    unsigned char red;
    unsigned char green;
    unsigned char blue;
};

DEFINE_LED_TRIGGER(led_fps_r_trigger);
DEFINE_LED_TRIGGER(led_fps_b_trigger);
DEFINE_LED_TRIGGER(led_fps_g_trigger);

#define ADDR_PAGE               (0xFF)
#define PAGE_FPS_HAYABUSA_REG   (0x26)
#define ADDR_FPS_HAYABUSA_REG   (0x8A)
#define PAGE_FPS_ROSETTA_REG    (0x26)
#define ADDR_FPS_ROSETTA_REG    (0x6D)
#define FPS_LED_INTERVAL        (135000)

enum {
	FPS_LED_STATE_NONE,
	FPS_LED_STATE_1HZ,
	FPS_LED_STATE_15HZ,
	FPS_LED_STATE_30HZ,
	FPS_LED_STATE_45HZ,
	FPS_LED_STATE_60HZ,
	FPS_LED_STATE_120HZ,
	MAX_FPS_LED_STATE
};

static const char *mdss_fps_led_state_str[MAX_FPS_LED_STATE] = {
	"NONE",
	"1HZ",
	"15HZ",
	"30HZ",
	"45HZ",
	"60HZ",
	"120HZ",
};

static const char mdss_fps_led_color[MAX_FPS_LED_STATE][3] = {
	[FPS_LED_STATE_NONE]  = {0, 0, 0},
	[FPS_LED_STATE_1HZ]   = {0, 0, 255},
	[FPS_LED_STATE_15HZ]  = {0, 255, 0},
	[FPS_LED_STATE_30HZ]  = {255, 255, 0},
	[FPS_LED_STATE_45HZ]  = {0, 255, 255},
	[FPS_LED_STATE_60HZ]  = {255, 0, 0},
	[FPS_LED_STATE_120HZ] = {255, 255, 255},
};

static struct {
	bool enable;
	bool suspend;
	bool panel_on;
	int state;
	int interval;
	int max_frame_rate;
	struct workqueue_struct *workq;
	struct delayed_work work;
} mdss_fps_led_ctx = {0};

static char bank[] = {ADDR_PAGE, PAGE_FPS_ROSETTA_REG};
static struct dsi_cmd_desc bank_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(bank)},
	bank
};
static unsigned char read_buf[2] = {0x00, 0x00};
static char read_cmd_param[] = {ADDR_FPS_ROSETTA_REG};
static struct dsi_cmd_desc read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 1, sizeof(read_cmd)},
	read_cmd_param
};

struct notifier_block fb_notif;
#endif /* MDSS_FPS_LED_PANEL_SUPPORT */

#define MDSS_DEBUG_CONSOLE(fmt, args...) \
		do { \
			int buflen = 0; \
			int remain = PROC_BUF_LENGTH - proc_buf_pos - 1; \
			if (remain > 0) { \
				buflen = snprintf(&proc_buf[proc_buf_pos], remain, fmt, ## args); \
				proc_buf_pos += (buflen > 0) ? buflen : 0; \
			} \
		} while (0)

#define MDSS_PROC_PRINT_CMD_DESC(cmds, size)												\
		do {																				\
			int i;																			\
			char buf[64];																	\
			struct mdss_dsi_cmd_desc *cmd;												\
			for (i = 0; i < size; i++) {													\
				cmd = &cmds[i];																\
				hex_dump_to_buffer(cmd->payload, cmd->dlen, 16, 1, buf, sizeof(buf), 0);	\
				pr_debug("%s: dtype=%02x dlen=%d mode=%02x wait=%d data=%s\n", __func__,	\
						(unsigned char)cmd->dtype, cmd->dlen, cmd->mode, cmd->wait, buf);	\
			}																				\
		} while (0)

#define MDSS_PROC_PRINT_RX_DATA(data, size)												\
		do {																				\
			char buf[64];																	\
			hex_dump_to_buffer(data, size, 16, 1, buf, sizeof(buf), 0);						\
			pr_debug("%s: rx_data=%s\n", __func__, buf);									\
		} while (0)

static unsigned char proc_buf[PROC_BUF_LENGTH] = {0};
static unsigned int  proc_buf_pos = 0;
static struct mdss_dsi_ctrl_pdata *mdss_dsi_ctrl = NULL;
static struct mdss_panel_data *mdss_panel_ctrl = NULL;
static void mdss_proc_clk_ctrl(bool onoff);
static int mdss_proc_callback_data = 0;
static int mdss_proc_collect_cmd_cnt = 0;
static struct dsi_cmd_desc mdss_proc_collect_cmds[MDSS_PROC_DSI_COLLECT_MAX];
static char mdss_proc_collect_payloads[MDSS_PROC_DSI_PAYLOAD_BUF_LEN];
static int mdss_proc_used_payloads = 0;
static struct semaphore mdss_proc_host_dsi_cmd_sem;


static int mdss_proc_is_cmdmode_eng_on(struct mdss_dsi_ctrl_pdata *ctrl);
static void mdss_proc_cmdmode_eng_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, int enable);
static int mdss_proc_mdp_cmd_clk_ctrl(bool onoff, enum mdss_dsi_clk_type clk_type);
static int mdss_proc_write(struct file *file, const char *buffer, unsigned long count, void *data);
static int mdss_proc_read(char *page, char **start, off_t offset, int count, int *eof, void *data);
static int mdss_proc_file_open(struct inode *inode, struct file *file);
static ssize_t mdss_proc_file_read(struct file *file, char __user *buf, size_t nbytes, loff_t *ppos);
static ssize_t mdss_proc_file_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos);
static void mdss_proc_debugfs_create(struct msm_fb_data_type *mfd);
static int mdss_panel_mipi_dsi_cmds_tx(int commit, struct mdss_dsi_cmd_desc *cmds, int cnt);
static int mdss_panel_mipi_dsi_cmds_rx(unsigned char *rbuf, struct mdss_dsi_cmd_desc *cmds, unsigned char size);
static void mdss_panel_dsi_wlog(struct mdss_dsi_cmd_desc *cmds, int cmdslen);
static void mdss_panel_dsi_rlog(char addr, char *rbuf, int len);
static int mdss_panel_dsi_write_reg(struct mdss_dsi_cmd_req *req);
static int mdss_panel_dsi_read_reg(struct mdss_dsi_cmd_req *req);
#ifdef MDSS_FPS_LED_PANEL_SUPPORT
static void mdss_fps_led_start(struct mdss_mdp_ctl *ctl);
static void mdss_fps_led_stop(void);
static void mdss_fps_led_work(struct work_struct *work);
static void mdss_fps_led_resume(void);
static void mdss_fps_led_suspend(void);
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
static int mdss_fps_set_led_video(struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_fps_set_led_cmd(struct mdss_dsi_ctrl_pdata *ctrl);
#endif /* MDSS_FPS_LED_PANEL_SUPPORT */
#ifdef MDSS_MFR_PANEL_SUPPORT
static int mdss_proc_mfr(struct mdss_dsi_ctrl_pdata *ctrl, int value);
static int mdss_proc_mfr_rosetta(struct mdss_dsi_ctrl_pdata *ctrl, int value);
static int mdss_proc_mfr_hayabusa(struct mdss_dsi_ctrl_pdata *ctrl, int value);
#endif

extern void mdss_mdp_video_transfer_ctrl(struct mdss_mdp_ctl *ctl, int onoff);

static void mdss_proc_lock_host_dsi_cmd(void)
{
	down(&mdss_proc_host_dsi_cmd_sem);
}

static void mdss_proc_unlock_host_dsi_cmd(void)
{
	up(&mdss_proc_host_dsi_cmd_sem);
}

static int mdss_proc_set_dsi_ctrl(struct msm_fb_data_type *mfd)
{
	int ret = -EIO;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("no panel connected\n");
		goto exit;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		goto exit;
	}

	if (ctrl_pdata->ndx != DSI_CTRL_0) {
		pr_err("%s: Invalid ndx\n", __func__);
		goto exit;
	}
	mdss_panel_ctrl = pdata;
	mdss_dsi_ctrl = ctrl_pdata;
	ret = 0;
exit:
	return ret;
}

static void mdss_proc_dsi_to_mdss_dsi(const struct mdss_dsi_cmd_desc *shdisp_cmds,  struct dsi_cmd_desc *mdss_cmds,
								int size, int isAck)
{
	int cnt;
	for (cnt = 0; cnt != size; cnt++) {
		if (mdss_proc_used_payloads + shdisp_cmds->dlen < MDSS_PROC_DSI_PAYLOAD_BUF_LEN) {
			mdss_cmds->dchdr.dtype  = shdisp_cmds->dtype;
			mdss_cmds->dchdr.last = shdisp_cmds->wait ? 1 : 0;
			mdss_cmds->dchdr.vc	    = 0;
			mdss_cmds->dchdr.ack    = isAck ? 1 : 0;
			mdss_cmds->dchdr.wait   = shdisp_cmds->wait ? ((shdisp_cmds->wait+999)/1000) : 0; /* mdss_dsi(ms) <- mdss_dsi(usec) */
			mdss_cmds->dchdr.dlen   = shdisp_cmds->dlen;
			mdss_cmds->payload      = mdss_proc_collect_payloads + mdss_proc_used_payloads;
			memcpy(mdss_cmds->payload, shdisp_cmds->payload, shdisp_cmds->dlen);
			mdss_proc_used_payloads += shdisp_cmds->dlen;
		} else {
			pr_err("LCDERR: buffer size over %s: shdisp_cmds->dlen=%d, mdss_proc_used_payloads = %d\n",
											__func__, shdisp_cmds->dlen, mdss_proc_used_payloads );
		}
		mdss_cmds++;
		shdisp_cmds++;
	}
}

static void mdss_proc_collect_cmd(struct mdss_dsi_cmd_desc *mdss_cmds, int size)
{
	if (mdss_proc_collect_cmd_cnt + size >= MDSS_PROC_DSI_COLLECT_MAX) {
		pr_err("LCDERR: buffer size over %s: size=%d, mdss_proc_collect_cmd_cnt = %d\n",
										__func__, size, mdss_proc_collect_cmd_cnt );
		return;
	}
	mdss_proc_dsi_to_mdss_dsi(mdss_cmds, &mdss_proc_collect_cmds[mdss_proc_collect_cmd_cnt], size, 0);
	mdss_proc_collect_cmd_cnt += size;
	pr_debug("%s: size=%d, mdss_proc_collect_cmd_cnt = %d\n", __func__, size, mdss_proc_collect_cmd_cnt );
}

static int mdss_proc_kick_collect_cmd(struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_cmd_desc *cmds)
{
	int ret = 0;
	int cmdengon;
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: begin cnt=%d\n", __func__, mdss_proc_collect_cmd_cnt);
	if (!mdss_proc_collect_cmd_cnt) {
		return 0;
	}

	mdss_proc_collect_cmds[mdss_proc_collect_cmd_cnt-1].dchdr.last   = 1;
	pr_debug("%s: kick_count = %d\n", __func__, mdss_proc_collect_cmd_cnt );

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.flags = CMD_REQ_COMMIT;
	if(cmds->mode) {
		pr_debug("%s: cmds->mode=0x%x\n", __func__, cmds->mode);
		cmdreq.flags |= cmds->mode;
	}
	cmdreq.cmds = mdss_proc_collect_cmds;
	cmdreq.cmds_cnt = mdss_proc_collect_cmd_cnt;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	cmdengon = mdss_proc_is_cmdmode_eng_on(ctrl);
	if(!cmdengon){
		mdss_proc_cmdmode_eng_ctrl(ctrl, 1);
	}
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if (!cmdengon) {
		mdss_proc_cmdmode_eng_ctrl(ctrl, 0);
	}

	mdss_proc_collect_cmd_cnt = 0;
	mdss_proc_used_payloads = 0;

	pr_debug("%s: end. ret=%d\n", __func__, ret);
	if (ret > 0) {
		return 0;
	} else {
		return -EIO;
	}
}

static int mdss_proc_is_cmdmode_eng_on(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int dsi_ctrl;
	int cmdmode_on = MDSS_DSI_CTRL_DSI_EN | MDSS_DSI_CTRL_CMD_MODE_EN;
	int ret = 0;

	mdss_proc_clk_ctrl(true);

	dsi_ctrl = MIPI_INP((ctrl->ctrl_base) + 0x0004);

	if ((dsi_ctrl&cmdmode_on) == cmdmode_on) {
		ret = 1;
	} else {
		ret = 0;
	}
	mdss_proc_clk_ctrl(false);

	return ret;
}

static void mdss_proc_cmdmode_eng_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, int enable)
{
	int mode;

	mode = (enable ? DSI_CMD_MODE : \
							(ctrl->panel_mode == DSI_CMD_MODE ? \
								DSI_CMD_MODE : DSI_VIDEO_MODE \
							) \
				);

	pr_debug("%s: request=%d\n", __func__, enable);

	mdss_dsi_op_mode_config(mode, &ctrl->panel_data);

	return;
}

static void mdss_proc_host_dsi_init_cbdata(void)
{
	mdss_proc_callback_data = 0xffff;
}

static int mdss_proc_host_dsi_get_cbdata(void)
{
	return mdss_proc_callback_data;
}

static void mdss_proc_host_dsi_cb(int data)
{
	mdss_proc_callback_data = data;
}

static int mdss_proc_host_dsi_tx(int commit, struct mdss_dsi_cmd_desc *mdss_cmds, int size)
{
	int ret = 0;

	pr_debug("%s: in commit=%d\n", __func__, commit);
	MDSS_PROC_PRINT_CMD_DESC(mdss_cmds, size);

	mdss_proc_lock_host_dsi_cmd();
	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p\n", __func__, mdss_dsi_ctrl);
		mdss_proc_unlock_host_dsi_cmd();
		return -EIO;
	}

	mdss_proc_collect_cmd(mdss_cmds, size);

	if (commit) {
		mdss_proc_clk_ctrl(true);

		ret = mdss_proc_kick_collect_cmd(mdss_dsi_ctrl, mdss_cmds);

		mdss_proc_clk_ctrl(false);
	}

	mdss_proc_unlock_host_dsi_cmd();

	pr_debug("%s: out ret=%d\n", __func__, ret);
	return ret;
}

static int mdss_proc_host_dsi_rx(struct mdss_dsi_cmd_desc *cmds, unsigned char *rx_data, int rx_size)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc mdss_cmds;
	char payload[2];
	int cmdengon;

	pr_debug("%s: in rx_size=%d\n", __func__, rx_size);
	MDSS_PROC_PRINT_CMD_DESC(cmds, 1);

	mdss_proc_lock_host_dsi_cmd();

	if ( !mdss_dsi_ctrl ) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p\n", __func__, mdss_dsi_ctrl);
		mdss_proc_unlock_host_dsi_cmd();
		return -EIO;
	}

	mdss_proc_clk_ctrl(true);

	ret = mdss_proc_kick_collect_cmd(mdss_dsi_ctrl, cmds);
	if (ret != 0) {
		pr_err("LCDERR: %s mdss_proc_kick_collect_cmd(cmds) ret=%d\n", __func__, ret);
		mdss_proc_clk_ctrl(false);
		mdss_proc_unlock_host_dsi_cmd();
		return -EIO;
	}

	memset(&cmdreq, 0, sizeof(cmdreq) );
	memset(&mdss_cmds, 0, sizeof(mdss_cmds) );
	mdss_proc_dsi_to_mdss_dsi(cmds, &mdss_cmds, 1, 1);
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	if(cmds->mode) {
		cmdreq.flags |= cmds->mode;
	}
	cmdreq.cmds = &mdss_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.cb = mdss_proc_host_dsi_cb;
	cmdreq.rbuf = rx_data;
	cmdreq.rlen = rx_size;

	if (mdss_cmds.dchdr.dlen>1) {
		payload[0] = mdss_cmds.payload[0];
		payload[1] = mdss_cmds.payload[1];
	} else {
		payload[0] = mdss_cmds.payload[0];
		payload[1] = 0;
	}

	mdss_cmds.payload = payload;
	mdss_proc_used_payloads = 0;

	cmdengon = mdss_proc_is_cmdmode_eng_on(mdss_dsi_ctrl);
	if (!cmdengon) {
		mdss_proc_cmdmode_eng_ctrl(mdss_dsi_ctrl, 1);
	}

	mdss_proc_host_dsi_init_cbdata();
	mdss_dsi_cmdlist_put(mdss_dsi_ctrl, &cmdreq);

	if (cmdreq.rlen != mdss_proc_host_dsi_get_cbdata()) {
		pr_err("LCDERR: %s callback_data=%d\n", __func__, mdss_proc_host_dsi_get_cbdata());
		if (!cmdengon) {
			mdss_proc_cmdmode_eng_ctrl(mdss_dsi_ctrl, 0);
		}
		mdss_proc_clk_ctrl(false);
		mdss_proc_unlock_host_dsi_cmd();
		return -EIO;
	}

	if (!cmdengon) {
		mdss_proc_cmdmode_eng_ctrl(mdss_dsi_ctrl, 0);
	}

	mdss_proc_clk_ctrl(false);
	mdss_proc_unlock_host_dsi_cmd();

	MDSS_PROC_PRINT_RX_DATA(rx_data, rx_size);
	pr_debug("%s: out\n", __func__);
	return 0;
}

static void mdss_proc_clk_ctrl(bool onoff)
{
	if ( !mdss_dsi_ctrl ){
		return;
	}
	if ( mdss_dsi_ctrl->panel_mode == DSI_CMD_MODE ) {
		mdss_proc_mdp_cmd_clk_ctrl(onoff, MDSS_DSI_ALL_CLKS);
	}
}

static int mdss_proc_mdp_cmd_clk_ctrl(bool onoff, enum mdss_dsi_clk_type clk_type)
{
	pr_debug("LCDDBG:[%s] enter - (onoff=%d)\n", __func__, onoff);

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR:[%s] invalid DSI control.\n", __func__);
		return -EINVAL;
	}

	if (onoff) {
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

		mdss_dsi_clk_ctrl(mdss_dsi_ctrl, mdss_dsi_ctrl->dsi_clk_handle,
			  clk_type, MDSS_DSI_CLK_ON);
	} else {
		mdss_dsi_clk_ctrl(mdss_dsi_ctrl, mdss_dsi_ctrl->dsi_clk_handle,
			clk_type, MDSS_DSI_CLK_OFF);

		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	}

	pr_debug("LCDDBG:[%s] leave - (ret=0)\n", __func__);
	return 0;
}

int mdss_proc_init(struct msm_fb_data_type *mfd)
{
	int ret = 0;
	ret = mdss_proc_set_dsi_ctrl(mfd);
	if(ret < 0) {
		pr_err("LCDERR:[%s] mdss_proc_set_dsi_ctrl\n", __func__);
		return ret;
	}

	mdss_proc_debugfs_create(mfd);

	sema_init(&mdss_proc_host_dsi_cmd_sem, 1);

#ifdef MDSS_FPS_LED_PANEL_SUPPORT
	mdss_fps_led_ctx.workq = create_singlethread_workqueue("mdss_fps_led_workq");
	INIT_DELAYED_WORK(&mdss_fps_led_ctx.work, mdss_fps_led_work);
#endif /* MDSS_FPS_LED_PANEL_SUPPORT */

	pr_info("[%s] success.\n", __func__);
	return 0;
}

static const struct file_operations mdss_proc_fops = {
	.owner			= THIS_MODULE,
	.open			= mdss_proc_file_open,
	.write			= mdss_proc_file_write,
	.read			= mdss_proc_file_read,
	.release		= single_release,
};

static void mdss_proc_debugfs_create(struct msm_fb_data_type *mfd)
{
	struct dentry * root;
	root = debugfs_create_dir("mdss", 0);

	if (!root) {
		pr_err("LCDERR:[%s] failed to create dbgfs root dir\n", __func__);
	}

	if (!debugfs_create_file("proc",
		S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH,
		root, mfd, &mdss_proc_fops)) {
		pr_err("LCDERR:[%s] failed to create dbgfs(mdss_proc)\n", __func__);
	}
}

static int mdss_proc_file_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}
static int mdss_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
#define MDSS_LEN_ID                   (2)
#define MDSS_LEN_PARAM                (4)
#define MDSS_PARAM_MAX                (4)

	unsigned long len = count;
	struct mdss_procfs mdss_pfs;
	char buf[MDSS_LEN_PARAM + 1];
	char kbuf[MDSS_LEN_ID + MDSS_PARAM_MAX * MDSS_LEN_PARAM];
	int i;
	int ret = 0;

	char *kbufex;
	unsigned char *param = NULL;
	int paramlen = 0;
	int needalloc = 0;
	bool foundinvalidparams = false;

	struct seq_file *s = file->private_data;
	struct msm_fb_data_type *mfd = s->private;

	struct mdss_mdp_ctl *ctl = mfd_to_ctl(mfd);

	proc_buf[0] = 0;
	proc_buf_pos = 0;
	MDSS_DEBUG_CONSOLE("NG");

	if (mfd->panel_power_state == MDSS_PANEL_POWER_OFF) {
		pr_err("LCDERR:[%s] panel off err\n", __func__);
		return count;
	}
	len--;
	/* Check length */
	if (len < MDSS_LEN_ID) {
		return count;
	}
	if (len > (MDSS_LEN_ID + MDSS_PARAM_MAX * MDSS_LEN_PARAM)) {
		len = MDSS_LEN_ID + MDSS_PARAM_MAX * MDSS_LEN_PARAM;
		needalloc = 1;
	}

	if (copy_from_user(kbuf, buffer, len)) {
		return -EFAULT;
	}
	/* Get FunctionID */
	memcpy(buf, kbuf, MDSS_LEN_ID);
	buf[MDSS_LEN_ID] = '\0';
	mdss_pfs.id = simple_strtol(buf, NULL, 10);
	mdss_pfs.par[0] = 0;
	mdss_pfs.par[1] = 0;
	mdss_pfs.par[2] = 0;
	mdss_pfs.par[3] = 0;
	if (!isdigit(*buf) || !isdigit(*(buf+1))) {
		pr_err("LCDERR:[%s] invalid func id = %s\n", __func__, buf);
		return count;
	}

	/* check Paramters */
	for (i = MDSS_LEN_ID; i < len; i++) {
		if (!isxdigit(*(kbuf+i))) {
			foundinvalidparams = true;
			break;
		}
	}

	if (foundinvalidparams) {
		pr_err("LCDERR:[%s] invalid param pos[%d] (%c)\n",
			__func__,i,  (*(kbuf+i)));
		return count;
	}

	/* Get Parameters */
	for (i = 0; (i + 1) * MDSS_LEN_PARAM <= (len - MDSS_LEN_ID); i++) {
		memcpy(buf, &(kbuf[MDSS_LEN_ID + i * MDSS_LEN_PARAM]), MDSS_LEN_PARAM);
		buf[MDSS_LEN_PARAM] = '\0';
		switch (mdss_pfs.id) {
		case MDSS_DEBUG_MFR_SET:
			mdss_pfs.par[i] = simple_strtol(buf, NULL, 10);
			break;
		default:
			mdss_pfs.par[i] = simple_strtol(buf, NULL, 16);
			break;
		}
	}

	printk("[MDSS] mdss_proc_write(%d, 0x%04x, 0x%04x, 0x%04x, 0x%04x)\n", mdss_pfs.id,
		mdss_pfs.par[0], mdss_pfs.par[1],
		mdss_pfs.par[2], mdss_pfs.par[3]);

	switch (mdss_pfs.id) {
	case MDSS_DEBUG_DSI_DCS_WRITE:
	case MDSS_DEBUG_DSI_GEN_WRITE:
		if (len < 8) {
			pr_err("(%d): DSI_WRITE param error\n", mdss_pfs.id);
			goto out;
		}
		needalloc = 1;
		break;
	}

	if (needalloc) {
		len = count - (MDSS_LEN_ID + 1);
		if (len > (1024 * MDSS_PARAM_MAX) - (MDSS_LEN_ID + 1)) {
		   len = (1024 * MDSS_PARAM_MAX) - (MDSS_LEN_ID + 1);
		}
		kbufex = kmalloc(len, GFP_KERNEL);
		if (!kbufex) {
			return -EFAULT;
		}
		buffer += MDSS_LEN_ID;
		if (copy_from_user(kbufex, buffer, len)) {
			kfree(kbufex);
			return -EFAULT;
		}
		paramlen = len / (MDSS_LEN_PARAM / 2);
		param = kmalloc(paramlen, GFP_KERNEL);
		if (!param) {
			kfree(kbufex);
			return -EFAULT;
		}
		/* Get Parameters */
		for (i = 0; i < paramlen; i++) {
			memcpy(buf, &(kbufex[i * (MDSS_LEN_PARAM / 2)]), (MDSS_LEN_PARAM / 2));
			buf[(MDSS_LEN_PARAM / 2)] = '\0';
			param[i] = simple_strtol(buf, NULL, 16);
		}
		kfree(kbufex);
	}

	switch (mdss_pfs.id) {
#ifdef MDSS_MFR_PANEL_SUPPORT
	case MDSS_DEBUG_MFR_SET:
		ret = mdss_proc_mfr(mdss_dsi_ctrl, mdss_pfs.par[0]);
		proc_buf[0] = 0;
		proc_buf_pos = 0;
		MDSS_DEBUG_CONSOLE(ret < 0 ? "NG" : "OK");
		break;
#endif
#ifdef MDSS_FPS_LED_PANEL_SUPPORT
	case MDSS_DEBUG_FPS_MFR:
		if (mdss_pfs.par[0]) {
			mdss_fps_led_start(ctl);
		} else {
			mdss_fps_led_stop();
		}
		break;
#endif /* MDSS_FPS_LED_PANEL_SUPPORT */
#if 0

	case MDSS_DEBUG_FLICKER_SET:
	{
		struct mdp_flicker_param flicker_req;
		flicker_req.request = mdss_pfs.par[0];
		flicker_req.vcom = mdss_pfs.par[1];
		ret = mdss_diag_set_flicker_param(mdss_panel_ctrl, flicker_req);
		proc_buf[0] = 0;
		proc_buf_pos = 0;
		MDSS_DEBUG_CONSOLE(ret ? "NG" : "OK");
		break;
	}
	case MDSS_DEBUG_FLICKER_GET:
	{
		struct mdp_flicker_param flicker_req;
		flicker_req.request = mdss_pfs.par[0];
		flicker_req.vcom = 0;
		ret = mdss_diag_get_flicker_param(mdss_panel_ctrl, &flicker_req);
		proc_buf[0] = 0;
		proc_buf_pos = 0;
		MDSS_DEBUG_CONSOLE(ret ? "NG" : "OK");
		MDSS_DEBUG_CONSOLE(",%02X", flicker_req.vcom);
		break;
	}
#endif
	case MDSS_DEBUG_DSI_GEN_WRITE:
	case MDSS_DEBUG_DSI_DCS_WRITE:
	{
		struct mdss_dsi_cmd_req dsi_req;
		unsigned char buf[MDSS_LCDDR_BUF_MAX];

		memset(&dsi_req, 0x00, sizeof(dsi_req));
		memset(buf, 0x00, sizeof(buf));
		dsi_req.data = buf;

		if (mdss_pfs.id == MDSS_DEBUG_DSI_GEN_WRITE) {
			dsi_req.dtype = MDSS_DTYPE_GEN_WRITE;
		} else {
			dsi_req.dtype = MDSS_DTYPE_DCS_WRITE;
		}
		dsi_req.size = param[0];
		dsi_req.addr = param[1];
		dsi_req.mode = param[2];

		if (dsi_req.size > MDSS_LCDDR_BUF_MAX) {
			pr_err("[%s]size over. max=%d\n", __func__, MDSS_LCDDR_BUF_MAX);
			goto out;
		}
		pr_debug(" Size    : %2d\n", dsi_req.size);
		pr_debug(" Address : %02Xh\n", dsi_req.addr);
		pr_debug(" Mode    : %02Xh\n", dsi_req.mode);
		for (i = 0; i < dsi_req.size; i++) {
			dsi_req.data[i] = param[i + 3];
			if ((i % 8) == 0) {
				printk("[%s]  WData    : ", __func__);
			}
			printk("%02X ", dsi_req.data[i]);
			if ((i % 8) == 7) {
				printk("\n");
			}
		}
		printk("\n");

		if (mdss_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
			mdss_mdp_video_transfer_ctrl(ctl, false);
		}
		ret = mdss_panel_dsi_write_reg(&dsi_req);
		if (mdss_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
			mdss_mdp_video_transfer_ctrl(ctl, true);
		}
		proc_buf[0] = 0;
		proc_buf_pos = 0;
		MDSS_DEBUG_CONSOLE(ret ? "NG" : "OK");
		break;
	}

	case MDSS_DEBUG_DSI_GEN_READ:
	case MDSS_DEBUG_DSI_DCS_READ:
	{
		struct mdss_dsi_cmd_req dsi_req;
		unsigned char buf[MDSS_LCDDR_BUF_MAX];
		struct mdss_panel_info *pinfo = NULL;

		memset(&dsi_req, 0x00, sizeof(dsi_req));
		memset(buf, 0x00, sizeof(buf));
		dsi_req.data = buf;

		if (mdss_pfs.id == MDSS_DEBUG_DSI_GEN_READ) {
			dsi_req.dtype = MDSS_DTYPE_GEN_READ;
		} else {
			dsi_req.dtype = MDSS_DTYPE_DCS_READ;
		}
		dsi_req.size    = ((mdss_pfs.par[0] >> 8) & 0x00FF);
		dsi_req.addr    = ( mdss_pfs.par[0]       & 0x00FF);
		dsi_req.mode    = ((mdss_pfs.par[1] >> 8) & 0x00FF);

		pinfo = &(mdss_panel_ctrl->panel_info);

		if (mdss_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
			mdss_mdp_video_transfer_ctrl(ctl, false);
		}
		pr_debug(" Size    : %2d\n", dsi_req.size);
		pr_debug(" Address : %02Xh\n", dsi_req.addr);
		pr_debug(" Mode    : %02Xh\n", dsi_req.mode);
		ret = mdss_panel_dsi_read_reg(&dsi_req);
		if (mdss_dsi_ctrl->panel_mode == DSI_VIDEO_MODE) {
			mdss_mdp_video_transfer_ctrl(ctl, true);
		}
		proc_buf[0] = 0;
		proc_buf_pos = 0;
		MDSS_DEBUG_CONSOLE(ret ? "NG" : "OK");
		if (!ret) {
			for (i=0; i != dsi_req.size; ++i) {
				MDSS_DEBUG_CONSOLE(",%02X", dsi_req.data[i]);
			}
		}

		break;
	}

	default:
		break;
	}

	printk("[MDSS] result : %d.\n", ret);

	if (needalloc) {
		kfree(param);
	}

out:

	return count;
}

static int mdss_proc_read(char *page, char **start, off_t offset, int count, int *eof, void *data)
{
	int len = 0;

	len += snprintf(page, count, "%s", proc_buf);
	proc_buf[0] = 0;
	proc_buf_pos = 0;

	return len;
}

static ssize_t mdss_proc_file_read(struct file *file, char __user *buf, size_t nbytes, loff_t *ppos)
{
	char    *page;
	ssize_t retval=0;
	int eof=0;
	ssize_t n, count;
	char    *start;
	unsigned long long pos;

	/*
	 * Gaah, please just use "seq_file" instead. The legacy /proc
	 * interfaces cut loff_t down to off_t for reads, and ignore
	 * the offset entirely for writes..
	 */
	pos = *ppos;
	if (pos > MAX_NON_LFS) {
		return 0;
	}
	if (nbytes > (MAX_NON_LFS - pos)) {
		nbytes = MAX_NON_LFS - pos;
	}

	if (!(page = (char*) __get_free_page(GFP_TEMPORARY))) {
		return -ENOMEM;
	}

	while ((nbytes > 0) && !eof) {
		count = min_t(size_t, PROC_BLOCK_SIZE, nbytes);

		start = NULL;
		n = mdss_proc_read(page, &start, *ppos,
				  count, &eof, NULL);

		if (n == 0) {    /* end of file */
			break;
		}
		if (n < 0) {  /* error */
			if (retval == 0)
				retval = n;
			break;
		}

		if (start == NULL) {
			if (n > PAGE_SIZE) {
				printk(KERN_ERR
					"proc_file_read: Apparent buffer overflow!\n");
				n = PAGE_SIZE;
			}
			n -= *ppos;
			if (n <= 0)
				break;
			if (n > count)
				n = count;
			start = page + *ppos;
		} else if (start < page) {
			if (n > PAGE_SIZE) {
				printk(KERN_ERR
					"proc_file_read: Apparent buffer overflow!\n");
				n = PAGE_SIZE;
			}
			if (n > count) {
				/*
				 * Don't reduce n because doing so might
				 * cut off part of a data block.
				 */
				printk(KERN_WARNING
					"proc_file_read: Read count exceeded\n");
			}
		} else /* start >= page */ {
			unsigned long startoff = (unsigned long)(start - page);
			if (n > (PAGE_SIZE - startoff)) {
				printk(KERN_ERR
					"proc_file_read: Apparent buffer overflow!\n");
				n = PAGE_SIZE - startoff;
			}
			if (n > count) {
				n = count;
			}
		}

		n -= copy_to_user(buf, start < page ? page : start, n);
		if (n == 0) {
			if (retval == 0) {
				retval = -EFAULT;
			}
			break;
		}

		*ppos += start < page ? (unsigned long)start : n;
		nbytes -= n;
		buf += n;
		retval += n;
	}
	free_page((unsigned long) page);

	return retval;
}

static ssize_t mdss_proc_file_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t rv = -EIO;

	rv = mdss_proc_write(file, buffer, count, NULL);

	return rv;
}

static int mdss_panel_dsi_write_reg(struct mdss_dsi_cmd_req *req)
{
	int retry = 5;
	int ret;
	char dtype;
	struct mdss_dsi_cmd_desc cmd[1];
	char cmd_buf[MDSS_LCDDR_BUF_MAX + 1];

	if (req->size > MDSS_LCDDR_BUF_MAX) {
		pr_err("size over, -EINVAL\n");
		return -EINVAL;
	}

	memset(cmd_buf, 0x00, sizeof(cmd_buf));
	cmd_buf[0] = req->addr;
	cmd_buf[1] = 0x00;
	memcpy(&cmd_buf[1], req->data, req->size);

	if (req->dtype != MDSS_DTYPE_GEN_WRITE) {
		if (req->size == 0) { /* 0 parameters */
			dtype = MDSS_DTYPE_DCS_WRITE;
		} else if (req->size == 1) { /* 1 parameter */
			dtype = MDSS_DTYPE_DCS_WRITE1;
		} else { /* Many parameters */
			dtype = MDSS_DTYPE_DCS_LWRITE;
		}
	} else {
		if (req->size == 0) { /* 0 parameters */
			dtype = MDSS_DTYPE_GEN_WRITE;
		} else if (req->size == 1) { /* 1 parameter */
			dtype = MDSS_DTYPE_GEN_WRITE1;
		} else if (req->size == 2) { /* 2 parameters */
			dtype = MDSS_DTYPE_GEN_WRITE2;
		} else { /* Many parameters */
			dtype = MDSS_DTYPE_GEN_LWRITE;
		}
	}

	cmd[0].dtype = dtype;
	cmd[0].mode = req->mode;
	cmd[0].wait = 0x00;
	cmd[0].dlen = req->size + 1;
	cmd[0].payload = cmd_buf;

	for (; retry >= 0; retry--) {
		ret = mdss_panel_mipi_dsi_cmds_tx(1, cmd, ARRAY_SIZE(cmd));
		if (ret == 0) {
			break;
		} else {
			pr_warn("mdss_panel_mipi_dsi_cmds_tx() failure. ret=%d retry=%d\n", ret, retry);
		}
	}

	if (ret != 0) {
		pr_err("mipi_dsi_cmds_tx error\n");
		return -EIO;
	}

	return 0;
}

static int mdss_panel_dsi_read_reg(struct mdss_dsi_cmd_req *req)
{
	int retry = 5;

	int ret;
	char dtype;
	struct mdss_dsi_cmd_desc cmd[1];
	char cmd_buf[2 + 1];

	pr_debug("in address:%02X, buf:%p, size:%d\n", req->addr, req->data, req->size);
	if ((req->size > MDSS_LCDDR_BUF_MAX) || (req->size == 0)) {
		pr_err("size over, -EINVAL\n");
		return -EINVAL;
	}

	cmd_buf[0] = req->addr;
	cmd_buf[1] = 0x00;

	if (req->dtype != MDSS_DTYPE_GEN_READ) {
		dtype = MDSS_DTYPE_DCS_READ;
	} else {
		if (req->size == 1) { /* 0 parameters */
			dtype = MDSS_DTYPE_GEN_READ;
		} else if (req->size == 2) { /* 1 parameter */
			dtype = MDSS_DTYPE_GEN_READ1;
		} else { /* 2 paramters */
			dtype = MDSS_DTYPE_GEN_READ2;
		}
	}
	cmd[0].dtype    = dtype;
	cmd[0].mode     = req->mode;
	cmd[0].wait     = 0x00;
	cmd[0].dlen     = 1;
	cmd[0].payload  = cmd_buf;

	for (; retry >= 0; retry--) {
		ret = mdss_panel_mipi_dsi_cmds_rx((unsigned char *)req->data, cmd, req->size);
		if (ret == 0) {
			break;
		} else {
			pr_warn("mdss_panel_mipi_dsi_cmds_rx() failure. ret=%d retry=%d\n", ret, retry);
		}
	}

	if (ret != 0) {
		pr_err("mipi_dsi_cmds_rx error\n");
		return -EIO;
	}

	pr_debug("out 0\n");
	return 0;
}

static int mdss_panel_mipi_dsi_cmds_tx(int commit, struct mdss_dsi_cmd_desc *cmds, int cnt)
{
	int ret;
	pr_debug("in cnt=%d\n", cnt);
	mdss_panel_dsi_wlog(cmds, cnt);
	ret = mdss_proc_host_dsi_tx(commit, cmds, cnt );
	pr_debug("out ret=%d\n", ret);
	return ret;
}

static int mdss_panel_mipi_dsi_cmds_rx(unsigned char *rbuf, struct mdss_dsi_cmd_desc *cmds, unsigned char size)
{
	int ret;
	pr_debug("in size:%d\n", size);
	mdss_panel_dsi_wlog(cmds, 1);
	ret = mdss_proc_host_dsi_rx(cmds, rbuf, size);
	mdss_panel_dsi_rlog(cmds->payload[0], (char *)rbuf, size);
	pr_debug("out ret:%d\n", ret);
	return ret;
}

static void mdss_panel_dsi_wlog(struct mdss_dsi_cmd_desc *cmds, int cmdslen)
{
#ifdef MDSS_LOG_ENABLE
	char buf[128];
	char *pbuf;
	int cmdcnt;
	int arylen;
	int writelen;

	for (cmdcnt = 0; cmdcnt != cmdslen; cmdcnt++) {
		int i;
		char dtype = cmds[cmdcnt].dtype;
		short payloadlen = cmds[cmdcnt].dlen;
		unsigned char *payload = (unsigned char *)cmds[cmdcnt].payload;

		pbuf = buf;
		arylen = sizeof(buf) / sizeof(*buf);

		writelen = snprintf(pbuf, arylen, "dtype= %02X, ", dtype);
		arylen -= writelen;
		pbuf += writelen;

		writelen = snprintf(pbuf, arylen, "payload= %02X ", payload[0]);
		arylen -= writelen;
		pbuf += writelen;

		for (i = 1; i != payloadlen; ++i) {
			if ((!((i - 1) % 16)) && (i != 1)) {
				int spacecnt = 23;
				*pbuf = '\n';
				*(pbuf+1) = '\0';
				pr_info("%s", buf);

				arylen = sizeof(buf) / sizeof(*buf);
				pbuf = buf;
				memset(pbuf, ' ', spacecnt);
				arylen -= spacecnt;
				pbuf += spacecnt;
			}
			writelen = snprintf(pbuf, arylen, "%02X ", payload[i]);
			arylen -= writelen;
			pbuf += writelen;
		}

		*pbuf = '\n';
		*(pbuf+1) = '\0';
		pr_info("%s", buf);
	}
#endif /* MDSS_LOG_ENABLE */
}

static void mdss_panel_dsi_rlog(char addr, char *rbuf, int len)
{
#ifdef MDSS_LOG_ENABLE
	char buf[128];
	char *pbuf;
	unsigned char *prbuf = (unsigned char *)rbuf;
	int arylen;
	int writelen;
	int i = 0;

	arylen = sizeof(buf) / sizeof(*buf);
	pbuf = buf;

	writelen = snprintf(pbuf, arylen, "addr = %02X, val = ", (unsigned char)addr);
	arylen -= writelen;
	pbuf += writelen;

	for (i = 0; i != len; ++i) {
		if ((!(i % 16)) && (i)) {
			int spacecnt = 17;
			*pbuf = '\n';
			*(pbuf+1) = '\0';
			pr_info("%s", buf);

			arylen = sizeof(buf) / sizeof(*buf);
			pbuf = buf;
			memset(pbuf, ' ', spacecnt);
			arylen -= spacecnt;
			pbuf += spacecnt;
		}
		writelen = snprintf(pbuf, arylen, "%02X ", *prbuf);
		arylen -= writelen;
		pbuf += writelen;
		prbuf++;
	}

	*pbuf = '\n';
	*(pbuf+1) = '\0';
	pr_info("%s", buf);
#endif /* MDSS_LOG_ENABLE */
}

#ifdef MDSS_FPS_LED_PANEL_SUPPORT
static int mdss_fps_led_read_reg(unsigned char *fps_reg)
{
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct dcs_cmd_req cmdreq;
	int ret = 0;

	if (mdss_dsi_ctrl) {
		ctrl = mdss_dsi_ctrl;
	} else {
		pr_err("%s mdss_dsi_ctrl=0x%p\n", __func__, mdss_dsi_ctrl);
		return -EIO;
	}
	/* bank set */
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &bank_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	/* fps read */
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = sizeof(read_buf);
	cmdreq.rbuf = read_buf;
	cmdreq.cb = NULL; /* call back */
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	if (ret >= 0) {
		*fps_reg = read_buf[0];
		pr_debug("%s: fps read succsess. fps=0x%x ret=%d\n", __func__, *fps_reg, ret);
		ret = 0;
	} else {
		pr_err("%s: fps read err. ret=%d\n", __func__, ret);
	}
	return ret;
}

static int mdss_led_set_color(struct shled_tri_led* led)
{
	pr_debug("%s: red=%d green=%d blue=%d\n",
			__func__, led->red, led->green, led->blue);
	led_trigger_event(led_fps_r_trigger, led->red);
	led_trigger_event(led_fps_g_trigger, led->green);
	led_trigger_event(led_fps_b_trigger, led->blue);
	return 0;
}

static void mdss_fps_led_set_color(int state)
{
	struct shled_tri_led led;

	led.red = mdss_fps_led_color[state][0];
	led.green = mdss_fps_led_color[state][1];
	led.blue = mdss_fps_led_color[state][2];
	mdss_led_set_color(&led);

}

static int mdss_fps_set_led_video(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int state = 0;
	int fps = mdss_panel_get_framerate(&(ctrl->panel_data.panel_info),FPS_RESOLUTION_HZ);

	pr_debug("%s: in\n", __func__);
	if (fps > 60) {
		state = FPS_LED_STATE_120HZ;
	} else if ((fps <= 60) && (fps > 45)) {
		state = FPS_LED_STATE_60HZ;
	} else if ((fps <= 45) && (fps > 30)) {
		state = FPS_LED_STATE_45HZ;
	} else if ((fps <= 30) && (fps > 15)) {
		state = FPS_LED_STATE_30HZ;
	} else if ((fps <= 15) && (fps > 10)) {
		state = FPS_LED_STATE_15HZ;
	} else {
		state = FPS_LED_STATE_1HZ;
	}
	pr_debug("%s: out state = %d\n", __func__, state);
	return state;
}

static int mdss_fps_set_led_cmd(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int bit_cnt = 0;
	unsigned char fps_reg = 0;
	int state = 0;
	int ret = 0;

	pr_debug("%s: in\n", __func__);
	ret = mdss_fps_led_read_reg(&fps_reg);
	if (ret == 0) {
		bit_cnt = hweight8(fps_reg);

		if (mdss_fps_led_ctx.max_frame_rate == 120) {
			if (bit_cnt >= 8) {
				state = FPS_LED_STATE_120HZ;
			} else if (bit_cnt >= 4) {
				state = FPS_LED_STATE_60HZ;
			} else if (bit_cnt >= 2) {
				state = FPS_LED_STATE_30HZ;
			} else if (bit_cnt >= 1) {
				state = FPS_LED_STATE_15HZ;
			} else {
				state = FPS_LED_STATE_1HZ;
			}
		} else {
			if (bit_cnt >= 8) {
				state = FPS_LED_STATE_60HZ;
			} else if (bit_cnt >= 4) {
				state = FPS_LED_STATE_30HZ;
			} else if (bit_cnt >= 2) {
				state = FPS_LED_STATE_15HZ;
			} else if (bit_cnt >= 1) {
				state = FPS_LED_STATE_1HZ;
			} else {
				state = FPS_LED_STATE_1HZ;
			}
		}
	}
	pr_debug("%s: out state = %d\n", __func__, state);
	return state;
}

static void mdss_fps_led_work(struct work_struct *work)
{
	int state, bef_state;
	unsigned char fps_reg = 0;

	pr_debug("%s: in\n", __func__);

	if (!mdss_fps_led_ctx.enable || mdss_fps_led_ctx.suspend) {
		pr_debug("out2 enable=%d suspend=%d\n",
				mdss_fps_led_ctx.enable,
				mdss_fps_led_ctx.suspend);
		return;
	}

	queue_delayed_work(
			mdss_fps_led_ctx.workq,
			&mdss_fps_led_ctx.work,
			usecs_to_jiffies(mdss_fps_led_ctx.interval));

	state = mdss_fps_led_ctx.state;
	bef_state = state;

	if (mdss_fps_led_ctx.panel_on) {
		if (!mdss_dsi_ctrl){
			pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p\n", __func__, mdss_dsi_ctrl);
			return;
		}
		if (mdss_dsi_ctrl->panel_mode == DSI_CMD_MODE) {
			state = mdss_fps_set_led_cmd(mdss_dsi_ctrl);
		} else {
			state = mdss_fps_set_led_video(mdss_dsi_ctrl);
		}
		pr_debug("fps_reg=%02x state=%s\n", fps_reg, mdss_fps_led_state_str[state]);
	} else {
		state = FPS_LED_STATE_NONE;
	}

	if (state != bef_state) {
		mdss_fps_led_set_color(state);
	}

	mdss_fps_led_ctx.state = state;

	pr_debug("%s: out\n", __func__);
}

static void mdss_fps_led_start(struct mdss_mdp_ctl *ctl)
{
	int ret = 0;
	struct mdss_data_type *mdata;
	pr_debug("%s: in\n", __func__);

	if (mdss_fps_led_ctx.enable) {
		pr_debug("%s: out2 enable=%d\n", __func__, mdss_fps_led_ctx.enable);
		return;
	}

	if (!mdss_fps_led_ctx.workq) {
		pr_err("%s: workq is NULL.\n", __func__);
		return;
	}

	mdata = mdss_mdp_get_mdata();
	if (!mdata) {
		pr_err("%s:failed to get mdata.\n", __func__);
		return;
	}

	mdss_fps_led_ctx.enable = true;
	mdss_fps_led_ctx.state = FPS_LED_STATE_NONE;
	mdss_fps_led_ctx.interval = FPS_LED_INTERVAL;
	mdss_fps_led_ctx.max_frame_rate = mdss_panel_get_framerate(&(ctl->panel_data->panel_info),FPS_RESOLUTION_HZ);
	pr_debug("%s: max_frame_rate=%d\n", __func__, mdss_fps_led_ctx.max_frame_rate);

	led_trigger_register_simple("led-fps-r-trigger", &led_fps_r_trigger);
	led_trigger_register_simple("led-fps-g-trigger", &led_fps_g_trigger);
	led_trigger_register_simple("led-fps-b-trigger", &led_fps_b_trigger);
	mdss_fps_led_set_color(FPS_LED_STATE_NONE);

	mdss_fps_led_ctx.panel_on = true;
	fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&fb_notif);
	pr_debug("%s:fb_register_client. rc=%d\n", __func__, ret);

	if (mdss_fps_led_ctx.panel_on) {
		mdss_fps_led_ctx.suspend = false;
		queue_delayed_work(
				mdss_fps_led_ctx.workq,
				&mdss_fps_led_ctx.work,
				usecs_to_jiffies(mdss_fps_led_ctx.interval));
	} else {
		mdss_fps_led_ctx.suspend = true;
	}

	pr_debug("%s: out\n", __func__);
}

static void mdss_fps_led_stop(void)
{
	pr_debug("%s: in\n", __func__);

	if (!mdss_fps_led_ctx.enable) {
		pr_debug("%s:out2 enable=%d\n", __func__, mdss_fps_led_ctx.enable);
		return;
	}

	if (!mdss_fps_led_ctx.workq) {
		pr_err("%s: workq is NULL.\n", __func__);
		return;
	}

	mdss_fps_led_ctx.enable = false;

	fb_unregister_client(&fb_notif);

	cancel_delayed_work_sync(&mdss_fps_led_ctx.work);

	mdss_fps_led_set_color(FPS_LED_STATE_NONE);

	led_trigger_unregister_simple(led_fps_r_trigger);
	led_trigger_unregister_simple(led_fps_g_trigger);
	led_trigger_unregister_simple(led_fps_b_trigger);

	pr_debug("%s: out\n", __func__);
}

static void mdss_fps_led_resume(void)
{
	pr_debug("%s: in\n", __func__);

	if (!mdss_fps_led_ctx.enable || !mdss_fps_led_ctx.suspend) {
		pr_debug("%s:out2 enable=%d suspend=%d\n", __func__,
				mdss_fps_led_ctx.enable,
				mdss_fps_led_ctx.suspend);
		return;
	}

	if (!mdss_fps_led_ctx.workq) {
		pr_err("%s: workq is NULL.\n", __func__);
		return;
	}

	mdss_fps_led_ctx.suspend = false;
	queue_delayed_work(
			mdss_fps_led_ctx.workq,
			&mdss_fps_led_ctx.work,
			usecs_to_jiffies(mdss_fps_led_ctx.interval * 3));

	pr_debug("%s: out\n", __func__);
}

static void mdss_fps_led_suspend(void)
{
	pr_debug("%s: in\n", __func__);

	if (!mdss_fps_led_ctx.enable || mdss_fps_led_ctx.suspend) {
		pr_debug("%s:out2 enable=%d suspend=%d\n", __func__,
				mdss_fps_led_ctx.enable,
				mdss_fps_led_ctx.suspend);
		return;
	}

	if (!mdss_fps_led_ctx.workq) {
		pr_err("%s: workq is NULL.\n", __func__);
		return;
	}

	mdss_fps_led_ctx.suspend = true;
	cancel_delayed_work_sync(&mdss_fps_led_ctx.work);
	mdss_fps_led_ctx.state = FPS_LED_STATE_NONE;

	mdss_fps_led_set_color(FPS_LED_STATE_NONE);

	pr_debug("%s: out\n", __func__);
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	pr_debug("%s: in\n", __func__);

	if (evdata && evdata->data) {
		blank = evdata->data;
		pr_debug("%s:fb request. event=%ld, blank=%d\n",__func__, event, *blank);
		switch (event) {
		case FB_EARLY_EVENT_BLANK:
			if (*blank == FB_BLANK_POWERDOWN) {
				if (mdss_fps_led_ctx.panel_on) {
					mdss_fps_led_ctx.panel_on = false;
					mdss_fps_led_suspend();
				}
			}
		case FB_EVENT_BLANK:
			if (*blank == FB_BLANK_UNBLANK) {
				if (!mdss_fps_led_ctx.panel_on) {
					mdss_fps_led_ctx.panel_on = true;
					mdss_fps_led_resume();
				}
			}
		}
	}
	pr_debug("%s:out. event=%ld", __func__, event);
	return 0;
}
#endif /* MDSS_FPS_LED_PANEL_SUPPORT */
#ifdef MDSS_MFR_PANEL_SUPPORT
static int mdss_proc_mfr(struct mdss_dsi_ctrl_pdata *ctrl, int value)
{
	int ret = 0;
	struct mdss_data_type *mdata;

	mdata = mdss_mdp_get_mdata();
	if (!mdata) {
		pr_err("%s:failed to get mdata.\n", __func__);
		return -EFAULT;
	}
	if (mdata->lcd_switch == MDSS_PANEL_ROSETTA) {
		ret = mdss_proc_mfr_rosetta(ctrl, value);
	} else {
		ret = mdss_proc_mfr_hayabusa(ctrl, value);
	}
	return ret;
}

static int mdss_proc_mfr_rosetta(struct mdss_dsi_ctrl_pdata *ctrl, int value)
{
	int ret = 0;
	unsigned char mfr[] = {0x62, 0x00};
	unsigned char refdet[] = {0x66, 0x20};
	struct dcs_cmd_req cmdreq;
	unsigned char mfr_proc_cmd_addr_value[][2] = {
		{0xFF, 0x26},
		{0x25, 0x78},
		{0x27, 0x78},
		{0x28, 0x25},
		{0x36, 0x08},
		{0x60, 0x00},
		{0x61, 0x00},

		{0x65, 0x9F},

		{0x41, 0x00},
		{0x42, 0x00},
		{0x43, 0x00},
		{0x44, 0x00},
		{0x45, 0x00},
		{0x46, 0x00},
		{0x47, 0x00},
		{0x48, 0x00},
		{0x49, 0x00},
		{0x4A, 0x00},
		{0x4B, 0x00},
		{0x4C, 0x00},
		{0x4D, 0x00},
		{0x4E, 0x00},
		{0x4F, 0x00},
		{0x50, 0x00},
		{0x51, 0x00},
		{0x40, 0x00},
		{0x31, 0x00},
		{0x32, 0x00},
		{0x33, 0x00},
		{0x34, 0x00},
		{0x35, 0x00},
		{0x30, 0x00},
		{0xFF, 0x10},
		{0xBC, 0x0A},
	};
	struct dsi_cmd_desc mfr_cmd[] = {
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[0]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[1]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[2]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[3]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[4]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[5]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[6]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[7]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, refdet},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[8]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[9]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[10]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[11]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[12]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[13]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[14]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[15]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[16]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[17]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[18]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[19]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[20]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[21]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[22]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[23]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[24]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[25]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[26]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[27]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[28]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[29]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[30]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[31]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[32]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[33]},
	};

	switch(value) {
	case 120:
		mfr[1] = 0x00;
		refdet[1] = 0x20;
		break;
	case 60:
		mfr[1] = 0x01;
		refdet[1] = 0x21;
		break;
	case 40:
		mfr[1] = 0x02;
		refdet[1] = 0x22;
		break;
	case 30:
		mfr[1] = 0x03;
		refdet[1] = 0x23;
		break;
	case 20:
		mfr[1] = 0x05;
		refdet[1] = 0x25;
		break;
	case 15:
		mfr[1] = 0x07;
		refdet[1] = 0x37;
		break;
	case 10:
		mfr[1] = 0x0B;
		refdet[1] = 0x37;
		break;
	case 1:
		mfr[1] = 0x77;
		refdet[1] = 0x37;
		break;
	default:
		printk("%s:error.not support mfr=%d\n", __func__, value);
		return -EPERM;
	}
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = mfr_cmd;
	cmdreq.cmds_cnt = ARRAY_SIZE(mfr_cmd);
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	pr_debug("%s:end mfr=%d ret=%d", __func__, value, ret);
	return ret;
}

static int mdss_proc_mfr_hayabusa(struct mdss_dsi_ctrl_pdata *ctrl, int value)
{
	int ret = 0;
	unsigned char mfr[] = {0x21, 0x00};
	unsigned char refdet[] = {0x23, 0x11};
	struct dcs_cmd_req cmdreq;
	unsigned char mfr_proc_cmd_addr_value[][2] = {
		{0xFF, 0x26},
		{0x1D, 0x80},
		{0x20, 0x00},
		{0x22, 0x9E},
		{0x0B, 0x03},
		{0x0C, 0x1B},
		{0x0D, 0x10},
		{0x0E, 0x06},
		{0x10, 0x01},
		{0x11, 0x9C},
		{0x12, 0x1C},
		{0x13, 0x02},
		{0x80, 0x00},
		{0x82, 0x00},
		{0x83, 0x11},
		{0x84, 0x11},
		{0x0A, 0x00},
		{0x15, 0x00},
		{0x16, 0x00},
		{0x17, 0x00},
		{0x18, 0x00},
		{0x19, 0x00},
		{0x14, 0x00},
		{0xFF, 0x10},
		{0xB0, 0x1A},
	};
	struct dsi_cmd_desc mfr_cmd[] = {
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[0]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[1]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[2]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[3]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, refdet},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[4]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[5]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[6]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[7]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[8]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[9]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[10]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[11]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[12]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[13]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[14]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[15]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[16]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[17]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[18]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[19]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[20]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[21]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[22]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[23]},
		{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, 2}, mfr_proc_cmd_addr_value[24]},
	};

	if(mdss_dsi_is_hw_config_split(ctrl->shared_data)) {
		switch(value) {
		case 120:
			mfr[1] = 0x00;
			refdet[1] = 0x11;
			break;
		case 60:
			mfr[1] = 0x01;
			refdet[1] = 0x11;
			break;
		case 40:
			mfr[1] = 0x02;
			refdet[1] = 0x11;
			break;
		case 30:
			mfr[1] = 0x03;
			refdet[1] = 0x11;
			break;
		case 20:
			mfr[1] = 0x05;
			refdet[1] = 0x11;
			break;
		case 15:
			mfr[1] = 0x07;
			refdet[1] = 0x11;
			break;
		case 10:
			mfr[1] = 0x0B;
			refdet[1] = 0x33;
			break;
		case 1:
			mfr[1] = 0x77;
			refdet[1] = 0x33;
			break;
		default:
			printk("%s:error.not support mfr=%d\n", __func__, value);
			return -EPERM;
		}
	} else {
		switch(value) {
		case 60:
			mfr[1] = 0x00;
			refdet[1] = 0x11;
			break;
		case 30:
			mfr[1] = 0x01;
			refdet[1] = 0x11;
			break;
		case 15:
			mfr[1] = 0x03;
			refdet[1] = 0x11;
			break;
		case 10:
			mfr[1] = 0x05;
			refdet[1] = 0x33;
			break;
		case 1:
			mfr[1] = 0x3B;
			refdet[1] = 0x33;
			break;
		default:
			pr_err("%s:error.not support mfr=%d\n", __func__, value);
			return -EPERM;
		}
	}
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = mfr_cmd;
	cmdreq.cmds_cnt = ARRAY_SIZE(mfr_cmd);
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	pr_debug("%s:end mfr=%d ret=%d", __func__, value, ret);
	return ret;
}
#endif