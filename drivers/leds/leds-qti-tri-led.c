/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/types.h>

#define TRILED_REG_TYPE			0x04
#define TRILED_REG_SUBTYPE		0x05
#define TRILED_REG_EN_CTL		0x46

/* TRILED_REG_EN_CTL */
#define TRILED_EN_CTL_MASK		GENMASK(7, 5)
#define TRILED_EN_CTL_MAX_BIT		7

#define TRILED_TYPE			0x19
#define TRILED_SUBTYPE_LED3H0L12	0x02
#define TRILED_SUBTYPE_LED2H0L12	0x03
#define TRILED_SUBTYPE_LED1H2L12	0x04

#define TRILED_NUM_MAX			3

#define PWM_PERIOD_DEFAULT_NS		1000000

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00038 */
#ifndef MAX
#define  MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif
#define QPNP_TRI_LED_RED	0
#define QPNP_TRI_LED_GREEN	1
#define QPNP_TRI_LED_BLUE	2
#endif /* CONFIG_LEDS_SHARP */

struct pwm_setting {
	u64	pre_period_ns;
	u64	period_ns;
	u64	duty_ns;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00037 */
	u64	period_ns_u64;
	u64	duty_ns_u64;
#endif /* CONFIG_LEDS_SHARP */
};

struct led_setting {
	u64			on_ms;
	u64			off_ms;
	enum led_brightness	brightness;
	bool			blink;
	bool			breath;
};

struct qpnp_led_dev {
	struct led_classdev	cdev;
	struct pwm_device	*pwm_dev;
	struct pwm_setting	pwm_setting;
	struct led_setting	led_setting;
	struct qpnp_tri_led_chip	*chip;
	struct mutex		lock;
	const char		*label;
	const char		*default_trigger;
	u8			id;
	bool			blinking;
	bool			breathing;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00038 */
	u32			calib;
#endif /* CONFIG_LEDS_SHARP */
};

struct qpnp_tri_led_chip {
	struct device		*dev;
	struct regmap		*regmap;
	struct qpnp_led_dev	*leds;
	struct mutex		bus_lock;
	int			num_leds;
	u16			reg_base;
	u8			subtype;
};

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00038 */
static char qpnp_led_system_color[32];
static u32 calib_enabled = 0;
module_param_string(system_color, qpnp_led_system_color, sizeof(qpnp_led_system_color), 0);
module_param(calib_enabled, uint, S_IRUGO | S_IWUSR);

static u32 qpnp_tri_led_calib_brightness(struct qpnp_led_dev *led)
{
	pr_debug("%s: in %s LED calib_enabled=%d\n", __func__, led->cdev.name, calib_enabled);
	if (led->led_setting.brightness && calib_enabled &&
		led->calib && !led->led_setting.blink && !led->led_setting.breath) {
		return MAX(led->led_setting.brightness * led->calib / LED_FULL, 1);
	} else {
		return led->cdev.brightness;
	}
}

static void qpnp_tri_led_calib_read_clrvari_param(struct qpnp_led_dev *led)
{
	struct device_node *root, *node;
	u32 color;
	char default_name[32];
	int rc = -EINVAL;

	pr_debug("%s: in %s LED node=%s\n", __func__, led->cdev.name, qpnp_led_system_color);

	root = of_find_node_by_name(NULL, "sharp,shled_leds_color_variation");
	if (!root) {
		pr_err("%s: could not find root node.\n", __func__);
		goto calib_read_error;
	}

	node = of_find_node_by_name(root, qpnp_led_system_color);
	if (!node) {
		pr_err("%s: could not find node %s, try default.\n", __func__, qpnp_led_system_color);
		rc = of_property_read_u32(root, "sharp,system-color-default", &color);
		if (rc < 0) {
			pr_err("%s: could not find default node.\n", __func__);
			goto calib_read_error;
		}
		snprintf(default_name, sizeof(default_name), "sharp,shled-leds-calib-%02x", color);
		pr_debug("%s: default_name=%s\n", __func__, default_name);
		node = of_find_node_by_name(root, default_name);
		if (!node) {
			pr_err("%s: could not find node %s.\n", __func__, default_name);
			goto calib_read_error;
		}
	}

	if (led->id == QPNP_TRI_LED_RED) {
		rc = of_property_read_u32(node, "red", &led->calib);
	} else if (led->id == QPNP_TRI_LED_GREEN) {
		rc = of_property_read_u32(node, "green", &led->calib);
	} else if (led->id == QPNP_TRI_LED_BLUE) {
		rc = of_property_read_u32(node, "blue", &led->calib);
	}
	pr_debug("%s: led->calib=%d.\n", __func__, led->calib);
	if (rc) {
		pr_err("%s: could not find calibration parameter param:%s led->id:%d.\n", __func__, qpnp_led_system_color, led->id);
		goto calib_read_error;
	}

	calib_enabled = 1;

	return;

calib_read_error:
	calib_enabled = 0;
	return;
}
#endif /* CONFIG_LEDS_SHARP */

static int qpnp_tri_led_read(struct qpnp_tri_led_chip *chip, u16 addr, u8 *val)
{
	int rc;
	unsigned int tmp;

	mutex_lock(&chip->bus_lock);
	rc = regmap_read(chip->regmap, chip->reg_base + addr, &tmp);
	if (rc < 0)
		dev_err(chip->dev, "Read addr 0x%x failed, rc=%d\n", addr, rc);
	else
		*val = (u8)tmp;
	mutex_unlock(&chip->bus_lock);

	return rc;
}

static int qpnp_tri_led_masked_write(struct qpnp_tri_led_chip *chip,
				u16 addr, u8 mask, u8 val)
{
	int rc;

	mutex_lock(&chip->bus_lock);
	rc = regmap_update_bits(chip->regmap, chip->reg_base + addr, mask, val);
	if (rc < 0)
		dev_err(chip->dev, "Update addr 0x%x to val 0x%x with mask 0x%x failed, rc=%d\n",
					addr, val, mask, rc);
	mutex_unlock(&chip->bus_lock);

	return rc;
}

static int __tri_led_config_pwm(struct qpnp_led_dev *led,
				struct pwm_setting *pwm)
{
	struct pwm_state pstate;
	int rc;

	pwm_get_state(led->pwm_dev, &pstate);
	pstate.enabled = !!(pwm->duty_ns != 0);
	pstate.period = pwm->period_ns;
	pstate.duty_cycle = pwm->duty_ns;
	pstate.output_type = led->led_setting.breath ?
		PWM_OUTPUT_MODULATED : PWM_OUTPUT_FIXED;
	/* Use default pattern in PWM device */
	pstate.output_pattern = NULL;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00037 */
	if (led->led_setting.blink) {
		pstate.period_u64 = pwm->period_ns_u64;
		pstate.duty_cycle_u64 = pwm->duty_ns_u64;
		rc = pwm_apply_state_u64(led->pwm_dev, &pstate);
	} else {
		pstate.period_u64 = 0;
		pstate.duty_cycle_u64 = 0;
		rc = pwm_apply_state(led->pwm_dev, &pstate);
	}
#else /* CONFIG_LEDS_SHARP */
	rc = pwm_apply_state(led->pwm_dev, &pstate);
#endif /* CONFIG_LEDS_SHARP */

	if (rc < 0)
		dev_err(led->chip->dev, "Apply PWM state for %s led failed, rc=%d\n",
					led->cdev.name, rc);

	return rc;
}

static int __tri_led_set(struct qpnp_led_dev *led)
{
	int rc = 0;
	u8 val = 0, mask = 0;

	rc = __tri_led_config_pwm(led, &led->pwm_setting);
	if (rc < 0) {
		dev_err(led->chip->dev, "Configure PWM for %s led failed, rc=%d\n",
					led->cdev.name, rc);
		return rc;
	}

	mask |= 1 << (TRILED_EN_CTL_MAX_BIT - led->id);

	if (led->pwm_setting.duty_ns == 0)
		val = 0;
	else
		val = mask;

	rc = qpnp_tri_led_masked_write(led->chip, TRILED_REG_EN_CTL,
							mask, val);
	if (rc < 0)
		dev_err(led->chip->dev, "Update addr 0x%x failed, rc=%d\n",
					TRILED_REG_EN_CTL, rc);

	return rc;
}

static int qpnp_tri_led_set(struct qpnp_led_dev *led)
{
	u64 on_ms, off_ms, period_ns, duty_ns;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00038 */
	u32 brightness = qpnp_tri_led_calib_brightness(led);
#else
	enum led_brightness brightness = led->led_setting.brightness;
#endif /* CONFIG_LEDS_SHARP */
	int rc = 0;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00037 */
	u64 period_ns_u64 = 0, duty_ns_u64 = 0;
#endif /* CONFIG_LEDS_SHARP */

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00038 */
	pr_debug("%s: brightness=%d calib_brightness=%d\n", __func__,
		led->led_setting.brightness, brightness);
#endif /* CONFIG_LEDS_SHARP */

	if (led->led_setting.blink) {
		on_ms = led->led_setting.on_ms;
		off_ms = led->led_setting.off_ms;

		duty_ns = on_ms * NSEC_PER_MSEC;
		period_ns = (on_ms + off_ms) * NSEC_PER_MSEC;

		if (period_ns < duty_ns && duty_ns != 0)
			period_ns = duty_ns + 1;

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00037 */
		duty_ns_u64 = on_ms * NSEC_PER_MSEC;
		period_ns_u64 = (on_ms + off_ms) * NSEC_PER_MSEC;

		led->pwm_setting.duty_ns_u64 = duty_ns_u64;
		led->pwm_setting.period_ns_u64 = period_ns_u64;
#endif /* CONFIG_LEDS_SHARP */
	} else {
		/* Use initial period if no blinking is required */
		period_ns = led->pwm_setting.pre_period_ns;

		if (brightness == LED_OFF)
			duty_ns = 0;

		duty_ns = period_ns * brightness;
		do_div(duty_ns, LED_FULL);

		if (period_ns < duty_ns && duty_ns != 0)
			period_ns = duty_ns + 1;
	}
	dev_dbg(led->chip->dev, "PWM settings for %s led: period = %lluns, duty = %lluns\n",
				led->cdev.name, period_ns, duty_ns);
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00038 */
	pr_debug("%s: duty_ns=%llu period_ns=%llu brightness=%d calib_enabled=%d calib=%d calib_brightness=%d\n",
		__func__, duty_ns, period_ns, led->led_setting.brightness, calib_enabled, led->calib, brightness);
#endif /* CONFIG_LEDS_SHARP */

	led->pwm_setting.duty_ns = duty_ns;
	led->pwm_setting.period_ns = period_ns;

	rc = __tri_led_set(led);
	if (rc < 0) {
		dev_err(led->chip->dev, "__tri_led_set %s failed, rc=%d\n",
				led->cdev.name, rc);
		return rc;
	}

	if (led->led_setting.blink) {
		led->cdev.brightness = LED_FULL;
		led->blinking = true;
		led->breathing = false;
	} else if (led->led_setting.breath) {
		led->cdev.brightness = LED_FULL;
		led->blinking = false;
		led->breathing = true;
	} else {
		led->cdev.brightness = led->led_setting.brightness;
		led->blinking = false;
		led->breathing = false;
	}

	return rc;
}

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00044 */
static void qpnp_tri_led_set_brightness(struct led_classdev *led_cdev,
		enum led_brightness brightness)
#else /* CONFIG_LEDS_SHARP */
static int qpnp_tri_led_set_brightness(struct led_classdev *led_cdev,
		enum led_brightness brightness)
#endif /* CONFIG_LEDS_SHARP */
{
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);
	int rc = 0;

	mutex_lock(&led->lock);
	if (brightness > LED_FULL)
		brightness = LED_FULL;

	if (brightness == led->led_setting.brightness &&
			!led->blinking && !led->breathing) {
		mutex_unlock(&led->lock);
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00044 */
		return;
#else /* CONFIG_LEDS_SHARP */
		return 0;
#endif /* CONFIG_LEDS_SHARP */
	}

	led->led_setting.brightness = brightness;
	if (!!brightness)
		led->led_setting.off_ms = 0;
	else
		led->led_setting.on_ms = 0;
	led->led_setting.blink = false;
	led->led_setting.breath = false;

	rc = qpnp_tri_led_set(led);
	if (rc)
		dev_err(led->chip->dev, "Set led failed for %s, rc=%d\n",
				led->label, rc);

	mutex_unlock(&led->lock);

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00044 */
	return;
#else /* CONFIG_LEDS_SHARP */
	return rc;
#endif /* CONFIG_LEDS_SHARP */
}

static enum led_brightness qpnp_tri_led_get_brightness(
			struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

static int qpnp_tri_led_set_blink(struct led_classdev *led_cdev,
		unsigned long *on_ms, unsigned long *off_ms)
{
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);
	int rc = 0;

	mutex_lock(&led->lock);
	if (led->blinking && *on_ms == led->led_setting.on_ms &&
			*off_ms == led->led_setting.off_ms) {
		dev_dbg(led_cdev->dev, "Ignore, on/off setting is not changed: on %lums, off %lums\n",
						*on_ms, *off_ms);
		mutex_unlock(&led->lock);
		return 0;
	}

	if (*on_ms == 0) {
		led->led_setting.blink = false;
		led->led_setting.breath = false;
		led->led_setting.brightness = LED_OFF;
	} else if (*off_ms == 0) {
		led->led_setting.blink = false;
		led->led_setting.breath = false;
		led->led_setting.brightness = led->cdev.brightness;
	} else {
		led->led_setting.on_ms = *on_ms;
		led->led_setting.off_ms = *off_ms;
		led->led_setting.blink = true;
		led->led_setting.breath = false;
	}

	rc = qpnp_tri_led_set(led);
	if (rc)
		dev_err(led->chip->dev, "Set led failed for %s, rc=%d\n",
				led->label, rc);

	mutex_unlock(&led->lock);
	return rc;
}

static ssize_t breath_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", led->led_setting.breath);
}

static ssize_t breath_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int rc;
	bool breath;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	rc = kstrtobool(buf, &breath);
	if (rc < 0)
		return rc;

	mutex_lock(&led->lock);
	if (led->breathing == breath)
		goto unlock;

	led->led_setting.blink = false;
	led->led_setting.breath = breath;
	led->led_setting.brightness = breath ? LED_FULL : LED_OFF;
	rc = qpnp_tri_led_set(led);
	if (rc < 0)
		dev_err(led->chip->dev, "Set led failed for %s, rc=%d\n",
				led->label, rc);

unlock:
	mutex_unlock(&led->lock);
	return (rc < 0) ? rc : count;
}

static DEVICE_ATTR(breath, 0644, breath_show, breath_store);
static const struct attribute *breath_attrs[] = {
	&dev_attr_breath.attr,
	NULL
};

static int qpnp_tri_led_register(struct qpnp_tri_led_chip *chip)
{
	struct qpnp_led_dev *led;
	int rc, i, j;

	for (i = 0; i < chip->num_leds; i++) {
		led = &chip->leds[i];
		mutex_init(&led->lock);
		led->cdev.name = led->label;
		led->cdev.max_brightness = LED_FULL;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00044 */
		led->cdev.brightness_set = qpnp_tri_led_set_brightness;
#else /* CONFIG_LEDS_SHARP */
		led->cdev.brightness_set_blocking = qpnp_tri_led_set_brightness;
#endif /* CONFIG_LEDS_SHARP */
		led->cdev.brightness_get = qpnp_tri_led_get_brightness;
		led->cdev.blink_set = qpnp_tri_led_set_blink;
		led->cdev.default_trigger = led->default_trigger;
		led->cdev.brightness = LED_OFF;
		led->cdev.flags |= LED_KEEP_TRIGGER;

		rc = devm_led_classdev_register(chip->dev, &led->cdev);
		if (rc < 0) {
			dev_err(chip->dev, "%s led class device registering failed, rc=%d\n",
							led->label, rc);
			goto err_out;
		}

		if (pwm_get_output_type_supported(led->pwm_dev)
				& PWM_OUTPUT_MODULATED) {
			rc = sysfs_create_files(&led->cdev.dev->kobj,
					breath_attrs);
			if (rc < 0) {
				dev_err(chip->dev, "Create breath file for %s led failed, rc=%d\n",
						led->label, rc);
				goto err_out;
			}
		}
	}

	return 0;

err_out:
	for (j = 0; j <= i; j++) {
		if (j < i)
			sysfs_remove_files(&chip->leds[j].cdev.dev->kobj,
					breath_attrs);
		mutex_destroy(&chip->leds[j].lock);
	}
	return rc;
}

static int qpnp_tri_led_hw_init(struct qpnp_tri_led_chip *chip)
{
	int rc = 0;
	u8 val;

	rc = qpnp_tri_led_read(chip, TRILED_REG_TYPE, &val);
	if (rc < 0) {
		dev_err(chip->dev, "Read REG_TYPE failed, rc=%d\n", rc);
		return rc;
	}

	if (val != TRILED_TYPE) {
		dev_err(chip->dev, "invalid subtype(%d)\n", val);
		return -ENODEV;
	}

	rc = qpnp_tri_led_read(chip, TRILED_REG_SUBTYPE, &val);
	if (rc < 0) {
		dev_err(chip->dev, "Read REG_SUBTYPE failed, rc=%d\n", rc);
		return rc;
	}

	chip->subtype = val;

	return 0;
}

static int qpnp_tri_led_parse_dt(struct qpnp_tri_led_chip *chip)
{
	struct device_node *node = chip->dev->of_node, *child_node;
	struct qpnp_led_dev *led;
	struct pwm_args pargs;
	const __be32 *addr;
	int rc = 0, id, i = 0;

	addr = of_get_address(chip->dev->of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(chip->dev, "Getting address failed\n");
		return -EINVAL;
	}
	chip->reg_base = be32_to_cpu(addr[0]);

	chip->num_leds = of_get_available_child_count(node);
	if (chip->num_leds == 0) {
		dev_err(chip->dev, "No led child node defined\n");
		return -ENODEV;
	}

	if (chip->num_leds > TRILED_NUM_MAX) {
		dev_err(chip->dev, "can't support %d leds(max %d)\n",
				chip->num_leds, TRILED_NUM_MAX);
		return -EINVAL;
	}

	chip->leds = devm_kcalloc(chip->dev, chip->num_leds,
			sizeof(struct qpnp_led_dev), GFP_KERNEL);
	if (!chip->leds)
		return -ENOMEM;

	for_each_available_child_of_node(node, child_node) {
		rc = of_property_read_u32(child_node, "led-sources", &id);
		if (rc) {
			dev_err(chip->dev, "Get led-sources failed, rc=%d\n",
							rc);
			return rc;
		}

		if (id >= TRILED_NUM_MAX) {
			dev_err(chip->dev, "only support 0~%d current source\n",
					TRILED_NUM_MAX - 1);
			return -EINVAL;
		}

		led = &chip->leds[i++];
		led->chip = chip;
		led->id = id;
		led->label =
			of_get_property(child_node, "label", NULL) ? :
							child_node->name;

		led->pwm_dev =
			devm_of_pwm_get(chip->dev, child_node, NULL);
		if (IS_ERR(led->pwm_dev)) {
			rc = PTR_ERR(led->pwm_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev, "Get pwm device for %s led failed, rc=%d\n",
							led->label, rc);
			return rc;
		}

		pwm_get_args(led->pwm_dev, &pargs);
		if (pargs.period == 0)
			led->pwm_setting.pre_period_ns = PWM_PERIOD_DEFAULT_NS;
		else
			led->pwm_setting.pre_period_ns = pargs.period;

		led->default_trigger = of_get_property(child_node,
				"linux,default-trigger", NULL);

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00038 */
		qpnp_tri_led_calib_read_clrvari_param(led);
#endif /* CONFIG_LEDS_SHARP */
	}

	return rc;
}

static int qpnp_tri_led_probe(struct platform_device *pdev)
{
	struct qpnp_tri_led_chip *chip;
	int rc = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "Getting regmap failed\n");
		return -EINVAL;
	}

	rc = qpnp_tri_led_parse_dt(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Devicetree properties parsing failed, rc=%d\n",
								rc);
		return rc;
	}

	mutex_init(&chip->bus_lock);

	rc = qpnp_tri_led_hw_init(chip);
	if (rc) {
		dev_err(chip->dev, "HW initialization failed, rc=%d\n", rc);
		goto destroy;
	}

	dev_set_drvdata(chip->dev, chip);
	rc = qpnp_tri_led_register(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Registering LED class devices failed, rc=%d\n",
								rc);
		goto destroy;
	}

	dev_dbg(chip->dev, "Tri-led module with subtype 0x%x is detected\n",
					chip->subtype);
	return 0;
destroy:
	mutex_destroy(&chip->bus_lock);
	dev_set_drvdata(chip->dev, NULL);

	return rc;
}

static int qpnp_tri_led_remove(struct platform_device *pdev)
{
	int i;
	struct qpnp_tri_led_chip *chip = dev_get_drvdata(&pdev->dev);

	mutex_destroy(&chip->bus_lock);
	for (i = 0; i < chip->num_leds; i++) {
		sysfs_remove_files(&chip->leds[i].cdev.dev->kobj, breath_attrs);
		mutex_destroy(&chip->leds[i].lock);
	}
	dev_set_drvdata(chip->dev, NULL);
	return 0;
}

static const struct of_device_id qpnp_tri_led_of_match[] = {
	{ .compatible = "qcom,tri-led",},
	{ },
};

static struct platform_driver qpnp_tri_led_driver = {
	.driver		= {
		.name		= "qcom,tri-led",
		.of_match_table	= qpnp_tri_led_of_match,
	},
	.probe		= qpnp_tri_led_probe,
	.remove		= qpnp_tri_led_remove,
};
module_platform_driver(qpnp_tri_led_driver);

MODULE_DESCRIPTION("QTI TRI_LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:qpnp-tri-led");
