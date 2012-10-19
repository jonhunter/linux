/*
 * ARM Cross Trigger Interface (CTI) Driver
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *  Jon Hunter <jon-hunter@ti.com>
 *
 * Based upon CTI Helpers by Ming Lei <ming.lei@canonical.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/hardware/coresight.h>

#include <linux/amba/bus.h>
#include <linux/amba/cti.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

/* The registers' definition is from section 3.2 of
 * Embedded Cross Trigger Revision: r0p0
 */
#define		CTICONTROL		0x000
#define		CTISTATUS		0x004
#define		CTILOCK			0x008
#define		CTIPROTECTION		0x00C
#define		CTIINTACK		0x010
#define		CTIAPPSET		0x014
#define		CTIAPPCLEAR		0x018
#define		CTIAPPPULSE		0x01c
#define		CTIINEN			0x020
#define		CTIOUTEN		0x0A0
#define		CTITRIGINSTATUS		0x130
#define		CTITRIGOUTSTATUS	0x134
#define		CTICHINSTATUS		0x138
#define		CTICHOUTSTATUS		0x13c
#define		CTIPERIPHID0		0xFE0
#define		CTIPERIPHID1		0xFE4
#define		CTIPERIPHID2		0xFE8
#define		CTIPERIPHID3		0xFEC
#define		CTIPCELLID0		0xFF0
#define		CTIPCELLID1		0xFF4
#define		CTIPCELLID2		0xFF8
#define		CTIPCELLID3		0xFFC
#define		CTI_MAX_CHANNELS	15
#define		CTI_MAX_TRIGGERS	7

#define cti_writel(v, c, x) (__raw_writel((v), (c)->base + (x)))
#define cti_readl(c, x) (__raw_readl((c)->base + (x)))

static DEFINE_SPINLOCK(cti_lock);
static LIST_HEAD(cti_list);

/**
 * cti_map_trigger - use the @chan to map @trig_in to @trig_out
 * @cti:	CTI instance
 * @trig_in:	trigger in number
 * @trig_out:	trigger out number
 * @chan:	channel number
 *
 * Maps one trigger in of @trig_in to one trigger out of @trig_out
 * using the channel @chan. The CTI module must not be enabled when
 * calling this function.
 */
int cti_map_trigger(struct cti *cti, int trig_in, int trig_out, int chan)
{
	u32 v;

	if (!cti)
		return -EINVAL;

	if (cti->enabled)
		return -EBUSY;

	if (chan > CTI_MAX_CHANNELS)
		return -EINVAL;

	if ((trig_in > CTI_MAX_TRIGGERS) || (trig_out > CTI_MAX_TRIGGERS))
		return -EINVAL;

	coresight_unlock(cti->base);

	v = cti_readl(cti, CTIINEN + trig_in * 4);
	v |= BIT(chan);
	cti_writel(v, cti, CTIINEN + trig_in * 4);
	v = cti_readl(cti, CTIOUTEN + trig_out * 4);
	v |= BIT(chan);
	cti_writel(v, cti, CTIOUTEN + trig_out * 4);
	cti->trig_out = trig_out;

	coresight_lock(cti->base);

	return 0;
}

/**
 * cti_enable - enable the CTI module
 * @cti: CTI instance
 *
 * Unlocks and enables the CTI module. The CTI module cannot be
 * programmed again until it has been disabled.
 */
int cti_enable(struct cti *cti)
{
	if (!cti || cti->enabled)
			return -EINVAL;

	coresight_unlock(cti->base);
	cti_writel(1, cti, CTICONTROL);
	cti->enabled = true;

	return 0;
}

/**
 * cti_disable - disable the CTI module
 * @cti: CTI instance
 *
 * Disables and locks the CTI module.
 */
int cti_disable(struct cti *cti)
{
	if (!cti || !cti->enabled)
		return -EINVAL;

	cti_writel(0, cti, CTICONTROL);
	cti->enabled = false;
	coresight_lock(cti->base);

	return 0;
}

/**
 * cti_irq_ack - acknowledges the CTI trigger output
 * @cti: CTI instance
 *
 * Acknowledges the CTI trigger output by writting to the appropriate
 * bit in the CTI interrupt acknowledge register.
 */
int cti_irq_ack(struct cti *cti)
{
	u32 v;

	if (!cti || !cti->enabled)
		return -EINVAL;

	v = cti_readl(cti, CTIINTACK);
	v |= BIT(cti->trig_out);
	cti_writel(v, cti, CTIINTACK);

	return 0;
}

/**
 * cti_get - acquire a CTI module
 * @name: name of CTI instance
 *
 * Acquires a CTI module from a list of CTI modules by name. If the CTI
 * module is already in use then return NULL, otherwise return a valid
 * handle to the CTI module.
 */
struct cti *cti_get(const char *name)
{
	struct cti *cti = NULL;
	unsigned long flags;

	if (!name)
		return NULL;

	spin_lock_irqsave(&cti_lock, flags);

	if (list_empty(&cti_list))
		goto out;

	list_for_each_entry(cti, &cti_list, node) {
		if (!strcmp(cti->name, name) && (!cti->reserved)) {
			cti->reserved = true;
			goto out;
		}
	}

out:
	spin_unlock_irqrestore(&cti_lock, flags);

	if (cti)
		pm_runtime_get_sync(cti->dev);

	return cti;
}

/**
 * cti_put - release handle to CTI module
 * @cti: CTI instance
 *
 * Releases a handle to CTI module that was previously acquired.
 */
void cti_put(struct cti *cti)
{
	if (!cti || !cti->reserved)
		return;

	cti->reserved = false;

	pm_runtime_put(cti->dev);
}

static int cti_probe(struct amba_device *dev, const struct amba_id *id)
{
	struct cti *cti;
	struct device_node *np = dev->dev.of_node;
	int rc;

	if (!np) {
		dev_err(&dev->dev, "device-tree not found!\n");
		return -ENODEV;
	}

	cti = devm_kzalloc(&dev->dev, sizeof(struct cti), GFP_KERNEL);
	if (!cti) {
		dev_err(&dev->dev, "memory allocation failed!\n");
		return -ENOMEM;
	}

	rc = of_property_read_string_index(np, "arm,cti-name", 0, &cti->name);
	if (rc) {
		dev_err(&dev->dev, "no name found for CTI!\n");
		return rc;
	}

	if (!dev->irq[0]) {
		dev_err(&dev->dev, "no CTI interrupt found!\n");
		return -ENODEV;
	}

	cti->irq = dev->irq[0];
	cti->base = of_iomap(np, 0);
	if (!cti->base) {
		dev_err(&dev->dev, "unable to map CTI registers!\n");
		return -ENOMEM;
	}

	cti->dev = &dev->dev;
	amba_set_drvdata(dev, cti);
	list_add_tail(&cti->node, &cti_list);

	/*
	 * AMBA bus driver has already enabled RPM and incremented
	 * use-count, so now we can safely decrement the use-count
	 * and allow the CTI driver to manage RPM for the device.
	 */
	pm_runtime_put(&dev->dev);

	dev_info(&dev->dev, "ARM CTI driver");

	return 0;
}

static const struct amba_id cti_ids[] = {
	{
		.id	= 0x003bb906,
		.mask	= 0x00ffffff,
	},
	{ 0, 0 },
};

static struct amba_driver cti_driver = {
	.drv		= {
		.name	= "cti",
	},
	.id_table	= cti_ids,
	.probe		= cti_probe,
};

static int __init cti_init(void)
{
	return amba_driver_register(&cti_driver);
}
subsys_initcall(cti_init);
