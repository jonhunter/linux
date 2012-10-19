/*
 * ARM Cross Trigger Interface Platform Driver
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *  Jon Hunter <jon-hunter@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef AMBA_CTI_H
#define AMBA_CTI_H

#include <linux/io.h>

/**
 * struct cti - Cross Trigger Interface (CTI) struct
 *
 * @node:	Connects CTI instance to list of CTI instances
 * @dev:	Pointer to device structure
 * @base:	Mapped virtual address of the CTI module
 * @name:	Name associated with CTI instance
 * @irq:	Interrupt associated with CTI instance
 * @trig_out:	Trigger output associated with interrupt (@irq)
 * @reserved:	Used to indicate if CTI instance has been allocated
 * @enabled:	Used to indicate if CTI instance has been enabled
 */
struct cti {
	struct list_head node;
	struct device *dev;
	void __iomem *base;
	const char *name;
	int irq;
	int trig_out;
	bool reserved;
	bool enabled;
};

#ifdef CONFIG_ARM_AMBA_CTI

int cti_map_trigger(struct cti *cti, int trig_in, int trig_out, int chan);
int cti_enable(struct cti *cti);
int cti_disable(struct cti *cti);
int cti_irq_ack(struct cti *cti);
struct cti *cti_get(const char *name);
void cti_put(struct cti *cti);

#else

static inline int cti_map_trigger(struct cti *cti, int trig_in, int trig_out,
				  int chan)
{
	return 0;
}

static inline int cti_enable(struct cti *cti)
{
	return 0;
}

static inline int cti_disable(struct cti *cti)
{
	return 0;
}

static inline int cti_irq_ack(struct cti *cti)
{
	return 0;
}

static inline struct cti *cti_get(const char *name)
{
	return NULL;
}

static inline void cti_put(struct cti *cti) {}

#endif /* ARM_AMBA_CTI */

#endif /* AMBA_CTI_H */
