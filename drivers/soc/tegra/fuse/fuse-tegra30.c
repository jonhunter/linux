// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2022, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/nvmem-consumer.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/random.h>

#include <soc/tegra/fuse.h>

#include "fuse.h"

#define FUSE_BEGIN	0x100

/* Tegra30 and later */
#define FUSE_VENDOR_CODE	0x100
#define FUSE_FAB_CODE		0x104
#define FUSE_LOT_CODE_0		0x108
#define FUSE_LOT_CODE_1		0x10c
#define FUSE_WAFER_ID		0x110
#define FUSE_X_COORDINATE	0x114
#define FUSE_Y_COORDINATE	0x118

#define FUSE_HAS_REVISION_INFO	BIT(0)

#if defined(CONFIG_ARCH_TEGRA_3x_SOC) || \
    defined(CONFIG_ARCH_TEGRA_114_SOC) || \
    defined(CONFIG_ARCH_TEGRA_124_SOC) || \
    defined(CONFIG_ARCH_TEGRA_132_SOC) || \
    defined(CONFIG_ARCH_TEGRA_210_SOC) || \
    defined(CONFIG_ARCH_TEGRA_186_SOC) || \
    defined(CONFIG_ARCH_TEGRA_194_SOC) || \
    defined(CONFIG_ARCH_TEGRA_234_SOC) || \
    defined(CONFIG_ARCH_TEGRA_241_SOC) || \
    defined(CONFIG_ARCH_TEGRA_264_SOC)
static u32 tegra30_fuse_read_early(struct tegra_fuse *fuse, unsigned int offset)
{
	if (WARN_ON(!fuse->base))
		return 0;

	return readl_relaxed(fuse->base + FUSE_BEGIN + offset);
}

static u32 tegra30_fuse_read(struct tegra_fuse *fuse, unsigned int offset)
{
	u32 value;
	int err;

	err = pm_runtime_resume_and_get(fuse->dev);
	if (err)
		return 0;

	value = readl_relaxed(fuse->base + FUSE_BEGIN + offset);

	pm_runtime_put(fuse->dev);

	return value;
}

static void __init tegra30_fuse_init(struct tegra_fuse *fuse)
{
	fuse->read_early = tegra30_fuse_read_early;
	fuse->read = tegra30_fuse_read;

	tegra_init_revision();

	if (fuse->soc->speedo_init)
		fuse->soc->speedo_init(&tegra_sku_info);
}
#endif

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
static const struct tegra_fuse_info tegra30_fuse_info = {
	.read = tegra30_fuse_read,
	.size = 0x2a4,
	.spare = 0x144,
};

const struct tegra_fuse_soc tegra30_fuse_soc = {
	.init = tegra30_fuse_init,
	.speedo_init = tegra30_init_speedo_data,
	.info = &tegra30_fuse_info,
	.soc_attr_group = &tegra_soc_attr_group,
	.clk_suspend_on = false,
};
#endif

#ifdef CONFIG_ARCH_TEGRA_114_SOC
static const struct nvmem_cell_info tegra114_fuse_cells[] = {
	{
		.name = "tsensor-cpu1",
		.offset = 0x084,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-cpu2",
		.offset = 0x088,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-common",
		.offset = 0x08c,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-cpu0",
		.offset = 0x098,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "xusb-pad-calibration",
		.offset = 0x0f0,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-cpu3",
		.offset = 0x12c,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-gpu",
		.offset = 0x154,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-mem0",
		.offset = 0x158,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-mem1",
		.offset = 0x15c,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-pllx",
		.offset = 0x160,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	},
};

static const struct nvmem_cell_lookup tegra114_fuse_lookups[] = {
	{
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration",
		.dev_id = "7009f000.padctl",
		.con_id = "calibration",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-common",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "common",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu0",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu0",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu1",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu1",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu2",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu2",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu3",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu3",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-mem0",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "mem0",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-mem1",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "mem1",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-gpu",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "gpu",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-pllx",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "pllx",
	},
};

static const struct tegra_fuse_info tegra114_fuse_info = {
	.read = tegra30_fuse_read,
	.size = 0x2a0,
	.spare = 0x180,
};

const struct tegra_fuse_soc tegra114_fuse_soc = {
	.init = tegra30_fuse_init,
	.speedo_init = tegra114_init_speedo_data,
	.info = &tegra114_fuse_info,
	.lookups = tegra114_fuse_lookups,
	.num_lookups = ARRAY_SIZE(tegra114_fuse_lookups),
	.cells = tegra114_fuse_cells,
	.num_cells = ARRAY_SIZE(tegra114_fuse_cells),
	.soc_attr_group = &tegra_soc_attr_group,
	.clk_suspend_on = false,
};
#endif

#if defined(CONFIG_ARCH_TEGRA_124_SOC) || defined(CONFIG_ARCH_TEGRA_132_SOC)
static const struct nvmem_cell_info tegra124_fuse_cells[] = {
	{
		.name = "tsensor-cpu1",
		.offset = 0x084,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-cpu2",
		.offset = 0x088,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-cpu0",
		.offset = 0x098,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "xusb-pad-calibration",
		.offset = 0x0f0,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-cpu3",
		.offset = 0x12c,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "sata-calibration",
		.offset = 0x124,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-gpu",
		.offset = 0x154,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-mem0",
		.offset = 0x158,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-mem1",
		.offset = 0x15c,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-pllx",
		.offset = 0x160,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-common",
		.offset = 0x180,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-realignment",
		.offset = 0x1fc,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	},
};

static const struct nvmem_cell_lookup tegra124_fuse_lookups[] = {
	{
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration",
		.dev_id = "7009f000.padctl",
		.con_id = "calibration",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "sata-calibration",
		.dev_id = "70020000.sata",
		.con_id = "calibration",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-common",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "common",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-realignment",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "realignment",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu0",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu0",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu1",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu1",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu2",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu2",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu3",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu3",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-mem0",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "mem0",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-mem1",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "mem1",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-gpu",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "gpu",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-pllx",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "pllx",
	},
};

static const struct tegra_fuse_info tegra124_fuse_info = {
	.read = tegra30_fuse_read,
	.size = 0x300,
	.spare = 0x200,
};

const struct tegra_fuse_soc tegra124_fuse_soc = {
	.init = tegra30_fuse_init,
	.speedo_init = tegra124_init_speedo_data,
	.info = &tegra124_fuse_info,
	.lookups = tegra124_fuse_lookups,
	.num_lookups = ARRAY_SIZE(tegra124_fuse_lookups),
	.cells = tegra124_fuse_cells,
	.num_cells = ARRAY_SIZE(tegra124_fuse_cells),
	.soc_attr_group = &tegra_soc_attr_group,
	.clk_suspend_on = true,
};
#endif

#if defined(CONFIG_ARCH_TEGRA_210_SOC)
static const struct nvmem_cell_info tegra210_fuse_cells[] = {
	{
		.name = "tsensor-cpu1",
		.offset = 0x084,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-cpu2",
		.offset = 0x088,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-cpu0",
		.offset = 0x098,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "xusb-pad-calibration",
		.offset = 0x0f0,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-cpu3",
		.offset = 0x12c,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "sata-calibration",
		.offset = 0x124,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-gpu",
		.offset = 0x154,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-mem0",
		.offset = 0x158,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-mem1",
		.offset = 0x15c,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-pllx",
		.offset = 0x160,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "tsensor-common",
		.offset = 0x180,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "gpu-calibration",
		.offset = 0x204,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "xusb-pad-calibration-ext",
		.offset = 0x250,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	},
};

static const struct nvmem_cell_lookup tegra210_fuse_lookups[] = {
	{
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu1",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu1",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu2",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu2",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu0",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu0",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration",
		.dev_id = "7009f000.padctl",
		.con_id = "calibration",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-cpu3",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "cpu3",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "sata-calibration",
		.dev_id = "70020000.sata",
		.con_id = "calibration",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-gpu",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "gpu",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-mem0",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "mem0",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-mem1",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "mem1",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-pllx",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "pllx",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "tsensor-common",
		.dev_id = "700e2000.thermal-sensor",
		.con_id = "common",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "gpu-calibration",
		.dev_id = "57000000.gpu",
		.con_id = "calibration",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration-ext",
		.dev_id = "7009f000.padctl",
		.con_id = "calibration-ext",
	},
};

static const struct tegra_fuse_info tegra210_fuse_info = {
	.read = tegra30_fuse_read,
	.size = 0x300,
	.spare = 0x280,
};

const struct tegra_fuse_soc tegra210_fuse_soc = {
	.init = tegra30_fuse_init,
	.speedo_init = tegra210_init_speedo_data,
	.info = &tegra210_fuse_info,
	.lookups = tegra210_fuse_lookups,
	.cells = tegra210_fuse_cells,
	.num_cells = ARRAY_SIZE(tegra210_fuse_cells),
	.num_lookups = ARRAY_SIZE(tegra210_fuse_lookups),
	.soc_attr_group = &tegra_soc_attr_group,
	.clk_suspend_on = false,
};
#endif

#if defined(CONFIG_ARCH_TEGRA_186_SOC)
static const struct nvmem_cell_info tegra186_fuse_cells[] = {
	{
		.name = "xusb-pad-calibration",
		.offset = 0x0f0,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "xusb-pad-calibration-ext",
		.offset = 0x250,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	},
};

static const struct nvmem_cell_lookup tegra186_fuse_lookups[] = {
	{
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration",
		.dev_id = "3520000.padctl",
		.con_id = "calibration",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration-ext",
		.dev_id = "3520000.padctl",
		.con_id = "calibration-ext",
	},
};

static const struct nvmem_keepout tegra186_fuse_keepouts[] = {
	{ .start = 0x01c, .end = 0x0f0 },
	{ .start = 0x138, .end = 0x198 },
	{ .start = 0x1d8, .end = 0x250 },
	{ .start = 0x280, .end = 0x290 },
	{ .start = 0x340, .end = 0x344 }
};

static const struct tegra_fuse_info tegra186_fuse_info = {
	.read = tegra30_fuse_read,
	.size = 0x478,
	.spare = 0x280,
};

const struct tegra_fuse_soc tegra186_fuse_soc = {
	.init = tegra30_fuse_init,
	.info = &tegra186_fuse_info,
	.lookups = tegra186_fuse_lookups,
	.num_lookups = ARRAY_SIZE(tegra186_fuse_lookups),
	.cells = tegra186_fuse_cells,
	.num_cells = ARRAY_SIZE(tegra186_fuse_cells),
	.keepouts = tegra186_fuse_keepouts,
	.num_keepouts = ARRAY_SIZE(tegra186_fuse_keepouts),
	.soc_attr_group = &tegra_soc_attr_group,
	.clk_suspend_on = false,
};
#endif

#if defined(CONFIG_ARCH_TEGRA_194_SOC)
static const struct nvmem_cell_info tegra194_fuse_cells[] = {
	{
		.name = "xusb-pad-calibration",
		.offset = 0x0f0,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "gpu-gcplex-config-fuse",
		.offset = 0x1c8,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "xusb-pad-calibration-ext",
		.offset = 0x250,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "gpu-pdi0",
		.offset = 0x300,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "gpu-pdi1",
		.offset = 0x304,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	},
};

static const struct nvmem_cell_lookup tegra194_fuse_lookups[] = {
	{
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration",
		.dev_id = "3520000.padctl",
		.con_id = "calibration",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration-ext",
		.dev_id = "3520000.padctl",
		.con_id = "calibration-ext",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "gpu-gcplex-config-fuse",
		.dev_id = "17000000.gpu",
		.con_id = "gcplex-config-fuse",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "gpu-pdi0",
		.dev_id = "17000000.gpu",
		.con_id = "pdi0",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "gpu-pdi1",
		.dev_id = "17000000.gpu",
		.con_id = "pdi1",
	},
};

static const struct nvmem_keepout tegra194_fuse_keepouts[] = {
	{ .start = 0x01c, .end = 0x0b8 },
	{ .start = 0x12c, .end = 0x198 },
	{ .start = 0x1a0, .end = 0x1bc },
	{ .start = 0x1d8, .end = 0x250 },
	{ .start = 0x270, .end = 0x290 },
	{ .start = 0x310, .end = 0x45c }
};

static const struct tegra_fuse_info tegra194_fuse_info = {
	.read = tegra30_fuse_read,
	.size = 0x650,
	.spare = 0x280,
};

const struct tegra_fuse_soc tegra194_fuse_soc = {
	.init = tegra30_fuse_init,
	.info = &tegra194_fuse_info,
	.lookups = tegra194_fuse_lookups,
	.num_lookups = ARRAY_SIZE(tegra194_fuse_lookups),
	.cells = tegra194_fuse_cells,
	.num_cells = ARRAY_SIZE(tegra194_fuse_cells),
	.keepouts = tegra194_fuse_keepouts,
	.num_keepouts = ARRAY_SIZE(tegra194_fuse_keepouts),
	.soc_attr_group = &tegra194_soc_attr_group,
	.clk_suspend_on = false,
};
#endif

#if defined(CONFIG_ARCH_TEGRA_234_SOC)
static const struct nvmem_cell_info tegra234_fuse_cells[] = {
	{
		.name = "xusb-pad-calibration",
		.offset = 0x0f0,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	}, {
		.name = "xusb-pad-calibration-ext",
		.offset = 0x250,
		.bytes = 4,
		.bit_offset = 0,
		.nbits = 32,
	},
};

static const struct nvmem_cell_lookup tegra234_fuse_lookups[] = {
	{
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration",
		.dev_id = "3520000.padctl",
		.con_id = "calibration",
	}, {
		.nvmem_name = "fuse",
		.cell_name = "xusb-pad-calibration-ext",
		.dev_id = "3520000.padctl",
		.con_id = "calibration-ext",
	},
};

static const struct nvmem_keepout tegra234_fuse_keepouts[] = {
	{ .start = 0x01c, .end = 0x064 },
	{ .start = 0x084, .end = 0x0a0 },
	{ .start = 0x0a4, .end = 0x0c8 },
	{ .start = 0x12c, .end = 0x164 },
	{ .start = 0x16c, .end = 0x184 },
	{ .start = 0x190, .end = 0x198 },
	{ .start = 0x1a0, .end = 0x204 },
	{ .start = 0x21c, .end = 0x2f0 },
	{ .start = 0x310, .end = 0x3d8 },
	{ .start = 0x400, .end = 0x420 },
	{ .start = 0x444, .end = 0x490 },
	{ .start = 0x4bc, .end = 0x4f0 },
	{ .start = 0x4f8, .end = 0x54c },
	{ .start = 0x57c, .end = 0x7e8 },
	{ .start = 0x8d0, .end = 0x8d8 },
	{ .start = 0xacc, .end = 0xf00 }
};

static const struct tegra_fuse_info tegra234_fuse_info = {
	.read = tegra30_fuse_read,
	.size = 0xf90,
	.spare = 0x280,
};

const struct tegra_fuse_soc tegra234_fuse_soc = {
	.init = tegra30_fuse_init,
	.info = &tegra234_fuse_info,
	.lookups = tegra234_fuse_lookups,
	.num_lookups = ARRAY_SIZE(tegra234_fuse_lookups),
	.cells = tegra234_fuse_cells,
	.num_cells = ARRAY_SIZE(tegra234_fuse_cells),
	.keepouts = tegra234_fuse_keepouts,
	.num_keepouts = ARRAY_SIZE(tegra234_fuse_keepouts),
	.soc_attr_group = &tegra194_soc_attr_group,
	.clk_suspend_on = false,
};
#endif

#if defined(CONFIG_ARCH_TEGRA_241_SOC)
static const struct tegra_fuse_info tegra241_fuse_info = {
	.read = tegra30_fuse_read,
	.size = 0x16008,
	.spare = 0xcf0,
};

static const struct nvmem_keepout tegra241_fuse_keepouts[] = {
	{ .start = 0xc, .end = 0x1600c }
};

const struct tegra_fuse_soc tegra241_fuse_soc = {
	.init = tegra30_fuse_init,
	.info = &tegra241_fuse_info,
	.keepouts = tegra241_fuse_keepouts,
	.num_keepouts = ARRAY_SIZE(tegra241_fuse_keepouts),
	.soc_attr_group = &tegra194_soc_attr_group,
};
#endif

#ifdef CONFIG_ARCH_TEGRA_264_SOC
static const struct nvmem_keepout tegra264_fuse_keepouts[] = {
	{ .start = 0x0042c, .end = 0x00434 },
	{ .start = 0x00450, .end = 0x00454 },
	{ .start = 0x00594, .end = 0x0059c },
	{ .start = 0x0088c, .end = 0x00890 },
	{ .start = 0x008a0, .end = 0x008a8 },
	{ .start = 0x008dc, .end = 0x008e4 },
	{ .start = 0x009d8, .end = 0x009dc },
	{ .start = 0x00a6c, .end = 0x00a70 },
	{ .start = 0x00a74, .end = 0x00a7c },
	{ .start = 0x00af4, .end = 0x00af8 },
	{ .start = 0x00b14, .end = 0x00b20 },
	{ .start = 0x00b44, .end = 0x00b4c },
	{ .start = 0x00b50, .end = 0x00b58 },
	{ .start = 0x00b5c, .end = 0x00b64 },
	{ .start = 0x00b68, .end = 0x00b70 },
	{ .start = 0x00bcc, .end = 0x00bd0 },
	{ .start = 0x00c0c, .end = 0x00c18 },
	{ .start = 0x00d80, .end = 0x00d8c },
	{ .start = 0x00eac, .end = 0x00eb4 },
	{ .start = 0x00eb8, .end = 0x00ebc },
	{ .start = 0x00f0c, .end = 0x00f10 },
	{ .start = 0x010d0, .end = 0x02000 },
	{ .start = 0x0201c, .end = 0x10164 },
	{ .start = 0x10184, .end = 0x101a0 },
	{ .start = 0x101a4, .end = 0x1029c },
	{ .start = 0x102a0, .end = 0x102cc },
	{ .start = 0x102d0, .end = 0x10408 },
	{ .start = 0x10410, .end = 0x1065c },
	{ .start = 0x1067c, .end = 0x107b0 },
	{ .start = 0x107b4, .end = 0x11108 },
	{ .start = 0x1110c, .end = 0x11118 },
	{ .start = 0x11120, .end = 0x111b8 },
	{ .start = 0x111c4, .end = 0x111c8 },
	{ .start = 0x111e8, .end = 0x111ec },
	{ .start = 0x111f0, .end = 0x11224 },
	{ .start = 0x11228, .end = 0x11268 },
	{ .start = 0x1126c, .end = 0x112b8 },
	{ .start = 0x112bc, .end = 0x112e4 },
	{ .start = 0x112e8, .end = 0x112ec },
	{ .start = 0x112f0, .end = 0x1131c },
	{ .start = 0x11350, .end = 0x1143c },
	{ .start = 0x11440, .end = 0x114c8 },
	{ .start = 0x114d0, .end = 0x11520 },
	{ .start = 0x11530, .end = 0x11540 },
	{ .start = 0x11544, .end = 0x11568 },
	{ .start = 0x1156c, .end = 0x115a8 },
	{ .start = 0x115ac, .end = 0x116cc },
	{ .start = 0x116d0, .end = 0x116dc },
	{ .start = 0x116e0, .end = 0x117b4 },
	{ .start = 0x117bc, .end = 0x11840 },
	{ .start = 0x11844, .end = 0x118b8 },
	{ .start = 0x118bc, .end = 0x11910 },
	{ .start = 0x11914, .end = 0x11950 },
	{ .start = 0x11958, .end = 0x119a0 },
	{ .start = 0x119ac, .end = 0x11c48 },
	{ .start = 0x11c50, .end = 0x11c5c },
	{ .start = 0x11c60, .end = 0x11cf8 },
	{ .start = 0x11cfc, .end = 0x11d18 },
	{ .start = 0x11d40, .end = 0x12100 },
	{ .start = 0x12104, .end = 0x12110 },
	{ .start = 0x12114, .end = 0x121fc },
	{ .start = 0x1221c, .end = 0x12220 },
	{ .start = 0x12224, .end = 0x12244 },
	{ .start = 0x1224c, .end = 0x12400 },
	{ .start = 0x12408, .end = 0x1246c },
	{ .start = 0x12470, .end = 0x127ac },
	{ .start = 0x127b0, .end = 0x12d60 },
	{ .start = 0x12d64, .end = 0x12e94 },
	{ .start = 0x12ea4, .end = 0x13104 },
	{ .start = 0x13108, .end = 0x1310c },
	{ .start = 0x13110, .end = 0x13114 },
	{ .start = 0x13118, .end = 0x13120 },
	{ .start = 0x13164, .end = 0x13184 },
	{ .start = 0x131a0, .end = 0x131a4 },
	{ .start = 0x131b8, .end = 0x131c4 },
	{ .start = 0x131c8, .end = 0x131e8 },
	{ .start = 0x131ec, .end = 0x131f0 },
	{ .start = 0x131fc, .end = 0x1321c },
	{ .start = 0x13220, .end = 0x13228 },
	{ .start = 0x13244, .end = 0x1324c },
	{ .start = 0x13268, .end = 0x1326c },
	{ .start = 0x1329c, .end = 0x132a0 },
	{ .start = 0x132ac, .end = 0x132c0 },
	{ .start = 0x132cc, .end = 0x132d0 },
	{ .start = 0x132e4, .end = 0x132e8 },
	{ .start = 0x132ec, .end = 0x132f0 },
	{ .start = 0x1331c, .end = 0x13350 },
	{ .start = 0x133f0, .end = 0x13410 },
	{ .start = 0x1342c, .end = 0x13434 },
	{ .start = 0x1343c, .end = 0x13444 },
	{ .start = 0x13450, .end = 0x13454 },
	{ .start = 0x1346c, .end = 0x13470 },
	{ .start = 0x134ac, .end = 0x134b0 },
	{ .start = 0x134c8, .end = 0x134d0 },
	{ .start = 0x13520, .end = 0x13530 },
	{ .start = 0x13540, .end = 0x13544 },
	{ .start = 0x13568, .end = 0x13570 },
	{ .start = 0x13594, .end = 0x1359c },
	{ .start = 0x135a8, .end = 0x135ac },
	{ .start = 0x13644, .end = 0x1364c },
	{ .start = 0x1365c, .end = 0x1367c },
	{ .start = 0x136cc, .end = 0x136e0 },
	{ .start = 0x136e4, .end = 0x13708 },
	{ .start = 0x1370c, .end = 0x13720 },
	{ .start = 0x137ac, .end = 0x137bc },
	{ .start = 0x137c0, .end = 0x137c8 },
	{ .start = 0x13814, .end = 0x13818 },
	{ .start = 0x13824, .end = 0x13828 },
	{ .start = 0x1382c, .end = 0x13830 },
	{ .start = 0x13834, .end = 0x1383c },
	{ .start = 0x13840, .end = 0x13844 },
	{ .start = 0x13854, .end = 0x13858 },
	{ .start = 0x13860, .end = 0x13884 },
	{ .start = 0x1388c, .end = 0x13890 },
	{ .start = 0x138a0, .end = 0x138a8 },
	{ .start = 0x138b8, .end = 0x138bc },
	{ .start = 0x138dc, .end = 0x138e4 },
	{ .start = 0x138ec, .end = 0x138f0 },
	{ .start = 0x13900, .end = 0x13918 },
	{ .start = 0x13920, .end = 0x13928 },
	{ .start = 0x13930, .end = 0x13958 },
	{ .start = 0x139a0, .end = 0x139b0 },
	{ .start = 0x139d0, .end = 0x139dc },
	{ .start = 0x13a6c, .end = 0x13a70 },
	{ .start = 0x13a74, .end = 0x13a7c },
	{ .start = 0x13af4, .end = 0x13af8 },
	{ .start = 0x13b14, .end = 0x13b20 },
	{ .start = 0x13b44, .end = 0x13b4c },
	{ .start = 0x13b50, .end = 0x13b58 },
	{ .start = 0x13b5c, .end = 0x13b64 },
	{ .start = 0x13b68, .end = 0x13b70 },
	{ .start = 0x13bcc, .end = 0x13bd8 },
	{ .start = 0x13c0c, .end = 0x13c18 },
	{ .start = 0x13c48, .end = 0x13c50 },
	{ .start = 0x13c5c, .end = 0x13c60 },
	{ .start = 0x13cf8, .end = 0x13cfc },
	{ .start = 0x13d18, .end = 0x13d40 },
	{ .start = 0x13d60, .end = 0x13d64 },
	{ .start = 0x13d74, .end = 0x13d8c },
	{ .start = 0x13e94, .end = 0x13ea4 },
	{ .start = 0x13eac, .end = 0x13eb4 },
	{ .start = 0x13eb8, .end = 0x13ebc },
	{ .start = 0x13ef4, .end = 0x13f04 },
	{ .start = 0x13f0c, .end = 0x13f10 },
	{ .start = 0x13fb8, .end = 0x1470c },
	{ .start = 0x14720, .end = 0x147c0 },
	{ .start = 0x147c8, .end = 0x14824 },
	{ .start = 0x14828, .end = 0x1482c },
	{ .start = 0x14830, .end = 0x14834 },
	{ .start = 0x1483c, .end = 0x14870 },
	{ .start = 0x14874, .end = 0x148ec },
	{ .start = 0x148f0, .end = 0x14900 },
	{ .start = 0x14910, .end = 0x14914 },
	{ .start = 0x14918, .end = 0x149ac },
	{ .start = 0x149b0, .end = 0x14bd0 },
	{ .start = 0x14bd8, .end = 0x152bc },
	{ .start = 0x152c0, .end = 0x156d4 },
	{ .start = 0x156d8, .end = 0x15814 },
	{ .start = 0x15818, .end = 0x15860 },
	{ .start = 0x15870, .end = 0x15920 },
	{ .start = 0x15924, .end = 0x15930 },
	{ .start = 0x15940, .end = 0x159d0 },
	{ .start = 0x159d8, .end = 0x15d74 },
	{ .start = 0x15d80, .end = 0x15ef4 },
	{ .start = 0x15efc, .end = 0x162ac },
	{ .start = 0x162b8, .end = 0x166d0 },
	{ .start = 0x166d4, .end = 0x166d8 },
	{ .start = 0x166dc, .end = 0x166e4 },
	{ .start = 0x166ec, .end = 0x16854 },
	{ .start = 0x16858, .end = 0x16874 },
	{ .start = 0x16884, .end = 0x16924 },
	{ .start = 0x16928, .end = 0x16940 },
	{ .start = 0x16950, .end = 0x16efc },
	{ .start = 0x16f04, .end = 0x17000 },
	{ .start = 0x17008, .end = 0x1f000 },
	{ .start = 0x1f010, .end = 0x1f014 },
	{ .start = 0x1f024, .end = 0x1f030 },
	{ .start = 0x1f04c, .end = 0x1f050 },
};

static const struct tegra_fuse_info tegra264_fuse_info = {
	.read = tegra30_fuse_read,
	.size = 0x1f094,
};

const struct tegra_fuse_soc tegra264_fuse_soc = {
	.init = tegra30_fuse_init,
	.info = &tegra264_fuse_info,
	.keepouts = tegra264_fuse_keepouts,
	.num_keepouts = ARRAY_SIZE(tegra264_fuse_keepouts),
	.soc_attr_group = &tegra194_soc_attr_group,
	.clk_suspend_on = false,
};
#endif
