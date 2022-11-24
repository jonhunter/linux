// SPDX-License-Identifier: GPL-2.0+
/*
 * P2U (PIPE to UPHY) driver for Tegra T194 SoC
 *
 * Copyright (C) 2019-2022 NVIDIA Corporation.
 *
 * Author: Vidya Sagar <vidyas@nvidia.com>
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <soc/tegra/bpmp.h>
#include <soc/tegra/bpmp-abi.h>

#define P2U_CONTROL_CMN			0x74
#define P2U_CONTROL_CMN_ENABLE_L2_EXIT_RATE_CHANGE		BIT(13)
#define P2U_CONTROL_CMN_SKP_SIZE_PROTECTION_EN			BIT(20)

#define P2U_PERIODIC_EQ_CTRL_GEN3	0xc0
#define P2U_PERIODIC_EQ_CTRL_GEN3_PERIODIC_EQ_EN		BIT(0)
#define P2U_PERIODIC_EQ_CTRL_GEN3_INIT_PRESET_EQ_TRAIN_EN	BIT(1)
#define P2U_PERIODIC_EQ_CTRL_GEN4	0xc4
#define P2U_PERIODIC_EQ_CTRL_GEN4_INIT_PRESET_EQ_TRAIN_EN	BIT(1)

#define P2U_RX_DEBOUNCE_TIME				0xa4
#define P2U_RX_DEBOUNCE_TIME_DEBOUNCE_TIMER_MASK	0xffff
#define P2U_RX_DEBOUNCE_TIME_DEBOUNCE_TIMER_VAL		160

#define P2U_DIR_SEARCH_CTRL				0xd4
#define P2U_DIR_SEARCH_CTRL_GEN4_FINE_GRAIN_SEARCH_TWICE	BIT(18)

#define P2U_RX_MARGIN_SW_INT_EN		0xe0
#define P2U_RX_MARGIN_SW_INT_EN_READINESS		BIT(0)
#define P2U_RX_MARGIN_SW_INT_EN_MARGIN_START		BIT(1)
#define P2U_RX_MARGIN_SW_INT_EN_MARGIN_CHANGE		BIT(2)
#define P2U_RX_MARGIN_SW_INT_EN_MARGIN_STOP		BIT(3)

#define P2U_RX_MARGIN_SW_INT		0xe4
#define P2U_RX_MARGIN_SW_INT_MASK			0xf
#define P2U_RX_MARGIN_SW_INT_READINESS			BIT(0)
#define P2U_RX_MARGIN_SW_INT_MARGIN_START		BIT(1)
#define P2U_RX_MARGIN_SW_INT_MARGIN_CHANGE		BIT(2)
#define P2U_RX_MARGIN_SW_INT_MARGIN_STOP		BIT(3)

#define P2U_RX_MARGIN_SW_STATUS		0xe8
#define P2U_RX_MARGIN_SW_STATUS_MARGIN_SW_READY		BIT(0)
#define P2U_RX_MARGIN_SW_STATUS_MARGIN_READY		BIT(1)
#define P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_STATUS	BIT(2)
#define P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_ERROR_STATUS	BIT(3)

#define P2U_RX_MARGIN_CTRL		0xec

#define P2U_RX_MARGIN_STATUS		0xf0
#define P2U_RX_MARGIN_STATUS_ERRORS_MASK		0xffff

enum margin_state {
	RX_MARGIN_START_CHANGE = 1,
	RX_MARGIN_STOP,
	RX_MARGIN_GET_MARGIN,
};

struct tegra_p2u_of_data {
	bool one_dir_search;
	bool lane_margin;
};

struct tegra_p2u {
	void __iomem *base;
	bool skip_sz_protection_en; /* Needed to support two retimers */
	struct tegra_p2u_of_data *of_data;
	struct device *dev;
	struct tegra_bpmp *bpmp;
	u32 id;
	atomic_t margin_state;
};

struct margin_ctrl {
	u32 en:1;
	u32 clr:1;
	u32 x:7;
	u32 y:6;
	u32 n_blks:8;
};

static inline void p2u_writel(struct tegra_p2u *phy, const u32 value,
			      const u32 reg)
{
	writel_relaxed(value, phy->base + reg);
}

static inline u32 p2u_readl(struct tegra_p2u *phy, const u32 reg)
{
	return readl_relaxed(phy->base + reg);
}

static int tegra_p2u_power_on(struct phy *x)
{
	struct tegra_p2u *phy = phy_get_drvdata(x);
	u32 val;

	if (phy->skip_sz_protection_en) {
		val = p2u_readl(phy, P2U_CONTROL_CMN);
		val |= P2U_CONTROL_CMN_SKP_SIZE_PROTECTION_EN;
		p2u_writel(phy, val, P2U_CONTROL_CMN);
	}

	val = p2u_readl(phy, P2U_PERIODIC_EQ_CTRL_GEN3);
	val &= ~P2U_PERIODIC_EQ_CTRL_GEN3_PERIODIC_EQ_EN;
	val |= P2U_PERIODIC_EQ_CTRL_GEN3_INIT_PRESET_EQ_TRAIN_EN;
	p2u_writel(phy, val, P2U_PERIODIC_EQ_CTRL_GEN3);

	val = p2u_readl(phy, P2U_PERIODIC_EQ_CTRL_GEN4);
	val |= P2U_PERIODIC_EQ_CTRL_GEN4_INIT_PRESET_EQ_TRAIN_EN;
	p2u_writel(phy, val, P2U_PERIODIC_EQ_CTRL_GEN4);

	val = p2u_readl(phy, P2U_RX_DEBOUNCE_TIME);
	val &= ~P2U_RX_DEBOUNCE_TIME_DEBOUNCE_TIMER_MASK;
	val |= P2U_RX_DEBOUNCE_TIME_DEBOUNCE_TIMER_VAL;
	p2u_writel(phy, val, P2U_RX_DEBOUNCE_TIME);

	if (phy->of_data->one_dir_search) {
		val = p2u_readl(phy, P2U_DIR_SEARCH_CTRL);
		val &= ~P2U_DIR_SEARCH_CTRL_GEN4_FINE_GRAIN_SEARCH_TWICE;
		p2u_writel(phy, val, P2U_DIR_SEARCH_CTRL);
	}

	if (phy->of_data->lane_margin && phy->bpmp) {
		p2u_writel(phy, P2U_RX_MARGIN_SW_INT_EN_READINESS |
			   P2U_RX_MARGIN_SW_INT_EN_MARGIN_START |
			   P2U_RX_MARGIN_SW_INT_EN_MARGIN_CHANGE |
			   P2U_RX_MARGIN_SW_INT_EN_MARGIN_STOP,
			   P2U_RX_MARGIN_SW_INT_EN);
	}

	return 0;
}

static int tegra_p2u_calibrate(struct phy *x)
{
	struct tegra_p2u *phy = phy_get_drvdata(x);
	u32 val;

	val = p2u_readl(phy, P2U_CONTROL_CMN);
	val |= P2U_CONTROL_CMN_ENABLE_L2_EXIT_RATE_CHANGE;
	p2u_writel(phy, val, P2U_CONTROL_CMN);

	return 0;
}

static const struct phy_ops ops = {
	.power_on = tegra_p2u_power_on,
	.calibrate = tegra_p2u_calibrate,
	.owner = THIS_MODULE,
};

static int tegra_p2u_set_margin_control(struct tegra_p2u *phy, u32 ctrl_data)
{
	struct tegra_bpmp_message msg;
	struct mrq_uphy_response resp;
	struct mrq_uphy_request req;
	struct margin_ctrl ctrl;
	int err;

	memcpy(&ctrl, &ctrl_data, sizeof(ctrl_data));

	memset(&req, 0, sizeof(req));
	memset(&resp, 0, sizeof(resp));

	req.lane = phy->id;
	req.cmd = CMD_UPHY_PCIE_LANE_MARGIN_CONTROL;
	req.uphy_set_margin_control.en = ctrl.en;
	req.uphy_set_margin_control.clr = ctrl.clr;
	req.uphy_set_margin_control.x = ctrl.x;
	req.uphy_set_margin_control.y = ctrl.y;
	req.uphy_set_margin_control.nblks = ctrl.n_blks;

	memset(&msg, 0, sizeof(msg));
	msg.mrq = MRQ_UPHY;
	msg.tx.data = &req;
	msg.tx.size = sizeof(req);
	msg.rx.data = &resp;
	msg.rx.size = sizeof(resp);

	err = tegra_bpmp_transfer(phy->bpmp, &msg);
	if (err)
		return err;
	if (msg.rx.ret)
		return -EINVAL;

	return 0;
}

static int tegra_p2u_get_margin_status(struct tegra_p2u *phy, u32 *val)
{
	struct tegra_bpmp_message msg;
	struct mrq_uphy_response resp;
	struct mrq_uphy_request req;
	int rc;

	req.lane = phy->id;
	req.cmd = CMD_UPHY_PCIE_LANE_MARGIN_STATUS;

	memset(&msg, 0, sizeof(msg));
	msg.mrq = MRQ_UPHY;
	msg.tx.data = &req;
	msg.tx.size = sizeof(req);
	msg.rx.data = &resp;
	msg.rx.size = sizeof(resp);

	rc = tegra_bpmp_transfer(phy->bpmp, &msg);
	if (rc)
		return rc;
	if (msg.rx.ret)
		return -EINVAL;

	*val = resp.uphy_get_margin_status.status;

	return 0;
}

static irqreturn_t tegra_p2u_irq_thread(int irq, void *arg)
{
	struct tegra_p2u *phy = arg;
	struct device *dev = phy->dev;
	u32 val;
	int state, ret;

	do {
		state = atomic_read(&phy->margin_state);
		switch (state) {
		case RX_MARGIN_START_CHANGE:
		case RX_MARGIN_STOP:
			/* Read margin control data and copy it to UPHY. */
			val = p2u_readl(phy, P2U_RX_MARGIN_CTRL);
			ret = tegra_p2u_set_margin_control(phy, val);
			if (ret) {
				dev_err(dev, "failed to set margin control: %d\n", ret);
				break;
			}

			p2u_writel(phy, P2U_RX_MARGIN_SW_STATUS_MARGIN_SW_READY |
				   P2U_RX_MARGIN_SW_STATUS_MARGIN_READY |
				   P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_STATUS |
				   P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_ERROR_STATUS,
				   P2U_RX_MARGIN_SW_STATUS);

			usleep_range(10, 11);

			if (state == RX_MARGIN_STOP) {
				/* Return from the loop if PCIe ctrl issues margin stop cmd. */
				p2u_writel(phy, P2U_RX_MARGIN_SW_STATUS_MARGIN_SW_READY |
					   P2U_RX_MARGIN_SW_STATUS_MARGIN_READY |
					   P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_ERROR_STATUS,
					   P2U_RX_MARGIN_SW_STATUS);

				return IRQ_HANDLED;
			}

			atomic_set(&phy->margin_state, RX_MARGIN_GET_MARGIN);
			break;

		case RX_MARGIN_GET_MARGIN:
			/*
			 * Read margin status from UPHY and program it in P2U_RX_MARGIN_STATUS
			 * register. This data will reflect in PCIe controller's margining lane
			 * status register.
			 */
			ret = tegra_p2u_get_margin_status(phy, &val);
			if (ret) {
				dev_err(dev, "failed to get margin status: %d\n", ret);
				break;
			}
			p2u_writel(phy, val & P2U_RX_MARGIN_STATUS_ERRORS_MASK,
				   P2U_RX_MARGIN_STATUS);

			p2u_writel(phy, P2U_RX_MARGIN_SW_STATUS_MARGIN_SW_READY |
				   P2U_RX_MARGIN_SW_STATUS_MARGIN_READY |
				   P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_ERROR_STATUS,
				   P2U_RX_MARGIN_SW_STATUS);

			msleep(20);
			break;

		default:
			dev_err(dev, "Invalid margin state: %d\n", state);
			return IRQ_HANDLED;
		};
	} while (1);

	return IRQ_HANDLED;
}

static irqreturn_t tegra_p2u_irq_handler(int irq, void *arg)
{
	struct tegra_p2u *phy = (struct tegra_p2u *)arg;
	u32 val = 0;
	irqreturn_t ret = IRQ_HANDLED;

	val = p2u_readl(phy, P2U_RX_MARGIN_SW_INT);
	p2u_writel(phy, val, P2U_RX_MARGIN_SW_INT);

	/*
	 * When PCIe link trains to Gen4, P2U HW generate READINESS interrupt. Set MARGIN_SW_READY
	 * and MARGIN_READY bits to enable P2U HW sample lane margin control data from PCIe
	 * controller's configuration space.
	 */
	if (val & P2U_RX_MARGIN_SW_INT_READINESS)
		p2u_writel(phy, P2U_RX_MARGIN_SW_STATUS_MARGIN_SW_READY |
			   P2U_RX_MARGIN_SW_STATUS_MARGIN_READY,
			   P2U_RX_MARGIN_SW_STATUS);

	/*
	 * P2U HW generates MARGIN_START or MARGIN_CHANGE interrupt after it copied margin control
	 * data to P2U_RX_MARGIN_CTRL register.
	 */
	if ((val & P2U_RX_MARGIN_SW_INT_MARGIN_START) ||
	    (val & P2U_RX_MARGIN_SW_INT_MARGIN_CHANGE)) {
		atomic_set(&phy->margin_state, RX_MARGIN_START_CHANGE);
		ret = IRQ_WAKE_THREAD;
	}

	/* P2U HW generates MARGIN_STOP interrupt when PCIe controller issues margin stop cmd. */
	if (val & P2U_RX_MARGIN_SW_INT_MARGIN_STOP)
		atomic_set(&phy->margin_state, RX_MARGIN_STOP);

	return ret;
}

static int tegra_p2u_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct phy *generic_phy;
	struct tegra_p2u *phy;
	int irq = -ENODEV;
	int ret;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	phy->dev = dev;
	platform_set_drvdata(pdev, phy);

	phy->of_data =
		(struct tegra_p2u_of_data *)of_device_get_match_data(dev);
	if (!phy->of_data)
		return -EINVAL;

	phy->base = devm_platform_ioremap_resource_byname(pdev, "ctl");
	if (IS_ERR(phy->base))
		return PTR_ERR(phy->base);

	phy->skip_sz_protection_en =
		of_property_read_bool(dev->of_node,
				      "nvidia,skip-sz-protect-en");

	platform_set_drvdata(pdev, phy);

	generic_phy = devm_phy_create(dev, NULL, &ops);
	if (IS_ERR(generic_phy))
		return PTR_ERR(generic_phy);

	phy_set_drvdata(generic_phy, phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	if (phy->of_data->lane_margin)
		irq = platform_get_irq(pdev, 0);

	if (irq < 0) {
		dev_warn(dev, "Device tree update required to enable lane margining\n");
	} else {
		ret = devm_request_threaded_irq(dev, irq, tegra_p2u_irq_handler,
						tegra_p2u_irq_thread, 0,
						"tegra-p2u-intr", phy);
		if (ret) {
			dev_err(dev, "failed to request intr irq\n");
			return ret;
		}

		ret = of_property_read_u32_index(dev->of_node, "nvidia,bpmp",
						 1, &phy->id);
		if (ret) {
			dev_err(dev, "failed to read P2U id: %d\n", ret);
			return ret;
		}

		phy->bpmp = tegra_bpmp_get(dev);
		if (IS_ERR(phy->bpmp))
			return PTR_ERR(phy->bpmp);
	}

	return 0;
}

static void tegra_p2u_remove(struct platform_device *pdev)
{
	struct tegra_p2u *phy = platform_get_drvdata(pdev);

	if (phy->bpmp)
		tegra_bpmp_put(phy->bpmp);
}

static const struct tegra_p2u_of_data tegra194_p2u_of_data = {
	.one_dir_search = false,
	.lane_margin = false,
};

static const struct tegra_p2u_of_data tegra234_p2u_of_data = {
	.one_dir_search = true,
	.lane_margin = true,
};

static const struct of_device_id tegra_p2u_id_table[] = {
	{
		.compatible = "nvidia,tegra194-p2u",
		.data = &tegra194_p2u_of_data,
	},
	{
		.compatible = "nvidia,tegra234-p2u",
		.data = &tegra234_p2u_of_data,
	},
	{}
};
MODULE_DEVICE_TABLE(of, tegra_p2u_id_table);

static struct platform_driver tegra_p2u_driver = {
	.probe = tegra_p2u_probe,
	.remove = tegra_p2u_remove,
	.driver = {
		.name = "tegra194-p2u",
		.of_match_table = tegra_p2u_id_table,
	},
};
module_platform_driver(tegra_p2u_driver);

MODULE_AUTHOR("Vidya Sagar <vidyas@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra194 PIPE2UPHY PHY driver");
MODULE_LICENSE("GPL v2");
