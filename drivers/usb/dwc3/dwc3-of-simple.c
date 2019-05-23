/**
 * dwc3-of-simple.c - OF glue layer for simple integrations
 *
 * Copyright (c) 2015 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Felipe Balbi <balbi@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This is a combination of the old dwc3-qcom.c by Ivan T. Ivanov
 * <iivanov@mm-sol.com> and the original patch adding support for Xilinx' SoC
 * by Subbaraya Sundeep Bhatta <subbaraya.sundeep.bhatta@xilinx.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

// MRFIXIT CHERRY-PICKED SPECIFIC FUNCTIONS FROM MAINLINE RESETS AND OF DRIVERS TO SUPPORT USB BACKPORTS 
#include <linux/overflow.h>
static DEFINE_MUTEX(reset_list_mutex);
static LIST_HEAD(reset_controller_list);

struct of_phandle_iterator {
	/* Common iterator information */
	const char *cells_name;
	int cell_count;
	const struct device_node *parent;

	/* List size information */
	const __be32 *list_end;
	const __be32 *phandle_end;

	/* Current position state */
	const __be32 *cur;
	uint32_t cur_count;
	phandle phandle;
	struct device_node *node;
};

int of_phandle_iterator_next(struct of_phandle_iterator *it)
{
	uint32_t count = 0;

	if (it->node) {
		of_node_put(it->node);
		it->node = NULL;
	}

	if (!it->cur || it->phandle_end >= it->list_end)
		return -ENOENT;

	it->cur = it->phandle_end;

	/* If phandle is 0, then it is an empty entry with no arguments. */
	it->phandle = be32_to_cpup(it->cur++);

	if (it->phandle) {

		/*
		 * Find the provider node and parse the #*-cells property to
		 * determine the argument length.
		 */
		it->node = of_find_node_by_phandle(it->phandle);

		if (it->cells_name) {
			if (!it->node) {
				pr_err("%pOF: could not find phandle\n",
				       it->parent);
				goto err;
			}

			if (of_property_read_u32(it->node, it->cells_name,
						 &count)) {
				pr_err("%pOF: could not get %s for %pOF\n",
				       it->parent,
				       it->cells_name,
				       it->node);
				goto err;
			}
		} else {
			count = it->cell_count;
		}

		/*
		 * Make sure that the arguments actually fit in the remaining
		 * property data length
		 */
		if (it->cur + count > it->list_end) {
			pr_err("%pOF: arguments longer than property\n",
			       it->parent);
			goto err;
		}
	}

	it->phandle_end = it->cur + count;
	it->cur_count = count;

	return 0;

err:
	if (it->node) {
		of_node_put(it->node);
		it->node = NULL;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(of_phandle_iterator_next);

int of_phandle_iterator_init(struct of_phandle_iterator *it,
		const struct device_node *np,
		const char *list_name,
		const char *cells_name,
		int cell_count)
{
	const __be32 *list;
	int size;

	memset(it, 0, sizeof(*it));

	list = of_get_property(np, list_name, &size);
	if (!list)
		return -ENOENT;

	it->cells_name = cells_name;
	it->cell_count = cell_count;
	it->parent = np;
	it->list_end = list + size / sizeof(*list);
	it->phandle_end = list;
	it->cur = list;

	return 0;
}
EXPORT_SYMBOL_GPL(of_phandle_iterator_init);


/**
 * APIs to manage an array of reset controls.
 */
/**
 * of_reset_control_get_count - Count number of resets available with a device
 *
 * @node: device node that contains 'resets'.
 *
 * Returns positive reset count on success, or error number on failure and
 * on count being zero.
 */
static int of_reset_control_get_count(struct device_node *node)
{
	int count;

	if (!node)
		return -EINVAL;

	count = of_count_phandle_with_args(node, "resets", "#reset-cells");
	if (count == 0)
		count = -ENOENT;

	return count;
}

/**
 * struct reset_controller_dev - reset controller entity that might
 *                               provide multiple reset controls
 * @ops: a pointer to device specific struct reset_control_ops
 * @owner: kernel module of the reset controller driver
 * @list: internal list of reset controller devices
 * @reset_control_head: head of internal list of requested reset controls
 * @dev: corresponding driver model device struct
 * @of_node: corresponding device tree node as phandle target
 * @of_reset_n_cells: number of cells in reset line specifiers
 * @of_xlate: translation function to translate from specifier as found in the
 *            device tree to id as given to the reset control ops
 * @nr_resets: number of reset controls in this reset controller device
 */
struct reset_controller_dev {
	const struct reset_control_ops *ops;
	struct module *owner;
	struct list_head list;
	struct list_head reset_control_head;
	struct device *dev;
	struct device_node *of_node;
	int of_reset_n_cells;
	int (*of_xlate)(struct reset_controller_dev *rcdev,
			const struct of_phandle_args *reset_spec);
	unsigned int nr_resets;
};

/**
 * struct reset_control - a reset control
 * @rcdev: a pointer to the reset controller device
 *         this reset control belongs to
 * @list: list entry for the rcdev's reset controller list
 * @id: ID of the reset controller in the reset
 *      controller device
 * @refcnt: Number of gets of this reset_control
 * @shared: Is this a shared (1), or an exclusive (0) reset_control?
 * @deassert_cnt: Number of times this reset line has been deasserted
 * @triggered_count: Number of times this reset line has been reset. Currently
 *                   only used for shared resets, which means that the value
 *                   will be either 0 or 1.
 */
struct reset_control {
	struct reset_controller_dev *rcdev;
	struct list_head list;
	unsigned int id;
	struct kref refcnt;
	bool shared;
	bool array;
	atomic_t deassert_count;
	atomic_t triggered_count;
};

static struct reset_control *__reset_control_get_internal(
				struct reset_controller_dev *rcdev,
				unsigned int index, bool shared)
{
	struct reset_control *rstc;

	lockdep_assert_held(&reset_list_mutex);

	list_for_each_entry(rstc, &rcdev->reset_control_head, list) {
		if (rstc->id == index) {
			if (WARN_ON(!rstc->shared || !shared))
				return ERR_PTR(-EBUSY);

			kref_get(&rstc->refcnt);
			return rstc;
		}
	}

	rstc = kzalloc(sizeof(*rstc), GFP_KERNEL);
	if (!rstc)
		return ERR_PTR(-ENOMEM);

	try_module_get(rcdev->owner);

	rstc->rcdev = rcdev;
	list_add(&rstc->list, &rcdev->reset_control_head);
	rstc->id = index;
	kref_init(&rstc->refcnt);
	rstc->shared = shared;

	return rstc;
}

struct reset_control *__of_reset_control_get(struct device_node *node,
				     const char *id, int index, bool shared,
				     bool optional)
{
	struct reset_control *rstc;
	struct reset_controller_dev *r, *rcdev;
	struct of_phandle_args args;
	int rstc_id;
	int ret;

	if (!node)
		return ERR_PTR(-EINVAL);

	if (id) {
		index = of_property_match_string(node,
						 "reset-names", id);
		if (index == -EILSEQ)
			return ERR_PTR(index);
		if (index < 0)
			return optional ? NULL : ERR_PTR(-ENOENT);
	}

	ret = of_parse_phandle_with_args(node, "resets", "#reset-cells",
					 index, &args);
	if (ret == -EINVAL)
		return ERR_PTR(ret);
	if (ret)
		return optional ? NULL : ERR_PTR(ret);

	mutex_lock(&reset_list_mutex);
	rcdev = NULL;
	list_for_each_entry(r, &reset_controller_list, list) {
		if (args.np == r->of_node) {
			rcdev = r;
			break;
		}
	}

	if (!rcdev) {
		rstc = ERR_PTR(-EPROBE_DEFER);
		goto out;
	}

	if (WARN_ON(args.args_count != rcdev->of_reset_n_cells)) {
		rstc = ERR_PTR(-EINVAL);
		goto out;
	}

	rstc_id = rcdev->of_xlate(rcdev, &args);
	if (rstc_id < 0) {
		rstc = ERR_PTR(rstc_id);
		goto out;
	}

	/* reset_list_mutex also protects the rcdev's reset_control list */
	rstc = __reset_control_get_internal(rcdev, rstc_id, shared);

out:
	mutex_unlock(&reset_list_mutex);
	of_node_put(args.np);

	return rstc;
}
EXPORT_SYMBOL_GPL(__of_reset_control_get);

static void __reset_control_release(struct kref *kref)
{
	struct reset_control *rstc = container_of(kref, struct reset_control,
						  refcnt);

	lockdep_assert_held(&reset_list_mutex);

	module_put(rstc->rcdev->owner);

	list_del(&rstc->list);
	kfree(rstc);
}

static void __reset_control_put_internal(struct reset_control *rstc)
{
	lockdep_assert_held(&reset_list_mutex);

	kref_put(&rstc->refcnt, __reset_control_release);
}

/**
 * struct reset_control_array - an array of reset controls
 * @base: reset control for compatibility with reset control API functions
 * @num_rstcs: number of reset controls
 * @rstc: array of reset controls
 */
struct reset_control_array {
	struct reset_control base;
	unsigned int num_rstcs;
	struct reset_control *rstc[];
};

/**
 * of_reset_control_array_get - Get a list of reset controls using
 *				device node.
 *
 * @np: device node for the device that requests the reset controls array
 * @shared: whether reset controls are shared or not
 * @optional: whether it is optional to get the reset controls
 *
 * Returns pointer to allocated reset_control_array on success or
 * error on failure
 */
struct reset_control *
of_reset_control_array_get(struct device_node *np, bool shared, bool optional)
{
	struct reset_control_array *resets;
	struct reset_control *rstc;
	int num, i;

	num = of_reset_control_get_count(np);
	if (num < 0)
		return optional ? NULL : ERR_PTR(num);

	resets = kzalloc(struct_size(resets, rstc, num), GFP_KERNEL);
	if (!resets)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < num; i++) {
		rstc = __of_reset_control_get(np, NULL, i, shared, optional);
		if (IS_ERR(rstc))
			goto err_rst;
		resets->rstc[i] = rstc;
	}
	resets->num_rstcs = num;
	resets->base.array = true;

	return &resets->base;

err_rst:
	mutex_lock(&reset_list_mutex);
	while (--i >= 0)
		__reset_control_put_internal(resets->rstc[i]);
	mutex_unlock(&reset_list_mutex);

	kfree(resets);

	return rstc;
}
EXPORT_SYMBOL_GPL(of_reset_control_array_get);









struct dwc3_of_simple {
	struct device		*dev;
	struct clk		**clks;
	int			num_clocks;
	struct reset_control	*resets;
	bool			pulse_resets;
	bool			need_reset;
};

static int dwc3_of_simple_probe(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple;
	struct device		*dev = &pdev->dev;
	struct device_node	*np = dev->of_node;

	unsigned int		count;
	int			ret;
	int			i;
	bool			shared_resets = false;

	simple = devm_kzalloc(dev, sizeof(*simple), GFP_KERNEL);
	if (!simple)
		return -ENOMEM;

	count = of_clk_get_parent_count(np);
	if (!count)
		return -ENOENT;

	simple->num_clocks = count;

	simple->clks = devm_kcalloc(dev, simple->num_clocks,
			sizeof(struct clk *), GFP_KERNEL);
	if (!simple->clks)
		return -ENOMEM;

	platform_set_drvdata(pdev, simple);
	simple->dev = dev;

	/*
	 * Some controllers need to toggle the usb3-otg reset before trying to
	 * initialize the PHY, otherwise the PHY times out.
	 */
	if (of_device_is_compatible(np, "rockchip,rk3399-dwc3"))
		simple->need_reset = true;

	simple->resets = of_reset_control_array_get(np, shared_resets, true);
	if (IS_ERR(simple->resets)) {
		ret = PTR_ERR(simple->resets);
		dev_err(dev, "failed to get device resets, err=%d\n", ret);
		return ret;
	}

	if (simple->pulse_resets) {
		ret = reset_control_reset(simple->resets);
		if (ret)
			goto err_resetc_put;
	} else {
		ret = reset_control_deassert(simple->resets);
		if (ret)
			goto err_resetc_put;
	}

	for (i = 0; i < simple->num_clocks; i++) {
		struct clk	*clk;

		clk = of_clk_get(np, i);
		if (IS_ERR(clk)) {
			while (--i >= 0) {
				clk_disable_unprepare(simple->clks[i]);
				clk_put(simple->clks[i]);
			}
			ret = PTR_ERR(clk);
			break;
		}

		ret = clk_prepare_enable(clk);
		if (ret < 0) {
			while (--i >= 0) {
				clk_disable_unprepare(simple->clks[i]);
				clk_put(simple->clks[i]);
			}
			clk_put(clk);
			break;
		}

		simple->clks[i] = clk;
	}

	if (ret)
		goto err_resetc_assert;

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret) {
		for (i = 0; i < simple->num_clocks; i++) {
			clk_disable_unprepare(simple->clks[i]);
			clk_put(simple->clks[i]);
		}

		goto err_resetc_assert;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	return 0;

err_resetc_assert:
	if (!simple->pulse_resets)
		reset_control_assert(simple->resets);

err_resetc_put:
	reset_control_put(simple->resets);
	return ret;
}

static int dwc3_of_simple_remove(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple = platform_get_drvdata(pdev);
	struct device		*dev = &pdev->dev;
	int			i;

	for (i = 0; i < simple->num_clocks; i++) {
		clk_unprepare(simple->clks[i]);
		clk_put(simple->clks[i]);
	}
	simple->num_clocks = 0;

	of_platform_depopulate(dev);

	if (!simple->pulse_resets)
		reset_control_assert(simple->resets);

	reset_control_put(simple->resets);

	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_set_suspended(dev);

	return 0;
}

#ifdef CONFIG_PM
static int dwc3_of_simple_runtime_suspend(struct device *dev)
{
	struct dwc3_of_simple	*simple = dev_get_drvdata(dev);
	int			i;

	for (i = 0; i < simple->num_clocks; i++)
		clk_disable(simple->clks[i]);

	return 0;
}

static int dwc3_of_simple_runtime_resume(struct device *dev)
{
	struct dwc3_of_simple	*simple = dev_get_drvdata(dev);
	int			ret;
	int			i;

	for (i = 0; i < simple->num_clocks; i++) {
		ret = clk_enable(simple->clks[i]);
		if (ret < 0) {
			while (--i >= 0)
				clk_disable(simple->clks[i]);
			return ret;
		}
	}

	return 0;
}

static int dwc3_of_simple_suspend(struct device *dev)
{
	struct dwc3_of_simple *simple = dev_get_drvdata(dev);

	if (simple->need_reset)
		reset_control_assert(simple->resets);

	return 0;
}

static int dwc3_of_simple_resume(struct device *dev)
{
	struct dwc3_of_simple *simple = dev_get_drvdata(dev);

	if (simple->need_reset)
		reset_control_deassert(simple->resets);

	return 0;
}
#endif

static const struct dev_pm_ops dwc3_of_simple_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_of_simple_suspend, dwc3_of_simple_resume)
	SET_RUNTIME_PM_OPS(dwc3_of_simple_runtime_suspend,
			dwc3_of_simple_runtime_resume, NULL)
};

static const struct of_device_id of_dwc3_simple_match[] = {
	{ .compatible = "qcom,dwc3" },
	{ .compatible = "xlnx,zynqmp-dwc3" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_dwc3_simple_match);

static struct platform_driver dwc3_of_simple_driver = {
	.probe		= dwc3_of_simple_probe,
	.remove		= dwc3_of_simple_remove,
	.driver		= {
		.name	= "dwc3-of-simple",
		.of_match_table = of_dwc3_simple_match,
		.pm	= &dwc3_of_simple_dev_pm_ops,
	},
};

module_platform_driver(dwc3_of_simple_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 OF Simple Glue Layer");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
