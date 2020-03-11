/*
 *  linux/drivers/devfreq/governor_mrfixedfreq5.c
 *
 */

#include <linux/devfreq.h>
#include <linux/module.h>
#include "governor.h"

static int devfreq_mrfixedfreq5_func(struct devfreq *df,
				    unsigned long *freq)
{
	/*
	 * target callback should be able to get floor value as
	 * said in devfreq.h
	 */
	*freq = 500000000;
	return 0;
}

static int devfreq_mrfixedfreq5_handler(struct devfreq *devfreq,
				unsigned int event, void *data)
{
	int ret = 0;

	if (event == DEVFREQ_GOV_START) {
		mutex_lock(&devfreq->lock);
		ret = update_devfreq(devfreq);
		mutex_unlock(&devfreq->lock);
		devfreq->last_status.update = true;
	} else if (event == DEVFREQ_GOV_STOP) {
		devfreq->last_status.update = false;
	}

	return ret;
}

static struct devfreq_governor devfreq_mrfixedfreq5 = {
	.name = "mrfixedfreq5",
	.get_target_freq = devfreq_mrfixedfreq5_func,
	.event_handler = devfreq_mrfixedfreq5_handler,
};

static int __init devfreq_mrfixedfreq5_init(void)
{
	return devfreq_add_governor(&devfreq_mrfixedfreq5);
}
subsys_initcall(devfreq_mrfixedfreq5_init);

static void __exit devfreq_mrfixedfreq5_exit(void)
{
	int ret;

	ret = devfreq_remove_governor(&devfreq_mrfixedfreq5);
	if (ret)
		pr_err("%s: failed remove governor %d\n", __func__, ret);

	return;
}
module_exit(devfreq_mrfixedfreq5_exit);
MODULE_LICENSE("GPL");
