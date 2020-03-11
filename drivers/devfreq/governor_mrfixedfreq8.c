/*
 *  linux/drivers/devfreq/governor_mrfixedfreq8.c
 *
 */

#include <linux/devfreq.h>
#include <linux/module.h>
#include "governor.h"

static int devfreq_mrfixedfreq8_func(struct devfreq *df,
				    unsigned long *freq)
{
	/*
	 * target callback should be able to get floor value as
	 * said in devfreq.h
	 */
	*freq = 800000000;
	return 0;
}

static int devfreq_mrfixedfreq8_handler(struct devfreq *devfreq,
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

static struct devfreq_governor devfreq_mrfixedfreq8 = {
	.name = "mrfixedfreq8",
	.get_target_freq = devfreq_mrfixedfreq8_func,
	.event_handler = devfreq_mrfixedfreq8_handler,
};

static int __init devfreq_mrfixedfreq8_init(void)
{
	return devfreq_add_governor(&devfreq_mrfixedfreq8);
}
subsys_initcall(devfreq_mrfixedfreq8_init);

static void __exit devfreq_mrfixedfreq8_exit(void)
{
	int ret;

	ret = devfreq_remove_governor(&devfreq_mrfixedfreq8);
	if (ret)
		pr_err("%s: failed remove governor %d\n", __func__, ret);

	return;
}
module_exit(devfreq_mrfixedfreq8_exit);
MODULE_LICENSE("GPL");
