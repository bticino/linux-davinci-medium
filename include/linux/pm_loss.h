/*
 * pm_loss.h - Power loss management related functions, header file
 *
 * Copyright (C) 2011 Davide Ciminaghi <ciminaghi@gnudd.com>
 * Copyright (C) 2011 BTicino S.p.A.
 *
 * This file is released under the GPLv2.
 */

#ifndef _LINUX_PM_LOSS_H
#define _LINUX_PM_LOSS_H

#include <linux/device.h>
#include <linux/pm.h>

struct pm_loss_policy ;

struct pm_loss_policy_ops {
	int (*bus_added)(struct pm_loss_policy *,
			 struct bus_type *);
	int (*bus_removed)(struct pm_loss_policy *,
			   struct bus_type *);
	void (*power_changed)(struct pm_loss_policy *,
			      enum sys_power_state);
	int (*start)(struct pm_loss_policy *);
	void (*stop)(struct pm_loss_policy *);
};

struct pm_loss_policy {
	const struct pm_loss_policy_ops *ops;
	void *priv;
	struct list_head list;
	struct kobject kobj;
};

#define to_policy(k) container_of(k, struct pm_loss_policy, kobj)

struct pm_loss_default_policy_table {
	struct module *owner ;
	const char *name ;
	struct pm_loss_default_policy_item *items;
	int nitems;
};

#ifdef CONFIG_PM_LOSS

#ifdef CONFIG_PM_LOSS_DEBUG
#define PM_LOSS_STR "pm_loss:"
#define pr_debug_pm_loss(a, args...) \
printk(KERN_INFO PM_LOSS_STR a, ## args)
#else
#define pr_debug_pm_loss(a, args...)
#endif

extern void pm_loss_power_changed(enum sys_power_state s);

extern struct pm_loss_policy *
pm_loss_register_policy(const struct pm_loss_policy_ops *ops,
			struct kobj_type *ktype,
			const char *name, void *priv);

extern int pm_loss_unregister_policy(struct pm_loss_policy *);

extern int pm_loss_set_policy(const char *name);

struct pm_loss_default_policy_item {
	const char *bus_name;
	int bus_priority ;
};

extern struct pm_loss_policy *
pm_loss_setup_default_policy(struct pm_loss_default_policy_table *);

extern int pm_loss_shutdown_default_policy(struct pm_loss_policy *);

#else /* !CONFIG_PM_LOSS */

static inline struct pm_loss_policy *
pm_loss_register_policy(const struct pm_loss_policy_ops *ops,
		       void *priv)
{
	return NULL ;
}

static inline int pm_loss_unregister_policy(struct pm_loss_policy *p)
{
	return -ENOSYS ;
}

static inline int pm_loss_set_policy(struct pm_loss_policy *p)
{
	return -ENOSYS ;
}

static inline struct pm_loss_policy *
pm_loss_setup_default_policy(struct pm_loss_default_policy_table *t)
{
	return NULL;
}

static inline int
pm_loss_shutdown_default_policy(struct pm_loss_policy *p)
{
	return -ENOSYS;
}

#endif /* !CONFIG_PM_LOSS */

#endif /* _LINUX_PM_LOSS_H */
