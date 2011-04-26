/*
 * drivers/base/power/loss.c - Power loss management related functions
 *
 * Copyright (C) 2011 Davide Ciminaghi <ciminaghi@gnudd.com>
 * Copyright (C) 2011 BTicino S.p.A.
 *
 * This file is released under the GPLv2.
 */

#ifdef CONFIG_PM_LOSS_DEBUG
#define DEBUG
#endif

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/pm_loss.h>
#include "../base.h"

#define LOSS_STR "pm_loss"

#define xstr(s) str(s)
#define str(s) #s
#define MAX_POLICY_NAME_LENGHT 20

struct power_loss_state_struct {
	spinlock_t lock;
	enum sys_power_state state;
	struct pm_loss_policy *active_policy;
	wait_queue_head_t wq;
	struct kobject *loss_kobj;
	struct kset *policies;
};

static struct power_loss_state_struct power_state;


void pm_loss_power_changed(enum sys_power_state s)
{
	pr_debug_pm_loss("pm_loss_power_changed, "
			 "power_state.active_policy = %p\n",
			 power_state.active_policy);
	if (!power_state.active_policy ||
	    !power_state.active_policy->ops ||
	    !power_state.active_policy->ops->power_changed) {
		pr_debug_pm_loss("pm_loss_power_changed with no active "
				 "policy or "
				 "policy with no sys_power_changed cb\n");
		return ;
	}
	pr_debug_pm_loss("pm_loss_power_changed(%d)\n", s);
	spin_lock_irq(&power_state.lock);
	if (power_state.state == s) {
		spin_unlock_irq(&power_state.lock);
		return ;
	}
	if (power_state.state == SYS_PWR_NOTIFYING) {
		DEFINE_WAIT(wait);

		for (;;) {
			prepare_to_wait(&power_state.wq, &wait,
					TASK_UNINTERRUPTIBLE);
			if (power_state.state != SYS_PWR_NOTIFYING)
				break;
			spin_unlock_irq(&power_state.lock);
			schedule();
			spin_lock_irq(&power_state.lock);
		}
	}
	power_state.state = SYS_PWR_NOTIFYING ;
	spin_unlock_irq(&power_state.lock);
	if (power_state.active_policy->ops->power_changed) {
		pr_debug_pm_loss("invoking sys_power_changed cb\n");
		power_state.active_policy->ops->power_changed
			(power_state.active_policy, s);
	}
	power_state.state = s;
}
EXPORT_SYMBOL(pm_loss_power_changed);

#define to_bus(obj) container_of(obj, struct bus_type_private, subsys.kobj)

static int __bus_added_removed(struct pm_loss_policy *p,
			       struct bus_type *bus, bool added)
{
	int ret = 0;
	spin_lock_irq(&power_state.lock);
	if (power_state.state == SYS_PWR_NOTIFYING) {
		DEFINE_WAIT(wait);

		for (;;) {
			prepare_to_wait(&power_state.wq, &wait,
					TASK_UNINTERRUPTIBLE);
			if (power_state.state != SYS_PWR_NOTIFYING)
				break;
			spin_unlock_irq(&power_state.lock);
			schedule();
			spin_lock_irq(&power_state.lock);
		}
	}
	if (added)
		ret = p->ops->bus_added ? p->ops->bus_added(p, bus) : 0;
	else
		ret = p->ops->bus_removed ? p->ops->bus_removed(p, bus) : 0;
	spin_unlock_irq(&power_state.lock);
	return ret;
}

static int bus_added(struct pm_loss_policy *p, struct bus_type *bus)
{
	return __bus_added_removed(p, bus, true);
}

static int bus_removed(struct pm_loss_policy *p, struct bus_type *bus)
{
	return __bus_added_removed(p, bus, false);
}

static int policy_add_busses(struct pm_loss_policy *p)
{
	struct kobject *k;
	int stat;

	if (!p || !p->ops)
		return -EINVAL;
	if (!p->ops->bus_added)
		return 0;

	pr_debug_pm_loss("policy_add_busses()\n");

	if (!bus_kset) {
		pr_debug_pm_loss("bus_kset still NULL\n");
		return 0;
	}

	spin_lock(&bus_kset->list_lock);
	list_for_each_entry(k, &bus_kset->list, entry) {
		spin_unlock(&bus_kset->list_lock);
		/* This might sleep */
		pr_debug_pm_loss("invoking bus_added()\n");
		stat = bus_added(p, to_bus(k)->bus);
		spin_lock(&bus_kset->list_lock);
	}
	spin_unlock(&bus_kset->list_lock);
	return 0;
}

static void __pm_loss_on_bus_added_removed(struct bus_type *bus, bool added)
{
	struct list_head *i;
	struct kobject *k;

	if (!power_state.policies)
		/* Not yet initialized */
		return ;

	spin_lock(&power_state.policies->list_lock);
	list_for_each(i, &power_state.policies->list) {
		k = container_of(i, struct kobject, entry);
		if (added)
			bus_added(to_policy(k), bus);
		else
			bus_removed(to_policy(k), bus);
	}
	spin_unlock(&power_state.policies->list_lock);
}

void pm_loss_on_bus_added(struct bus_type *bus)
{
	__pm_loss_on_bus_added_removed(bus, true);
}
EXPORT_SYMBOL(pm_loss_on_bus_added);

void pm_loss_on_bus_removed(struct bus_type *bus)
{
	__pm_loss_on_bus_added_removed(bus, false);
}
EXPORT_SYMBOL(pm_loss_on_bus_removed);

struct pm_loss_policy *
pm_loss_register_policy(const struct pm_loss_policy_ops *ops,
			struct kobj_type *ktype,
			const char *name, void *priv)
{
	struct pm_loss_policy *out = NULL;
	int ret ;

	if (strlen(name) > MAX_POLICY_NAME_LENGHT)
		printk(KERN_WARNING LOSS_STR "warning : name %s is too long, "
		       "will be truncated\n", name);

	out = kzalloc(sizeof(struct pm_loss_policy), GFP_KERNEL);
	if (!out)
		return NULL;
	out->ops = ops;
	out->priv = priv;
	out->kobj.kset = power_state.policies;

	if (policy_add_busses(out) < 0)
		goto err0;

	ret = kobject_init_and_add(&out->kobj, ktype, NULL, "%s", name);
	if (ret < 0)
		goto err1;

	kobject_uevent(&out->kobj, KOBJ_ADD);

	return out;

 err1:
	kobject_put(&out->kobj);
 err0:
	kfree(out);
	return NULL;
}
EXPORT_SYMBOL(pm_loss_register_policy);

int pm_loss_unregister_policy(struct pm_loss_policy *p)
{
	pr_debug_pm_loss("pm_loss_unregister_policy(%p)\n", p);
	if (power_state.active_policy == p) {
		pr_debug_pm_loss("cannot unregister policy (busy)\n");
		return -EBUSY ;
	}
	kobject_del(&p->kobj);
	kobject_put(&p->kobj);
	return 0;
}
EXPORT_SYMBOL(pm_loss_unregister_policy);

int pm_loss_set_policy(const char *name)
{
	struct kobject *o;
	struct pm_loss_policy *p ;
	spin_lock_irq(&power_state.lock);
	o = kset_find_obj(power_state.policies, name);
	if (!o) {
		spin_unlock_irq(&power_state.lock);
		return -ENODEV;
	}
	p = to_policy(o);
	if (power_state.active_policy) {
		if (power_state.active_policy->ops->stop)
			power_state.active_policy->ops->stop
				(power_state.active_policy);
	}
	power_state.active_policy = p;
	if (power_state.active_policy->ops->start)
		power_state.active_policy->ops->start
			(power_state.active_policy);
	spin_unlock_irq(&power_state.lock);
	return 0;
}
EXPORT_SYMBOL(pm_loss_set_policy);

/*
 * Default release function for pm_loss_policy object
 */
static void pm_loss_policy_release(struct kobject *kobj)
{
	struct pm_loss_policy *p;
	p = to_policy(kobj);
	pr_debug_pm_loss("pm_loss_policy_release(%p)\n", p);
	kfree(p);
}

/*
 * Dummy nop policy implementation (all null pointers)
 */
static const struct pm_loss_policy_ops nop_policy_ops = {
};

static struct sysfs_ops nop_sysfs_ops = {
	.show = NULL,
	.store = NULL,
};

static struct attribute *nop_default_attrs[] = {
	NULL,
};

static struct kobj_type nop_ktype = {
	.sysfs_ops = &nop_sysfs_ops,
	.release = pm_loss_policy_release,
	.default_attrs = nop_default_attrs,
};

/*
 * Default policy implementation
 */

struct pm_loss_default_policy_data {
	struct pm_loss_default_policy_table *table;
	struct list_head busses ;
};

struct pm_loss_default_policy_bus_data {
	struct bus_type *bus;
	int priority ;
	struct list_head list;
};

static struct pm_loss_default_policy_item *
__find_bus_by_name(const char *name,
		   struct pm_loss_default_policy_table *table)
{
	int i;
	struct pm_loss_default_policy_item *out = NULL;
	for (i = 0; i < table->nitems; i++)
		if (!strcmp(table->items[i].bus_name, name)) {
			out = &table->items[i];
			break;
		}
	return out;
}

static void __add_bus(struct pm_loss_default_policy_data *data,
		      struct pm_loss_default_policy_bus_data *new)
{
	struct pm_loss_default_policy_bus_data *bd;
	list_for_each_entry(bd, &data->busses, list)
		if (new->priority > bd->priority) {
			list_add_tail(&new->list, &bd->list);
			return;
		}
	list_add_tail(&new->list, &bd->list);
}

static int pm_loss_default_policy_bus_added(struct pm_loss_policy *p,
					   struct bus_type *bus)
{
	struct pm_loss_default_policy_item *item ;
	struct pm_loss_default_policy_data *pd = p->priv;
	struct pm_loss_default_policy_bus_data *bd =
		kzalloc(sizeof(struct pm_loss_default_policy_bus_data),
			GFP_KERNEL);

	BUG_ON(!pd);
	if (!bd)
		return -ENOMEM;
	pr_debug_pm_loss("pm_loss_default_policy_bus_added(), name = %s\n",
		 bus->name);
	item = __find_bus_by_name(bus->name, pd->table);
	if (!item)
		return 0;

	pr_debug_pm_loss("bus_found\n");

	bd->priority = item->bus_priority ;
	bd->bus = bus ;
	__add_bus(pd, bd);

	return 0;
}

static int pm_loss_default_policy_bus_removed(struct pm_loss_policy *p,
					     struct bus_type *bus)
{
	struct pm_loss_default_policy_data *pd = p->priv;
	struct pm_loss_default_policy_bus_data *bd, *tmp;

	BUG_ON(!pd);
	list_for_each_entry_safe(bd, tmp, &pd->busses, list)
		if (bd->bus == bus) {
			list_del(&bd->list);
			kfree(bd);
			break;
		}
	return 0;
}

struct bus_sys_pwr_changed_data {
	struct bus_type *bus;
	enum sys_power_state s;
};

static int __bus_sys_pwr_changed(struct device *dev, void *_d)
{
	struct bus_sys_pwr_changed_data *data =
		(struct bus_sys_pwr_changed_data *)_d;
	struct bus_type *bus = data->bus;

	BUG_ON(!dev);
	BUG_ON(!bus);

	pr_debug_pm_loss("__bus_sys_pwr_changed(%p), bus %s\n", dev,
			 bus->name);

	if (bus->pm && bus->pm->power_changed)
		bus->pm->power_changed(dev, data->s);
	return 0;
}


static void
pm_loss_default_policy_sys_power_changed(struct pm_loss_policy *p,
					enum sys_power_state s)
{
	struct pm_loss_default_policy_data *pd = p->priv;
	struct pm_loss_default_policy_bus_data *bd ;
	struct bus_sys_pwr_changed_data cb_data ;

	pr_debug_pm_loss("pm_loss_default_policy_sys_power_changed()\n");

	BUG_ON(!pd);

	list_for_each_entry(bd, &pd->busses, list) {
		struct bus_type *bus = bd->bus;
		const struct dev_pm_ops *pm = bus->pm;
		pr_debug_pm_loss("bus = %s\n", bus->name);
		if (pm && pm->power_changed) {
			cb_data.bus = bus ; cb_data.s = s;
			pr_debug_pm_loss("before bus_for_each_dev\n");
			bus_for_each_dev(bus, NULL, (void *)&cb_data,
					 __bus_sys_pwr_changed);
		}
	}
}

static int pm_loss_default_policy_start(struct pm_loss_policy *p)
{
	return 0;
}

static void pm_loss_default_policy_stop(struct pm_loss_policy *p)
{
	struct pm_loss_default_policy_data *pd = p->priv;
	if (pd->table->owner)
		module_put(pd->table->owner);
}

static const struct pm_loss_policy_ops default_policy_ops = {
	.bus_added = pm_loss_default_policy_bus_added,
	.bus_removed = pm_loss_default_policy_bus_removed,
	.power_changed = pm_loss_default_policy_sys_power_changed,
	.start = pm_loss_default_policy_start,
	.stop = pm_loss_default_policy_stop,
};

/* sysfs stuff */

struct dflt_bus_table_attribute {
	struct attribute attr;
	ssize_t (*show)(struct pm_loss_policy *,
			struct dflt_bus_table_attribute *attr,
			char *buf);
};
#define to_dflt_bus_table_attr(x) \
container_of(x, struct dflt_bus_table_attribute, attr)


static ssize_t bus_table_show(struct pm_loss_policy *p,
			      struct dflt_bus_table_attribute *attr,
			      char *buf)
{
	struct pm_loss_default_policy_data *pd = p->priv;
	struct pm_loss_default_policy_bus_data *bd ;
	char *ptr = buf;

	BUG_ON(!pd);

	list_for_each_entry(bd, &pd->busses, list) {
		struct bus_type *bus = bd->bus;
		ptr += snprintf(ptr, PAGE_SIZE - (ptr - buf), "%s:%d,",
				bus->name, bd->priority);
	}
	return ptr - buf;
}

static struct dflt_bus_table_attribute bus_table_attribute =
__ATTR_RO(bus_table);


static struct attribute *dflt_default_attrs[] = {
	&bus_table_attribute.attr,
	NULL,
};

/* dflt sysfs ops */

static ssize_t dflt_show(struct kobject *kobj, struct attribute *_attr,
			 char *buf)
{
	if (!strcmp(_attr->name, "bus_table")) {
		struct dflt_bus_table_attribute *attr =
			to_dflt_bus_table_attr(_attr);
		struct pm_loss_policy *p =
			to_policy(kobj);
		return attr->show(p, attr, buf);
	}
	return -EIO;
}

static ssize_t dflt_store(struct kobject *kobj,
			  struct attribute *attr,
			  const char *buf, size_t len)
{
	return -EIO;
}

static struct sysfs_ops dflt_sysfs_ops = {
	.show = dflt_show,
	.store = dflt_store,
};

static struct kobj_type dflt_ktype = {
	.sysfs_ops = &dflt_sysfs_ops,
	.release = pm_loss_policy_release,
	.default_attrs = dflt_default_attrs,
};

struct pm_loss_policy *
pm_loss_setup_default_policy(struct pm_loss_default_policy_table *table)
{
	struct pm_loss_policy *p ;
	struct pm_loss_default_policy_data *data ;

	if (!table)
		return NULL;

	if (table->owner && try_module_get(table->owner) < 0)
		return NULL;

	data = kzalloc(sizeof(struct pm_loss_default_policy_data), GFP_KERNEL);
	if (!data)
		return NULL;
	INIT_LIST_HEAD(&data->busses);
	data->table = table;

	p = pm_loss_register_policy(&default_policy_ops, &dflt_ktype,
				    table->name, data);
	if (!p) {
		kfree(data);
		return p;
	}
	return p;
}
EXPORT_SYMBOL(pm_loss_setup_default_policy);

int pm_loss_shutdown_default_policy(struct pm_loss_policy *p)
{
	int ret;
	pr_debug_pm_loss("pm_loss_shutdown_default_policy()\n");
	ret = pm_loss_unregister_policy(p);
	if (ret < 0)
		pr_debug_pm_loss("cannot unregister policy\n");
	return ret;
}
EXPORT_SYMBOL(pm_loss_shutdown_default_policy);

static ssize_t active_policy_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	return sprintf(buf, "%s\n",
		       kobject_name(&power_state.active_policy->kobj));
}


static ssize_t active_policy_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	char pname[MAX_POLICY_NAME_LENGHT];
	int ret;

	sscanf(buf, "%" xstr(MAX_POLICY_NAME_LENGHT) "s", pname);
	ret = pm_loss_set_policy(pname);
	return ret < 0 ? ret : count;
}

static struct kobj_attribute active_policy_attribute =
	__ATTR(active_policy, 0644, active_policy_show, active_policy_store);

#ifdef CONFIG_PM_LOSS_SIM
static ssize_t pwrfail_sim_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int n;
	sscanf(buf, "%d", &n);
	pm_loss_power_changed(n ? SYS_PWR_FAILING : SYS_PWR_GOOD);
	return count;
}


static struct kobj_attribute pwrfail_sim_attribute =
	__ATTR(pwrfail_sim, 0200, NULL, pwrfail_sim_store);
#endif


static struct attribute *loss_attrs[] = {
	&active_policy_attribute.attr,
#ifdef CONFIG_PM_LOSS_SIM
	&pwrfail_sim_attribute.attr,
#endif
	NULL,
};

static struct attribute_group loss_attr_group = {
	.attrs = loss_attrs,
};

/*
 * Init function
 */
int __init pm_loss_init(void)
{
	struct pm_loss_policy *p ;
	int ret = 0;

	pr_debug_pm_loss("pm_loss_init()\n");
	spin_lock_init(&power_state.lock);
	power_state.state = SYS_PWR_GOOD;
	init_waitqueue_head(&power_state.wq);
	power_state.active_policy = NULL;
	power_state.loss_kobj = kobject_create_and_add("loss", power_kobj);
	if (!power_state.loss_kobj) {
		ret = -ENOMEM;
		goto err0;
	}
	power_state.policies =
		kset_create_and_add("policies", NULL, power_state.loss_kobj);
	if (!power_state.policies) {
		ret = -ENOMEM;
		goto err1;
	}
	ret = sysfs_create_group(power_state.loss_kobj, &loss_attr_group);
	if (ret < 0)
		goto err2;
	pr_debug_pm_loss("invoking pm_loss_register_policy (nop)\n");
	p = pm_loss_register_policy(&nop_policy_ops, &nop_ktype, "nop", NULL);
	BUG_ON(!p);
	pr_debug_pm_loss("setting nop policy as current one\n");
	pm_loss_set_policy("nop");
	return ret;

 err2:
	kset_unregister(power_state.policies);
 err1:
	kobject_put(power_state.loss_kobj);
 err0:
	return ret;
}

core_initcall(pm_loss_init);
