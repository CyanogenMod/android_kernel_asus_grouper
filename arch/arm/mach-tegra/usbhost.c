#include <linux/kobject.h>
#include <linux/sysfs.h>

extern int smb347_event_fi(void);
extern int smb347_event_fastcharge(void);

// TODO: need to persist all 3 values 

/* ----------------------------------------- */
int usbhost_fixed_install_mode = 0;

static ssize_t fixed_install_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", usbhost_fixed_install_mode);
}

static ssize_t fixed_install_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%du", &usbhost_fixed_install_mode);
    // TODO: event -> smb347_charger.c
    smb347_event_fi();
    return count;
}

static struct kobj_attribute fixed_install_mode_attribute = 
    __ATTR(usbhost_fixed_install_mode, 0666, fixed_install_mode_show, fixed_install_mode_store);

/* ----------------------------------------- */
int usbhost_fastcharge_in_host_mode = 0;

static ssize_t fastcharge_in_host_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", usbhost_fastcharge_in_host_mode);
}

static ssize_t fastcharge_in_host_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%du", &usbhost_fastcharge_in_host_mode);
    // TODO: event -> smb347_charger.c
    smb347_event_fastcharge();
    return count;
}

static struct kobj_attribute fastcharge_in_host_mode_attribute = 
    __ATTR(usbhost_fastcharge_in_host_mode, 0666, fastcharge_in_host_mode_show, fastcharge_in_host_mode_store);

/* ----------------------------------------- */
int usbhost_hotplug_on_boot = 0;

static ssize_t hotplug_on_boot_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", usbhost_hotplug_on_boot);
}

static ssize_t hotplug_on_boot_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%du", &usbhost_hotplug_on_boot);
    return count;
}

static struct kobj_attribute hotplug_on_boot_attribute = 
    __ATTR(usbhost_hotplug_on_boot, 0666, hotplug_on_boot_show, hotplug_on_boot_store);




static struct attribute *attrs[] = {
    &fixed_install_mode_attribute.attr,
    &hotplug_on_boot_attribute.attr,
    &fastcharge_in_host_mode_attribute.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

static struct kobject *usbhost_kobj;

int usbhost_init(void)
{
	int retval;

	usbhost_fixed_install_mode = 1;
	usbhost_hotplug_on_boot = 0;
	usbhost_fastcharge_in_host_mode = 0;

    usbhost_kobj = kobject_create_and_add("usbhost", kernel_kobj);
    if (!usbhost_kobj) {
            return -ENOMEM;
    }
    retval = sysfs_create_group(usbhost_kobj, &attr_group);
    if (retval)
            kobject_put(usbhost_kobj);
    return retval;
}

void usbhost_exit(void)
{
	kobject_put(usbhost_kobj);
}

module_init(usbhost_init);
module_exit(usbhost_exit);

