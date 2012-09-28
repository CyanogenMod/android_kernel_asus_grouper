#!/tmp/busybox sh
cd /tmp
mkdir ramdisk
cd ramdisk
/tmp/busybox gzip -dc ../boot.img-ramdisk.gz | /tmp/busybox cpio -i
if [ -z `/tmp/busybox grep init.d init.rc` ]; then
echo '' >>init.rc
echo '# Execute files in /etc/init.d before booting' >>init.rc
echo 'service userinit /system/xbin/busybox run-parts /system/etc/init.d' >>init.rc
echo '    oneshot' >>init.rc
echo '    class late_start' >>init.rc
echo '    user root' >>init.rc
echo '    group root' >>init.rc
fi
if [ -f charger ]; then
/tmp/busybox chmod +x charger
fi
#/tmp/busybox sed -i 's/persist.sys.usb.config=adb/persist.sys.usb.config=mass_storage,adb/g' default.prop
#/tmp/busybox sed -i 's/persist.sys.usb.config=adb/persist.sys.usb.config=mass_storage/g' default.prop
/tmp/busybox sh -c "/tmp/busybox find . | /tmp/busybox cpio -o -H newc | /tmp/busybox gzip > ../boot.img-ramdisk-new.gz"
cd ../
