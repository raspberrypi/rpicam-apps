#
# Regular cron jobs for the libcamera-apps package
#
0 4	* * *	root	[ -x /usr/bin/libcamera-apps_maintenance ] && /usr/bin/libcamera-apps_maintenance
