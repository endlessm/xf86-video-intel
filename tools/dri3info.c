/*
 * Copyright (c) 2015 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <X11/Xlib.h>
#include <X11/Xlib-xcb.h>
#include <xcb/xcb.h>
#include <xcb/dri3.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <drm.h>
#include <xf86drm.h>

static int dri3_open(Display *dpy)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	xcb_dri3_open_cookie_t cookie;
	xcb_dri3_open_reply_t *reply;

	cookie = xcb_dri3_open(c, RootWindow(dpy, DefaultScreen(dpy)), None);
	reply = xcb_dri3_open_reply(c, cookie, NULL);

	if (!reply)
		return -1;

	if (reply->nfd != 1)
		return -1;

	return xcb_dri3_open_reply_fds(c, reply)[0];
}

static void get_device_path(int fd, char *buf, int len)
{
	struct stat remote, local;
	int i;

	if (fstat(fd, &remote))
		goto out;

	for (i = 0; i < 16; i++) {
		snprintf(buf, len, "/dev/dri/card%d", i);
		if (stat(buf, &local))
			continue;

		if (local.st_mode == remote.st_mode &&
		    local.st_rdev == remote.st_rdev)
			return;

		snprintf(buf, len, "/dev/dri/renderD%d", i + 128);
		if (stat(buf, &local))
			continue;

		if (local.st_mode == remote.st_mode &&
		    local.st_rdev == remote.st_rdev)
			return;
	}

out:
	strncpy(buf, "unknown path", len);
}

static void get_driver_name(int fd, char *name, int len)
{
	drm_version_t version;

	memset(name, 0, len);
	memset(&version, 0, sizeof(version));
	version.name_len = len;
	version.name = name;

	(void)drmIoctl(fd, DRM_IOCTL_VERSION, &version);
}

static void info(const char *dpyname)
{
	Display *dpy;
	int device;
	char device_path[1024];
	char driver_name[1024];

	dpy = XOpenDisplay(dpyname);
	if (dpy == NULL) {
		printf("Unable to connect to display '%s'\n",
		       dpyname ?: getenv("DISPLAY") ?: "unset");
		return;
	}

	device = dri3_open(dpy);
	if (device < 0) {
		printf("Unable to connect to DRI3 on display '%s'\n",
		       DisplayString(dpy));
		return;
	}

	get_device_path(device, device_path, sizeof(device_path));
	get_driver_name(device, driver_name, sizeof(driver_name));

	printf("Connected to DRI3 on display '%s', using fd %d: matches %s, driver %s\n",
	       DisplayString(dpy), device, device_path, driver_name);

	XCloseDisplay(dpy);
	close(device);
}

int main(int argc, char **argv)
{
	int i;

	if (argc > 1) {
		for (i = 1; i < argc; i++)
			info(argv[i]);
	} else
		info(NULL);

	return 0;
}
