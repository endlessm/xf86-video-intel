/***************************************************************************

 Copyright 2014 Intel Corporation.  All Rights Reserved.
 Copyright 2014 Red Hat, Inc.

 Permission is hereby granted, free of charge, to any person obtaining a
 copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sub license, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice (including the
 next paragraph) shall be included in all copies or substantial portions
 of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 IN NO EVENT SHALL INTEL, AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
 THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 **************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>

#include "backlight.h"
#include "fd.h"

/* Enough for 10 digits of backlight + '\n' + '\0' */
#define BACKLIGHT_VALUE_LEN 12

/*
 * Unfortunately this is not as simple as I would like it to be. If selinux is
 * dropping dbus messages pkexec may block *forever*.
 *
 * Backgrounding pkexec by doing System("pkexec ...&") does not work because
 * that detaches pkexec from its parent at which point its security checks
 * fail and it refuses to execute the helper.
 *
 * So we're left with spawning a helper child which gets levels to set written
 * to it through a pipe. This turns the blocking forever problem from a hung
 * machine problem into a simple backlight control not working problem.
 */

#ifdef __OpenBSD__

#include <dev/wscons/wsconsio.h>

int backlight_set(struct backlight *b, int level)
{
	struct wsdisplay_param param;

	if (b->iface == NULL)
		return;

	if ((unsigned)level > b->max)
		level = b->max;

	memset(&param, 0, sizeof(param));
	param.param = WSDISPLAYIO_PARAM_BRIGHTNESS;
	param.curval = level;

	return ioctl(xf86Info.consoleFd, WSDISPLAYIO_SETPARAM, &param);
}

int backlight_get(struct backlight *b)
{
	struct wsdisplay_param param;

	if (b->iface == NULL)
		return -1;

	memset(&param, 0, sizeof(param));
	param.param = WSDISPLAYIO_PARAM_BRIGHTNESS;

	if (ioctl(xf86Info.consoleFd, WSDISPLAYIO_GETPARAM, &param))
		return -1;

	return param.curval;
}

int backlight_open(struct backlight *b, char *iface)
{
	struct wsdisplay_param param;

	memset(&param, 0, sizeof(param));
	param.param = WSDISPLAYIO_PARAM_BRIGHTNESS;

	if (ioctl(xf86Info.consoleFd, WSDISPLAYIO_GETPARAM, &param) == -1)
		return -1;

	b->iface = strdup("wscons");
	if (b->iface == NULL)
		return -1;

	b->max = param.max;
	b->fd = -1;

	return param.curval;
}

#else

static int
is_sysfs_fd(int fd)
{
	struct stat st;
	return fstat(fd, &st) == 0 && major(st.st_dev) == 0;
}

static int
__backlight_read(const char *iface, const char *file)
{
	char buf[1024];
	int fd, val;

	snprintf(buf, sizeof(buf), "%s/%s/%s", BACKLIGHT_CLASS, iface, file);
	fd = open(buf, O_RDONLY);
	if (fd == -1)
		return -1;

	if (is_sysfs_fd(fd)) {
		val = read(fd, buf, BACKLIGHT_VALUE_LEN - 1);
		if (val > 0) {
			buf[val] = '\0';
			val = atoi(buf);
		} else
			val = -1;
	} else
		val = -1;
	close(fd);

	return val;
}

int backlight_exists(const char *iface)
{
	if (__backlight_read(iface, "brightness") < 0)
		return 0;

	if (__backlight_read(iface, "max_brightness") <= 0)
		return 0;

	return 1;
}

static int __backlight_init(struct backlight *b, char *iface, int fd)
{
	b->fd = fd_set_cloexec(fd_set_nonblock(fd));
	b->iface = iface;
	return 1;
}

static int __backlight_direct_init(struct backlight *b, char *iface)
{
	char path[1024];
	int fd;

	snprintf(path, sizeof(path), "%s/%s/brightness", BACKLIGHT_CLASS, iface);
	fd = open(path, O_RDWR);
	if (fd < 0)
		return 0;

	if (!is_sysfs_fd(fd)) {
		close(fd);
		return 0;
	}

	return __backlight_init(b, iface, fd);
}

static int __backlight_helper_init(struct backlight *b, char *iface)
{
#if USE_BACKLIGHT_HELPER
	struct stat st;
	char *env[] = { NULL };
	int use_pkexec = 0;
	int fds[2];

	/* If system policy is to disallow setuid helpers,
	 * we fallback to invoking PolicyKit. However, as pkexec
	 * is quite troublesome and not universally available, we
	 * still try the old fashioned and simple method first.
	 * Either way, we have to trust that it is our backlight-helper
	 * that is run and that we have scrutinised it carefully.
	 */
	if (stat(PREFIX_PATH "/libexec/xf86-video-intel-backlight-helper", &st))
		return 0;

	if ((st.st_mode & (S_IFREG | S_ISUID | S_IXUSR)) != (S_IFREG | S_ISUID | S_IXUSR)) {
		if (system("pkexec --version"))
			return 0;

		use_pkexec = 1;
	}

	if (pipe(fds))
		return 0;

	switch ((b->pid = fork())) {
	case 0:
		close(fds[1]);
		dup2(fds[0], 0);
		close(fds[0]);
		if (use_pkexec) {
			execlp("pkexec", "pkexec",
			       PREFIX_PATH "/libexec/xf86-video-intel-backlight-helper",
			       iface, (char *)0);
		} else {
			execle(PREFIX_PATH "/libexec/xf86-video-intel-backlight-helper",
			       "xf86-video-intel-backlight-helper",
			       iface, (char *)0, env);
		}
		_exit(1);
		/* unreachable fallthrough */
	case -1:
		close(fds[1]);
		close(fds[0]);
		return 0;

	default:
		close(fds[0]);
		return __backlight_init(b, iface, fds[1]);
	}
#else
	return 0;
#endif
}

int backlight_open(struct backlight *b, char *iface)
{
	int level;

	if (iface == NULL)
		return -1;

	b->max = __backlight_read(iface, "max_brightness");
	if (b->max <= 0)
		return -1;

	level = __backlight_read(iface, "brightness");
	if (level < 0)
		return -1;

	if (!__backlight_direct_init(b, iface) &&
	    !__backlight_helper_init(b, iface))
		return -1;

	return level;
}

int backlight_set(struct backlight *b, int level)
{
	char val[BACKLIGHT_VALUE_LEN];
	int len, ret = 0;

	if (b->iface == NULL)
		return 0;

	if ((unsigned)level > b->max)
		level = b->max;

	len = snprintf(val, BACKLIGHT_VALUE_LEN, "%d\n", level);
	if (write(b->fd, val, len) != len)
		ret = -1;

	return ret;
}

int backlight_get(struct backlight *b)
{
	int level;

	if (b->iface == NULL)
		return -1;

	level = __backlight_read(b->iface, "brightness");
	if (level > b->max)
		level = b->max;
	else if (level < 0)
		level = -1;
	return level;
}
#endif

void backlight_disable(struct backlight *b)
{
	if (b->iface == NULL)
		return;

	if (b->fd != -1)
		close(b->fd);

	free(b->iface);
	b->iface = NULL;
}

void backlight_close(struct backlight *b)
{
	backlight_disable(b);
	if (b->pid)
		waitpid(b->pid, NULL, 0);
}
