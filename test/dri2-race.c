#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/Xfixes.h>
#include <X11/Xlib-xcb.h>
#include <xcb/xcb.h>
#include <xcb/xcbext.h>
#include <xcb/dri2.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

#include <xf86drm.h>
#include <drm.h>

#include "dri2.h"

#define COUNT 60

static uint32_t upper_32_bits(uint64_t val)
{
	return val >> 32;
}

static uint32_t lower_32_bits(uint64_t val)
{
	return val & 0xffffffff;
}

static int dri2_open(Display *dpy)
{
	drm_auth_t auth;
	char *driver, *device;
	int fd;

	if (!DRI2Connect(dpy, DefaultRootWindow(dpy), &driver, &device))
		return -1;

	printf ("Connecting to %s driver on %s\n", driver, device);

	fd = open(device, O_RDWR);
	if (fd < 0)
		return -1;

	if (drmIoctl(fd, DRM_IOCTL_GET_MAGIC, &auth))
		return -1;

	if (!DRI2Authenticate(dpy, DefaultRootWindow(dpy), auth.magic))
		return -1;

	return fd;
}

static void swap_buffers(Display *dpy, Window win, int divisor,
			 unsigned int *attachments, int nattachments)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	unsigned int seq[2];

	seq[0] = xcb_dri2_swap_buffers_unchecked(c, win,
						 0, 0, 0, divisor, 0, 0).sequence;


	seq[1] = xcb_dri2_get_buffers_unchecked(c, win,
						nattachments, nattachments,
						attachments).sequence;

	xcb_flush(c);
	xcb_discard_reply(c, seq[0]);
	xcb_discard_reply(c, seq[1]);
}

static void race_window(Display *dpy, int width, int height,
			unsigned int *attachments, int nattachments,
			const char *name)
{
	Window win;
	XSetWindowAttributes attr;
	int count, loop;
	DRI2Buffer *buffers;

	printf("%s(%s)\n", __func__, name);

	/* Be nasty and install a fullscreen window on top so that we
	 * can guarantee we do not get clipped by children.
	 */
	attr.override_redirect = 1;
	loop = 100;
	do {
		win = XCreateWindow(dpy, DefaultRootWindow(dpy),
				    0, 0, width, height, 0,
				    DefaultDepth(dpy, DefaultScreen(dpy)),
				    InputOutput,
				    DefaultVisual(dpy, DefaultScreen(dpy)),
				    CWOverrideRedirect, &attr);
		XMapWindow(dpy, win);

		DRI2CreateDrawable(dpy, win);

		buffers = DRI2GetBuffers(dpy, win, &width, &height,
					 attachments, nattachments, &count);
		if (count != nattachments)
			return;

		free(buffers);
		for (count = 0; count < loop; count++)
			DRI2SwapBuffers(dpy, win, 0, 0, 0);
		XDestroyWindow(dpy, win);
		printf("."); fflush(stdout);
	} while (--loop);
	printf("*\n");

	loop = 100;
	do {
		win = XCreateWindow(dpy, DefaultRootWindow(dpy),
				    0, 0, width, height, 0,
				    DefaultDepth(dpy, DefaultScreen(dpy)),
				    InputOutput,
				    DefaultVisual(dpy, DefaultScreen(dpy)),
				    CWOverrideRedirect, &attr);
		XMapWindow(dpy, win);

		DRI2CreateDrawable(dpy, win);

		buffers = DRI2GetBuffers(dpy, win, &width, &height,
					 attachments, nattachments, &count);
		if (count != nattachments)
			return;

		free(buffers);
		for (count = 0; count < loop; count++)
			DRI2SwapBuffers(dpy, win, 0, 1, 0);
		XDestroyWindow(dpy, win);
		printf("."); fflush(stdout);
	} while (--loop);
	printf("*\n");

	loop = 100;
	do {
		win = XCreateWindow(dpy, DefaultRootWindow(dpy),
				    0, 0, width, height, 0,
				    DefaultDepth(dpy, DefaultScreen(dpy)),
				    InputOutput,
				    DefaultVisual(dpy, DefaultScreen(dpy)),
				    CWOverrideRedirect, &attr);
		XMapWindow(dpy, win);

		DRI2CreateDrawable(dpy, win);

		buffers = DRI2GetBuffers(dpy, win, &width, &height,
					 attachments, nattachments, &count);
		if (count != nattachments)
			return;

		free(buffers);
		for (count = 0; count < loop; count++)
			swap_buffers(dpy, win, 0, attachments, nattachments);
		XDestroyWindow(dpy, win);
		printf("."); fflush(stdout);
	} while (--loop);
	printf("*\n");

	loop = 100;
	do {
		win = XCreateWindow(dpy, DefaultRootWindow(dpy),
				    0, 0, width, height, 0,
				    DefaultDepth(dpy, DefaultScreen(dpy)),
				    InputOutput,
				    DefaultVisual(dpy, DefaultScreen(dpy)),
				    CWOverrideRedirect, &attr);
		XMapWindow(dpy, win);

		DRI2CreateDrawable(dpy, win);

		buffers = DRI2GetBuffers(dpy, win, &width, &height,
					 attachments, nattachments, &count);
		if (count != nattachments)
			return;

		free(buffers);
		for (count = 0; count < loop; count++)
			swap_buffers(dpy, win, 1, attachments, nattachments);
		XDestroyWindow(dpy, win);
		printf("."); fflush(stdout);
	} while (--loop);
	printf("*\n");

	loop = 100;
	do {
		uint64_t ignore, msc;
		xcb_connection_t *c = XGetXCBConnection(dpy);

		win = XCreateWindow(dpy, DefaultRootWindow(dpy),
				    0, 0, width, height, 0,
				    DefaultDepth(dpy, DefaultScreen(dpy)),
				    InputOutput,
				    DefaultVisual(dpy, DefaultScreen(dpy)),
				    CWOverrideRedirect, &attr);
		XMapWindow(dpy, win);

		DRI2CreateDrawable(dpy, win);
		DRI2GetMSC(dpy, win, &ignore, &msc, &ignore);
		for (count = 0; count < loop; count++)
			xcb_discard_reply(c,
					  xcb_dri2_wait_msc(c, win,
							    upper_32_bits(msc + count + 1),
							    lower_32_bits(msc + count + 1),
							    0, 1, 0, 0).sequence);
		XFlush(dpy);
		XDestroyWindow(dpy, win);
		printf("."); fflush(stdout);
	} while (--loop);
	printf("*\n");

	XSync(dpy, 1);
	sleep(2);
	XSync(dpy, 1);
}

static void race_client(int width, int height,
			unsigned int *attachments, int nattachments,
			const char *name)
{
	XSetWindowAttributes attr;
	int count, loop;

	printf("%s(%s)\n", __func__, name);

	/* Be nasty and install a fullscreen window on top so that we
	 * can guarantee we do not get clipped by children.
	 */
	attr.override_redirect = 1;
	loop = 100;
	do {
		Display *dpy = XOpenDisplay(NULL);
		Window win = XCreateWindow(dpy, DefaultRootWindow(dpy),
					   0, 0, width, height, 0,
					   DefaultDepth(dpy, DefaultScreen(dpy)),
					   InputOutput,
					   DefaultVisual(dpy, DefaultScreen(dpy)),
					   CWOverrideRedirect, &attr);

		XMapWindow(dpy, win);

		DRI2CreateDrawable(dpy, win);
		free(DRI2GetBuffers(dpy, win, &width, &height,
				    attachments, nattachments, &count));
		if (count != nattachments)
			return;

		for (count = 0; count < loop; count++)
			DRI2SwapBuffers(dpy, win, 0, 0, 0);
		XCloseDisplay(dpy);
		printf("."); fflush(stdout);
	} while (--loop);
	printf("*\n");

	loop = 100;
	do {
		Display *dpy = XOpenDisplay(NULL);
		Window win = XCreateWindow(dpy, DefaultRootWindow(dpy),
					   0, 0, width, height, 0,
					   DefaultDepth(dpy, DefaultScreen(dpy)),
					   InputOutput,
					   DefaultVisual(dpy, DefaultScreen(dpy)),
					   CWOverrideRedirect, &attr);

		XMapWindow(dpy, win);

		DRI2CreateDrawable(dpy, win);
		free(DRI2GetBuffers(dpy, win, &width, &height,
				    attachments, nattachments, &count));
		if (count != nattachments)
			return;

		for (count = 0; count < loop; count++)
			swap_buffers(dpy, win, 0, attachments, nattachments);
		XCloseDisplay(dpy);
		printf("."); fflush(stdout);
	} while (--loop);
	printf("*\n");

	loop = 100;
	do {
		Display *dpy = XOpenDisplay(NULL);
		Window win = XCreateWindow(dpy, DefaultRootWindow(dpy),
					   0, 0, width, height, 0,
					   DefaultDepth(dpy, DefaultScreen(dpy)),
					   InputOutput,
					   DefaultVisual(dpy, DefaultScreen(dpy)),
					   CWOverrideRedirect, &attr);

		XMapWindow(dpy, win);

		DRI2CreateDrawable(dpy, win);
		free(DRI2GetBuffers(dpy, win, &width, &height,
				    attachments, nattachments, &count));
		if (count != nattachments)
			return;

		for (count = 0; count < loop; count++)
			swap_buffers(dpy, win, 1, attachments, nattachments);
		XCloseDisplay(dpy);
		printf("."); fflush(stdout);
	} while (--loop);
	printf("*\n");

	loop = 100;
	do {
		uint64_t ignore, msc;
		Display *dpy = XOpenDisplay(NULL);
		xcb_connection_t *c = XGetXCBConnection(dpy);
		Window win = XCreateWindow(dpy, DefaultRootWindow(dpy),
					   0, 0, width, height, 0,
					   DefaultDepth(dpy, DefaultScreen(dpy)),
					   InputOutput,
					   DefaultVisual(dpy, DefaultScreen(dpy)),
					   CWOverrideRedirect, &attr);

		XMapWindow(dpy, win);

		DRI2CreateDrawable(dpy, win);
		DRI2GetMSC(dpy, win, &ignore, &msc, &ignore);
		for (count = 0; count < loop; count++)
			xcb_discard_reply(c,
					  xcb_dri2_wait_msc(c, win,
							    upper_32_bits(msc + count + 1),
							    lower_32_bits(msc + count + 1),
							    0, 1, 0, 0).sequence);
		XFlush(dpy);
		XCloseDisplay(dpy);
		printf("."); fflush(stdout);
	} while (--loop);
	printf("*\n");
}

int main(void)
{
	Display *dpy;
	int width, height, fd;
	unsigned int attachments[] = {
		DRI2BufferBackLeft,
		DRI2BufferFrontLeft,
	};

	dpy = XOpenDisplay (NULL);
	if (dpy == NULL)
		return 77;

	fd = dri2_open(dpy);
	if (fd < 0)
		return 1;

	width = WidthOfScreen(DefaultScreenOfDisplay(dpy));
	height = HeightOfScreen(DefaultScreenOfDisplay(dpy));
	race_window(dpy, width, height, attachments, 1, "fullscreen");
	race_window(dpy, width, height, attachments, 2, "fullscreen (with front)");
	race_client(width, height, attachments, 1, "fullscreen");
	race_client(width, height, attachments, 2, "fullscreen (with front)");

	width /= 2;
	height /= 2;
	race_window(dpy, width, height, attachments, 1, "windowed");
	race_window(dpy, width, height, attachments, 2, "windowed (with front)");
	race_client(width, height, attachments, 1, "windowed");
	race_client(width, height, attachments, 2, "windowed (with front)");

	return 0;
}
