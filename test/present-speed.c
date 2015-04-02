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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xlib-xcb.h>
#include <X11/xshmfence.h>
#include <X11/Xutil.h>
#include <X11/Xlibint.h>
#include <X11/extensions/dpms.h>
#include <X11/extensions/randr.h>
#include <X11/extensions/Xrandr.h>
#include <xcb/xcb.h>
#include <xcb/present.h>
#include <xcb/dri3.h>
#include <xf86drm.h>
#include <i915_drm.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <setjmp.h>
#include <signal.h>

#include "dri3.h"

static int _x_error_occurred;
static uint32_t stamp;

static int
_check_error_handler(Display     *display,
		     XErrorEvent *event)
{
	printf("X11 error from display %s, serial=%ld, error=%d, req=%d.%d\n",
	       DisplayString(display),
	       event->serial,
	       event->error_code,
	       event->request_code,
	       event->minor_code);
	_x_error_occurred++;
	return False; /* ignored */
}

static void *setup_msc(Display *dpy, Window win)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	uint32_t id = xcb_generate_id(c);

	xcb_present_select_input(c, id, win, XCB_PRESENT_EVENT_MASK_COMPLETE_NOTIFY);
	return xcb_register_for_special_xge(c, &xcb_present_id, id, &stamp);
}

static void teardown_msc(Display *dpy, void *q)
{
	xcb_unregister_for_special_event(XGetXCBConnection(dpy), q);
}

static double elapsed(const struct timespec *start,
		      const struct timespec *end)
{
	return 1e6*(end->tv_sec - start->tv_sec) + (end->tv_nsec - start->tv_nsec)/1000;
}

static void run(Display *dpy, Window win, const char *name, int use_dri3)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	struct timespec start, end;
	Pixmap pixmap[4];
	int busy[4];
	struct dri3_fence fence[4];
	Window root;
	unsigned int width, height;
	unsigned border, depth;
	int i, j, n, back = 0;
	int completed = 0;
	void *Q;

	XGetGeometry(dpy, win,
		     &root, &i, &j, &width, &height, &border, &depth);

	_x_error_occurred = 0;

	for (n = 0; n < 4; n++) {
		pixmap[n] = XCreatePixmap(dpy, win, width, height, depth);
		if (use_dri3) {
			if (dri3_create_fence(dpy, win, &fence[n]))
				return;
			/* start idle */
			xshmfence_trigger(fence[n].addr);
		}
		busy[n] = 0;
	}

	Q = setup_msc(dpy, win);
	clock_gettime(CLOCK_MONOTONIC, &start);
	do {
		for (n = 0; n < 1000; n++) {
			Pixmap p = 0;
			for (i = 0; i < 4; i++) {
				j = (back + i) % 4;
				if (!busy[j]) {
					p = pixmap[j];
					break;
				}
			}
			if (p == 0) {
				xcb_present_complete_notify_event_t *ce;
				xcb_generic_event_t *ev;

				ev = xcb_wait_for_special_event(c, Q);
				if (ev == NULL)
					abort();

				do {
					ce = (xcb_present_complete_notify_event_t *)ev;
					if (ce->kind == XCB_PRESENT_COMPLETE_KIND_PIXMAP) {
						completed++;
						busy[ce->serial] = 0;
						if (p == 0)
							p = pixmap[j = ce->serial];
					}
					free(ev);
				} while ((ev = xcb_poll_for_special_event(c, Q)));
			}

			back = j;
			busy[back] = 1;
			if (use_dri3) {
				xshmfence_await(fence[back].addr);
				xshmfence_reset(fence[back].addr);
			}
			xcb_present_pixmap(c, win, p, back,
					   0, /* valid */
					   0, /* update */
					   0, /* x_off */
					   0, /* y_off */
					   None,
					   None, /* wait fence */
					   use_dri3 ? fence[back].xid : None,
					   XCB_PRESENT_OPTION_ASYNC,
					   0, /* target msc */
					   0, /* divisor */
					   0, /* remainder */
					   0, NULL);
			xcb_flush(c);
			back++;
		}
		clock_gettime(CLOCK_MONOTONIC, &end);
	} while (end.tv_sec < start.tv_sec + 10);

	for (n = 0; n < 4; n++) {
		if (use_dri3)
			dri3_fence_free(dpy, &fence[n]);
		XFreePixmap(dpy, pixmap[n]);
	}

	XSync(dpy, True);
	teardown_msc(dpy, Q);
	if (_x_error_occurred)
		abort();

	printf("%s%s: Completed %d presents in %.1fs, %.3fus each (%.1f FPS)\n",
	       name, use_dri3 ? " (dri3)" : "",
	       completed, elapsed(&start, &end) / 1000000,
	       elapsed(&start, &end) / completed,
	       completed / (elapsed(&start, &end) / 1000000));
}

static int has_present(Display *dpy)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	xcb_generic_error_t *error = NULL;
	void *reply;

	reply = xcb_present_query_version_reply(c,
						xcb_present_query_version(c,
									  XCB_PRESENT_MAJOR_VERSION,
									  XCB_PRESENT_MINOR_VERSION),
						&error);

	free(reply);
	free(error);
	if (reply == NULL) {
		fprintf(stderr, "Present not supported on %s\n", DisplayString(dpy));
		return 0;
	}

	return 1;
}

static int dri3_query_version(Display *dpy, int *major, int *minor)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	xcb_dri3_query_version_reply_t *reply;
	xcb_generic_error_t *error;

	*major = *minor = -1;

	reply = xcb_dri3_query_version_reply(c,
					     xcb_dri3_query_version(c,
								    XCB_DRI3_MAJOR_VERSION,
								    XCB_DRI3_MINOR_VERSION),
					     &error);
	free(error);
	if (reply == NULL)
		return -1;

	*major = reply->major_version;
	*minor = reply->minor_version;
	free(reply);

	return 0;
}

static int has_dri3(Display *dpy)
{
	const xcb_query_extension_reply_t *ext;
	int major, minor;

	ext = xcb_get_extension_data(XGetXCBConnection(dpy), &xcb_dri3_id);
	if (ext == NULL || !ext->present)
		return 0;

	if (dri3_query_version(dpy, &major, &minor) < 0)
		return 0;

	return major >= 0;
}

static inline XRRScreenResources *_XRRGetScreenResourcesCurrent(Display *dpy, Window window)
{
	XRRScreenResources *res;

	res = XRRGetScreenResourcesCurrent(dpy, window);
	if (res == NULL)
		res = XRRGetScreenResources(dpy, window);

	return res;
}

static XRRModeInfo *lookup_mode(XRRScreenResources *res, int id)
{
	int i;

	for (i = 0; i < res->nmode; i++) {
		if (res->modes[i].id == id)
			return &res->modes[i];
	}

	return NULL;
}

static void fullscreen(Display *dpy, Window win)
{
	Atom atom = XInternAtom(dpy, "_NET_WM_STATE_FULLSCREEN", False);
	XChangeProperty(dpy, win,
			XInternAtom(dpy, "_NET_WM_STATE", False),
			XA_ATOM, 32, PropModeReplace,
			(unsigned char *)&atom, 1);
}

static void loop(Display *dpy, XRRScreenResources *res, int use_dri3)
{
	Window root = DefaultRootWindow(dpy);
	Window win;
	XSetWindowAttributes attr;
	int i, j;

	attr.override_redirect = 1;

	run(dpy, root, "off", use_dri3);

	for (i = 0; i < res->noutput; i++) {
		XRROutputInfo *output;
		XRRModeInfo *mode;

		output = XRRGetOutputInfo(dpy, res, res->outputs[i]);
		if (output == NULL)
			continue;

		mode = NULL;
		if (res->nmode)
			mode = lookup_mode(res, output->modes[0]);

		for (j = 0; mode && j < 2*output->ncrtc; j++) {
			int c = j;
			if (c >= output->ncrtc)
				c = 2*output->ncrtc - j - 1;

			printf("[%d, %d] -- OUTPUT:%ld, CRTC:%ld: %dx%d\n",
			       i, c, (long)res->outputs[i], (long)output->crtcs[c],
			       mode->width, mode->height);
			XRRSetCrtcConfig(dpy, res, output->crtcs[c], CurrentTime,
					 0, 0, output->modes[0], RR_Rotate_0, &res->outputs[i], 1);

			run(dpy, root, "root", use_dri3);

			win = XCreateWindow(dpy, root,
					    0, 0, mode->width, mode->height, 0,
					    DefaultDepth(dpy, DefaultScreen(dpy)),
					    InputOutput,
					    DefaultVisual(dpy, DefaultScreen(dpy)),
					    CWOverrideRedirect, &attr);
			fullscreen(dpy, win);
			XMapWindow(dpy, win);
			run(dpy, win, "fullscreen", use_dri3);
			XDestroyWindow(dpy, win);

			win = XCreateWindow(dpy, root,
					    0, 0, mode->width, mode->height, 0,
					    DefaultDepth(dpy, DefaultScreen(dpy)),
					    InputOutput,
					    DefaultVisual(dpy, DefaultScreen(dpy)),
					    CWOverrideRedirect, &attr);
			XMapWindow(dpy, win);
			run(dpy, win, "windowed", use_dri3);
			XDestroyWindow(dpy, win);

			win = XCreateWindow(dpy, root,
					    0, 0, mode->width/2, mode->height/2, 0,
					    DefaultDepth(dpy, DefaultScreen(dpy)),
					    InputOutput,
					    DefaultVisual(dpy, DefaultScreen(dpy)),
					    CWOverrideRedirect, &attr);
			XMapWindow(dpy, win);
			run(dpy, win, "half", use_dri3);
			XDestroyWindow(dpy, win);

			XRRSetCrtcConfig(dpy, res, output->crtcs[c], CurrentTime,
					 0, 0, None, RR_Rotate_0, NULL, 0);
		}

		XRRFreeOutputInfo(output);
	}
}

int main(void)
{
	Display *dpy;
	XRRScreenResources *res;
	XRRCrtcInfo **original_crtc;
	int i;

	dpy = XOpenDisplay(NULL);
	if (dpy == NULL)
		return 77;

	if (!has_present(dpy))
		return 77;

	if (DPMSQueryExtension(dpy, &i, &i))
		DPMSDisable(dpy);

	signal(SIGALRM, SIG_IGN);
	XSetErrorHandler(_check_error_handler);

	res = NULL;
	if (XRRQueryVersion(dpy, &i, &i))
		res = _XRRGetScreenResourcesCurrent(dpy, DefaultRootWindow(dpy));
	if (res == NULL)
		return 77;

	original_crtc = malloc(sizeof(XRRCrtcInfo *)*res->ncrtc);
	for (i = 0; i < res->ncrtc; i++)
		original_crtc[i] = XRRGetCrtcInfo(dpy, res, res->crtcs[i]);

	printf("noutput=%d, ncrtc=%d\n", res->noutput, res->ncrtc);
	for (i = 0; i < res->ncrtc; i++)
		XRRSetCrtcConfig(dpy, res, res->crtcs[i], CurrentTime,
				 0, 0, None, RR_Rotate_0, NULL, 0);

	loop(dpy, res, 0);
	if (has_dri3(dpy))
		loop(dpy, res, 1);

	for (i = 0; i < res->ncrtc; i++)
		XRRSetCrtcConfig(dpy, res, res->crtcs[i], CurrentTime,
				 original_crtc[i]->x,
				 original_crtc[i]->y,
				 original_crtc[i]->mode,
				 original_crtc[i]->rotation,
				 original_crtc[i]->outputs,
				 original_crtc[i]->noutput);

	if (DPMSQueryExtension(dpy, &i, &i))
		DPMSEnable(dpy);
	return 0;
}
