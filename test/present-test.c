/*
 * Copyright (c) 2014 Intel Corporation
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
#include <X11/Xlib-xcb.h>
#include <X11/xshmfence.h>
#include <X11/Xutil.h>
#include <X11/Xlibint.h>
#include <X11/extensions/dpms.h>
#include <X11/extensions/randr.h>
#include <X11/extensions/Xrandr.h>
#include <X11/extensions/Xrender.h>
#include <X11/extensions/XShm.h>
#if HAVE_X11_EXTENSIONS_SHMPROTO_H
#include <X11/extensions/shmproto.h>
#elif HAVE_X11_EXTENSIONS_SHMSTR_H
#include <X11/extensions/shmstr.h>
#else
#error Failed to find the right header for X11 MIT-SHM protocol definitions
#endif
#include <xcb/xcb.h>
#include <xcb/present.h>
#include <xcb/xfixes.h>
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

#include <sys/mman.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <pciaccess.h>

#include "dri3.h"

#define ALIGN(x, y) (((x) + (y) - 1) & -(y))
#define PAGE_ALIGN(x) ALIGN(x, 4096)

#define GTT I915_GEM_DOMAIN_GTT
#define CPU I915_GEM_DOMAIN_CPU

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

static int is_i915_device(int fd)
{
	drm_version_t version;
	char name[5] = "";

	memset(&version, 0, sizeof(version));
	version.name_len = 4;
	version.name = name;

	if (drmIoctl(fd, DRM_IOCTL_VERSION, &version))
		return 0;

	return strcmp("i915", name) == 0;
}

static int is_intel(int fd)
{
	struct drm_i915_getparam gp;
	int ret;

	/* Confirm that this is a i915.ko device with GEM/KMS enabled */
	ret = is_i915_device(fd);
	if (ret) {
		gp.param = I915_PARAM_HAS_GEM;
		gp.value = &ret;
		if (drmIoctl(fd, DRM_IOCTL_I915_GETPARAM, &gp))
			ret = 0;
	}
	return ret;
}

static void *setup_msc(Display *dpy,  Window win)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	xcb_void_cookie_t cookie;
	uint32_t id = xcb_generate_id(c);
	xcb_generic_error_t *error;
	void *q;

	cookie = xcb_present_select_input_checked(c, id, win, XCB_PRESENT_EVENT_MASK_COMPLETE_NOTIFY);
	q = xcb_register_for_special_xge(c, &xcb_present_id, id, &stamp);

	error = xcb_request_check(c, cookie);
	assert(error == NULL);

	return q;
}

static uint64_t check_msc(Display *dpy, Window win, void *q, uint64_t last_msc, uint64_t *ust)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	static uint32_t serial = 1;
	uint64_t msc = 0;
	int complete = 0;

	xcb_present_notify_msc(c, win, serial, 0, 0, 0);
	xcb_flush(c);

	do {
		xcb_present_complete_notify_event_t *ce;
		xcb_generic_event_t *ev;

		ev = xcb_wait_for_special_event(c, q);
		if (ev == NULL)
			break;

		ce = (xcb_present_complete_notify_event_t *)ev;
		if (ce->kind == XCB_PRESENT_COMPLETE_KIND_NOTIFY_MSC &&
		    ce->serial == serial) {
			msc = ce->msc;
			if (ust)
				*ust = ce->ust;
			complete = 1;
		}
		free(ev);
	} while (!complete);

	if ((int64_t)(msc - last_msc) < 0) {
		printf("Invalid MSC: was %llu, now %llu\n",
		       (long long)last_msc, (long long)msc);
	}

	if (++serial == 0)
		serial = 1;

	return msc;
}

static uint64_t msc_interval(Display *dpy, Window win, void *q)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	uint64_t msc, ust;
	int complete = 0;

	msc = check_msc(dpy, win, q, 0, NULL);

	xcb_present_notify_msc(c, win, 0xc0ffee00, msc, 0, 0);
	xcb_present_notify_msc(c, win, 0xc0ffee01, msc + 10, 0, 0);
	xcb_flush(c);

	ust = msc = 0;
	do {
		xcb_present_complete_notify_event_t *ce;
		xcb_generic_event_t *ev;

		ev = xcb_wait_for_special_event(c, q);
		if (ev == NULL)
			break;

		ce = (xcb_present_complete_notify_event_t *)ev;
		if (ce->kind == XCB_PRESENT_COMPLETE_KIND_NOTIFY_MSC &&
		    ce->serial == 0xc0ffee00) {
			msc -= ce->msc;
			ust -= ce->ust;
			complete++;
		}
		if (ce->kind == XCB_PRESENT_COMPLETE_KIND_NOTIFY_MSC &&
		    ce->serial == 0xc0ffee01) {
			msc += ce->msc;
			ust += ce->ust;
			complete++;
		}
		free(ev);
	} while (complete != 2);

	return (ust + msc/2) / msc;
}

static void teardown_msc(Display *dpy, void *q)
{
	xcb_unregister_for_special_event(XGetXCBConnection(dpy), q);
}

static int test_whole(Display *dpy)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	Pixmap pixmap;
	struct dri3_fence fence;
	Window root;
	unsigned int width, height;
	unsigned border, depth;
	int x, y, ret = 1;

	XGetGeometry(dpy, DefaultRootWindow(dpy),
		     &root, &x, &y, &width, &height, &border, &depth);

	if (dri3_create_fence(dpy, root, &fence))
		return 0;

	printf("Testing whole screen flip: %dx%d\n", width, height);
	_x_error_occurred = 0;

	xshmfence_reset(fence.addr);

	pixmap = XCreatePixmap(dpy, root, width, height, depth);
	xcb_present_pixmap(c, root, pixmap, 0,
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   fence.xid,
			   XCB_PRESENT_OPTION_NONE,
			   0, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	XFreePixmap(dpy, pixmap);

	pixmap = XCreatePixmap(dpy, root, width, height, depth);
	xcb_present_pixmap(c, root, pixmap, 0,
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   None, /* sync fence */
			   XCB_PRESENT_OPTION_NONE,
			   0, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	XFreePixmap(dpy, pixmap);
	XFlush(dpy);

	ret = !!xshmfence_await(fence.addr);
	dri3_fence_free(dpy, &fence);

	XSync(dpy, True);
	ret += !!_x_error_occurred;

	return ret;
}

static uint64_t flush_flips(Display *dpy, Window win, Pixmap pixmap, void *Q, uint64_t *ust)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	uint64_t msc;
	int complete;

	msc = check_msc(dpy, win, Q, 0, NULL);
	xcb_present_pixmap(c, win, pixmap,
			   0xdeadbeef, /* serial */
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   None,
			   XCB_PRESENT_OPTION_NONE,
			   msc + 60, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	xcb_flush(c);
	complete = 0;
	do {
		xcb_present_complete_notify_event_t *ce;
		xcb_generic_event_t *ev;

		ev = xcb_wait_for_special_event(c, Q);
		if (ev == NULL)
			break;

		ce = (xcb_present_complete_notify_event_t *)ev;
		complete = (ce->kind == XCB_PRESENT_COMPLETE_KIND_PIXMAP &&
			    ce->serial == 0xdeadbeef);
		free(ev);
	} while (!complete);
	XSync(dpy, True);

	return check_msc(dpy, win, Q, msc, ust);
}

static int test_double(Display *dpy, void *Q)
{
#define COUNT (15*60)
	xcb_connection_t *c = XGetXCBConnection(dpy);
	Pixmap pixmap;
	Window root;
	unsigned int width, height;
	unsigned border, depth;
	int x, y, n, ret;
	struct {
		uint64_t msc, ust;
	} frame[COUNT+1];
	int offset = 0;

	XGetGeometry(dpy, DefaultRootWindow(dpy),
		     &root, &x, &y, &width, &height, &border, &depth);

	printf("Testing whole screen flip double buffering: %dx%d\n", width, height);
	_x_error_occurred = 0;

	pixmap = XCreatePixmap(dpy, root, width, height, depth);
	flush_flips(dpy, root, pixmap, Q, NULL);
	for (n = 0; n <= COUNT; n++) {
		int complete;

		xcb_present_pixmap(c, root, pixmap, n,
				   0, /* valid */
				   0, /* update */
				   0, /* x_off */
				   0, /* y_off */
				   None,
				   None, /* wait fence */
				   None,
				   XCB_PRESENT_OPTION_NONE,
				   0, /* target msc */
				   0, /* divisor */
				   0, /* remainder */
				   0, NULL);
		xcb_flush(c);

		complete = 0;
		do {
			xcb_present_complete_notify_event_t *ce;
			xcb_generic_event_t *ev;

			ev = xcb_wait_for_special_event(c, Q);
			if (ev == NULL)
				break;

			ce = (xcb_present_complete_notify_event_t *)ev;
			if (ce->kind == XCB_PRESENT_COMPLETE_KIND_PIXMAP &&
			    ce->serial == n) {
				frame[n].msc = ce->msc;
				frame[n].ust = ce->ust;
				complete = 1;
			}
			free(ev);
		} while (!complete);
	}
	XFreePixmap(dpy, pixmap);

	XSync(dpy, True);
	ret = !!_x_error_occurred;

	if (frame[COUNT].msc - frame[0].msc != COUNT) {
		printf("Expected %d frames interval, %d elapsed instead\n",
		       COUNT, (int)(frame[COUNT].msc - frame[0].msc));
		for (n = 0; n <= COUNT; n++) {
			if (frame[n].msc - frame[0].msc != n + offset) {
				printf("frame[%d]: msc=%03lld, ust=%lld\n", n,
				       (long long)(frame[n].msc - frame[0].msc),
				       (long long)(frame[n].ust - frame[0].ust));
				offset = frame[n].msc - frame[0].msc - n;
				ret++;
			}
		}
	}

	return ret;
}

static int test_future(Display *dpy, void *Q)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	Pixmap pixmap;
	struct dri3_fence fence;
	Window root;
	unsigned int width, height;
	unsigned border, depth;
	int x, y, ret = 0, n;
	uint64_t msc, ust;
	int complete;
	int early = 0, late = 0;
	int earliest = 0, latest = 0;
	uint64_t interval;

	XGetGeometry(dpy, DefaultRootWindow(dpy),
		     &root, &x, &y, &width, &height, &border, &depth);

	if (dri3_create_fence(dpy, root, &fence))
		return 0;

	printf("Testing whole screen flips into the future: %dx%d\n", width, height);
	_x_error_occurred = 0;

	interval = msc_interval(dpy, root, Q);

	pixmap = XCreatePixmap(dpy, root, width, height, depth);
	msc = flush_flips(dpy, root, pixmap, Q, &ust);
	for (n = 1; n <= 10; n++)
		xcb_present_pixmap(c, root, pixmap,
				   n, /* serial */
				   0, /* valid */
				   0, /* update */
				   0, /* x_off */
				   0, /* y_off */
				   None,
				   None, /* wait fence */
				   None,
				   XCB_PRESENT_OPTION_NONE,
				   msc + 60 + n*15*60, /* target msc */
				   0, /* divisor */
				   0, /* remainder */
				   0, NULL);
	xcb_present_pixmap(c, root, pixmap,
			   0xdeadbeef, /* serial */
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   None,
			   XCB_PRESENT_OPTION_NONE,
			   msc + 60 + n*15*60, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	xcb_flush(c);

	complete = 0;
	do {
		xcb_present_complete_notify_event_t *ce;
		xcb_generic_event_t *ev;

		ev = xcb_wait_for_special_event(c, Q);
		if (ev == NULL)
			break;

		ce = (xcb_present_complete_notify_event_t *)ev;
		assert(ce->kind == XCB_PRESENT_COMPLETE_KIND_PIXMAP);

		if (ce->serial == 0xdeadbeef) {
			int64_t time;

			time = ce->ust - (ust + (60 + 15*60*n) * interval);
			if (time < -(int64_t)interval) {
				fprintf(stderr,
					"\tflips completed too early by %lldms\n",
					(long long)(-time / 1000));
			} else if (time > (int64_t)interval) {
				fprintf(stderr,
					"\tflips completed too late by %lldms\n",
					(long long)(time / 1000));
			}
			complete = 1;
		} else {
			int diff = (int64_t)(ce->msc - (15*60*ce->serial + msc + 60));
			if (diff < 0) {
				if (-diff > earliest) {
					fprintf(stderr, "\tframe %d displayed early by %d frames\n", ce->serial, diff);
					earliest = -diff;
				}
				early++;
				ret++;
			} else if (diff > 0) {
				if (diff > latest) {
					fprintf(stderr, "\tframe %d displayed late by %d frames\n", ce->serial, diff);
					latest = diff;
				}
				late++;
				ret++;
			}
		}
		free(ev);
	} while (!complete);

	if (early)
		printf("\t%d frames shown too early (worst %d)!\n", early, earliest);
	if (late)
		printf("\t%d frames shown too late (worst %d)!\n", late, latest);

	ret += !!_x_error_occurred;

	return ret;
}

static int test_exhaustion(Display *dpy, void *Q)
{
#define N_VBLANKS 256 /* kernel event queue length: 128 vblanks */
	xcb_connection_t *c = XGetXCBConnection(dpy);
	Pixmap pixmap;
	struct dri3_fence fence;
	Window root;
	xcb_xfixes_region_t region;
	unsigned int width, height;
	unsigned border, depth;
	int x, y, ret = 0, n;
	uint64_t target, final;

	XGetGeometry(dpy, DefaultRootWindow(dpy),
		     &root, &x, &y, &width, &height, &border, &depth);

	if (dri3_create_fence(dpy, root, &fence))
		return 0;

	printf("Testing whole screen flips with long vblank queues: %dx%d\n", width, height);
	_x_error_occurred = 0;

	region = xcb_generate_id(c);
	xcb_xfixes_create_region(c, region, 0, NULL);

	target = check_msc(dpy, root, Q, 0, NULL);
	pixmap = XCreatePixmap(dpy, root, width, height, depth);
	xshmfence_reset(fence.addr);
	for (n = N_VBLANKS; n--; )
		xcb_present_pixmap(c, root, pixmap, 0,
				   0, /* valid */
				   region, /* update */
				   0, /* x_off */
				   0, /* y_off */
				   None,
				   None, /* wait fence */
				   None,
				   XCB_PRESENT_OPTION_NONE,
				   target + N_VBLANKS, /* target msc */
				   1, /* divisor */
				   0, /* remainder */
				   0, NULL);
	xcb_present_pixmap(c, root, pixmap, 0,
			   region, /* valid */
			   region, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   fence.xid,
			   XCB_PRESENT_OPTION_NONE,
			   target, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	xcb_flush(c);

	XSync(dpy, True);
	ret += !!xshmfence_await(fence.addr);

	final = check_msc(dpy, root, Q, 0, NULL);
	if (final < target) {
		printf("\tFirst flip too early, MSC was %llu, expected %llu\n",
		       (long long)final, (long long)target);
		ret++;
	} else if (final > target + 1) {
		printf("\tFirst flip too late, MSC was %llu, expected %llu\n",
		       (long long)final, (long long)target);
		ret++;
	}

	xshmfence_reset(fence.addr);
	xcb_present_pixmap(c, root, pixmap, 0,
			   region, /* valid */
			   region, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   fence.xid,
			   XCB_PRESENT_OPTION_NONE,
			   target + N_VBLANKS, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	xcb_flush(c);
	ret += !!xshmfence_await(fence.addr);

	final = check_msc(dpy, root, Q, 0, NULL);
	if (final < target + N_VBLANKS) {
		printf("\tLast flip too early, MSC was %llu, expected %llu\n",
		       (long long)final, (long long)(target + N_VBLANKS));
		ret++;
	} else if (final > target + N_VBLANKS + 1) {
		printf("\tLast flip too late, MSC was %llu, expected %llu\n",
		       (long long)final, (long long)(target + N_VBLANKS));
		ret++;
	}

	flush_flips(dpy, root, pixmap, Q, NULL);

	XFreePixmap(dpy, pixmap);
	xcb_xfixes_destroy_region(c, region);
	dri3_fence_free(dpy, &fence);

	XSync(dpy, True);
	ret += !!_x_error_occurred;

	return ret;
#undef N_VBLANKS
}

static int test_accuracy(Display *dpy, void *Q)
{
#define N_VBLANKS (60 * 120) /* ~2 minutes */
	xcb_connection_t *c = XGetXCBConnection(dpy);
	Pixmap pixmap;
	Window root;
	unsigned int width, height;
	unsigned border, depth;
	int x, y, ret = 0, n;
	uint64_t target;
	int early = 0, late = 0;
	int earliest = 0, latest = 0;
	int complete;

	XGetGeometry(dpy, DefaultRootWindow(dpy),
		     &root, &x, &y, &width, &height, &border, &depth);

	printf("Testing whole screen flip accuracy: %dx%d\n", width, height);
	_x_error_occurred = 0;

	pixmap = XCreatePixmap(dpy, root, width, height, depth);
	target = flush_flips(dpy, root, pixmap, Q, NULL);
	for (n = 0; n <= N_VBLANKS; n++)
		xcb_present_pixmap(c, root, pixmap,
				   n, /* serial */
				   0, /* valid */
				   0, /* update */
				   0, /* x_off */
				   0, /* y_off */
				   None,
				   None, /* wait fence */
				   None,
				   XCB_PRESENT_OPTION_NONE,
				   target + 60 + n, /* target msc */
				   0, /* divisor */
				   0, /* remainder */
				   0, NULL);
	xcb_present_pixmap(c, root, pixmap,
			   0xdeadbeef, /* serial */
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   None,
			   XCB_PRESENT_OPTION_NONE,
			   target + 60 + n, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	xcb_flush(c);

	complete = 0;
	do {
		xcb_present_complete_notify_event_t *ce;
		xcb_generic_event_t *ev;

		ev = xcb_wait_for_special_event(c, Q);
		if (ev == NULL)
			break;

		ce = (xcb_present_complete_notify_event_t *)ev;
		assert(ce->kind == XCB_PRESENT_COMPLETE_KIND_PIXMAP);

		if (ce->serial != 0xdeadbeef) {
			int diff = (int64_t)(ce->msc - (target + ce->serial + 60));
			if (diff < 0) {
				if (-diff > earliest) {
					fprintf(stderr, "\tframe %d displayed early by %d frames\n", ce->serial, -diff);
					earliest = -diff;
				}
				early++;
				ret++;
			} else if (diff > 0) {
				if (diff > latest) {
					fprintf(stderr, "\tframe %d displayed late by %d frames\n", ce->serial, diff);
					latest = diff;
				}
				ret++;
				late++;
			}
		} else
			complete = 1;
		free(ev);
	} while (!complete);

	if (early)
		printf("\t%d frames shown too early (worst %d)!\n", early, earliest);
	if (late)
		printf("\t%d frames shown too late (worst %d)!\n", late, latest);

	XFreePixmap(dpy, pixmap);

	XSync(dpy, True);
	ret += !!_x_error_occurred;

	return ret;
#undef N_VBLANKS
}

static int test_modulus(Display *dpy, void *Q)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	Pixmap pixmap;
	Window root;
	unsigned int width, height;
	unsigned border, depth;
	xcb_xfixes_region_t region;
	int x, y, ret = 0;
	uint64_t target;
	int early = 0, late = 0;
	int earliest = 0, latest = 0;
	int complete;

	XGetGeometry(dpy, DefaultRootWindow(dpy),
		     &root, &x, &y, &width, &height, &border, &depth);

	printf("Testing whole screen flip modulus: %dx%d\n", width, height);
	_x_error_occurred = 0;

	region = xcb_generate_id(c);
	xcb_xfixes_create_region(c, region, 0, NULL);

	pixmap = XCreatePixmap(dpy, root, width, height, depth);
	target = flush_flips(dpy, root, pixmap, Q, NULL);
	for (x = 1; x <= 7; x++) {
		for (y = 0; y < x; y++) {
			xcb_present_pixmap(c, root, pixmap,
					   y << 16 | x, /* serial */
					   region, /* valid */
					   region, /* update */
					   0, /* x_off */
					   0, /* y_off */
					   None,
					   None, /* wait fence */
					   None,
					   XCB_PRESENT_OPTION_NONE,
					   0, /* target msc */
					   x, /* divisor */
					   y, /* remainder */
					   0, NULL);
		}
	}
	xcb_present_pixmap(c, root, pixmap,
			   0xdeadbeef, /* serial */
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   None,
			   XCB_PRESENT_OPTION_NONE,
			   target + 2*x, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	xcb_flush(c);

	complete = 0;
	do {
		xcb_present_complete_notify_event_t *ce;
		xcb_generic_event_t *ev;

		ev = xcb_wait_for_special_event(c, Q);
		if (ev == NULL)
			break;

		ce = (xcb_present_complete_notify_event_t *)ev;
		if (ce->kind != XCB_PRESENT_COMPLETE_KIND_PIXMAP)
			break;

		assert(ce->serial);
		if (ce->serial != 0xdeadbeef) {
			uint64_t msc;
			int diff;

			x = ce->serial & 0xffff;
			y = ce->serial >> 16;

			msc = target;
			msc -= target % x;
			msc += y;
			if (msc <= target)
				msc += x;

			diff = (int64_t)(ce->msc - msc);
			if (diff < 0) {
				ret += -diff;
				if (-diff > earliest) {
					fprintf(stderr, "\tframe (%d, %d) displayed early by %d frames\n", y, x, -diff);
					earliest = -diff;
				}
				early++;
			} else if (diff > 0) {
				ret += diff;
				if (diff > latest) {
					fprintf(stderr, "\tframe (%d, %d) displayed late by %d frames\n", y, x, diff);
					latest = diff;
				}
				late++;
			}
		} else
			complete = 1;
		free(ev);
	} while (!complete);

	if (early)
		printf("\t%d frames shown too early (worst %d)!\n", early, earliest);
	if (late)
		printf("\t%d frames shown too late (worst %d)!\n", late, latest);

	XFreePixmap(dpy, pixmap);
	xcb_xfixes_destroy_region(c, region);

	XSync(dpy, True);
	ret += !!_x_error_occurred;

	return ret;
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

static int for_each_crtc(Display *dpy,
			  int (*func)(Display *dpy,
				      RRCrtc crtc,
				      int width, int height,
				      void *closure),
			  void *closure)
{
	XRRScreenResources *res;
	XRRCrtcInfo **original_crtc;
	int i, j, err = 0;

	if (!XRRQueryVersion(dpy, &i, &j))
		return -1;

	res = _XRRGetScreenResourcesCurrent(dpy, DefaultRootWindow(dpy));
	if (res == NULL)
		return -1;

	original_crtc = malloc(sizeof(XRRCrtcInfo *)*res->ncrtc);
	for (i = 0; i < res->ncrtc; i++)
		original_crtc[i] = XRRGetCrtcInfo(dpy, res, res->crtcs[i]);

	for (i = 0; i < res->noutput; i++) {
		XRROutputInfo *output;
		XRRModeInfo *mode;

		output = XRRGetOutputInfo(dpy, res, res->outputs[i]);
		if (output == NULL)
			continue;

		mode = NULL;
		if (res->nmode)
			mode = lookup_mode(res, output->modes[0]);

		for (j = 0; mode && j < output->ncrtc; j++) {
			printf("[%d, %d] -- OUTPUT:%ld, CRTC:%ld\n",
			       i, j, (long)res->outputs[i], (long)output->crtcs[j]);
			XRRSetCrtcConfig(dpy, res, output->crtcs[j], CurrentTime,
					 0, 0, output->modes[0], RR_Rotate_0, &res->outputs[i], 1);
			XSync(dpy, True);

			err += func(dpy, output->crtcs[j], mode->width, mode->height, closure);

			XRRSetCrtcConfig(dpy, res, output->crtcs[j], CurrentTime,
					 0, 0, None, RR_Rotate_0, NULL, 0);
			XSync(dpy, True);
		}

		XRRFreeOutputInfo(output);
	}

	for (i = 0; i < res->ncrtc; i++)
		XRRSetCrtcConfig(dpy, res, res->crtcs[i], CurrentTime,
				 original_crtc[i]->x,
				 original_crtc[i]->y,
				 original_crtc[i]->mode,
				 original_crtc[i]->rotation,
				 original_crtc[i]->outputs,
				 original_crtc[i]->noutput);

	free(original_crtc);
	XRRFreeScreenResources(res);

	return err;
}

struct test_crtc {
	Window win;
	int depth;
	unsigned flags;

	struct dri3_fence fence;
	void *queue;
	uint64_t msc;
};
#define SYNC 0x1
#define FUTURE 0x2

static int __test_crtc(Display *dpy, RRCrtc crtc,
		       int width, int height,
		       void *closure)
{
	struct test_crtc *test = closure;
	Pixmap pixmap;
	int err = 0;

	test->msc = check_msc(dpy, test->win, test->queue, test->msc, NULL);

	if (test->flags & SYNC)
		xshmfence_reset(test->fence.addr);

	pixmap = XCreatePixmap(dpy, test->win, width, height, test->depth);
	xcb_present_pixmap(XGetXCBConnection(dpy),
			   test->win, pixmap,
			   0, /* sbc */
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   crtc,
			   None, /* wait fence */
			   test->flags & SYNC ? test->fence.xid : None,
			   XCB_PRESENT_OPTION_NONE,
			   test->msc, /* target msc */
			   1, /* divisor */
			   0, /* remainder */
			   0, NULL);
	if (test->flags & SYNC) {
		Pixmap tmp = XCreatePixmap(dpy, test->win, width, height, test->depth);
		xcb_present_pixmap(XGetXCBConnection(dpy),
				   test->win, tmp,
				   1, /* sbc */
				   0, /* valid */
				   0, /* update */
				   0, /* x_off */
				   0, /* y_off */
				   crtc,
				   None, /* wait fence */
				   None, /* sync fence */
				   XCB_PRESENT_OPTION_NONE,
				   test->msc + (test->flags & FUTURE ? 5 * 16 : 1), /* target msc */
				   1, /* divisor */
				   0, /* remainder */
				   0, NULL);
		XFreePixmap(dpy, tmp);
		XFlush(dpy);
		err += !!xshmfence_await(test->fence.addr);
	}
	XFreePixmap(dpy, pixmap);

	test->msc = check_msc(dpy, test->win, test->queue, test->msc, NULL);
	return err;
}

static int test_crtc(Display *dpy, void *queue, uint64_t last_msc)
{
	struct test_crtc test;
	int err = 0;

	XSync(dpy, True);
	_x_error_occurred = 0;

	test.win = DefaultRootWindow(dpy);
	test.depth = DefaultDepth(dpy, DefaultScreen(dpy));
	if (dri3_create_fence(dpy, test.win, &test.fence))
		return -1;
	test.queue = queue;
	test.msc = last_msc;

	printf("Testing each crtc, without waiting for each flip\n");
	test.flags = 0;
	test.msc = check_msc(dpy, test.win, test.queue, test.msc, NULL);
	err += for_each_crtc(dpy, __test_crtc, &test);
	test.msc = check_msc(dpy, test.win, test.queue, test.msc, NULL);

	printf("Testing each crtc, waiting for flips to complete\n");
	test.flags = SYNC;
	test.msc = check_msc(dpy, test.win, test.queue, test.msc, NULL);
	err += for_each_crtc(dpy, __test_crtc, &test);
	test.msc = check_msc(dpy, test.win, test.queue, test.msc, NULL);

	printf("Testing each crtc, with future flips\n");
	test.flags = FUTURE | SYNC;
	test.msc = check_msc(dpy, test.win, test.queue, test.msc, NULL);
	err += for_each_crtc(dpy, __test_crtc, &test);
	test.msc = check_msc(dpy, test.win, test.queue, test.msc, NULL);

	dri3_fence_free(dpy, &test.fence);
	XSync(dpy, True);
	err += !!_x_error_occurred;

	if (err)
		printf("%s: failures=%d\n", __func__, err);

	return err;
}

static int
can_use_shm(Display *dpy)
{
	int major, minor, has_pixmap;

	if (!XShmQueryExtension(dpy))
		return 0;

	XShmQueryVersion(dpy, &major, &minor, &has_pixmap);
	return has_pixmap;
}

static int test_shm(Display *dpy)
{
	Window win = DefaultRootWindow(dpy);
	XShmSegmentInfo shm;
	Pixmap pixmap;
	Window root;
	unsigned int width, height;
	unsigned border, depth;
	int x, y, ret = 1;

	if (!can_use_shm(dpy))
		return 0;

	_x_error_occurred = 0;

	XGetGeometry(dpy, win, &root, &x, &y,
		     &width, &height, &border, &depth);

	printf("Using %dx%d SHM\n", width, height);

	shm.shmid = shmget(IPC_PRIVATE, height * 4*width, IPC_CREAT | 0666);
	if (shm.shmid == -1)
		return 0;

	shm.shmaddr = shmat(shm.shmid, 0, 0);
	if (shm.shmaddr == (char *) -1)
		goto rmid;

	shm.readOnly = False;
	XShmAttach(dpy, &shm);

	pixmap = XShmCreatePixmap(dpy, DefaultRootWindow(dpy),
				  shm.shmaddr, &shm, width, height, 24);
	if (_x_error_occurred)
		goto detach;

	xcb_present_pixmap(XGetXCBConnection(dpy),
			   win, pixmap,
			   0, /* sbc */
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   None,
			   XCB_PRESENT_OPTION_NONE,
			   0, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	XFreePixmap(dpy, pixmap);

	XSync(dpy, True);
	if (_x_error_occurred)
		goto detach;

	ret = 0;
detach:
	XShmDetach(dpy, &shm);
	shmdt(shm.shmaddr);
	XSync(dpy, False);
rmid:
	shmctl(shm.shmid, IPC_RMID, NULL);
	return ret;
}

static uint32_t gem_create(int fd, int size)
{
	struct drm_i915_gem_create create;

	create.handle = 0;
	create.size = size;
	(void)drmIoctl(fd, DRM_IOCTL_I915_GEM_CREATE, &create);

	return create.handle;
}

struct local_i915_gem_caching {
	uint32_t handle;
	uint32_t caching;
};

#define LOCAL_I915_GEM_SET_CACHING	0x2f
#define LOCAL_IOCTL_I915_GEM_SET_CACHING DRM_IOW(DRM_COMMAND_BASE + LOCAL_I915_GEM_SET_CACHING, struct local_i915_gem_caching)

static int gem_set_caching(int fd, uint32_t handle, int caching)
{
	struct local_i915_gem_caching arg;

	arg.handle = handle;
	arg.caching = caching;

	return drmIoctl(fd, LOCAL_IOCTL_I915_GEM_SET_CACHING, &arg) == 0;
}

static int gem_export(int fd, uint32_t handle)
{
	struct drm_prime_handle args;

	args.handle = handle;
	args.flags = O_CLOEXEC;

	if (drmIoctl(fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &args))
		return -1;

	return args.fd;
}

static void gem_close(int fd, uint32_t handle)
{
	struct drm_gem_close close;

	close.handle = handle;
	(void)drmIoctl(fd, DRM_IOCTL_GEM_CLOSE, &close);
}

static int test_dri3(Display *dpy)
{
	Window win = DefaultRootWindow(dpy);
	Pixmap pixmap;
	Window root;
	unsigned int width, height;
	unsigned border, depth;
	unsigned stride, size;
	int x, y, ret = 1;
	int device, handle;
	int bpp;

	device = dri3_open(dpy);
	if (device < 0)
		return 0;

	if (!is_intel(device))
		return 0;

	printf("Opened Intel DRI3 device\n");

	XGetGeometry(dpy, win, &root, &x, &y,
		     &width, &height, &border, &depth);

	switch (depth) {
	case 8: bpp = 8; break;
	case 15: case 16: bpp = 16; break;
	case 24: case 32: bpp = 32; break;
	default: return 0;
	}

	stride = width * bpp/8;
	size = PAGE_ALIGN(stride * height);
	printf("Creating DRI3 %dx%d (source stride=%d, size=%d) for GTT\n",
	       width, height, stride, size);

	pixmap = 0;
	handle = gem_create(device, size);
	if (handle) {
		pixmap = dri3_create_pixmap(dpy, root,
					     width, height, depth,
					     gem_export(device, handle), bpp, stride, size);
		gem_close(device, handle);
	}
	if (pixmap == 0)
		goto fail;

	xcb_present_pixmap(XGetXCBConnection(dpy),
			   win, pixmap,
			   0, /* sbc */
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   None,
			   XCB_PRESENT_OPTION_NONE,
			   0, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	XFreePixmap(dpy, pixmap);

	XSync(dpy, True);
	if (_x_error_occurred)
		goto fail;

	printf("Creating DRI3 %dx%d (source stride=%d, size=%d) for CPU\n",
	       width, height, stride, size);

	pixmap = 0;
	handle = gem_create(device, size);
	if (handle) {
		gem_set_caching(device, handle, CPU);
		handle = dri3_create_pixmap(dpy, root,
					     width, height, depth,
					     gem_export(device, handle), bpp, stride, size);
		gem_close(device, handle);
	}
	if (pixmap == 0)
		goto fail;

	xcb_present_pixmap(XGetXCBConnection(dpy),
			   win, pixmap,
			   0, /* sbc */
			   0, /* valid */
			   0, /* update */
			   0, /* x_off */
			   0, /* y_off */
			   None,
			   None, /* wait fence */
			   None,
			   XCB_PRESENT_OPTION_NONE,
			   0, /* target msc */
			   0, /* divisor */
			   0, /* remainder */
			   0, NULL);
	XFreePixmap(dpy, pixmap);

	XSync(dpy, True);
	if (_x_error_occurred)
		goto fail;

	ret = 0;
fail:
	close(device);
	return ret;
}

static int has_present(Display *dpy)
{
	xcb_connection_t *c = XGetXCBConnection(dpy);
	xcb_generic_error_t *error = NULL;
	void *reply;

	reply = xcb_xfixes_query_version_reply(c,
					       xcb_xfixes_query_version(c,
									XCB_XFIXES_MAJOR_VERSION,
									XCB_XFIXES_MINOR_VERSION),
					       &error);
	free(reply);
	free(error);
	if (reply == NULL) {
		fprintf(stderr, "XFixes not supported on %s\n", DisplayString(dpy));
		return 0;
	}

	reply = xcb_dri3_query_version_reply(c,
					     xcb_dri3_query_version(c,
								    XCB_DRI3_MAJOR_VERSION,
								    XCB_DRI3_MINOR_VERSION),
					     &error);
	free(reply);
	free(error);
	if (reply == NULL) {
		fprintf(stderr, "DRI3 not supported on %s\n", DisplayString(dpy));
		return 0;
	}

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

int main(void)
{
	Display *dpy;
	Window root;
	int dummy;
	int error = 0;
	uint64_t last_msc;
	void *queue;

	dpy = XOpenDisplay(NULL);
	if (dpy == NULL)
		return 77;

	if (!has_present(dpy))
		return 77;

	if (DPMSQueryExtension(dpy, &dummy, &dummy))
		DPMSDisable(dpy);

	root = DefaultRootWindow(dpy);

	signal(SIGALRM, SIG_IGN);
	XSetErrorHandler(_check_error_handler);

	queue = setup_msc(dpy, root);
	last_msc = check_msc(dpy, root, queue, 0, NULL);

	error += test_whole(dpy);
	last_msc = check_msc(dpy, root, queue, last_msc, NULL);

	error += test_double(dpy, queue);
	last_msc = check_msc(dpy, root, queue, last_msc, NULL);

	error += test_future(dpy, queue);
	last_msc = check_msc(dpy, root, queue, last_msc, NULL);

	error += test_accuracy(dpy, queue);
	last_msc = check_msc(dpy, root, queue, last_msc, NULL);

	error += test_modulus(dpy, queue);
	last_msc = check_msc(dpy, root, queue, last_msc, NULL);

	error += test_exhaustion(dpy, queue);
	last_msc = check_msc(dpy, root, queue, last_msc, NULL);

	error += test_crtc(dpy, queue, last_msc);
	last_msc = check_msc(dpy, root, queue, last_msc, NULL);

	error += test_shm(dpy);
	last_msc = check_msc(dpy, root, queue, last_msc, NULL);

	error += test_dri3(dpy);
	last_msc = check_msc(dpy, root, queue, last_msc, NULL);

	teardown_msc(dpy, queue);

	if (DPMSQueryExtension(dpy, &dummy, &dummy))
		DPMSEnable(dpy);
	return !!error;
}
