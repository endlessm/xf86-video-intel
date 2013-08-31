/*
 * Copyright © 2013 Intel Corporation
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <X11/Xlib.h>

#include <X11/Xlibint.h>
#include <X11/extensions/XShm.h>
#include <X11/extensions/shmproto.h>
#include <X11/extensions/Xdamage.h>
#include <X11/extensions/Xrandr.h>
#include <X11/extensions/Xrender.h>
#include <X11/extensions/record.h>
#include <X11/Xcursor/Xcursor.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/timerfd.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <getopt.h>
#include <limits.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>

#if 0
#define DBG(x) printf x
#else
#define DBG(x)
#endif

struct display {
	Display *dpy;

	int damage_event, damage_error;
	int xfixes_event, xfixes_error;
	int rr_event, rr_error;
	Window root;
	Damage damage;

	XRenderPictFormat *root_format;
	XRenderPictFormat *rgb24_format;

	Cursor invisible_cursor;
	Cursor visible_cursor;

	int cursor_x;
	int cursor_y;
	int cursor_moved;
	int cursor_visible;
	int cursor;

	int flush;
};

struct output {
	struct display *display;
	Display *dpy;
	char *name;
	RROutput rr_output;
	XShmSegmentInfo shm;
	Window window;
	Picture win_picture;
	Picture pix_picture;
	Pixmap pixmap;
	GC gc;

	int depth;
	int serial;

	int has_shm;
	int has_shm_pixmap;
	int shm_opcode;
	int shm_event;

	int x, y;
	XRRModeInfo mode;
	Rotation rotation;
};

struct clone {
	struct output src, dst;

	XShmSegmentInfo shm;
	XImage image;

	int width, height;
	struct { int x1, x2, y1, y2; } damaged;
	int rr_update;
};

struct context {
	struct display *display;
	struct clone *clones;
	int num_clones;
	int num_display;
};

static int xlib_vendor_is_xorg(Display *dpy)
{
	const char *const vendor = ServerVendor(dpy);
	return strstr(vendor, "X.Org") || strstr(vendor, "Xorg");
}

#define XORG_VERSION_ENCODE(major,minor,patch,snap) \
    (((major) * 10000000) + ((minor) * 100000) + ((patch) * 1000) + snap)

static int _x_error_occurred;

static int
_check_error_handler(Display     *display,
		     XErrorEvent *event)
{
	_x_error_occurred = 1;
	return False; /* ignored */
}

static int
can_use_shm(Display *dpy,
	    Window window,
	    int *shm_event,
	    int *shm_opcode,
	    int *shm_pixmap)
{
	XShmSegmentInfo shm;
	int (*old_handler)(Display *display, XErrorEvent *event);
	Status success;
	XExtCodes *codes;
	int major, minor, has_shm, has_pixmap;

	if (!XShmQueryExtension(dpy))
		return 0;

	XShmQueryVersion(dpy, &major, &minor, &has_pixmap);

	shm.shmid = shmget(IPC_PRIVATE, 0x1000, IPC_CREAT | 0600);
	if (shm.shmid == -1)
		return 0;

	shm.readOnly = 0;
	shm.shmaddr = shmat (shm.shmid, NULL, 0);
	if (shm.shmaddr == (char *) -1) {
		shmctl(shm.shmid, IPC_RMID, NULL);
		return 0;
	}

	_x_error_occurred = 0;

	XSync(dpy, False);
	old_handler = XSetErrorHandler(_check_error_handler);

	success = XShmAttach(dpy, &shm);
	XSync(dpy, False);

	has_shm = success && _x_error_occurred == 0;

	codes = XInitExtension(dpy, SHMNAME);
	if (codes == NULL)
		has_pixmap = 0;

	/* As libXext sets the SEND_EVENT bit in the ShmCompletionEvent,
	 * the Xserver may crash if it does not take care when processing
	 * the event type. For instance versions of Xorg prior to 1.11.1
	 * exhibited this bug, and was fixed by:
	 *
	 * commit 2d2dce558d24eeea0eb011ec9ebaa6c5c2273c39
	 * Author: Sam Spilsbury <sam.spilsbury@canonical.com>
	 * Date:   Wed Sep 14 09:58:34 2011 +0800
	 *
	 * Remove the SendEvent bit (0x80) before doing range checks on event type.
	 */
	if (has_pixmap &&
	    xlib_vendor_is_xorg(dpy) &&
	    VendorRelease(dpy) < XORG_VERSION_ENCODE(1,11,0,1))
		has_pixmap = 0;

	if (has_pixmap) {
		XShmCompletionEvent e;

		e.type = codes->first_event;
		e.send_event = 1;
		e.serial = 1;
		e.drawable = window;
		e.major_code = codes->major_opcode;
		e.minor_code = X_ShmPutImage;

		e.shmseg = shm.shmid;
		e.offset = 0;

		XSendEvent(dpy, e.drawable, False, 0, (XEvent *)&e);
		XSync(dpy, False);

		has_pixmap = _x_error_occurred == 0;
	}

	if (success)
		XShmDetach(dpy, &shm);

	XSync(dpy, False);
	XSetErrorHandler(old_handler);

	shmctl(shm.shmid, IPC_RMID, NULL);
	shmdt(shm.shmaddr);

	if (has_pixmap) {
		*shm_opcode = codes->major_opcode;
		*shm_event = codes->first_event;
		*shm_pixmap = has_pixmap;
	}

	return has_shm;
}

static int mode_equal(const XRRModeInfo *a, const XRRModeInfo *b)
{
	return (a->width == b->width &&
		a->height == b->height &&
		a->dotClock == b->dotClock &&
		a->hSyncStart == b->hSyncStart &&
		a->hSyncEnd == b->hSyncEnd &&
		a->hTotal == b->hTotal &&
		a->hSkew == b->hSkew &&
		a->vSyncStart == b->vSyncStart &&
		a->vSyncEnd == b->vSyncEnd &&
		a->vTotal == b->vTotal &&
		a->modeFlags == b->modeFlags);
}

static RROutput find_output(Display *dpy, const char *name)
{
	XRRScreenResources *res;
	RROutput ret = 0;
	int i;

	res = XRRGetScreenResourcesCurrent(dpy, DefaultRootWindow(dpy));
	if (res == NULL)
		return 0;

	for (i = 0; ret == 0 && i < res->noutput; i++) {
		XRROutputInfo *o = XRRGetOutputInfo(dpy, res, res->outputs[i]);
		if (strcmp(o->name, name) == 0)
			ret = res->outputs[i];
		XRRFreeOutputInfo(o);
	}
	XRRFreeScreenResources(res);

	return ret;
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

static int clone_update_dst(struct output *src, struct output *dst)
{
	XRRScreenResources *src_res, *dst_res;
	XRROutputInfo *src_info, *dst_info;
	int i, j, ret = ENOENT;

	assert(src->rr_output);
	assert(dst->rr_output);

	src_res = XRRGetScreenResources(src->dpy, src->window);
	if (src_res == NULL)
		goto err;

	src_info = XRRGetOutputInfo(src->dpy, src_res, src->rr_output);
	if (src_info == NULL)
		goto err;

	dst_res = XRRGetScreenResourcesCurrent(dst->dpy, dst->window);
	if (dst_res == NULL)
		goto err;

	dst_info = XRRGetOutputInfo(dst->dpy, dst_res, dst->rr_output);
	if (dst_info == NULL)
		goto err;

	/* Clear all current UserModes on the output, including any active ones */
	if (dst_info->crtc)
		XRRSetCrtcConfig(dst->dpy, dst_res, dst_info->crtc, CurrentTime,
				0, 0, None, RR_Rotate_0, NULL, 0);
	for (i = 0; i < dst_info->nmode; i++)
		XRRDeleteOutputMode(dst->dpy, dst->rr_output, dst_info->modes[i]);

	/* Create matching modes for the real output on the virtual */
	for (i = 0; i < src_info->nmode; i++) {
		XRRModeInfo *mode, *old;
		RRMode id;

		mode = lookup_mode(src_res, src_info->modes[i]);
		if (mode == NULL)
			continue;
		for (j = 0; j < i; j++) {
			old = lookup_mode(src_res, src_info->modes[j]);
			if (old && mode_equal(mode, old)) {
				mode = NULL;
				break;
			}
		}
		if (mode == NULL)
			continue;

		id = 0;
		for (j = 0; j < dst_res->nmode; j++) {
			old = &dst_res->modes[j];
			if (mode_equal(mode, old)) {
				id = old->id;
				break;
			}
		}
		if (id == 0) {
			XRRModeInfo m;
			char buf[256];

			/* XXX User names must be unique! */
			m = *mode;
			m.nameLength = snprintf(buf, sizeof(buf),
						"%s.%ld-%s", dst->name, (long)src_info->modes[i], mode->name);
			m.name = buf;

			id = XRRCreateMode(dst->dpy, dst->window, &m);
		}

		XRRAddOutputMode(dst->dpy, dst->rr_output, id);
	}
	ret = 0;

err:
	if (dst_info)
		XRRFreeOutputInfo(dst_info);
	if (dst_res)
		XRRFreeScreenResources(dst_res);
	if (src_info)
		XRRFreeOutputInfo(src_info);
	if (src_res)
		XRRFreeScreenResources(src_res);

	return ret;
}

static int get_current_config(struct output *output)
{
	XRRScreenResources *res;
	int i, ret = ENOENT;
	XRROutputInfo *o;
	RRMode mode = 0;

	res = XRRGetScreenResourcesCurrent(output->dpy, output->window);
	if (res == NULL)
		return ENOMEM;

	o = XRRGetOutputInfo(output->dpy, res, output->rr_output);
	if (o) {
		XRRCrtcInfo *c = NULL;
		if (o->crtc)
			c = XRRGetCrtcInfo(output->dpy, res, o->crtc);
		if (c) {
			output->rotation = c->rotation;
			output->x = c->x;
			output->y = c->y;
			mode = c->mode;
			ret = 0;
		} else
			ret = ENOMEM;
		XRRFreeOutputInfo(o);
	}
	if (ret == 0) {
		ret = EINVAL;
		for (i = 0; ret == EINVAL && i < res->nmode; i++) {
			XRRModeInfo *m = &res->modes[i];
			if (m->id != mode)
				continue;

			output->mode = *m;
			ret = 0;
		}
	}
	XRRFreeScreenResources(res);

	return ret;
}

static int set_config(struct output *output, const struct output *config)
{
	XRRScreenResources *res;
	XRROutputInfo *o;
	XRRCrtcInfo *c;
	int i, ret = ENOENT;
	RRCrtc rr_crtc = 0;
	RRMode rr_mode = 0;
	int x = 0, y = 0;

	res = XRRGetScreenResourcesCurrent(output->dpy, output->window);
	if (res == NULL)
		return ENOMEM;

	o = XRRGetOutputInfo(output->dpy, res, res->outputs[i]);
	if (o) {
		rr_crtc = o->crtcs[0];
		XRRFreeOutputInfo(o);
	}
	if (rr_crtc == 0)
		goto err;

	c = XRRGetCrtcInfo(output->dpy, res, rr_crtc);
	if (c) {
		x = c->x;
		y = c->y;
		XRRFreeCrtcInfo(c);
	}

	for (i = 0; i < res->nmode; i++) {
		if (mode_equal(&config->mode, &res->modes[i])) {
			rr_mode = res->modes[i].id;
			break;
		}
	}
	if (rr_mode == 0)
		goto err;

	XRRSetCrtcConfig(output->dpy, res, rr_crtc, CurrentTime,
			 x, y, rr_mode, config->rotation,
			 &output->rr_output, 1);
	output->x = x;
	output->y = y;
	output->rotation = config->rotation;
	output->mode = config->mode;
	output->display->flush = 1;
	ret = 0;

err:
	XRRFreeScreenResources(res);
	return ret;
}

static void init_image(struct clone *clone)
{
	XImage *image = &clone->image;
	int ret;

	image->width = clone->width;
	image->height = clone->height;
	image->format = ZPixmap;
	image->byte_order = LSBFirst;
	image->bitmap_unit = 32;
	image->bitmap_bit_order = LSBFirst;
	image->red_mask = 0xff << 16;
	image->green_mask = 0xff << 8;
	image->blue_mask = 0xff << 0;;
	image->xoffset = 0;
	image->bitmap_pad = 32;
	image->depth = 24;
	image->data = clone->shm.shmaddr;
	image->bytes_per_line = 4*clone->width;
	image->bits_per_pixel = 32;

	ret = XInitImage(image);
	assert(ret);
}

static void output_create_xfer(struct clone *clone, struct output *output)
{
	if (output->has_shm_pixmap) {
		DBG(("%s-%s: creating shm pixmap\n", DisplayString(output->dpy), output->name));
		if (output->pixmap)
			XFreePixmap(output->dpy, output->pixmap);
		output->pixmap = XShmCreatePixmap(output->dpy, output->window,
						  clone->shm.shmaddr, &clone->shm,
						  clone->width, clone->height, 24);
		if (output->pix_picture) {
			XRenderFreePicture(output->dpy, output->pix_picture);
			output->pix_picture = None;
		}
	}
	if (output->display->rgb24_format != output->display->root_format) {
		DBG(("%s-%s: creating picture\n", DisplayString(output->dpy), output->name));
		if (output->win_picture == None)
			output->win_picture = XRenderCreatePicture(output->dpy, output->window,
								   output->display->root_format, 0, NULL);
		if (output->pixmap == None)
			output->pixmap = XCreatePixmap(output->dpy, output->window,
						       clone->width, clone->height, 24);
		if (output->pix_picture == None)
			output->pix_picture = XRenderCreatePicture(output->dpy, output->pixmap,
								   output->display->rgb24_format, 0, NULL);
	}

	if (output->gc == None) {
		XGCValues gcv;

		gcv.graphics_exposures = False;
		gcv.subwindow_mode = IncludeInferiors;

		output->gc = XCreateGC(output->dpy, output->pixmap ?: output->window, GCGraphicsExposures | GCSubwindowMode, &gcv);
	}
}

static int clone_create_xfer(struct clone *clone)
{
	DBG(("%s-%s create xfer\n",
	     DisplayString(clone->dst.dpy), clone->dst.name));

	clone->width = clone->src.mode.width;
	clone->height = clone->src.mode.height;

	if (clone->shm.shmaddr)
		shmdt(clone->shm.shmaddr);

	clone->shm.shmid = shmget(IPC_PRIVATE, clone->height * clone->width * 4, IPC_CREAT | 0666);
	if (clone->shm.shmid == -1)
		return errno;

	clone->shm.shmaddr = shmat(clone->shm.shmid, 0, 0);
	if (clone->shm.shmaddr == (char *) -1) {
		shmctl(clone->shm.shmid, IPC_RMID, NULL);
		return ENOMEM;
	}

	clone->shm.readOnly = 0;

	init_image(clone);

	if (clone->src.has_shm) {
		XShmAttach(clone->src.dpy, &clone->shm);
		XSync(clone->src.dpy, False);
	}
	if (clone->dst.has_shm) {
		XShmAttach(clone->dst.dpy, &clone->shm);
		XSync(clone->dst.dpy, False);
	}

	shmctl(clone->shm.shmid, IPC_RMID, NULL);

	output_create_xfer(clone, &clone->src);
	output_create_xfer(clone, &clone->dst);

	clone->damaged.x1 = clone->src.x;
	clone->damaged.x2 = clone->src.x + clone->width;
	clone->damaged.y1 = clone->src.y;
	clone->damaged.y2 = clone->src.y + clone->height;

	clone->dst.display->flush = 1;
	return 0;
}

static int clone_update_src(struct clone *clone)
{
	int ret;

	DBG(("%s-%s clone %s\n",
	     DisplayString(clone->dst.dpy), clone->dst.name, clone->src.name));

	ret = get_current_config(&clone->src);
	if (ret)
		return ret;

	set_config(&clone->dst, &clone->src);

	if (clone->src.mode.width != clone->width ||
	    clone->src.mode.height != clone->height)
		clone_create_xfer(clone);

	clone->damaged.x1 = clone->src.x;
	clone->damaged.x2 = clone->src.x + clone->width;
	clone->damaged.y1 = clone->src.y;
	clone->damaged.y2 = clone->src.y + clone->height;

	return 0;
}

static void clone_update(struct clone *clone, int reconfigure)
{
	if (reconfigure)
		clone_update_src(clone);

	if (!clone->rr_update)
		return;

	DBG(("%s-%s cloning modes\n",
	     DisplayString(clone->dst.dpy), clone->dst.name));

	clone_update_dst(&clone->dst, &clone->src);
	clone->rr_update = 0;
}

static Cursor display_load_invisible_cursor(struct display *display)
{
	char zero[8] = {};
	XColor black = {};
	Pixmap bitmap = XCreateBitmapFromData(display->dpy, display->root, zero, 8, 8);
	return XCreatePixmapCursor(display->dpy, bitmap, bitmap, &black, &black, 0, 0);
}

static void display_load_visible_cursor(struct display *display, XFixesCursorImage *cur)
{
	XcursorImage image;

	memset(&image, 0, sizeof(image));
	image.width = cur->width;
	image.height = cur->height;
	image.size = image.width;
	if (image.height > image.size)
		image.size = image.height;
	image.xhot = cur->xhot;
	image.yhot = cur->yhot;
	image.pixels = (void *)cur->pixels;

	if (display->visible_cursor)
		XFreeCursor(display->dpy, display->visible_cursor);

	DBG(("%s updating cursor\n", DisplayString(display->dpy)));
	display->visible_cursor = XcursorImageLoadCursor(display->dpy, &image);

	display->cursor_moved++;
	display->cursor_visible += display->cursor != display->invisible_cursor;
}

static void display_start_cursor_move(struct display *display)
{
	display->cursor_moved = 0;
	display->cursor_visible = 0;
}

static void display_cursor_move(struct display *display, int x, int y, int visible)
{
	display->cursor_moved++;
	display->cursor_visible += visible;
	if (visible) {
		display->cursor_x = x;
		display->cursor_y = y;
	}
}

static void display_end_cursor_move(struct display *display)
{
	Cursor cursor;
	int x, y;

	if (!display->cursor_moved)
		return;

	if (display->cursor_visible) {
		x = display->cursor_x;
		y = display->cursor_y;
	} else {
		x = display->cursor_x++ & 31;
		y = display->cursor_y++ & 31;
	}

	XWarpPointer(display->dpy, None, display->root, 0, 0, 0, 0, x, y);
	display->flush = 1;

	cursor = None;
	if (display->cursor_visible)
		cursor = display->visible_cursor;
	if (cursor == None)
		cursor = display->invisible_cursor;
	if (cursor != display->cursor) {
		XDefineCursor(display->dpy, display->root, cursor);
		display->cursor = cursor;
	}
}

static void clone_move_cursor(struct clone *c, int x, int y)
{
	int visible;

	DBG(("%s-%s moving cursor (%d, %d) [(%d, %d), (%d, %d)]\n",
	     DisplayString(c->dst.dpy), c->dst.name,
	     x, y,
	     c->src.x, c->src.y,
	     c->src.x + c->width, c->src.y + c->height));

	visible = (x >= c->src.x && x < c->src.x + c->width &&
		   y >= c->src.y && y < c->src.y + c->height);

	x += c->dst.x - c->src.x;
	y += c->dst.y - c->src.y;

	display_cursor_move(c->dst.display, x, y, visible);
}

static int clone_output_init(struct clone *clone, struct output *output,
			     struct display *display, const char *name)
{
	Display *dpy = display->dpy;

	DBG(("%s(%s, %s)\n", __func__, DisplayString(dpy), name));

	output->display = display;
	output->dpy = dpy;

	output->rr_output = find_output(dpy, name);
	if (output->rr_output == 0)
		return ENOENT;

	output->name = strdup(name);
	if (output->name == NULL)
		return ENOMEM;

	output->depth = DefaultDepth(dpy, DefaultScreen(dpy));
	output->window = display->root;
	output->has_shm = can_use_shm(dpy, output->window,
				      &output->shm_event,
				      &output->shm_opcode,
				      &output->has_shm_pixmap);

	return 0;
}

static void send_shm(struct output *o, int serial)
{
	XShmCompletionEvent e;

	if (o->shm_event == 0) {
		XSync(o->dpy, False);
		return;
	}

	e.type = o->shm_event;
	e.send_event = 1;
	e.serial = serial;
	e.drawable = o->pixmap;
	e.major_code = o->shm_opcode;
	e.minor_code = X_ShmPutImage;
	e.shmseg = 0;
	e.offset = 0;

	XSendEvent(o->dpy, o->window, False, 0, (XEvent *)&e);
	o->serial = serial;
}

static void get_src(struct clone *c, const XRectangle *clip)
{
	DBG(("%s-%s get_src(%d,%d)x(%d,%d)\n", DisplayString(c->dst.dpy), c->dst.name,
	     clip->x, clip->y, clip->width, clip->height));
	if (c->src.win_picture) {
		XRenderComposite(c->src.dpy, PictOpSrc,
				 c->src.win_picture, 0, c->src.pix_picture,
				 clip->x, clip->y,
				 0, 0,
				 0, 0,
				 clip->width, clip->height);
		if (c->src.has_shm_pixmap) {
			XSync(c->src.dpy, False);
		} else if (c->src.has_shm) {
			c->image.width = clip->width;
			c->image.height = clip->height;
			c->image.obdata = (char *)&c->shm;
			XShmGetImage(c->src.dpy, c->src.pixmap, &c->image,
				     clip->x, clip->y, AllPlanes);
		} else {
			c->image.width = c->width;
			c->image.height = c->height;
			c->image.obdata = 0;
			XGetSubImage(c->src.dpy, c->src.pixmap,
				     clip->x, clip->y, clip->width, clip->height,
				     AllPlanes, ZPixmap,
				     &c->image, 0, 0);
		}
	} else if (c->src.pixmap) {
		XCopyArea(c->src.dpy, c->src.window, c->src.pixmap, c->src.gc,
			  clip->x, clip->y,
			  clip->width, clip->height,
			  0, 0);
		XSync(c->src.dpy, False);
	} else if (c->src.has_shm) {
		c->image.width = clip->width;
		c->image.height = clip->height;
		c->image.obdata = (char *)&c->shm;
		XShmGetImage(c->src.dpy, c->src.window, &c->image,
			     clip->x, clip->y, AllPlanes);
	} else {
		c->image.width = c->width;
		c->image.height = c->height;
		c->image.obdata = 0;
		XGetSubImage(c->src.dpy, c->src.window,
			     clip->x, clip->y, clip->width, clip->height,
			     AllPlanes, ZPixmap,
			     &c->image, 0, 0);
	}
}

static void put_dst(struct clone *c, const XRectangle *clip)
{
	DBG(("%s-%s put_dst(%d,%d)x(%d,%d)\n", DisplayString(c->dst.dpy), c->dst.name,
	     clip->x, clip->y, clip->width, clip->height));
	if (c->dst.win_picture) {
		int serial;
		if (c->dst.has_shm_pixmap) {
		} else if (c->dst.has_shm) {
			c->image.width = clip->width;
			c->image.height = clip->height;
			c->image.obdata = (char *)&c->shm;
			XShmPutImage(c->dst.dpy, c->dst.pixmap, c->dst.gc, &c->image,
				     0, 0,
				     0, 0,
				     clip->width, clip->height,
				     False);
		} else {
			c->image.width = c->width;
			c->image.height = c->height;
			c->image.obdata = 0;
			XPutImage(c->dst.dpy, c->dst.pixmap, c->dst.gc, &c->image,
				  0, 0,
				  0, 0,
				  clip->width, clip->height);
		}
		serial = NextRequest(c->dst.dpy);
		XRenderComposite(c->dst.dpy, PictOpSrc,
				 c->dst.pix_picture, 0, c->dst.win_picture,
				 0, 0,
				 0, 0,
				 clip->x, clip->y,
				 clip->width, clip->height);
		if (c->dst.has_shm)
			send_shm(&c->dst, serial);
	} else if (c->dst.pixmap) {
		int serial = NextRequest(c->dst.dpy);
		XCopyArea(c->dst.dpy, c->dst.pixmap, c->dst.window, c->dst.gc,
			  0, 0,
			  clip->width, clip->height,
			  clip->x, clip->y);
		send_shm(&c->dst, serial);
	} else if (c->dst.has_shm) {
		c->image.width = clip->width;
		c->image.height = clip->height;
		c->image.obdata = (char *)&c->shm;
		c->dst.serial = NextRequest(c->dst.dpy);
		XShmPutImage(c->dst.dpy, c->dst.window, c->dst.gc, &c->image,
			     0, 0,
			     clip->x, clip->y,
			     clip->width, clip->height,
			     True);
	} else {
		c->image.width = c->width;
		c->image.height = c->height;
		c->image.obdata = 0;
		XPutImage(c->dst.dpy, c->dst.window, c->dst.gc, &c->image,
			  0, 0,
			  clip->x, clip->y,
			  clip->width, clip->height);
		c->dst.serial = 0;
	}

	c->dst.display->flush = 1;
}

static int clone_paint(struct clone *c)
{
	XRectangle clip;

	DBG(("%s-%s paint clone\n",
	     DisplayString(c->dst.dpy), c->dst.name));

	if (c->damaged.x1 < c->src.x)
		c->damaged.x1 = c->src.x;
	if (c->damaged.x2 > c->src.x + c->width)
		c->damaged.x2 = c->src.x + c->width;
	if (c->damaged.x2 <= c->damaged.x1)
		goto done;

	if (c->damaged.y1 < c->src.y)
		c->damaged.y1 = c->src.y;
	if (c->damaged.y2 > c->src.y + c->height)
		c->damaged.y2 = c->src.y + c->height;
	if (c->damaged.y2 <= c->damaged.y1)
		goto done;

	if (c->dst.serial > LastKnownRequestProcessed(c->dst.dpy))
		return EAGAIN;

	clip.x = c->damaged.x1;
	clip.y = c->damaged.y1;
	clip.width  = c->damaged.x2 - c->damaged.x1;
	clip.height = c->damaged.y2 - c->damaged.y1;
	get_src(c, &clip);

	clip.x += c->dst.x - c->src.x;
	clip.y += c->dst.y - c->src.y;
	put_dst(c, &clip);

done:
	c->damaged.x2 = c->damaged.y2 = INT_MIN;
	c->damaged.x1 = c->damaged.y1 = INT_MAX;
	return 0;
}

static void clone_damage(struct clone *c, const XRectangle *rec)
{
	if (rec->x < c->damaged.x1)
		c->damaged.x1 = rec->x;
	if (rec->x + rec->width > c->damaged.x2)
		c->damaged.x2 = rec->x + rec->width;
	if (rec->y < c->damaged.y1)
		c->damaged.y1 = rec->y;
	if (rec->y + rec->height > c->damaged.y2)
		c->damaged.y2 = rec->y + rec->height;
}

static void usage(const char *arg0)
{
	printf("usage: %s [-d <source display>] [<target display>] <target output>...\n", arg0);
}

static void record_callback(XPointer closure, XRecordInterceptData *data)
{
	struct context *ctx = (struct context *)closure;
	int n;

	if (data->category == XRecordFromServer) {
		const xEvent *e = (const xEvent *)data->data;

		if (e->u.u.type == MotionNotify) {
			for (n = 0; n < ctx->num_clones; n++)
				clone_move_cursor(&ctx->clones[n],
						  e->u.keyButtonPointer.rootX,
						  e->u.keyButtonPointer.rootY);
		}
	}

	XRecordFreeData(data);
}

static int record_mouse(struct context *ctx)
{
	Display *dpy;
	XRecordRange *rr;
	XRecordClientSpec rcs;
	XRecordContext rc;

	DBG(("%s(%s)\n", __func__, DisplayString(ctx->display[0].dpy)));

	dpy = XOpenDisplay(DisplayString(ctx->display[0].dpy));
	if (dpy == NULL)
		return -ECONNREFUSED;

	rr = XRecordAllocRange();
	if (rr == NULL)
		return -ENOMEM;

	rr->device_events.first = rr->device_events.last = MotionNotify;

	rcs = XRecordAllClients;
	rc = XRecordCreateContext(dpy, 0, &rcs, 1, &rr, 1);

	XSync(dpy, False);

	if (!XRecordEnableContextAsync(dpy, rc, record_callback, (XPointer)ctx))
		return -EINVAL;

	ctx->display[ctx->num_display].dpy = dpy;
	return ConnectionNumber(dpy);
}

static int timer(int hz)
{
	struct itimerspec it;
	int fd;

	fd = timerfd_create(CLOCK_MONOTONIC_COARSE, TFD_NONBLOCK);
	if (fd < 0)
		fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
	if (fd < 0)
		return -ETIME;

	it.it_interval.tv_sec = 0;
	it.it_interval.tv_nsec = 1000000000 / hz;
	it.it_value = it.it_interval;
	if (timerfd_settime(fd, 0, &it, NULL) < 0) {
		close(fd);
		return -ETIME;
	}

	return fd;
}

static int display_init_core(struct display *display)
{
	Display *dpy = display->dpy;
	Visual *visual;
	int major, minor;

	display->root = DefaultRootWindow(dpy);

	visual = DefaultVisual(dpy, DefaultScreen(dpy));
	if (visual->bits_per_rgb != 8 ||
	    visual->red_mask != 0xff << 16 ||
	    visual->green_mask != 0xff << 8 ||
	    visual->blue_mask != 0xff << 0) {
		DBG(("%s: has non-RGB24 visual, forcing render (bpp=%d, red=%lx, green=%lx, blue=%lx)\n",
		     DisplayString(dpy), visual->bits_per_rgb,
		     (long)visual->red_mask,
		     (long)visual->green_mask,
		     (long)visual->blue_mask));

		if (!XRenderQueryVersion(dpy, &major, &minor)) {
			fprintf(stderr, "Render extension not supported by %s\n", DisplayString(dpy));
			return -EINVAL;
		}

		display->root_format = XRenderFindVisualFormat(dpy, visual);
		display->rgb24_format = XRenderFindStandardFormat(dpy, PictStandardRGB24);

		DBG(("%s: root=%x, rgb24=%x\n", DisplayString(dpy),
		     (int)display->root_format->id, (int)display->rgb24_format->id));
	}

	if (!XRRQueryExtension(dpy, &display->rr_event, &display->rr_error)) {
		fprintf(stderr, "RandR extension not supported by %s\n", DisplayString(dpy));
		return -EINVAL;
	}


	display->invisible_cursor = display_load_invisible_cursor(display);
	display->cursor = None;

	return 0;
}

static int display_open(struct display *display, const char *name)
{
	int ret;

	DBG(("%s(%s)\n", __func__, name));

	display->dpy = XOpenDisplay(name);
	if (display->dpy == NULL) {
		fprintf(stderr, "Unable to connect to %s\n", name);
		return -ECONNREFUSED;
	}

	ret = display_init_core(display);
	if (ret)
		return ret;

	return ConnectionNumber(display->dpy);
}

static int bumblebee_open(struct display *display)
{
	char buf[256];
	struct sockaddr_un addr;
	int fd, len;

	fd = socket(PF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
	if (fd < 0)
		goto err;

	addr.sun_family = AF_UNIX;
	strcpy(addr.sun_path, optarg && *optarg ? optarg : "/var/run/bumblebee.socket");
	if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
		goto err;

	/* Ask bumblebee to start the second server */
	buf[0] = 'C';
	if (send(fd, &buf, 1, 0) != 1 || (len = recv(fd, &buf, 255, 0)) <= 0) {
		close(fd);
		goto err;
	}
	buf[len] = '\0';

	/* Query the display name */
	strcpy(buf, "Q VirtualDisplay");
	if (send(fd, buf, 17, 0) != 17 || (len = recv(fd, buf, 255, 0)) <= 0)
		goto err;
	buf[len] = '\0';
	close(fd);

	if (strncmp(buf, "Value: ", 7))
		goto err;

	while (isspace(buf[--len]))
		buf[len] = '\0';

	display->dpy = XOpenDisplay(buf+7);
	if (display->dpy == NULL) {
		fprintf(stderr, "Unable to connect to bumblebee Xserver on %s\n", buf+7);
		return -ECONNREFUSED;
	}

	len = display_init_core(display);
	if (len)
		return len;

	return ConnectionNumber(display->dpy);

err:
	fprintf(stderr, "Unable to connect to bumblebee\n");
	return -ECONNREFUSED;
}

static int display_init_damage(struct display *display)
{
	DBG(("%s(%s)\n", __func__, DisplayString(display->dpy)));

	if (!XDamageQueryExtension(display->dpy, &display->damage_event, &display->damage_error) ||
	    !XFixesQueryExtension(display->dpy, &display->xfixes_event, &display->xfixes_error)) {
		fprintf(stderr, "Damage/Fixes extension not supported by %s\n", DisplayString(display->dpy));
		return EINVAL;
	}

	display->damage = XDamageCreate(display->dpy, display->root, XDamageReportRawRectangles);
	if (display->damage == 0)
		return EACCES;

	return 0;
}

static void display_init_randr_hpd(struct display *display)
{
	int major, minor;

	DBG(("%s(%s)\n", __func__, DisplayString(display->dpy)));

	if (!XRRQueryVersion(display->dpy, &major, &minor))
		return;

	if (major > 1 || (major == 1 && minor >= 2))
		XRRSelectInput(display->dpy, display->root, RROutputChangeNotifyMask);
}

static void display_flush(struct display *display)
{
	if (!display->flush)
		return;

	DBG(("%s(%s)\n", __func__, DisplayString(display->dpy)));

	XFlush(display->dpy);
	display->flush = 0;
}

static int context_init(struct context *ctx, int num_clones)
{
	memset(ctx, 0, sizeof(*ctx));

	ctx->display = calloc(num_clones+2, sizeof(struct display));
	if (ctx->display == NULL)
		return ENOMEM;

	ctx->clones = calloc(num_clones, sizeof(struct clone));
	if (ctx->clones == NULL)
		return ENOMEM;

	return 0;
}

int main(int argc, char **argv)
{
	struct context ctx;
	const char *src_name = NULL;
	struct pollfd *pfd;
	uint64_t count;
	int nfd, enable_timer = 0;
	int i, ret, daemonize = 1;

	while ((i = getopt(argc, argv, "d:fh")) != -1) {
		switch (i) {
		case 'd':
			src_name = optarg;
			break;
		case 'f':
			daemonize = 0;
			break;
		case 'h':
		default:
			usage(argv[0]);
			exit(0);
		}
	}

	count = argc - optind;
	if (count < 2) {
		usage(argv[0]);
		exit(EINVAL);
	}

	pfd = malloc(sizeof(struct pollfd) * (count+3));
	if (pfd == NULL)
		return ENOMEM;

	ret = context_init(&ctx, count);
	if (ret)
		return ret;

	nfd = 1;
	pfd->events = POLLIN;
	pfd->fd = display_open(&ctx.display[0], src_name);
	if (pfd->fd < 0)
		return -pfd->fd;

	ret = display_init_damage(&ctx.display[0]);
	if (ret)
		return ret;

	XRRSelectInput(ctx.display[0].dpy, ctx.display[0].root, RRScreenChangeNotifyMask);
	XFixesSelectCursorInput(ctx.display[0].dpy, ctx.display[0].root, XFixesDisplayCursorNotifyMask);
	ctx.num_display++;

	for (i = optind; i < argc; i++) {
		char buf[80];

		if (strchr(argv[i], ':')) {
			pfd[nfd].fd = display_open(&ctx.display[ctx.num_display++], argv[i]);
			if (pfd[nfd].fd < 0)
				return -pfd[nfd].fd;
			pfd[nfd].events = POLLIN;
			nfd++;

			display_init_randr_hpd(&ctx.display[ctx.num_display-1]);
			continue;
		}
		if (nfd == 1) {
			pfd[nfd].fd = bumblebee_open(&ctx.display[ctx.num_display++]);
			if (pfd[nfd].fd < 0)
				return -pfd[nfd].fd;
			pfd[nfd].events = POLLIN;
			nfd++;

			display_init_randr_hpd(&ctx.display[ctx.num_display-1]);
		}

		sprintf(buf, "VIRTUAL%d", ctx.num_clones+1);
		ret = clone_output_init(&ctx.clones[ctx.num_clones], &ctx.clones[ctx.num_clones].src, &ctx.display[0], buf);
		if (ret) {
			while (++i < argc)
				ctx.num_clones += strchr(argv[i], ':') == NULL;
			fprintf(stderr,
				"No preallocated VirtualHead found for argv[i].\n"
				"Please increase the number of VirtualHeads in xorg.conf:\n"
				"  Section \"Device\"\n"
				"    Identifier \"<identifier>\"\n"
				"    Driver \"intel\"\n"
				"    Option \"VirtualHeads\" \"%d\"\n"
				"    ...\n"
				"  EndSection\n", ctx.num_clones+1);
			return ret;
		}

		ret = clone_output_init(&ctx.clones[ctx.num_clones], &ctx.clones[ctx.num_clones].dst, &ctx.display[ctx.num_display-1], argv[i]);
		if (ret) {
			fprintf(stderr, "Unable to find output \"%s\" on display \"%s\"\n",
				argv[i], DisplayString(ctx.display[nfd-1].dpy));
			return ret;
		}

		ret = clone_update_dst(&ctx.clones[ctx.num_clones].dst, &ctx.clones[ctx.num_clones].src);
		if (ret) {
			fprintf(stderr, "Failed to clone output \"%s\" from display \"%s\"\n",
				argv[i], DisplayString(ctx.display[nfd-1].dpy));
			return ret;
		}

		ctx.num_clones++;
	}

	pfd[nfd].fd = record_mouse(&ctx);
	if (pfd[nfd].fd < 0) {
		fprintf(stderr, "XTEST extension not supported by display \"%s\"\n", DisplayString(ctx.display[0].dpy));
		return -pfd[nfd].fd;
	}
	pfd[nfd].events = POLLIN;
	nfd++;

	pfd[nfd].fd = timer(60);
	if (pfd[nfd].fd < 0) {
		fprintf(stderr, "Failed to setup timer\n");
		return -pfd[nfd].fd;
	}
	pfd[nfd].events = POLLIN;

	if (daemonize && daemon(0, 0))
		return EINVAL;

	while (1) {
		XEvent e;
		int reconfigure = 0;

		ret = poll(pfd, nfd + enable_timer, -1);
		if (ret <= 0)
			break;

		for (i = 1; i < ctx.num_display; i++)
			display_start_cursor_move(&ctx.display[i]);

		if (pfd[0].revents) {
			int damaged = 0;

			do {
				XNextEvent(ctx.display[0].dpy, &e);

				if (e.type == ctx.display[0].damage_event + XDamageNotify ) {
					const XDamageNotifyEvent *de = (const XDamageNotifyEvent *)&e;
					for (i = 0; i < ctx.num_clones; i++)
						clone_damage(&ctx.clones[i], &de->area);
					if (!enable_timer)
						enable_timer = read(pfd[nfd].fd, &count, sizeof(count)) > 0;
					damaged++;
				} else if (e.type == ctx.display[0].xfixes_event + XFixesCursorNotify) {
					XFixesCursorImage *cur;

					cur = XFixesGetCursorImage(ctx.display[0].dpy);
					if (cur == NULL)
						continue;

					for (i = 1; i < ctx.num_display; i++)
						display_load_visible_cursor(&ctx.display[i], cur);

					XFree(cur);
				} else if (e.type == ctx.display[0].rr_event + RRScreenChangeNotify) {
					reconfigure = 1;
					if (!enable_timer)
						enable_timer = read(pfd[nfd].fd, &count, sizeof(count)) > 0;
				} else {
					DBG(("unknown event %d\n", e.type));
				}
			} while (XPending(ctx.display[0].dpy) || poll(pfd, 1, 0) > 0);

			if (damaged)
				XDamageSubtract(ctx.display[0].dpy, ctx.display[0].damage, None, None);
			ret--;
		}

		for (i = 1; ret && i < ctx.num_display; i++) {
			if (pfd[i].revents == 0)
				continue;

			do {
				XNextEvent(ctx.display[i].dpy, &e);

				if (e.type == ctx.display[i].rr_event + RRNotify) {
					XRRNotifyEvent *re = (XRRNotifyEvent *)&e;
					if (re->subtype == RRNotify_OutputChange) {
						XRROutputPropertyNotifyEvent *ro = (XRROutputPropertyNotifyEvent *)re;
						int j;

						for (j = 0; j < ctx.num_clones; j++) {
							if (ctx.clones[j].dst.display == &ctx.display[i] &&
							    ctx.clones[j].dst.rr_output == ro->output)
								ctx.clones[j].rr_update = 1;
						}
					}
				}
			} while (XPending(ctx.display[i].dpy) || poll(&pfd[i], 1, 0) > 0);

			ret--;
		}

		for (i = 0; i < ctx.num_clones; i++)
			clone_update(&ctx.clones[i], reconfigure);

		if (enable_timer && read(pfd[nfd].fd, &count, sizeof(count)) > 0 && count > 0) {
			ret = 0;
			for (i = 0; i < ctx.num_clones; i++)
				ret |= clone_paint(&ctx.clones[i]);
			enable_timer = ret != 0;
		}

		XPending(ctx.display[ctx.num_display].dpy);

		for (i = 1; i < ctx.num_display; i++)
			display_end_cursor_move(&ctx.display[i]);

		for (i = 0; i < ctx.num_display; i++)
			display_flush(&ctx.display[i]);
	}

	return 0;
}
