/*
 * Copyright © 2009 Nokia Corporation
 * Copyright © 2010 Movial Creative Technologies Oy
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <X11/X.h>
#include <X11/Xutil.h> /* for XDestroyImage */
#include <pixman.h> /* for pixman blt functions */

#include "test.h"

static const struct format {
	const char *name;
	pixman_format_code_t pixman_format;
} formats[] = {
	{ "a8r8g8b8", PIXMAN_a8r8g8b8 },
	{ "x8r8g8b8", PIXMAN_x8r8g8b8 },
	{ "a8", PIXMAN_a8 },
	{ "a4", PIXMAN_a4 },
	{ "a1", PIXMAN_a1 },
};

static const struct op {
	const char *name;
} ops[] = {
	[PictOpClear] = { "Clear" },
	[PictOpSrc] = { "Src" },
	[PictOpDst] = { "Dst" },
	[PictOpOver] = { "Over" },
	[PictOpOverReverse] = { "OverReverse" },
	[PictOpIn] = { "In" },
	[PictOpInReverse] = { "InReverse" },
	[PictOpOut] = { "Out" },
	[PictOpOutReverse] = { "OutReverse" },
	[PictOpAtop] = { "Atop" },
	[PictOpAtopReverse] = { "AtopReverse" },
	[PictOpXor] = { "Xor" },
	[PictOpAdd] = { "Add" },
	[PictOpSaturate] = { "Saturate" },
};

static Picture source_pixmap(struct test_display *t, struct test_target *target, int format)
{
	XRenderColor render_color = { 0x8000, 0x8000, 0x8000, 0x8000 };
	Pixmap pixmap;
	Picture picture;

	pixmap = XCreatePixmap(t->dpy, t->root,
			       target->width, target->height,
			       PIXMAN_FORMAT_DEPTH(formats[format].pixman_format));

	picture = XRenderCreatePicture(t->dpy, pixmap,
				       XRenderFindStandardFormat(t->dpy, format),
				       0, NULL);
	XFreePixmap(t->dpy, pixmap);

	XRenderFillRectangle(t->dpy, PictOpSrc, picture, &render_color,
			     0, 0, target->width, target->height);

	return picture;
}

static Picture source_a8r8g8b8(struct test_display *t, struct test_target *target)
{
	return source_pixmap(t, target, 0);
}

static Picture source_x8r8g8b8(struct test_display *t, struct test_target *target)
{
	return source_pixmap(t, target, 1);
}

static Picture source_a8(struct test_display *t, struct test_target *target)
{
	return source_pixmap(t, target, 2);
}

static Picture source_a4(struct test_display *t, struct test_target *target)
{
	return source_pixmap(t, target, 3);
}

static Picture source_a1(struct test_display *t, struct test_target *target)
{
	return source_pixmap(t, target, 3);
}

static Picture source_1x1r(struct test_display *t, struct test_target *target)
{
	XRenderColor render_color = { 0x8000, 0x8000, 0x8000, 0x8000 };
	XRenderPictureAttributes pa;
	Pixmap pixmap;
	Picture picture;

	pa.repeat = RepeatNormal;

	pixmap = XCreatePixmap(t->dpy, t->root, 1, 1, 32);
	picture = XRenderCreatePicture(t->dpy, pixmap,
				       XRenderFindStandardFormat(t->dpy, 0),
				       CPRepeat, &pa);
	XFreePixmap(t->dpy, pixmap);

	XRenderFillRectangle(t->dpy, PictOpSrc, picture, &render_color,
			     0, 0, 1, 1);

	return picture;
}

static Picture source_solid(struct test_display *t, struct test_target *target)
{
	XRenderColor render_color = { 0x8000, 0x8000, 0x8000, 0x8000 };
	return XRenderCreateSolidFill(t->dpy, &render_color);
}

static Picture source_linear_horizontal(struct test_display *t, struct test_target *target)
{
	XRenderColor colors[2] = {{0}, {0xffff, 0xffff, 0xffff, 0xffff}};
	XFixed stops[2] = {0, 0xffff};
	XLinearGradient gradient = { {0, 0}, {target->width << 16, 0}};

	return XRenderCreateLinearGradient(t->dpy, &gradient, stops, colors, 2);
}

static Picture source_linear_vertical(struct test_display *t, struct test_target *target)
{
	XRenderColor colors[2] = {{0}, {0xffff, 0xffff, 0xffff, 0xffff}};
	XFixed stops[2] = {0, 0xffff};
	XLinearGradient gradient = { {0, 0}, {0, target->height << 16}};

	return XRenderCreateLinearGradient(t->dpy, &gradient, stops, colors, 2);
}

static Picture source_linear_diagonal(struct test_display *t, struct test_target *target)
{
	XRenderColor colors[2] = {{0}, {0xffff, 0xffff, 0xffff, 0xffff}};
	XFixed stops[2] = {0, 0xffff};
	XLinearGradient gradient = { {0, 0}, {target->width << 16, target->height << 16}};

	return XRenderCreateLinearGradient(t->dpy, &gradient, stops, colors, 2);
}

static Picture source_radial_concentric(struct test_display *t, struct test_target *target)
{
	XRenderColor colors[2] = {{0}, {0xffff, 0xffff, 0xffff, 0xffff}};
	XFixed stops[2] = {0, 0xffff};
	XRadialGradient gradient = {
		{
			((target->width << 16) + 1) / 2,
			((target->height << 16) + 1) / 2,
			0,
		},
		{
			((target->width << 16) + 1) / 2,
			((target->height << 16) + 1) / 2,
			target->width << 15,
		}
	};

	return XRenderCreateRadialGradient(t->dpy, &gradient, stops, colors, 2);
}

static Picture source_radial_generic(struct test_display *t, struct test_target *target)
{
	XRenderColor colors[2] = {{0}, {0xffff, 0xffff, 0xffff, 0xffff}};
	XFixed stops[2] = {0, 0xffff};
	XRadialGradient gradient = {
		{ 0, 0, target->width << 14, },
		{ target->width << 16, target->height << 16, target->width << 14, }
	};

	return XRenderCreateRadialGradient(t->dpy, &gradient, stops, colors, 2);
}

static const struct {
	Picture (*create)(struct test_display *, struct test_target *);
	const char *name;
} source[] = {
	{ source_a8r8g8b8, "a8r8g8b8 pixmap" },
	{ source_x8r8g8b8, "x8r8g8b8 pixmap" },
	{ source_a8, "a8 pixmap" },
	{ source_a4, "a4 pixmap" },
	{ source_a1, "a1 pixmap" },
	{ source_1x1r, "a8r8g8b8 1x1R pixmap" },
	{ source_solid, "solid" },
	{ source_linear_horizontal, "linear (horizontal gradient)" },
	{ source_linear_vertical, "linear (vertical gradient)" },
	{ source_linear_diagonal, "linear (diagonal gradient)" },
	{ source_radial_concentric, "radial (concentric)" },
	{ source_radial_generic, "radial (generic)" },
};

static double _bench(struct test_display *t, enum target target_type,
		     int op, int src, int loops)
{
	XRenderColor render_color = { 0x8000, 0x8000, 0x8000, 0x8000 };
	struct test_target target;
	Picture picture;
	struct timespec tv;
	double elapsed;

	test_target_create_render(t, target_type, &target);
	XRenderFillRectangle(t->dpy, PictOpClear, target.picture, &render_color,
			     0, 0, target.width, target.height);

	picture = source[src].create(t, &target);

	test_timer_start(t, &tv);
	while (loops--)
		XRenderComposite(t->dpy, op,
				 picture, 0, target.picture,
				 0, 0,
				 0, 0,
				 0, 0,
				 target.width, target.height);
	elapsed = test_timer_stop(t, &tv);

	XRenderFreePicture(t->dpy, picture);
	test_target_destroy_render(t, &target);

	return elapsed;
}

static void bench(struct test *t, enum target target, int op, int src)
{
	double out, ref;

	ref = _bench(&t->ref, target, op, src, 1000);
	out = _bench(&t->out, target, op, src, 1000);

	fprintf (stdout, "%28s with %s: ref=%f, out=%f\n",
		 source[src].name, ops[op].name, ref, out);
}

int main(int argc, char **argv)
{
	struct test test;
	unsigned op, src;

	test_init(&test, argc, argv);

	for (op = 0; op < sizeof(ops)/sizeof(ops[0]); op++) {
		for (src = 0; src < sizeof(source)/sizeof(source[0]); src++)
			bench(&test, ROOT, op, src);
		fprintf (stdout, "\n");
	}

	return 0;
}
