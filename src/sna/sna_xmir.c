/***************************************************************************

 Copyright 2013 Intel Corporation.  All Rights Reserved.

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

#include <assert.h>

#include "sna.h"

#if XMIR

/* Theory of Operation
 * -------------------
 *
 *  1. Clients render to their pixmaps and Windows aggregating damage.
 *  2. Before blocking, we walk the list of dirty Windows and submit
 *     any damage to Mir. This consumes the xfer buffer.
 *  3. Clients continue to render and we accumulate damage. However,
 *     as there is now no xfer buffer free, damage accumulates.
 *  4. Mir reports that its exchange has complete and gives us a new
 *     transport buffer.
 *  5. Before going to sleep, we iterate over dirty Windows and copy
 *     their damage into the xfer buffer and send back to Mir.
 *
 *  Clients render uninterrupted, but we only send damage to Mir once
 *  every frame.
 */

#define FORCE_FULL_REDRAW 0

static void
sna_xmir_copy_to_mir(xmir_window *xmir_win, RegionPtr region)
{
	PixmapPtr src = get_window_pixmap(xmir_window_to_windowptr(xmir_win));
	struct sna *sna = to_sna_from_pixmap(src);
	const BoxRec *dst_box = xmir_window_get_drawable_region(xmir_win);
	struct sna_pixmap *priv;
	struct kgem_bo *bo;
	BoxRec *box;
	const int pitch = xmir_window_get_stride(xmir_win);
	int16_t sx, sy, dx, dy;
	int n;

#if FORCE_FULL_REDRAW
	RegionRec whole = { { 0, 0, src->drawable.width, src->drawable.height } };
	region = &whole;
#endif

	assert(region);

	DBG(("%s: copying region (%d, %d), (%d, %d) x %d, dst box=(%d, %d), (%d, %d), pitch=%d, fd=%d\n",
	     __FUNCTION__,
	     region->extents.x1, region->extents.y1,
	     region->extents.x2, region->extents.y2,
	     REGION_NUM_RECTS(region),
	     dst_box->x1, dst_box->y1,
	     dst_box->x2, dst_box->y2,
	     pitch, xmir_window_get_fd(xmir_win)));

	box = REGION_RECTS(region);
	n = REGION_NUM_RECTS(region);
	if (n == 0)
		return;

	/* XXX size is bogus, but only used for sanity checks */
	bo = kgem_create_for_prime(&sna->kgem,
				   xmir_window_get_fd(xmir_win),
				   pitch * (dst_box->y2 - dst_box->y1));
	if (bo == NULL)
		return;

	bo->pitch = pitch;
	bo->scanout = true; /* presume the worst (almost always true) */

	if (get_window_deltas(src, &sx, &sy))
		RegionTranslate(region, sx, sy);

	dx = sx + dst_box->x1;
	dy = sy + dst_box->y1;

	priv = sna_pixmap_move_area_to_gpu(src, &region->extents, MOVE_READ);
	if (priv && sna->render.copy_boxes(sna, GXcopy,
					   src, priv->gpu_bo, 0, 0,
					   src, bo, -dx, -dy,
					   box, n, COPY_LAST)) {
		kgem_submit(&sna->kgem);
		n = 0;
	} else {
		void *dst = kgem_bo_map__gtt(&sna->kgem, bo);
		if (dst && sna_drawable_move_region_to_cpu(&src->drawable,
							   region, MOVE_READ)) {
			kgem_bo_sync__gtt(&sna->kgem, bo);
			do {
				memcpy_blt(src->devPrivate.ptr, dst,
					   src->drawable.bitsPerPixel,
					   src->devKind, bo->pitch,
					   box->x1, box->y1,
					   box->x1 - dx, box->y1 - dy,
					   box->x2 - box->x1,
					   box->y2 - box->y1);
			} while (--n);
		}
	}

	if (sx | sy)
		RegionTranslate(region, -sx, -sy);

	if (n == 0)
		xmir_submit_rendering_for_window(xmir_win, region);

	bo->scanout = false; /* but don't confuse our caching! */
	kgem_bo_destroy(&sna->kgem, bo);
}

static xmir_driver sna_xmir_driver = {
	XMIR_DRIVER_VERSION,
	sna_xmir_copy_to_mir
};

bool sna_xmir_create(struct sna *sna)
{
	if (!xorgMir)
		return true;

	sna->xmir = xmir_screen_create(sna->scrn);
	if (sna->xmir == NULL)
		return false;

	sna->flags |= SNA_IS_HOSTED;
	return true;
}

bool sna_xmir_pre_init(struct sna *sna)
{
	if (sna->xmir == NULL)
		return true;

	return xmir_screen_pre_init(sna->scrn, sna->xmir, &sna_xmir_driver);
}

void sna_xmir_init(struct sna *sna, ScreenPtr screen)
{
	if (sna->xmir == NULL)
		return;

	xmir_screen_init(screen, sna->xmir);
}

void sna_xmir_post_damage(struct sna *sna)
{
	if (sna->xmir == NULL)
		return;

	xmir_screen_for_each_damaged_window(sna->xmir,
					    sna_xmir_copy_to_mir);
}

#endif
