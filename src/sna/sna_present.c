/*
 * Copyright © 2014 Intel Corporation
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <xf86drm.h>

#include "sna.h"

#include <xf86.h>
#include <present.h>

static present_screen_info_rec present_info;

struct sna_present_event {
	xf86CrtcPtr crtc;
	struct sna *sna;
	struct list link;
	uint64_t *event_id;
	uint64_t target_msc;
	int n_event_id;
};

static void sna_present_unflip(ScreenPtr screen, uint64_t event_id);

static inline struct sna_present_event *
to_present_event(uintptr_t  data)
{
	return (struct sna_present_event *)(data & ~3);
}

#define MARK_PRESENT(x) ((void *)((uintptr_t)(x) | 2))

static int pipe_from_crtc(RRCrtcPtr crtc)
{
	return crtc ? sna_crtc_to_pipe__safe(crtc->devPrivate) : -1;
}

static uint32_t pipe_select(int pipe)
{
	if (pipe > 1)
		return pipe << DRM_VBLANK_HIGH_CRTC_SHIFT;
	else if (pipe > 0)
		return DRM_VBLANK_SECONDARY;
	else
		return 0;
}

static inline int sna_wait_vblank(struct sna *sna, union drm_wait_vblank *vbl, int pipe)
{
	DBG(("%s(pipe=%d, waiting until seq=%u%s)\n",
	     __FUNCTION__, pipe, vbl->request.sequence,
	     vbl->request.type & DRM_VBLANK_RELATIVE ? " [relative]" : ""));
	vbl->request.type |= pipe_select(pipe);
	return drmIoctl(sna->kgem.fd, DRM_IOCTL_WAIT_VBLANK, vbl);
}

static uint64_t gettime_ust64(void)
{
	struct timespec tv;

	if (clock_gettime(CLOCK_MONOTONIC, &tv))
		return GetTimeInMicros();

	return ust64(tv.tv_sec, tv.tv_nsec / 1000);
}

static void vblank_complete(struct sna_present_event *info,
			    uint64_t ust, uint64_t msc)
{
	int n;

	DBG(("%s: %d events complete\n", __FUNCTION__, info->n_event_id));
	for (n = 0; n < info->n_event_id; n++) {
		DBG(("%s: pipe=%d tv=%d.%06d msc=%lld (target=%lld), event=%lld complete%s\n", __FUNCTION__,
		     sna_crtc_to_pipe(info->crtc),
		     (int)(ust / 1000000), (int)(ust % 1000000),
		     (long long)msc, (long long)info->target_msc,
		     (long long)info->event_id[n],
		     info->target_msc && msc == (uint32_t)info->target_msc ? "" : ": MISS"));
		present_event_notify(info->event_id[n], ust, msc);
	}
	if (info->n_event_id > 1)
		free(info->event_id);
	list_del(&info->link);
	free(info);
}

static uint32_t msc_to_delay(xf86CrtcPtr crtc, uint64_t target)
{
	const DisplayModeRec *mode = &crtc->desiredMode;
	const struct ust_msc *swap = sna_crtc_last_swap(crtc);
	int64_t delay, subframe;

	delay = (target - swap->msc) * mode->VTotal * mode->HTotal / mode->Clock;
	subframe = gettime_ust64() - swap_ust(swap);
	subframe /= 1000;
	if (subframe < delay)
		delay -= subframe;
	else
		delay = 0;

	DBG(("%s: sleep %d frames, %llu ms\n", __FUNCTION__,
	     (int)(target - swap->msc), (long long)delay));
	assert(delay >= 0);
	return delay;
}

static CARD32 sna_fake_vblank_handler(OsTimerPtr timer, CARD32 now, void *data)
{
	struct sna_present_event *info = data;
	union drm_wait_vblank vbl;
	uint64_t msc, ust;

	DBG(("%s(event=%lldx%d, now=%d)\n", __FUNCTION__, (long long)info->event_id[0], info->n_event_id, now));

	VG_CLEAR(vbl);
	vbl.request.type = DRM_VBLANK_RELATIVE;
	vbl.request.sequence = 0;
	if (sna_wait_vblank(info->sna, &vbl, sna_crtc_to_pipe(info->crtc)) == 0) {
		ust = ust64(vbl.reply.tval_sec, vbl.reply.tval_usec);
		msc = sna_crtc_record_vblank(info->crtc, &vbl);
		DBG(("%s: event=%lld, target msc=%lld, now %lld\n",
		     __FUNCTION__, (long long)info->event_id, (long long)info->target_msc, (long long)msc));
		if (msc < info->target_msc) {
			uint32_t delay = msc_to_delay(info->crtc, info->target_msc);
			if (delay)
				return delay;
		}
	} else {
		const struct ust_msc *swap = sna_crtc_last_swap(info->crtc);
		ust = swap_ust(swap);
		msc = swap->msc;
		DBG(("%s: event=%lld, CRTC OFF, target msc=%lld, was %lld\n",
		     __FUNCTION__, (long long)info->event_id, (long long)info->target_msc, (long long)msc));
	}

	vblank_complete(info, ust, msc);
	free(timer);
	return 0;
}

static bool sna_fake_vblank(struct sna_present_event *info)
{
	uint64_t msc = sna_crtc_last_swap(info->crtc)->msc;
	uint32_t delay;

	assert(info->n_event_id == 1);

	if (msc < info->target_msc)
		delay = msc_to_delay(info->crtc, info->target_msc);
	else
		delay = 0;

	DBG(("%s(event=%lld, target_msc=%lld, msc=%lld, delay=%ums)\n",
	     __FUNCTION__, (long long)info->event_id, (long long)info->target_msc, msc, delay));
	if (delay == 0) {
		const struct ust_msc *swap = sna_crtc_last_swap(info->crtc);
		present_event_notify(info->event_id[0], swap_ust(swap), swap->msc);
		list_del(&info->link);
		free(info);
		return true;
	}

	return TimerSet(NULL, 0, delay, sna_fake_vblank_handler, info);
}

static RRCrtcPtr
sna_present_get_crtc(WindowPtr window)
{
	struct sna *sna = to_sna_from_drawable(&window->drawable);
	BoxRec box;
	xf86CrtcPtr crtc;

	DBG(("%s\n", __FUNCTION__));

	box.x1 = window->drawable.x;
	box.y1 = window->drawable.y;
	box.x2 = box.x1 + window->drawable.width;
	box.y2 = box.y1 + window->drawable.height;

	crtc = sna_covering_crtc(sna, &box, NULL);
	if (crtc)
		return crtc->randr_crtc;

	return NULL;
}

static int
sna_present_get_ust_msc(RRCrtcPtr crtc, CARD64 *ust, CARD64 *msc)
{
	struct sna *sna = to_sna_from_screen(crtc->pScreen);
	int pipe = pipe_from_crtc(crtc);
	union drm_wait_vblank vbl;

	DBG(("%s(pipe=%d)\n", __FUNCTION__, pipe));

	VG_CLEAR(vbl);
	vbl.request.type = DRM_VBLANK_RELATIVE;
	vbl.request.sequence = 0;
	if (sna_wait_vblank(sna, &vbl, pipe) == 0) {
		*ust = ust64(vbl.reply.tval_sec, vbl.reply.tval_usec);
		*msc = sna_crtc_record_vblank(crtc->devPrivate, &vbl);
	} else {
		const struct ust_msc *swap = sna_crtc_last_swap(crtc->devPrivate);
		*ust = swap_ust(swap);
		*msc = swap->msc;
	}

	DBG(("%s: pipe=%d, tv=%d.%06d msc=%lld\n", __FUNCTION__, pipe,
	     (int)(*ust / 1000000), (int)(*ust % 1000000),
	     (long long)*msc));

	return Success;
}

void
sna_present_vblank_handler(struct drm_event_vblank *event)
{
	struct sna_present_event *info = to_present_event(event->user_data);

	vblank_complete(info,
			ust64(event->tv_sec, event->tv_usec),
			sna_crtc_record_event(info->crtc, event));
}

static int
sna_present_queue_vblank(RRCrtcPtr crtc, uint64_t event_id, uint64_t msc)
{
	struct sna *sna = to_sna_from_screen(crtc->pScreen);
	struct sna_present_event *info, *tmp;
	const struct ust_msc *swap;
	union drm_wait_vblank vbl;

	DBG(("%s(pipe=%d, event=%lld, msc=%lld)\n",
	     __FUNCTION__, pipe_from_crtc(crtc),
	     (long long)event_id, (long long)msc));

	swap = sna_crtc_last_swap(crtc->devPrivate);
	assert((int64_t)(msc - swap->msc) >= 0);
	if ((int64_t)(msc - swap->msc) <= 0) {
		DBG(("%s: pipe=%d tv=%d.%06d msc=%lld (target=%lld), event=%lld complete\n", __FUNCTION__,
		     pipe_from_crtc(crtc),
		     swap->tv_sec, swap->tv_usec,
		     (long long)swap->msc, (long long)msc,
		     (long long)event_id));
		present_event_notify(event_id, swap_ust(swap), swap->msc);
		return Success;
	}

	list_for_each_entry(tmp, &sna->present.vblank_queue, link) {
		if (tmp->target_msc == msc && tmp->crtc == crtc->devPrivate) {
			uint64_t *events = tmp->event_id;

			if (is_power_of_two(tmp->n_event_id)) {
				events = malloc(2*sizeof(uint64_t)*tmp->n_event_id);
				if (events == NULL)
					goto fail;

				memcpy(events,
				       tmp->event_id,
				       tmp->n_event_id*sizeof(uint64_t));
				if (tmp->n_event_id != 1)
					free(tmp->event_id);
				tmp->event_id = events;
			}

			DBG(("%s: appending event=%lld to vblank %lld x %d\n",
			     __FUNCTION__, (long long)event_id, (long long)msc, tmp->n_event_id+1));
			events[tmp->n_event_id++] = event_id;
			return Success;
		}
		if ((int64_t)(tmp->target_msc - msc) > 0)
			break;
	}

fail:
	info = malloc(sizeof(struct sna_present_event) + sizeof(uint64_t));
	if (info == NULL)
		return BadAlloc;

	info->crtc = crtc->devPrivate;
	info->sna = sna;
	info->target_msc = msc;
	info->event_id = (uint64_t *)(info + 1);
	info->event_id[0] = event_id;
	info->n_event_id = 1;
	list_add_tail(&info->link, &tmp->link);

	VG_CLEAR(vbl);
	vbl.request.type = DRM_VBLANK_ABSOLUTE | DRM_VBLANK_EVENT;
	vbl.request.sequence = msc;
	vbl.request.signal = (uintptr_t)MARK_PRESENT(info);
	if (sna_wait_vblank(sna, &vbl, sna_crtc_to_pipe(info->crtc))) {
		DBG(("%s: vblank enqueue failed\n", __FUNCTION__));
		if (!sna_fake_vblank(info)) {
			list_del(&info->link);
			free(info);
			return BadAlloc;
		}
	}

	return Success;
}

static void
sna_present_abort_vblank(RRCrtcPtr crtc, uint64_t event_id, uint64_t msc)
{
	DBG(("%s(pipe=%d, event=%lld, msc=%lld)\n",
	     __FUNCTION__, pipe_from_crtc(crtc),
	     (long long)event_id, (long long)msc));
}

static void
sna_present_flush(WindowPtr window)
{
	PixmapPtr pixmap = get_window_pixmap(window);
	struct sna_pixmap *priv;

	DBG(("%s(pixmap=%ld)\n", __FUNCTION__, pixmap->drawable.serialNumber));

	priv = sna_pixmap_move_to_gpu(pixmap, MOVE_READ | MOVE_ASYNC_HINT | __MOVE_FORCE);
	if (priv && priv->gpu_bo)
		kgem_scanout_flush(&to_sna_from_pixmap(pixmap)->kgem, priv->gpu_bo);
}

static bool
check_flip__crtc(struct sna *sna,
		 RRCrtcPtr crtc)
{
	if (!sna_crtc_is_on(crtc->devPrivate)) {
		DBG(("%s: CRTC off\n", __FUNCTION__));
		return false;
	}

	assert(sna->scrn->vtSema);

	if (!sna->mode.front_active) {
		DBG(("%s: DPMS off, no flips\n", __FUNCTION__));
		return FALSE;
	}

	if (sna->mode.rr_active) {
		DBG(("%s: RandR transformation active\n", __FUNCTION__));
		return false;
	}

	return true;
}

static Bool
sna_present_check_flip(RRCrtcPtr crtc,
		       WindowPtr window,
		       PixmapPtr pixmap,
		       Bool sync_flip)
{
	struct sna *sna = to_sna_from_pixmap(pixmap);
	struct sna_pixmap *flip;

	DBG(("%s(pipe=%d, pixmap=%ld, sync_flip=%d)\n",
	     __FUNCTION__,
	     pipe_from_crtc(crtc),
	     pixmap->drawable.serialNumber,
	     sync_flip));

	if (!sna->scrn->vtSema) {
		DBG(("%s: VT switched away, no flips\n", __FUNCTION__));
		return FALSE;
	}

	if (sna->flags & SNA_NO_FLIP) {
		DBG(("%s: flips not suported\n", __FUNCTION__));
		return FALSE;
	}

	if (sync_flip) {
		if ((sna->flags & SNA_HAS_FLIP) == 0) {
			DBG(("%s: sync flips not suported\n", __FUNCTION__));
			return FALSE;
		}
	} else {
		if ((sna->flags & SNA_HAS_ASYNC_FLIP) == 0) {
			DBG(("%s: async flips not suported\n", __FUNCTION__));
			return FALSE;
		}
	}

	if (!check_flip__crtc(sna, crtc)) {
		DBG(("%s: flip invalid for CRTC\n", __FUNCTION__));
		return FALSE;
	}

	flip = sna_pixmap(pixmap);
	if (flip == NULL) {
		DBG(("%s: unattached pixmap\n", __FUNCTION__));
		return FALSE;
	}

	if (flip->cpu_bo && IS_STATIC_PTR(flip->ptr)) {
		DBG(("%s: SHM pixmap\n", __FUNCTION__));
		return FALSE;
	}

	return TRUE;
}

static Bool
flip__async(struct sna *sna,
	    RRCrtcPtr crtc,
	    uint64_t event_id,
	    uint64_t target_msc,
	    struct kgem_bo *bo)
{
	DBG(("%s(pipe=%d, event=%lld, handle=%d)\n",
	     __FUNCTION__,
	     pipe_from_crtc(crtc),
	     (long long)event_id,
	     bo->handle));

	if (!sna_page_flip(sna, bo, NULL, NULL)) {
		DBG(("%s: async pageflip failed\n", __FUNCTION__));
		present_info.capabilities &= ~PresentCapabilityAsync;
		return FALSE;
	}

	DBG(("%s: pipe=%d tv=%d.%06d msc=%lld (target=%lld), event=%lld complete\n", __FUNCTION__,
	     pipe_from_crtc(crtc),
	     gettime_ust64() / 1000000, gettime_ust64() % 1000000,
	     crtc ? (long long)sna_crtc_last_swap(crtc->devPrivate)->msc : 0LL,
	     (long long)target_msc, (long long)event_id));
	present_event_notify(event_id, gettime_ust64(), target_msc);
	return TRUE;
}

static void
present_flip_handler(struct drm_event_vblank *event, void *data)
{
	struct sna_present_event *info = data;
	struct ust_msc swap;

	DBG(("%s(sequence=%d): event=%lld\n", __FUNCTION__, event->sequence, (long long)info->event_id[0]));
	assert(info->n_event_id == 1);

	if (info->crtc == NULL) {
		swap.tv_sec = event->tv_sec;
		swap.tv_usec = event->tv_usec;
		swap.msc = event->sequence;
	} else
		swap = *sna_crtc_last_swap(info->crtc);

	DBG(("%s: pipe=%d, tv=%d.%06d msc=%lld (target %lld), event=%lld complete%s\n", __FUNCTION__,
	     info->crtc ? sna_crtc_to_pipe(info->crtc) : -1,
	     swap.tv_sec, swap.tv_usec, (long long)swap.msc,
	     (long long)info->target_msc,
	     (long long)info->event_id[0],
	     info->target_msc && info->target_msc == swap.msc ? "" : ": MISS"));
	present_event_notify(info->event_id[0], swap_ust(&swap), swap.msc);

	if (info->sna->present.unflip) {
		DBG(("%s: executing queued unflip (event=%lld)\n", __FUNCTION__, info->sna->present.unflip));
		sna_present_unflip(xf86ScrnToScreen(info->sna->scrn),
				   info->sna->present.unflip);
		info->sna->present.unflip = 0;
	}
	free(info);
}

static Bool
flip(struct sna *sna,
     RRCrtcPtr crtc,
     uint64_t event_id,
     uint64_t target_msc,
     struct kgem_bo *bo)
{
	struct sna_present_event *info;

	DBG(("%s(pipe=%d, event=%lld, handle=%d)\n",
	     __FUNCTION__,
	     pipe_from_crtc(crtc),
	     (long long)event_id,
	     bo->handle));

	info = malloc(sizeof(struct sna_present_event)+sizeof(uint64_t));
	if (info == NULL)
		return FALSE;

	info->crtc = crtc ? crtc->devPrivate : NULL;
	info->sna = sna;
	info->event_id = (uint64_t *)(info + 1);
	info->event_id[0] = event_id;
	info->n_event_id = 1;
	info->target_msc = target_msc;

	if (!sna_page_flip(sna, bo, present_flip_handler, info)) {
		DBG(("%s: pageflip failed\n", __FUNCTION__));
		free(info);
		return FALSE;
	}

	return TRUE;
}

static struct kgem_bo *
get_flip_bo(PixmapPtr pixmap)
{
	struct sna *sna = to_sna_from_pixmap(pixmap);
	struct sna_pixmap *priv;

	DBG(("%s(pixmap=%ld)\n", __FUNCTION__, pixmap->drawable.serialNumber));

	priv = sna_pixmap_move_to_gpu(pixmap, MOVE_READ | __MOVE_SCANOUT | __MOVE_FORCE);
	if (priv == NULL) {
		DBG(("%s: cannot force pixmap to the GPU\n", __FUNCTION__));
		return NULL;
	}

	if (priv->gpu_bo->scanout)
		return priv->gpu_bo;

	if (sna->kgem.has_llc && !wedged(sna)) {
		struct kgem_bo *bo;
		uint32_t tiling;

		tiling = I915_TILING_NONE;
		if ((sna->flags & SNA_LINEAR_FB) == 0)
			tiling = I915_TILING_X;

		bo = kgem_create_2d(&sna->kgem,
				    pixmap->drawable.width,
				    pixmap->drawable.height,
				    pixmap->drawable.bitsPerPixel,
				    tiling, CREATE_SCANOUT | CREATE_CACHED);
		if (bo) {
			BoxRec box;

			box.x1 = box.y1 = 0;
			box.x2 = pixmap->drawable.width;
			box.y2 = pixmap->drawable.height;

			if (sna->render.copy_boxes(sna, GXcopy,
						   &pixmap->drawable, priv->gpu_bo, 0, 0,
						   &pixmap->drawable, bo, 0, 0,
						   &box, 1, 0)) {
				sna_pixmap_unmap(pixmap, priv);
				kgem_bo_destroy(&sna->kgem, priv->gpu_bo);

				priv->gpu_bo = bo;
			} else
				kgem_bo_destroy(&sna->kgem, bo);
		}
	}

	if (sna->flags & SNA_LINEAR_FB &&
	    priv->gpu_bo->tiling &&
	    !sna_pixmap_change_tiling(pixmap, I915_TILING_NONE)) {
		DBG(("%s: invalid tiling for scanout, user requires linear\n", __FUNCTION__));
		return NULL;
	}

	if (priv->gpu_bo->tiling == I915_TILING_Y &&
	    !sna_pixmap_change_tiling(pixmap, I915_TILING_X)) {
		DBG(("%s: invalid Y-tiling, cannot convert\n", __FUNCTION__));
		return NULL;
	}

	return priv->gpu_bo;
}

static Bool
sna_present_flip(RRCrtcPtr crtc,
		 uint64_t event_id,
		 uint64_t target_msc,
		 PixmapPtr pixmap,
		 Bool sync_flip)
{
	struct sna *sna = to_sna_from_pixmap(pixmap);
	struct kgem_bo *bo;

	DBG(("%s(pipe=%d, event=%lld, msc=%lld, pixmap=%ld, sync?=%d)\n",
	     __FUNCTION__,
	     pipe_from_crtc(crtc),
	     (long long)event_id,
	     (long long)target_msc,
	     pixmap->drawable.serialNumber, sync_flip));

	if (!check_flip__crtc(sna, crtc)) {
		DBG(("%s: flip invalid for CRTC\n", __FUNCTION__));
		return FALSE;
	}

	assert(sna->present.unflip == 0);

	if (sna->flags & SNA_TEAR_FREE)
		sna->mode.shadow_enabled = false;
	assert(!sna->mode.shadow_enabled);

	if (sna->mode.flip_active) {
		DBG(("%s: flips still pending\n", __FUNCTION__));
		return FALSE;
	}

	bo = get_flip_bo(pixmap);
	if (bo == NULL) {
		DBG(("%s: flip invalid bo\n", __FUNCTION__));
		return FALSE;
	}

	if (sync_flip)
		return flip(sna, crtc, event_id, target_msc, bo);
	else
		return flip__async(sna, crtc, event_id, target_msc, bo);
}

static void
sna_present_unflip(ScreenPtr screen, uint64_t event_id)
{
	struct sna *sna = to_sna_from_screen(screen);
	struct kgem_bo *bo;

	DBG(("%s(event=%lld)\n", __FUNCTION__, (long long)event_id));
	if (sna->mode.front_active == 0 || sna->mode.rr_active) {
		const struct ust_msc *swap;

		DBG(("%s: no CRTC active, perform no-op flip\n", __FUNCTION__));

notify:
		swap = sna_crtc_last_swap(sna_primary_crtc(sna));
		DBG(("%s: pipe=%d, tv=%d.%06d msc=%lld, event=%lld complete\n", __FUNCTION__,
		     -1,
		     swap->tv_sec, swap->tv_usec, (long long)swap->msc,
		     (long long)event_id));
		present_event_notify(event_id, swap_ust(swap), swap->msc);
		return;
	}

	if (sna->mode.flip_active) {
		DBG(("%s: %d outstanding flips, queueing unflip\n", __FUNCTION__, sna->mode.flip_active));
		assert(sna->present.unflip == 0);
		sna->present.unflip = event_id;
		return;
	}

	if (sna->flags & SNA_TEAR_FREE)
		sna->mode.shadow_enabled = sna->mode.shadow_damage != NULL;

	bo = get_flip_bo(screen->GetScreenPixmap(screen));
	if (bo == NULL) {
reset_mode:
		DBG(("%s: failed, trying to restore original mode\n", __FUNCTION__));
		xf86SetDesiredModes(sna->scrn);
		goto notify;
	}

	assert(sna_pixmap(screen->GetScreenPixmap(screen))->pinned & PIN_SCANOUT);

	if (sna->flags & SNA_HAS_ASYNC_FLIP) {
		DBG(("%s: trying async flip restore\n", __FUNCTION__));
		if (flip__async(sna, NULL, event_id, 0, bo))
			return;
	}

	if (!flip(sna, NULL, event_id, 0, bo))
		goto reset_mode;
}

static present_screen_info_rec present_info = {
	.version = PRESENT_SCREEN_INFO_VERSION,

	.get_crtc = sna_present_get_crtc,
	.get_ust_msc = sna_present_get_ust_msc,
	.queue_vblank = sna_present_queue_vblank,
	.abort_vblank = sna_present_abort_vblank,
	.flush = sna_present_flush,

	.capabilities = PresentCapabilityNone,
	.check_flip = sna_present_check_flip,
	.flip = sna_present_flip,
	.unflip = sna_present_unflip,
};

bool sna_present_open(struct sna *sna, ScreenPtr screen)
{
	DBG(("%s(num_crtc=%d)\n", __FUNCTION__, sna->mode.num_real_crtc));

	if (sna->mode.num_real_crtc == 0)
		return false;

	sna_present_update(sna);
	list_init(&sna->present.vblank_queue);

	return present_screen_init(screen, &present_info);
}

void sna_present_update(struct sna *sna)
{
	if (sna->flags & SNA_HAS_ASYNC_FLIP)
		present_info.capabilities |= PresentCapabilityAsync;
	else
		present_info.capabilities &= ~PresentCapabilityAsync;

	DBG(("%s: has_async_flip? %d\n", __FUNCTION__,
	     !!(present_info.capabilities & PresentCapabilityAsync)));
}

void sna_present_close(struct sna *sna, ScreenPtr screen)
{
	DBG(("%s()\n", __FUNCTION__));
}
