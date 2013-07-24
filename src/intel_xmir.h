#ifndef INTEL_XMIR_H
#define INTEL_XMIR_H

#include <xorg-server.h>

#if XMIR
#include <xf86Priv.h>
#include <xmir.h>
#else
typedef struct xmir_screen xmir_screen;
#define xorgMir 0
#define xmir_get_drm_fd(id) -1
#endif

#endif /* INTEL_XMIR_H */
