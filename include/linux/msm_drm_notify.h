/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _MSM_DRM_NOTIFY_H_
#define _MSM_DRM_NOTIFY_H_

#include <linux/notifier.h>
#include <uapi/drm/sde_drm.h>

/* A hardware display blank change occurred */
#define MSM_DRM_EVENT_BLANK			0x01
/* A hardware display blank early change occurred */
#define MSM_DRM_EARLY_EVENT_BLANK		0x02

enum {
	/* panel: power on */
	MSM_DRM_BLANK_UNBLANK	= SDE_MODE_DPMS_ON,
	/* panel: low-power 1 */
	MSM_DRM_BLANK_LP1	= SDE_MODE_DPMS_LP1,
	/* panel: low-power 2 */
	MSM_DRM_BLANK_LP2	= SDE_MODE_DPMS_LP2,
	/* panel: standby */
	MSM_DRM_DPMS_STANDBY	= SDE_MODE_DPMS_STANDBY,
	/* panel: suspend */
	MSM_DRM_BLANK_SUSPEND	= SDE_MODE_DPMS_SUSPEND,
	/* panel: power off */
	MSM_DRM_BLANK_POWERDOWN	= SDE_MODE_DPMS_OFF,
};

enum msm_drm_display_id {
	/* primary display */
	MSM_DRM_PRIMARY_DISPLAY,
	/* external display */
	MSM_DRM_EXTERNAL_DISPLAY,
	MSM_DRM_DISPLAY_MAX
};

struct msm_drm_notifier {
	enum msm_drm_display_id id;
	void *data;
};

int msm_drm_register_client(struct notifier_block *nb);
int msm_drm_unregister_client(struct notifier_block *nb);
int msm_drm_notifier_call_chain(unsigned long val, void *v);

#endif
