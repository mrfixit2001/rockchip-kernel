/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_fbdev.h"

#define PREFERRED_BPP		32

static int rockchip_fbdev_mmap(struct fb_info *info,
			       struct vm_area_struct *vma)
{
	struct drm_fb_helper *helper = info->par;
	struct rockchip_drm_private *private = helper->dev->dev_private;

	return rockchip_gem_mmap_buf(private->fbdev_bo, vma);
}

static struct dma_buf *rockchip_fbdev_get_dma_buf(struct fb_info *info)
{
	struct dma_buf *buf = NULL;
	struct drm_fb_helper *helper = info->par;
	struct rockchip_drm_private *private = helper->dev->dev_private;
	struct drm_device *dev = helper->dev;

	if (dev->driver->gem_prime_export) {
		buf = dev->driver->gem_prime_export(dev, private->fbdev_bo,
						    O_RDWR);
		if (buf)
			drm_gem_object_reference(private->fbdev_bo);
	}

	return buf;
}

static struct fb_ops rockchip_drm_fbdev_ops = {
	.owner		= THIS_MODULE,
	.fb_mmap	= rockchip_fbdev_mmap,
	.fb_fillrect	= drm_fb_helper_cfb_fillrect,
	.fb_copyarea	= drm_fb_helper_cfb_copyarea,
	.fb_imageblit	= drm_fb_helper_cfb_imageblit,
	.fb_check_var	= drm_fb_helper_check_var,
	.fb_set_par	= drm_fb_helper_set_par,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= drm_fb_helper_pan_display,
	.fb_setcmap	= drm_fb_helper_setcmap,
	.fb_dmabuf_export	= rockchip_fbdev_get_dma_buf,
};

static int rockchip_drm_fbdev_create(struct drm_fb_helper *helper,
				     struct drm_fb_helper_surface_size *sizes)
{
	struct rockchip_drm_private *private = helper->dev->dev_private;
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct drm_device *dev = helper->dev;
	struct rockchip_gem_object *rk_obj;
	struct drm_framebuffer *fb;
	unsigned int bytes_per_pixel;
	unsigned long offset;
	struct fb_info *fbi;
	size_t size;
	int ret;
	char *options = NULL;

	bytes_per_pixel = DIV_ROUND_UP(sizes->surface_bpp, 8);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;

	// Allow framebuffer size to be specified in kernel options as "video=rkfb:800x600" (for example)
	if (!fb_get_options("rkfb", &options)) {
		if(options != NULL) {
			char *x;
			long xval, yval;
			DRM_DEBUG_KMS("RKFB VIDEO OPTION: %s\n", options);
			x = strsep(&options,"x");
			if(x != NULL && kstrtol(x, 0, &xval) == 0 && kstrtol(options, 0, &yval) == 0) {
				mode_cmd.width = xval;
				mode_cmd.height = yval;
				sizes->fb_width = xval;
				sizes->fb_height = yval;
				DRM_DEBUG_KMS("RKFB WIDTH x HEIGHT: %d x %d\n", mode_cmd.width, mode_cmd.height);
			}
		}
	}

	mode_cmd.pitches[0] = mode_cmd.width * bytes_per_pixel;
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
		sizes->surface_depth);

	size = mode_cmd.pitches[0] * mode_cmd.height;

	rk_obj = rockchip_gem_create_object(dev, size, true, 0);
	if (IS_ERR(rk_obj))
		return -ENOMEM;

	private->fbdev_bo = &rk_obj->base;

	fbi = drm_fb_helper_alloc_fbi(helper);
	if (IS_ERR(fbi)) {
		dev_err(dev->dev, "Failed to create framebuffer info.\n");
		ret = PTR_ERR(fbi);
		goto err_rockchip_gem_free_object;
	}

	helper->fb = rockchip_drm_framebuffer_init(dev, &mode_cmd,
						   private->fbdev_bo);
	if (IS_ERR(helper->fb)) {
		dev_err(dev->dev, "Failed to allocate DRM framebuffer.\n");
		ret = PTR_ERR(helper->fb);
		goto err_release_fbi;
	}

	fbi->par = helper;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->fbops = &rockchip_drm_fbdev_ops;

	fb = helper->fb;
	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, sizes->fb_width, sizes->fb_height);

	offset = fbi->var.xoffset * bytes_per_pixel;
	offset += fbi->var.yoffset * fb->pitches[0];

	dev->mode_config.fb_base = 0;
	fbi->screen_base = rk_obj->kvaddr + offset;
	fbi->screen_size = rk_obj->base.size;
	fbi->fix.smem_len = rk_obj->base.size;

	DRM_DEBUG_KMS("FB [%dx%d]-%d kvaddr=%p offset=%ld size=%zu\n",
		      fb->width, fb->height, fb->depth, rk_obj->kvaddr,
		      offset, size);

	fbi->skip_vt_switch = true;

	return 0;

err_release_fbi:
	drm_fb_helper_release_fbi(helper);
err_rockchip_gem_free_object:
	rockchip_gem_free_object(&rk_obj->base);
	return ret;
}

static const struct drm_fb_helper_funcs rockchip_drm_fb_helper_funcs = {
	.gamma_set = rockchip_vop_crtc_fb_gamma_set,
	.gamma_get = rockchip_vop_crtc_fb_gamma_get,
	.fb_probe = rockchip_drm_fbdev_create,
};

int rockchip_drm_fbdev_init(struct drm_device *dev)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct drm_fb_helper *helper;
	unsigned int num_crtc;
	int ret;

	if (!dev->mode_config.num_crtc || !dev->mode_config.num_connector)
		return -EINVAL;

	num_crtc = dev->mode_config.num_crtc;

	helper = devm_kzalloc(dev->dev, sizeof(*helper), GFP_KERNEL);
	if (!helper)
		return -ENOMEM;

	drm_fb_helper_prepare(dev, helper, &rockchip_drm_fb_helper_funcs);

	ret = drm_fb_helper_init(dev, helper, num_crtc, ROCKCHIP_MAX_CONNECTOR);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to initialize drm fb helper - %d.\n",
			ret);
		return ret;
	}

	ret = drm_fb_helper_single_add_all_connectors(helper);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to add connectors - %d.\n", ret);
		goto err_drm_fb_helper_fini;
	}

	ret = drm_fb_helper_initial_config(helper, PREFERRED_BPP);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to set initial hw config - %d.\n",
			ret);
		goto err_drm_fb_helper_fini;
	}

	private->fbdev_helper = helper;

	return 0;

err_drm_fb_helper_fini:
	drm_fb_helper_fini(helper);
	return ret;
}

void rockchip_drm_fbdev_fini(struct drm_device *dev)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct drm_fb_helper *helper = private->fbdev_helper;

	if (!helper)
		return;

	drm_fb_helper_unregister_fbi(helper);
	drm_fb_helper_release_fbi(helper);

	if (helper->fb)
		drm_framebuffer_unreference(helper->fb);

	drm_fb_helper_fini(helper);
}
