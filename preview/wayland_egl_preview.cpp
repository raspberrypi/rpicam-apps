/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, Raspberry Pi Ltd.
 *
 * wayland_egl_preview.cpp - Native Wayland EGL-based preview window.
 *
 * This is the native-Wayland counterpart to egl_preview.cpp. The GL path
 * (dmabuf -> EGLImage -> GL_OES_EGL_image_external -> draw) is identical; only
 * the windowing layer differs. On a Wayland compositor this avoids the
 * XWayland round-trip (and its per-frame texture-format copy) that the X11 EGL
 * preview incurs.
 */

#include <cstring>
#include <map>
#include <poll.h>
#include <string>

#include "core/options.hpp"

#include "preview.hpp"

#include <libdrm/drm_fourcc.h>

#include <wayland-client.h>
#include <wayland-egl.h>

#include "xdg-decoration-unstable-v1-client-protocol.h"
#include "xdg-shell-client-protocol.h"

#include <epoxy/egl.h>
#include <epoxy/gl.h>

class WaylandEglPreview : public Preview
{
public:
	WaylandEglPreview(Options const *options);
	~WaylandEglPreview();
	virtual void SetInfoText(const std::string &text) override;
	// Display the buffer. You get given the fd back in the BufferDoneCallback
	// once its available for re-use.
	virtual void Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info) override;
	// Reset the preview window, clearing the current buffers and being ready to
	// show new ones.
	virtual void Reset() override;
	// Check if the compositor has closed the preview.
	virtual bool Quit() override;
	// Return the maximum image size allowed.
	virtual void MaxImageSize(unsigned int &w, unsigned int &h) const override
	{
		w = max_image_width_;
		h = max_image_height_;
	}

private:
	struct Buffer
	{
		Buffer() : fd(-1)
		{
		}
		int fd;
		size_t size;
		StreamInfo info;
		GLuint texture;
	};
	void makeWindow(char const *name);
	void makeBuffer(int fd, size_t size, StreamInfo const &info, Buffer &buffer);
	void configureGl(int image_width, int image_height, int window_width, int window_height);
	void dispatchPending();

	// Wayland registry callbacks.
	static void registryGlobal(void *data, wl_registry *registry, uint32_t name, const char *interface,
							   uint32_t version);
	static void registryGlobalRemove(void *data, wl_registry *registry, uint32_t name);
	static void wmBasePing(void *data, xdg_wm_base *wm_base, uint32_t serial);
	static void xdgSurfaceConfigure(void *data, xdg_surface *xdg_surface, uint32_t serial);
	static void xdgToplevelConfigure(void *data, xdg_toplevel *toplevel, int32_t width, int32_t height,
									 wl_array *states);
	static void xdgToplevelClose(void *data, xdg_toplevel *toplevel);

	// Wayland objects.
	wl_display *display_;
	wl_registry *registry_;
	wl_compositor *compositor_;
	xdg_wm_base *wm_base_;
	zxdg_decoration_manager_v1 *decoration_manager_;
	wl_surface *surface_;
	xdg_surface *xdg_surface_;
	xdg_toplevel *xdg_toplevel_;
	zxdg_toplevel_decoration_v1 *toplevel_decoration_;
	wl_egl_window *egl_window_;

	// EGL/GL objects.
	EGLDisplay egl_display_;
	EGLConfig egl_config_;
	EGLContext egl_context_;
	EGLSurface egl_surface_;
	GLuint gl_program_;
	// Vertex data for the letterboxed quad. Owned per-instance as it is the
	// client-side array bound via glVertexAttribPointer, so it must persist for
	// as long as it is bound, and is rewritten on every resize.
	float verts_[8];

	std::map<int, Buffer> buffers_; // map the DMABUF's fd to the Buffer
	int last_fd_;
	bool first_time_;
	bool quit_;
	// Size of preview window.
	int x_;
	int y_;
	int width_;
	int height_;
	// Latest size requested by the compositor (0 if none pending).
	int pending_width_;
	int pending_height_;
	// Dimensions of the last image shown, so we can re-letterbox on resize.
	int image_width_;
	int image_height_;
	unsigned int max_image_width_;
	unsigned int max_image_height_;
};

static GLint compile_shader(GLenum target, const char *source)
{
	GLuint s = glCreateShader(target);
	glShaderSource(s, 1, (const GLchar **)&source, NULL);
	glCompileShader(s);

	GLint ok;
	glGetShaderiv(s, GL_COMPILE_STATUS, &ok);

	if (!ok)
	{
		GLchar *info;
		GLint size;

		glGetShaderiv(s, GL_INFO_LOG_LENGTH, &size);
		info = (GLchar *)malloc(size);

		glGetShaderInfoLog(s, size, NULL, info);
		throw std::runtime_error("failed to compile shader: " + std::string(info) + "\nsource:\n" +
								 std::string(source));
	}

	return s;
}

static GLint link_program(GLint vs, GLint fs)
{
	GLint prog = glCreateProgram();
	glAttachShader(prog, vs);
	glAttachShader(prog, fs);
	glLinkProgram(prog);

	GLint ok;
	glGetProgramiv(prog, GL_LINK_STATUS, &ok);
	if (!ok)
	{
		/* Some drivers return a size of 1 for an empty log.  This is the size
		 * of a log that contains only a terminating NUL character.
		 */
		GLint size;
		GLchar *info = NULL;
		glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &size);
		if (size > 1)
		{
			info = (GLchar *)malloc(size);
			glGetProgramInfoLog(prog, size, NULL, info);
		}

		throw std::runtime_error("failed to link: " + std::string(info ? info : "<empty log>"));
	}

	return prog;
}

// Build the GL program and vertex data for the given image and window sizes,
// letterboxing the image to preserve its aspect ratio. Unlike the X11 preview
// this may be called repeatedly (on live resize), so the previous program is
// deleted first.
void WaylandEglPreview::configureGl(int image_width, int image_height, int window_width, int window_height)
{
	float w_factor = image_width / (float)window_width;
	float h_factor = image_height / (float)window_height;
	float max_dimension = std::max(w_factor, h_factor);
	w_factor /= max_dimension;
	h_factor /= max_dimension;

	char vs[256];
	snprintf(vs, sizeof(vs),
			 "attribute vec4 pos;\n"
			 "varying vec2 texcoord;\n"
			 "\n"
			 "void main() {\n"
			 "  gl_Position = pos;\n"
			 "  texcoord.x = pos.x / %f + 0.5;\n"
			 "  texcoord.y = 0.5 - pos.y / %f;\n"
			 "}\n",
			 2.0 * w_factor, 2.0 * h_factor);
	vs[sizeof(vs) - 1] = 0;
	GLint vs_s = compile_shader(GL_VERTEX_SHADER, vs);
	const char *fs = "#extension GL_OES_EGL_image_external : enable\n"
					 "precision mediump float;\n"
					 "uniform samplerExternalOES s;\n"
					 "varying vec2 texcoord;\n"
					 "void main() {\n"
					 "  gl_FragColor = texture2D(s, texcoord);\n"
					 "}\n";
	GLint fs_s = compile_shader(GL_FRAGMENT_SHADER, fs);

	if (gl_program_)
		glDeleteProgram(gl_program_);
	gl_program_ = link_program(vs_s, fs_s);
	glDeleteShader(vs_s);
	glDeleteShader(fs_s);

	glUseProgram(gl_program_);

	verts_[0] = -w_factor;
	verts_[1] = -h_factor;
	verts_[2] = w_factor;
	verts_[3] = -h_factor;
	verts_[4] = w_factor;
	verts_[5] = h_factor;
	verts_[6] = -w_factor;
	verts_[7] = h_factor;
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, verts_);
	glEnableVertexAttribArray(0);

	glViewport(0, 0, window_width, window_height);
}

void WaylandEglPreview::registryGlobal(void *data, wl_registry *registry, uint32_t name, const char *interface,
									   uint32_t version)
{
	WaylandEglPreview *self = static_cast<WaylandEglPreview *>(data);

	if (!strcmp(interface, wl_compositor_interface.name))
	{
		self->compositor_ = static_cast<wl_compositor *>(
			wl_registry_bind(registry, name, &wl_compositor_interface, std::min(version, 4u)));
	}
	else if (!strcmp(interface, xdg_wm_base_interface.name))
	{
		self->wm_base_ = static_cast<xdg_wm_base *>(wl_registry_bind(registry, name, &xdg_wm_base_interface, 1));
		static const xdg_wm_base_listener listener = { .ping = wmBasePing };
		xdg_wm_base_add_listener(self->wm_base_, &listener, self);
	}
	else if (!strcmp(interface, zxdg_decoration_manager_v1_interface.name))
	{
		self->decoration_manager_ = static_cast<zxdg_decoration_manager_v1 *>(
			wl_registry_bind(registry, name, &zxdg_decoration_manager_v1_interface, 1));
	}
}

void WaylandEglPreview::registryGlobalRemove(void *data, wl_registry *registry, uint32_t name)
{
}

void WaylandEglPreview::wmBasePing(void *data, xdg_wm_base *wm_base, uint32_t serial)
{
	xdg_wm_base_pong(wm_base, serial);
}

void WaylandEglPreview::xdgSurfaceConfigure(void *data, xdg_surface *xdg_surface, uint32_t serial)
{
	xdg_surface_ack_configure(xdg_surface, serial);
}

void WaylandEglPreview::xdgToplevelConfigure(void *data, xdg_toplevel *toplevel, int32_t width, int32_t height,
											 wl_array *states)
{
	WaylandEglPreview *self = static_cast<WaylandEglPreview *>(data);
	// A zero size means "you decide"; keep our current size in that case.
	if (width > 0 && height > 0)
	{
		self->pending_width_ = width;
		self->pending_height_ = height;
	}
}

void WaylandEglPreview::xdgToplevelClose(void *data, xdg_toplevel *toplevel)
{
	WaylandEglPreview *self = static_cast<WaylandEglPreview *>(data);
	self->quit_ = true;
}

WaylandEglPreview::WaylandEglPreview(Options const *options)
	: Preview(options), display_(nullptr), registry_(nullptr), compositor_(nullptr), wm_base_(nullptr),
	  decoration_manager_(nullptr), surface_(nullptr), xdg_surface_(nullptr), xdg_toplevel_(nullptr),
	  toplevel_decoration_(nullptr), egl_window_(nullptr), egl_display_(EGL_NO_DISPLAY), egl_context_(EGL_NO_CONTEXT),
	  egl_surface_(EGL_NO_SURFACE), gl_program_(0), last_fd_(-1), first_time_(true), quit_(false), pending_width_(0),
	  pending_height_(0), image_width_(0), image_height_(0), max_image_width_(0), max_image_height_(0)
{
	display_ = wl_display_connect(NULL);
	if (!display_)
		throw std::runtime_error("Couldn't connect to Wayland display");

	registry_ = wl_display_get_registry(display_);
	static const wl_registry_listener listener = { .global = registryGlobal, .global_remove = registryGlobalRemove };
	wl_registry_add_listener(registry_, &listener, this);

	// A round-trip is enough to receive all the globals advertised by the compositor.
	wl_display_roundtrip(display_);

	if (!compositor_ || !wm_base_)
		throw std::runtime_error("Wayland compositor is missing wl_compositor or xdg_wm_base");

	x_ = options_->Get().preview_x;
	y_ = options_->Get().preview_y;
	width_ = options_->Get().preview_width;
	height_ = options_->Get().preview_height;
	makeWindow("rpicam-app");

	// gl_setup() has to happen later, once we're sure we're in the display thread.
}

WaylandEglPreview::~WaylandEglPreview()
{
	WaylandEglPreview::Reset();

	if (egl_surface_ != EGL_NO_SURFACE)
		eglDestroySurface(egl_display_, egl_surface_);
	if (egl_context_ != EGL_NO_CONTEXT)
		eglDestroyContext(egl_display_, egl_context_);
	if (egl_display_ != EGL_NO_DISPLAY)
		eglTerminate(egl_display_);

	if (egl_window_)
		wl_egl_window_destroy(egl_window_);
	if (toplevel_decoration_)
		zxdg_toplevel_decoration_v1_destroy(toplevel_decoration_);
	if (xdg_toplevel_)
		xdg_toplevel_destroy(xdg_toplevel_);
	if (xdg_surface_)
		xdg_surface_destroy(xdg_surface_);
	if (surface_)
		wl_surface_destroy(surface_);
	if (decoration_manager_)
		zxdg_decoration_manager_v1_destroy(decoration_manager_);
	if (wm_base_)
		xdg_wm_base_destroy(wm_base_);
	if (compositor_)
		wl_compositor_destroy(compositor_);
	if (registry_)
		wl_registry_destroy(registry_);
	if (display_)
		wl_display_disconnect(display_);
}

void WaylandEglPreview::makeWindow(char const *name)
{
	// Default behaviour here is to use a 1024x768 window (unless we're going fullscreen,
	// in which case the compositor tells us the size via a configure event).
	if (width_ == 0 || height_ == 0)
	{
		width_ = 1024;
		height_ = 768;
	}

	// Mesa's eglGetDisplay auto-detects a wl_display pointer as a Wayland
	// display. We avoid eglGetPlatformDisplay as that is core EGL 1.5, which is
	// not available everywhere (and libepoxy aborts on a missing entry point).
	egl_display_ = eglGetDisplay(reinterpret_cast<EGLNativeDisplayType>(display_));
	if (egl_display_ == EGL_NO_DISPLAY)
		throw std::runtime_error("eglGetDisplay() failed");

	EGLint egl_major, egl_minor;
	if (!eglInitialize(egl_display_, &egl_major, &egl_minor))
		throw std::runtime_error("eglInitialize() failed");

	static const EGLint attribs[] = { EGL_RED_SIZE,	 1, EGL_GREEN_SIZE,		 1,
									  EGL_BLUE_SIZE, 1, EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
									  EGL_NONE };
	EGLint num_configs;
	if (!eglChooseConfig(egl_display_, attribs, &egl_config_, 1, &num_configs) || num_configs == 0)
		throw std::runtime_error("couldn't get an EGL visual config");

	// Build the Wayland surface and its xdg-shell roles.
	surface_ = wl_compositor_create_surface(compositor_);

	xdg_surface_ = xdg_wm_base_get_xdg_surface(wm_base_, surface_);
	static const xdg_surface_listener surf_listener = { .configure = xdgSurfaceConfigure };
	xdg_surface_add_listener(xdg_surface_, &surf_listener, this);

	xdg_toplevel_ = xdg_surface_get_toplevel(xdg_surface_);
	// configure_bounds (v4) and wm_capabilities (v5) are never sent as we bind
	// xdg_wm_base at version 1, but must be initialised to satisfy -Werror.
	static const xdg_toplevel_listener top_listener = { .configure = xdgToplevelConfigure,
														.close = xdgToplevelClose,
														.configure_bounds = nullptr,
														.wm_capabilities = nullptr };
	xdg_toplevel_add_listener(xdg_toplevel_, &top_listener, this);
	xdg_toplevel_set_title(xdg_toplevel_, name);
	xdg_toplevel_set_app_id(xdg_toplevel_, name);

	// Ask the compositor to draw server-side decorations (a titlebar), matching
	// the framed window the X11 EGL preview gets from the window manager. If the
	// compositor doesn't support the protocol we simply get an undecorated window.
	if (decoration_manager_)
	{
		toplevel_decoration_ = zxdg_decoration_manager_v1_get_toplevel_decoration(decoration_manager_, xdg_toplevel_);
		zxdg_toplevel_decoration_v1_set_mode(toplevel_decoration_, ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE);
	}

	if (options_->Get().fullscreen)
		xdg_toplevel_set_fullscreen(xdg_toplevel_, NULL);

	// Commit the role and wait for the initial configure so we know our size.
	wl_surface_commit(surface_);
	wl_display_roundtrip(display_);

	if (pending_width_ && pending_height_)
	{
		width_ = pending_width_;
		height_ = pending_height_;
		pending_width_ = pending_height_ = 0;
	}

	egl_window_ = wl_egl_window_create(surface_, width_, height_);
	if (!egl_window_)
		throw std::runtime_error("wl_egl_window_create() failed");

	eglBindAPI(EGL_OPENGL_ES_API);

	static const EGLint ctx_attribs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
	egl_context_ = eglCreateContext(egl_display_, egl_config_, EGL_NO_CONTEXT, ctx_attribs);
	if (egl_context_ == EGL_NO_CONTEXT)
		throw std::runtime_error("eglCreateContext failed");

	egl_surface_ =
		eglCreateWindowSurface(egl_display_, egl_config_, reinterpret_cast<EGLNativeWindowType>(egl_window_), NULL);
	if (egl_surface_ == EGL_NO_SURFACE)
		throw std::runtime_error("eglCreateWindowSurface failed");

	// We have to do eglMakeCurrent in the thread where it will run, but we must do it
	// here temporarily so as to get the maximum texture size.
	eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, egl_context_);
	int max_texture_size = 0;
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture_size);
	max_image_width_ = max_image_height_ = max_texture_size;
	// This "undoes" the previous eglMakeCurrent.
	eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
}

static void get_colour_space_info(std::optional<libcamera::ColorSpace> const &cs, EGLint &encoding, EGLint &range)
{
	encoding = EGL_ITU_REC601_EXT;
	range = EGL_YUV_NARROW_RANGE_EXT;

	if (cs == libcamera::ColorSpace::Sycc)
		range = EGL_YUV_FULL_RANGE_EXT;
	else if (cs == libcamera::ColorSpace::Smpte170m)
		/* all good */;
	else if (cs == libcamera::ColorSpace::Rec709)
		encoding = EGL_ITU_REC709_EXT;
	else
		LOG(1, "WaylandEglPreview: unexpected colour space " << libcamera::ColorSpace::toString(cs));
}

void WaylandEglPreview::makeBuffer(int fd, size_t size, StreamInfo const &info, Buffer &buffer)
{
	if (first_time_)
	{
		// This stuff has to be delayed until we know we're in the thread doing the display.
		if (!eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_))
			throw std::runtime_error("eglMakeCurrent failed");
		image_width_ = info.width;
		image_height_ = info.height;
		configureGl(image_width_, image_height_, width_, height_);
		first_time_ = false;
	}

	buffer.fd = fd;
	buffer.size = size;
	buffer.info = info;

	EGLint encoding, range;
	get_colour_space_info(info.colour_space, encoding, range);

	EGLint attribs[] = { EGL_WIDTH,
						 static_cast<EGLint>(info.width),
						 EGL_HEIGHT,
						 static_cast<EGLint>(info.height),
						 EGL_LINUX_DRM_FOURCC_EXT,
						 DRM_FORMAT_YUV420,
						 EGL_DMA_BUF_PLANE0_FD_EXT,
						 fd,
						 EGL_DMA_BUF_PLANE0_OFFSET_EXT,
						 0,
						 EGL_DMA_BUF_PLANE0_PITCH_EXT,
						 static_cast<EGLint>(info.stride),
						 EGL_DMA_BUF_PLANE1_FD_EXT,
						 fd,
						 EGL_DMA_BUF_PLANE1_OFFSET_EXT,
						 static_cast<EGLint>(info.stride * info.height),
						 EGL_DMA_BUF_PLANE1_PITCH_EXT,
						 static_cast<EGLint>(info.stride / 2),
						 EGL_DMA_BUF_PLANE2_FD_EXT,
						 fd,
						 EGL_DMA_BUF_PLANE2_OFFSET_EXT,
						 static_cast<EGLint>(info.stride * info.height + (info.stride / 2) * (info.height / 2)),
						 EGL_DMA_BUF_PLANE2_PITCH_EXT,
						 static_cast<EGLint>(info.stride / 2),
						 EGL_YUV_COLOR_SPACE_HINT_EXT,
						 encoding,
						 EGL_SAMPLE_RANGE_HINT_EXT,
						 range,
						 EGL_NONE };

	EGLImage image = eglCreateImageKHR(egl_display_, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, attribs);
	if (!image)
		throw std::runtime_error("failed to import fd " + std::to_string(fd));

	glGenTextures(1, &buffer.texture);
	glBindTexture(GL_TEXTURE_EXTERNAL_OES, buffer.texture);
	glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, image);

	eglDestroyImageKHR(egl_display_, image);
}

void WaylandEglPreview::SetInfoText(const std::string &text)
{
	if (!text.empty())
		xdg_toplevel_set_title(xdg_toplevel_, text.c_str());
}

// Read and dispatch any pending Wayland events without ever blocking. This uses
// the prepare_read/read_events idiom rather than wl_display_dispatch(), as the
// latter can block on a partial read; this is called every frame from Show() and
// Quit() on the preview thread, so a stall would delay frame delivery.
void WaylandEglPreview::dispatchPending()
{
	// Drain anything already queued so we can safely prepare to read.
	while (wl_display_prepare_read(display_) != 0)
		wl_display_dispatch_pending(display_);
	wl_display_flush(display_);

	pollfd pfd = { wl_display_get_fd(display_), POLLIN, 0 };
	if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN))
		wl_display_read_events(display_); // data is available, so this won't block
	else
		wl_display_cancel_read(display_);

	wl_display_dispatch_pending(display_);
}

void WaylandEglPreview::Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info)
{
	Buffer &buffer = buffers_[fd];
	if (buffer.fd == -1)
		makeBuffer(fd, span.size(), info, buffer);

	// Apply any pending resize the compositor asked for, re-letterboxing the image.
	if (pending_width_ && pending_height_ && (pending_width_ != width_ || pending_height_ != height_))
	{
		width_ = pending_width_;
		height_ = pending_height_;
		wl_egl_window_resize(egl_window_, width_, height_, 0, 0);
		configureGl(image_width_, image_height_, width_, height_);
	}
	pending_width_ = pending_height_ = 0;

	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);

	glBindTexture(GL_TEXTURE_EXTERNAL_OES, buffer.texture);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	EGLBoolean success [[maybe_unused]] = eglSwapBuffers(egl_display_, egl_surface_);

	dispatchPending();

	if (last_fd_ >= 0)
		done_callback_(last_fd_);
	last_fd_ = fd;
}

void WaylandEglPreview::Reset()
{
	for (auto &it : buffers_)
		glDeleteTextures(1, &it.second.texture);
	buffers_.clear();
	last_fd_ = -1;
	if (gl_program_)
	{
		glDeleteProgram(gl_program_);
		gl_program_ = 0;
	}
	eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	first_time_ = true;
}

bool WaylandEglPreview::Quit()
{
	dispatchPending();
	return quit_;
}

static Preview *Create(Options const *options)
{
	return new WaylandEglPreview(options);
}

static RegisterPreview reg("wayland-egl", &Create);
