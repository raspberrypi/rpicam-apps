/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * egl_preview.cpp - X/EGL-based preview window.
 */

#include <libdrm/drm_fourcc.h>

#include "egl_preview.hpp"

#include <X11/Xutil.h>

static GLint compile_shader(GLenum target, const char *source)
{
	GLuint s = glCreateShader(target);
	glShaderSource(s, 1, (const GLchar **) &source, NULL);
	glCompileShader(s);

	GLint ok;
	glGetShaderiv(s, GL_COMPILE_STATUS, &ok);

	if (!ok) {
		GLchar *info;
		GLint size;

		glGetShaderiv(s, GL_INFO_LOG_LENGTH, &size);
		info = (GLchar *)malloc(size);

		glGetShaderInfoLog(s, size, NULL, info);
		throw std::runtime_error("failed to compile shader: " + std::string(info) +
								 "\nsource:\n" + std::string(source));
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
	if (!ok) {
		/* Some drivers return a size of 1 for an empty log.  This is the size
		 * of a log that contains only a terminating NUL character.
		 */
		GLint size;
		GLchar *info = NULL;
		glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &size);
		if (size > 1) {
			info = (GLchar *)malloc(size);
			glGetProgramInfoLog(prog, size, NULL, info);
		}

		throw std::runtime_error("failed to link: " +
								 std::string(info ? info : "<empty log>"));
	}

	return prog;
}

static void gl_setup(int width, int height, int window_width, int window_height)
{
	float w_factor = width / (float)window_width;
	float h_factor = height / (float)window_height;
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
			"}\n", 2.0 * w_factor, 2.0 * h_factor);
	vs[sizeof(vs) - 1] = 0;
	GLint vs_s = compile_shader(GL_VERTEX_SHADER, vs);
	const char *fs =
		"#extension GL_OES_EGL_image_external : enable\n"
		"precision mediump float;\n"
		"uniform samplerExternalOES s;\n"
		"varying vec2 texcoord;\n"
		"void main() {\n"
		"  gl_FragColor = texture2D(s, texcoord);\n"
		"}\n";
	GLint fs_s = compile_shader(GL_FRAGMENT_SHADER, fs);
	GLint prog = link_program(vs_s, fs_s);

	glUseProgram(prog);

	static const float verts[] = {
		-w_factor, -h_factor,
		w_factor, -h_factor,
		w_factor, h_factor,
		-w_factor, h_factor
	};
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, verts);
	glEnableVertexAttribArray(0);
}

EglPreview::EglPreview(Options const &options) : last_fd_(-1), first_time_(true), Preview(options)
{
   display_ = XOpenDisplay(NULL);
   if (!display_)
	   throw std::runtime_error("Couldn't open X display");

   egl_display_ = eglGetDisplay(display_);
   if (!egl_display_)
	   throw std::runtime_error("eglGetDisplay() failed");

   EGLint egl_major, egl_minor;

   if (!eglInitialize(egl_display_, &egl_major, &egl_minor))
	   throw std::runtime_error("eglInitialize() failed");

   x_ = options_.preview_x;
   y_ = options_.preview_y;
   width_ = options_.preview_width;
   height_ = options_.preview_height;
   makeWindow("libcamera-app");

   // gl_setup() has to happen later, once we're sure we're in the display thread.
}

EglPreview::~EglPreview()
{
}

static void no_border(Display *display, Window window)
{
	static const unsigned MWM_HINTS_DECORATIONS = (1 << 1);
	static const int PROP_MOTIF_WM_HINTS_ELEMENTS = 5;

	typedef struct
	{
		unsigned long       flags;
		unsigned long       functions;
		unsigned long       decorations;
		long                inputMode;
		unsigned long       status;
	} PropMotifWmHints;

	PropMotifWmHints motif_hints;
	Atom prop, proptype;
	unsigned long flags = 0;

	/* setup the property */
	motif_hints.flags = MWM_HINTS_DECORATIONS;
	motif_hints.decorations = flags;

	/* get the atom for the property */
	prop = XInternAtom(display, "_MOTIF_WM_HINTS", True );
	if (!prop) {
		/* something went wrong! */
		return;
	}

	/* not sure this is correct, seems to work, XA_WM_HINTS didn't work */
	proptype = prop;

	XChangeProperty(display, window,                /* display, window */
                    prop, proptype,                 /* property, type */
                    32,                             /* format: 32-bit datums */
                    PropModeReplace,                /* mode */
                    (unsigned char *) &motif_hints, /* data */
                    PROP_MOTIF_WM_HINTS_ELEMENTS    /* nelements */
					);
}

void EglPreview::makeWindow(char const *name)
{
	int screen_num = DefaultScreen(display_);
	XSetWindowAttributes attr;
	unsigned long mask;
	Window root = RootWindow(display_, screen_num);
	Window win;
	int screen_width = DisplayWidth(display_, screen_num);
	int screen_height = DisplayHeight(display_, screen_num);

	// Default behaviour here is to use a 1024x768 window.
	if (width_ == 0 || height_ == 0)
	{
		width_ = 1024;
		height_ = 768;
	}

	if (options_.fullscreen ||
		x_ + width_ > screen_width || y_ + height_ > screen_height)
	{
		x_ = y_ = 0;
		width_ = DisplayWidth(display_, screen_num);
		height_ = DisplayHeight(display_, screen_num);
	}

	static const EGLint attribs[] =
		{
			EGL_RED_SIZE, 1,
			EGL_GREEN_SIZE, 1,
			EGL_BLUE_SIZE, 1,
			EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
			EGL_NONE
		};
	EGLConfig config;
	EGLint num_configs;
	if (!eglChooseConfig(egl_display_, attribs, &config, 1, &num_configs))
		throw std::runtime_error("couldn't get an EGL visual config");

	EGLint vid;
	if (!eglGetConfigAttrib(egl_display_, config, EGL_NATIVE_VISUAL_ID, &vid))
		throw std::runtime_error("eglGetConfigAttrib() failed\n");

	XVisualInfo visTemplate = { .visualid = (long unsigned int)vid };
	int num_visuals;
	XVisualInfo *visinfo = XGetVisualInfo(display_, VisualIDMask,
                                         &visTemplate, &num_visuals);

	/* window attributes */
	attr.background_pixel = 0;
	attr.border_pixel = 0;
	attr.colormap = XCreateColormap(display_, root, visinfo->visual, AllocNone);
	attr.event_mask = StructureNotifyMask | ExposureMask | KeyPressMask;
	/* XXX this is a bad way to get a borderless window! */
	mask = CWBackPixel | CWBorderPixel | CWColormap | CWEventMask;

	window_ = XCreateWindow(display_, root, x_, y_, width_, height_,
							0, visinfo->depth, InputOutput,
							visinfo->visual, mask, &attr);

	if (options_.fullscreen)
		no_border(display_, window_);

	/* set hints and properties */
	{
		XSizeHints sizehints;
		sizehints.x = x_;
		sizehints.y = y_;
		sizehints.width  = width_;
		sizehints.height = height_;
		sizehints.flags = USSize | USPosition;
		XSetNormalHints(display_, window_, &sizehints);
		XSetStandardProperties(display_, window_, name, name,
							   None, (char **)NULL, 0, &sizehints);
	}

	eglBindAPI(EGL_OPENGL_ES_API);

	static const EGLint ctx_attribs[] = {
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};
	egl_context_ = eglCreateContext(egl_display_, config, EGL_NO_CONTEXT, ctx_attribs );
	if (!egl_context_)
		throw std::runtime_error("eglCreateContext failed");

	XFree(visinfo);

	XMapWindow(display_, window_);

	// This stops the window manager from closing the window, so we get an event instead.
	wm_delete_window_ = XInternAtom(display_, "WM_DELETE_WINDOW", False);
	XSetWMProtocols(display_, window_, &wm_delete_window_, 1);

	egl_surface_ = eglCreateWindowSurface(egl_display_, config,
										  window_, NULL);
	if (!egl_surface_)
		throw std::runtime_error("eglCreateWindowSurface failed");

	// Can't do eglMakeCurrent yet because this might not be the display thread.
}

void EglPreview::makeBuffer(int fd, size_t size, int width, int height, int stride,
							   Buffer &buffer)
{
	if (first_time_)
	{
		// This stuff has to be delayed until we know we're in the thread doing the display.
		if (!eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_))
			throw std::runtime_error("eglMakeCurrent failed");
		gl_setup(width, height, width_, height_);
		first_time_ = false;
	}

	buffer.fd = fd;
	buffer.size = size;
	buffer.width = width;
	buffer.height = height;
	buffer.stride = stride;

	EGLint attribs[] = {
		EGL_WIDTH, width,
		EGL_HEIGHT, height,
		EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_YUV420,
		EGL_DMA_BUF_PLANE0_FD_EXT, fd,
		EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
		EGL_DMA_BUF_PLANE0_PITCH_EXT, stride,
		EGL_DMA_BUF_PLANE1_FD_EXT, fd,
		EGL_DMA_BUF_PLANE1_OFFSET_EXT, stride * height,
		EGL_DMA_BUF_PLANE1_PITCH_EXT, stride / 2,
		EGL_DMA_BUF_PLANE2_FD_EXT, fd,
		EGL_DMA_BUF_PLANE2_OFFSET_EXT, stride * height + (stride / 2) * (height / 2),
		EGL_DMA_BUF_PLANE2_PITCH_EXT, stride / 2,
		EGL_NONE
	};

	EGLImage image = eglCreateImageKHR(egl_display_,
									   EGL_NO_CONTEXT,
									   EGL_LINUX_DMA_BUF_EXT,
									   NULL, attribs);
	if (!image)
		throw std::runtime_error("failed to import fd " + std::to_string(fd));

	glGenTextures(1, &buffer.texture);
	glBindTexture(GL_TEXTURE_EXTERNAL_OES, buffer.texture);
	glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, image);

	eglDestroyImageKHR(egl_display_, image);
}

void EglPreview::Show(int fd, size_t size, int width, int height, int stride)
{
	Buffer &buffer = buffers_[fd];
	if (buffer.fd == -1)
		makeBuffer(fd, size, width, height, stride, buffer);

	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);

	glBindTexture(GL_TEXTURE_EXTERNAL_OES, buffer.texture);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	EGLBoolean success = eglSwapBuffers(egl_display_, egl_surface_);
	if (last_fd_ >= 0)
		done_callback_(last_fd_);
	last_fd_ = fd;
}

void EglPreview::Reset()
{
	for (auto &it : buffers_)
		glDeleteTextures(1, &it.second.texture);
	buffers_.clear();
	last_fd_ = -1;
}

bool EglPreview::Quit()
{
	XEvent event;
	while (XCheckTypedWindowEvent(display_, window_, ClientMessage, &event))
	{
		if (event.xclient.data.l[0] == wm_delete_window_)
			return true;
	}
	return false;
}
