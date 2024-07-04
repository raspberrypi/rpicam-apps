/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * qt_preview.cpp - Qt preview window
 */

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string.h>
#include <thread>

// This header must be before the QT headers, as the latter #defines slot and emit!
#include "core/options.hpp"

#include <QApplication>
#include <QImage>
#include <QMainWindow>
#include <QPaintEvent>
#include <QPainter>
#include <QWidget>

#include "preview.hpp"

class MyMainWindow : public QMainWindow
{
public:
	MyMainWindow() : QMainWindow() {}
	bool quit = false;
protected:
	void closeEvent(QCloseEvent *event) override
	{
		event->ignore();
		quit = true;
	}
};

class MyWidget : public QWidget
{
public:
	MyWidget(QWidget *parent, int w, int h) : QWidget(parent), size(w, h)
	{
		image = QImage(size, QImage::Format_RGB888);
		image.fill(0);
	}
	QSize size;
	QImage image;
protected:
	void paintEvent(QPaintEvent *) override
	{
		QPainter painter(this);
		painter.drawImage(rect(), image, image.rect());
	}
	QSize sizeHint() const override { return size; }
};

class QtPreview : public Preview
{
public:
	QtPreview(Options const *options) : Preview(options)
	{
		window_width_ = options->preview_width;
		window_height_ = options->preview_height;
		if (window_width_ % 2 || window_height_ % 2)
			throw std::runtime_error("QtPreview: expect even dimensions");
		// This preview window is expensive, so make it small by default.
		if (window_width_ == 0 || window_height_ == 0)
			window_width_ = 512, window_height_ = 384;
		// As a hint, reserve twice the binned width for our widest current camera (V3)
		tmp_stripe_.reserve(4608);
		thread_ = std::thread(&QtPreview::threadFunc, this, options);
		std::unique_lock lock(mutex_);
		while (!pane_)
			cond_var_.wait(lock);
		LOG(2, "Made Qt preview");
	}
	~QtPreview()
	{
		application_->exit();
		thread_.join();
	}
	void SetInfoText(const std::string &text) override { main_window_->setWindowTitle(QString::fromStdString(text)); }
	virtual void Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info) override
	{
		// Quick and simple nearest-neighbour-ish resampling is used here.
		// We further share U,V samples between adjacent output pixel pairs
		// (even when downscaling) to speed up the conversion.
		unsigned x_step = (info.width << 16) / window_width_;
		unsigned y_step = (info.height << 16) / window_height_;

		// Choose the right matrix to convert YUV back to RGB.
		static const float YUV2RGB[3][9] = {
			{ 1.0,   0.0, 1.402, 1.0,   -0.344, -0.714, 1.0,   1.772, 0.0 }, // JPEG
			{ 1.164, 0.0, 1.596, 1.164, -0.392, -0.813, 1.164, 2.017, 0.0 }, // SMPTE170M
			{ 1.164, 0.0, 1.793, 1.164, -0.213, -0.533, 1.164, 2.112, 0.0 }, // Rec709
		};
		int offsetY;
		float coeffY, coeffVR, coeffUG, coeffVG, coeffUB;
		if (info.colour_space == libcamera::ColorSpace::Smpte170m)
		{
			offsetY = 16;
			coeffY = YUV2RGB[1][0];
			coeffVR = YUV2RGB[1][2];
			coeffUG = YUV2RGB[1][4];
			coeffVG = YUV2RGB[1][5];
			coeffUB = YUV2RGB[1][7];
		}
		else if (info.colour_space == libcamera::ColorSpace::Rec709)
		{
			offsetY = 16;
			coeffY = YUV2RGB[2][0];
			coeffVR = YUV2RGB[2][2];
			coeffUG = YUV2RGB[2][4];
			coeffVG = YUV2RGB[2][5];
			coeffUB = YUV2RGB[2][7];
		}
		else
		{
			offsetY = 0;
			coeffY = YUV2RGB[0][0];
			coeffVR = YUV2RGB[0][2];
			coeffUG = YUV2RGB[0][4];
			coeffVG = YUV2RGB[0][5];
			coeffUB = YUV2RGB[0][7];
			if (info.colour_space != libcamera::ColorSpace::Sycc)
				LOG(1, "QtPreview: unexpected colour space " << libcamera::ColorSpace::toString(info.colour_space));
		}

		// Because the source buffer is uncached, and we want to read it a byte at a time,
		// take a copy of each row used. This is a speedup provided memcpy() is vectorized.
		tmp_stripe_.resize(2 * info.stride);
		uint8_t const *Y_start = span.data();
		uint8_t *Y_row = &tmp_stripe_[0];
		uint8_t *U_row = Y_row + info.stride;
		uint8_t *V_row = U_row + (info.stride >> 1);

		// Possibly this should be locked in case a repaint is happening? In practice the risk
		// is only that there might be some tearing, so I don't think we worry. We could speed
		// it up by getting the ISP to supply RGB, but I'm not sure I want to handle that extra
		// possibility in our main application code, so we'll put up with the slow conversion.
		for (unsigned int y = 0; y < window_height_; y++)
		{
			unsigned row = (y * y_step) >> 16;
			uint8_t *dest = pane_->image.scanLine(y);
			unsigned x_pos = x_step >> 1;

			memcpy(Y_row, Y_start + row * info.stride, info.stride);
			memcpy(U_row, Y_start + ((4 * info.height + row) >> 1) * (info.stride >> 1), info.stride >> 1);
			memcpy(V_row, Y_start + ((5 * info.height + row) >> 1) * (info.stride >> 1), info.stride >> 1);

			for (unsigned int x = 0; x < window_width_; x += 2)
			{
				int Y0 = Y_row[x_pos >> 16];
				x_pos += x_step;
				int Y1 = Y_row[x_pos >> 16];
				int U = U_row[x_pos >> 17];
				int V = V_row[x_pos >> 17];
				x_pos += x_step;
				Y0 -= offsetY;
				Y1 -= offsetY;
				U -= 128;
				V -= 128;
				int R0 = coeffY * Y0 + coeffVR * V;
				int G0 = coeffY * Y0 + coeffUG * U + coeffVG * V;
				int B0 = coeffY * Y0 + coeffUB * U;
				int R1 = coeffY * Y1 + coeffVR * V;
				int G1 = coeffY * Y1 + coeffUG * U + coeffVG * V;
				int B1 = coeffY * Y1 + coeffUB * U;
				*(dest++) = std::clamp(R0, 0, 255);
				*(dest++) = std::clamp(G0, 0, 255);
				*(dest++) = std::clamp(B0, 0, 255);
				*(dest++) = std::clamp(R1, 0, 255);
				*(dest++) = std::clamp(G1, 0, 255);
				*(dest++) = std::clamp(B1, 0, 255);
			}
		}

		pane_->update();

		// Return the buffer to the camera system.
		done_callback_(fd);
	}
	// Reset the preview window, clearing the current buffers and being ready to
	// show new ones.
	void Reset() override {}
	// Check if preview window has been shut down.
	bool Quit() override { return main_window_->quit; }
	// There is no particular limit to image sizes, though large images will be very slow.
	virtual void MaxImageSize(unsigned int &w, unsigned int &h) const override { w = h = 0; }

private:
	void threadFunc(Options const *options)
	{
		// This acts as Qt's event loop. Really Qt prefers to own the application's event loop
		// but we've supplied our own and only want Qt for rendering. This works, but I
		// wouldn't write a proper Qt application like this.
		int argc = 0;
		char **argv = NULL;
		QApplication application(argc, argv);
		application_ = &application;
		MyMainWindow main_window;
		main_window_ = &main_window;
		MyWidget pane(&main_window, window_width_, window_height_);
		main_window.setCentralWidget(&pane);
		// Need to get the window border sizes (it seems to be unreasonably difficult...)
		main_window.move(options->preview_x + 2, options->preview_y + 28);
		main_window.show();
		pane_ = &pane;
		cond_var_.notify_one();
		application.exec();
	}
	QApplication *application_ = nullptr;
	MyMainWindow *main_window_ = nullptr;
	MyWidget *pane_ = nullptr;
	std::thread thread_;
	unsigned int window_width_, window_height_;
	std::mutex mutex_;
	std::condition_variable cond_var_;
	std::vector<uint8_t> tmp_stripe_;
};

Preview *make_qt_preview(Options const *options)
{
	return new QtPreview(options);
}
