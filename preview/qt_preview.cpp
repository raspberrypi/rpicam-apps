/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * qt_preview.cpp - Qt preview window
 */

#include <condition_variable>
#include <iostream>
#include <mutex>
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
		// Cache the x sampling locations for speed. This is a quick nearest neighbour resize.
		if (last_image_width_ != info.width)
		{
			last_image_width_ = info.width;
			x_locations_.resize(window_width_);
			for (unsigned int i = 0; i < window_width_; i++)
				x_locations_[i] = (i * (info.width - 1) + (window_width_ - 1) / 2) / (window_width_ - 1);
		}

		uint8_t *Y_start = span.data();
		uint8_t *U_start = Y_start + info.stride * info.height;
		int uv_size = (info.stride / 2) * (info.height / 2);

		// Choose the right matrix to convert YUV back to RGB.
		static const float YUV2RGB[3][9] = {
			{ 1.0,   0.0, 1.402, 1.0,   -0.344, -0.714, 1.0,   1.772, 0.0 }, // JPEG
			{ 1.164, 0.0, 1.596, 1.164, -0.392, -0.813, 1.164, 2.017, 0.0 }, // SMPTE170M
			{ 1.164, 0.0, 1.793, 1.164, -0.213, -0.533, 1.164, 2.112, 0.0 }, // Rec709
		};
		const float *M = YUV2RGB[0];
		if (info.colour_space == libcamera::ColorSpace::Sycc)
			M = YUV2RGB[0];
		else if (info.colour_space == libcamera::ColorSpace::Smpte170m)
			M = YUV2RGB[1];
		else if (info.colour_space == libcamera::ColorSpace::Rec709)
			M = YUV2RGB[2];
		else
			LOG(1, "QtPreview: unexpected colour space " << libcamera::ColorSpace::toString(info.colour_space));

		// Possibly this should be locked in case a repaint is happening? In practice the risk
		// is only that there might be some tearing, so I don't think we worry. We could speed
		// it up by getting the ISP to supply RGB, but I'm not sure I want to handle that extra
		// possibility in our main application code, so we'll put up with the slow conversion.
		for (unsigned int y = 0; y < window_height_; y++)
		{
			int row = (y * (info.height - 1) + (window_height_ - 1) / 2) / (window_height_ - 1);
			uint8_t *Y_row = Y_start + row * info.stride;
			uint8_t *U_row = U_start + (row / 2) * (info.stride / 2);
			uint8_t *V_row = U_row + uv_size;
			uint8_t *dest = pane_->image.scanLine(y);
			for (unsigned int x = 0; x < window_width_;)
			{
				int y_off0 = x_locations_[x++];
				int y_off1 = x_locations_[x++];
				int uv_off0 = y_off0 >> 1;
				int uv_off1 = y_off0 >> 1;
				int Y0 = Y_row[y_off0];
				int Y1 = Y_row[y_off1];
				int U0 = U_row[uv_off0];
				int V0 = V_row[uv_off0];
				int U1 = U_row[uv_off1];
				int V1 = V_row[uv_off1];
				U0 -= 128;
				V0 -= 128;
				U1 -= 128;
				V1 -= 128;
				int R0 = M[0] * Y0 + M[2] * V0;
				int G0 = M[3] * Y0 + M[4] * U0 + M[5] * V0;
				int B0 = M[6] * Y0 + M[7] * U0;
				int R1 = M[0] * Y1 + M[2] * V1;
				int G1 = M[3] * Y1 + M[4] * U1 + M[5] * V1;
				int B1 = M[6] * Y1 + M[7] * U1;
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
	std::vector<uint16_t> x_locations_;
	unsigned int last_image_width_ = 0;
	unsigned int window_width_, window_height_;
	std::mutex mutex_;
	std::condition_variable cond_var_;
};

Preview *make_qt_preview(Options const *options)
{
	return new QtPreview(options);
}
