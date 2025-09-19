/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021-2024, Raspberry Pi (Trading) Ltd.
 *
 * qt_preview.cpp - Qt preview widget used by CLI and GUI applications.
 */

#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>

#include "core/options.hpp"

#include <QApplication>
#include <QMainWindow>
#include <QMetaObject>
#include <QPainter>
#include <QPaintEvent>
#include <QPointer>
#include <QSizePolicy>
#include <QThread>
#include <QVBoxLayout>

#include "qt_preview.hpp"

namespace
{

class QtPreviewWindow : public QMainWindow
{
public:
        QtPreviewWindow() : QMainWindow() {}
        bool quit = false;

protected:
        void closeEvent(QCloseEvent *event) override
        {
                event->ignore();
                quit = true;
        }
};

struct ColourMatrix
{
        int offset_y;
        float coeff_y;
        float coeff_vr;
        float coeff_ug;
        float coeff_vg;
        float coeff_ub;
};

ColourMatrix selectColourMatrix(const StreamInfo &info)
{
        static constexpr ColourMatrix matrices[3] = {
                { 0, 1.0f, 1.402f, -0.344f, -0.714f, 1.772f },      // JPEG
                { 16, 1.164f, 1.596f, -0.392f, -0.813f, 2.017f },  // SMPTE170M
                { 16, 1.164f, 1.793f, -0.213f, -0.533f, 2.112f },  // Rec709
        };

        if (info.colour_space == libcamera::ColorSpace::Smpte170m)
                return matrices[1];
        if (info.colour_space == libcamera::ColorSpace::Rec709)
                return matrices[2];
        if (info.colour_space && info.colour_space != libcamera::ColorSpace::Sycc)
                LOG(1, "QtPreview: unexpected colour space " << libcamera::ColorSpace::toString(info.colour_space));
        return matrices[0];
}

} // namespace

namespace QtPreviewIntegration
{
namespace
{
std::mutex &parentMutex()
{
        static std::mutex mutex;
        return mutex;
}

QPointer<QWidget> &previewParent()
{
        static QPointer<QWidget> parent;
        return parent;
}

} // namespace

void registerPreviewParent(QWidget *parent)
{
        std::lock_guard<std::mutex> lock(parentMutex());
        previewParent() = parent;
}

QWidget *takePreviewParent()
{
        std::lock_guard<std::mutex> lock(parentMutex());
        QWidget *parent = previewParent();
        previewParent().clear();
        return parent;
}

} // namespace QtPreviewIntegration

QtPreviewWidget::QtPreviewWidget(unsigned int width, unsigned int height, QWidget *parent)
        : QWidget(parent), window_width_(width), window_height_(height)
{
        if (window_width_ % 2 || window_height_ % 2)
                throw std::runtime_error("QtPreview: expect even dimensions");
        if (!window_width_ || !window_height_)
                window_width_ = 512, window_height_ = 384;
        ensureImage();
        tmp_stripe_.reserve(4608);
        setAttribute(Qt::WA_OpaquePaintEvent);
        setAttribute(Qt::WA_NoSystemBackground);
}

void QtPreviewWidget::ensureImage()
{
        if (image_.size() != QSize(window_width_, window_height_))
        {
                image_ = QImage(QSize(window_width_, window_height_), QImage::Format_RGB888);
                image_.fill(Qt::black);
        }
}

void QtPreviewWidget::renderFrame(const QByteArray &data, const QtFrameInfo &info)
{
        if (!image_.width() || !image_.height())
                ensureImage();

        const uint8_t *src = reinterpret_cast<const uint8_t *>(data.constData());
        if (!src)
                return;

        unsigned x_step = (info.width << 16) / window_width_;
        unsigned y_step = (info.height << 16) / window_height_;

        ColourMatrix matrix;
        matrix.offset_y = info.colour_matrix == 0 ? 0 : 16;
        if (info.colour_matrix == 1)
                matrix = { 16, 1.164f, 1.596f, -0.392f, -0.813f, 2.017f };
        else if (info.colour_matrix == 2)
                matrix = { 16, 1.164f, 1.793f, -0.213f, -0.533f, 2.112f };
        else
                matrix = { 0, 1.0f, 1.402f, -0.344f, -0.714f, 1.772f };

        tmp_stripe_.resize(2 * info.stride);
        uint8_t *Y_row = tmp_stripe_.data();
        uint8_t *U_row = Y_row + info.stride;
        uint8_t *V_row = U_row + (info.stride >> 1);

        for (unsigned int y = 0; y < window_height_; y++)
        {
                unsigned row = (y * y_step) >> 16;
                uint8_t *dest = image_.scanLine(y);
                unsigned x_pos = x_step >> 1;

                memcpy(Y_row, src + row * info.stride, info.stride);
                memcpy(U_row, src + ((4 * info.height + row) >> 1) * (info.stride >> 1), info.stride >> 1);
                memcpy(V_row, src + ((5 * info.height + row) >> 1) * (info.stride >> 1), info.stride >> 1);

                for (unsigned int x = 0; x < window_width_; x += 2)
                {
                        int Y0 = Y_row[x_pos >> 16] - matrix.offset_y;
                        x_pos += x_step;
                        int Y1 = Y_row[x_pos >> 16] - matrix.offset_y;
                        int U = U_row[x_pos >> 17] - 128;
                        int V = V_row[x_pos >> 17] - 128;
                        x_pos += x_step;

                        int R0 = matrix.coeff_y * Y0 + matrix.coeff_vr * V;
                        int G0 = matrix.coeff_y * Y0 + matrix.coeff_ug * U + matrix.coeff_vg * V;
                        int B0 = matrix.coeff_y * Y0 + matrix.coeff_ub * U;
                        int R1 = matrix.coeff_y * Y1 + matrix.coeff_vr * V;
                        int G1 = matrix.coeff_y * Y1 + matrix.coeff_ug * U + matrix.coeff_vg * V;
                        int B1 = matrix.coeff_y * Y1 + matrix.coeff_ub * U;

                        *(dest++) = std::clamp(R0, 0, 255);
                        *(dest++) = std::clamp(G0, 0, 255);
                        *(dest++) = std::clamp(B0, 0, 255);
                        *(dest++) = std::clamp(R1, 0, 255);
                        *(dest++) = std::clamp(G1, 0, 255);
                        *(dest++) = std::clamp(B1, 0, 255);
                }
        }

        update();
        emit frameUpdated();
}

void QtPreviewWidget::clearFrame()
{
        ensureImage();
        image_.fill(Qt::black);
        update();
}

void QtPreviewWidget::setInfoText(const QString &text)
{
        info_text_ = text;
        update();
}

void QtPreviewWidget::paintEvent(QPaintEvent *event)
{
        Q_UNUSED(event);
        QPainter painter(this);
        painter.drawImage(rect(), image_, image_.rect());
        if (!info_text_.isEmpty())
        {
                        painter.setPen(Qt::green);
                        painter.drawText(rect().adjusted(8, 8, -8, -8), Qt::AlignTop | Qt::AlignLeft, info_text_);
        }
}

QtPreview::QtPreview(Options const *options)
        : Preview(options)
{
        qRegisterMetaType<QtFrameInfo>();

        window_width_ = options->Get().preview_width;
        window_height_ = options->Get().preview_height;
        if (window_width_ % 2 || window_height_ % 2)
                throw std::runtime_error("QtPreview: expect even dimensions");
        if (!window_width_ || !window_height_)
                window_width_ = 512, window_height_ = 384;

        QWidget *parent = QtPreviewIntegration::takePreviewParent();
        if (parent)
        {
                initialiseEmbeddedWidget(parent);
        }
        else
        {
                initialiseStandaloneWindow(options);
        }
}

QtPreview::~QtPreview()
{
        if (owns_event_loop_)
        {
                if (application_)
                        QMetaObject::invokeMethod(application_, "quit", Qt::QueuedConnection);
                if (thread_.joinable())
                        thread_.join();
        }
        else if (owns_widget_ && pane_)
        {
                pane_->deleteLater();
        }
}

void QtPreview::initialiseEmbeddedWidget(QWidget *parent)
{
        pane_ = parent->findChild<QtPreviewWidget *>();
        if (!pane_)
        {
                pane_ = new QtPreviewWidget(window_width_, window_height_, parent);
                pane_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
                if (!parent->layout())
                {
                        auto *layout = new QVBoxLayout(parent);
                        layout->setContentsMargins(0, 0, 0, 0);
                        layout->addWidget(pane_);
                }
                else
                        parent->layout()->addWidget(pane_);
        }
        owns_widget_ = false;
}

void QtPreview::initialiseStandaloneWindow(Options const *options)
{
        owns_event_loop_ = true;
        thread_ = std::thread(&QtPreview::runStandaloneEventLoop, this, options);
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] { return pane_ != nullptr; });
}

void QtPreview::runStandaloneEventLoop(Options const *options)
{
        int argc = 0;
        char **argv = nullptr;
        QApplication application(argc, argv);
        application_ = &application;

        QtPreviewWindow main_window;
        window_ = &main_window;
        QtPreviewWidget pane(window_width_, window_height_, &main_window);
        pane_ = &pane;
        main_window.setCentralWidget(&pane);
        main_window.move(options->Get().preview_x + 2, options->Get().preview_y + 28);
        main_window.show();

        {
                std::lock_guard<std::mutex> lock(mutex_);
                cond_var_.notify_all();
        }

        application.exec();
        quit_ = true;
        pane_ = nullptr;
        window_ = nullptr;
        application_ = nullptr;
}

void QtPreview::SetInfoText(const std::string &text)
{
        if (!pane_)
                return;
        QString info = QString::fromStdString(text);
        QMetaObject::invokeMethod(pane_, "setInfoText", Qt::QueuedConnection, Q_ARG(QString, info));
        if (window_)
                QMetaObject::invokeMethod(window_, [window = window_, info]() { window->setWindowTitle(info); }, Qt::QueuedConnection);
}

void QtPreview::Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info)
{
        if (!pane_)
                return;

        ColourMatrix matrix = selectColourMatrix(info);
        QtFrameInfo frame_info;
        frame_info.width = info.width;
        frame_info.height = info.height;
        frame_info.stride = info.stride;
        if (matrix.offset_y == 16 && matrix.coeff_vr == 1.596f)
                frame_info.colour_matrix = 1;
        else if (matrix.offset_y == 16 && matrix.coeff_vr == 1.793f)
                frame_info.colour_matrix = 2;
        else
                frame_info.colour_matrix = 0;

        QByteArray data(reinterpret_cast<const char *>(span.data()), span.size());
        QMetaObject::invokeMethod(pane_, "renderFrame", Qt::QueuedConnection, Q_ARG(QByteArray, data), Q_ARG(QtFrameInfo, frame_info));

        if (done_callback_)
                done_callback_(fd);
}

void QtPreview::Reset()
{
        if (!pane_)
                return;
        QMetaObject::invokeMethod(pane_, "clearFrame", Qt::QueuedConnection);
}

bool QtPreview::Quit()
{
        return quit_;
}

void QtPreview::MaxImageSize(unsigned int &w, unsigned int &h) const
{
        w = h = 0;
}

void QtPreview::RegisterPreviewParent(QWidget *parent)
{
        QtPreviewIntegration::registerPreviewParent(parent);
}

static Preview *Create(Options const *options)
{
        return new QtPreview(options);
}

static RegisterPreview reg("qt", &Create);

