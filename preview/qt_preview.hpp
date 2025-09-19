/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021-2024, Raspberry Pi (Trading) Ltd.
 *
 * qt_preview.hpp - Qt preview widget facade used by GUI and CLI applications.
 */

#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <QByteArray>
#include <QImage>
#include <QMetaType>
#include <QPointer>
#include <QString>
#include <QWidget>

#include "preview.hpp"

struct QtFrameInfo
{
        unsigned int width = 0;
        unsigned int height = 0;
        unsigned int stride = 0;
        int colour_matrix = 0;
};

Q_DECLARE_METATYPE(QtFrameInfo)

class QtPreviewWidget : public QWidget
{
        Q_OBJECT

public:
        QtPreviewWidget(unsigned int width, unsigned int height, QWidget *parent = nullptr);

public slots:
        void renderFrame(const QByteArray &data, const QtFrameInfo &info);
        void clearFrame();
        void setInfoText(const QString &text);

signals:
        void frameUpdated();

protected:
        void paintEvent(QPaintEvent *event) override;
        QSize sizeHint() const override { return QSize(window_width_, window_height_); }

private:
        void ensureImage();

        unsigned int window_width_;
        unsigned int window_height_;
        QImage image_;
        QString info_text_;
        std::vector<uint8_t> tmp_stripe_;
};

class QtPreviewWindow;

class QtPreview : public QObject, public Preview
{
        Q_OBJECT

public:
        explicit QtPreview(Options const *options);
        ~QtPreview() override;

        void SetInfoText(const std::string &text) override;
        void Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info) override;
        void Reset() override;
        bool Quit() override;
        void MaxImageSize(unsigned int &w, unsigned int &h) const override;

        QWidget *widget() const { return pane_; }

        static void RegisterPreviewParent(QWidget *parent);

private:
        void initialiseStandaloneWindow(Options const *options);
        void initialiseEmbeddedWidget(QWidget *parent);
        void runStandaloneEventLoop(Options const *options);

        QtPreviewWidget *pane_ = nullptr;
        QtPreviewWindow *window_ = nullptr;
        QApplication *application_ = nullptr;
        std::thread thread_;
        unsigned int window_width_ = 0;
        unsigned int window_height_ = 0;
        bool quit_ = false;
        bool owns_widget_ = false;
        bool owns_event_loop_ = false;
        std::mutex mutex_;
        std::condition_variable cond_var_;
};

namespace QtPreviewIntegration
{
void registerPreviewParent(QWidget *parent);
QWidget *takePreviewParent();
}

