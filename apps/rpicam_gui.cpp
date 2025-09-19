/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * rpicam_gui.cpp - Qt based GUI application for Raspberry Pi cameras.
 */

#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QComboBox>
#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QObject>
#include <QPushButton>
#include <QString>
#include <QSpinBox>
#include <QStatusBar>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>

#include "core/frame_info.hpp"
#include "core/rpicam_app.hpp"
#include "core/rpicam_encoder.hpp"
#include "core/still_options.hpp"
#include "core/video_options.hpp"

#include "image/image.hpp"

#include "output/output.hpp"

#include "preview/qt_preview.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
namespace fs = std::filesystem;

class RPiCamStillApp : public RPiCamApp
{
public:
        RPiCamStillApp() : RPiCamApp(std::make_unique<StillOptions>()) {}

        StillOptions *GetOptions() const { return static_cast<StillOptions *>(RPiCamApp::GetOptions()); }
};

static std::string generate_filename(StillOptions const *options)
{
        char filename[128];
        std::string folder = options->Get().output;
        if (!folder.empty() && folder.back() != '/')
                folder += "/";
        if (options->Get().datetime)
        {
                std::time_t raw_time;
                std::time(&raw_time);
                char time_string[32];
                std::tm *time_info = std::localtime(&raw_time);
                std::strftime(time_string, sizeof(time_string), "%m%d%H%M%S", time_info);
                snprintf(filename, sizeof(filename), "%s%s.%s", folder.c_str(), time_string, options->Get().encoding.c_str());
        }
        else if (options->Get().timestamp)
                snprintf(filename, sizeof(filename), "%s%u.%s", folder.c_str(), (unsigned)time(NULL), options->Get().encoding.c_str());
        else if (!options->Get().output.empty())
                snprintf(filename, sizeof(filename), options->Get().output.c_str(), options->Get().framestart);
        else
                snprintf(filename, sizeof(filename), "capture_%u.%s", options->Get().framestart, options->Get().encoding.c_str());
        filename[sizeof(filename) - 1] = 0;
        return std::string(filename);
}

static void update_latest_link(std::string const &filename, StillOptions const *options)
{
        if (!options->Get().latest.empty())
        {
                fs::path link { options->Get().latest };
                fs::path target { filename };
                if (fs::exists(link) && !fs::remove(link))
                        LOG_ERROR("WARNING: could not delete latest link " << options->Get().latest);
                else
                {
                        fs::create_symlink(target, link);
                        LOG(2, "Link " << options->Get().latest << " created");
                }
        }
}

static std::string save_image(RPiCamStillApp &app, CompletedRequestPtr &payload, libcamera::Stream *stream,
                                                        std::string const &filename)
{
        StillOptions const *options = app.GetOptions();
        StreamInfo info = app.GetStreamInfo(stream);
        BufferReadSync r(&app, payload->buffers[stream]);
        const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
        if (stream == app.RawStream())
                dng_save(mem, info, payload->metadata, filename, app.CameraModel(), options);
        else if (options->Get().encoding == "jpg")
                jpeg_save(mem, info, payload->metadata, filename, app.CameraModel(), options);
        else if (options->Get().encoding == "png")
                png_save(mem, info, filename, options);
        else if (options->Get().encoding == "bmp")
                bmp_save(mem, info, filename, options);
        else
                yuv_save(mem, info, filename, options);
        LOG(2, "Saved image " << info.width << " x " << info.height << " to file " << filename);
        return filename;
}

static std::vector<std::string> save_images(RPiCamStillApp &app, CompletedRequestPtr &payload)
{
        StillOptions *options = app.GetOptions();
        std::vector<std::string> paths;
        std::string filename = generate_filename(options);
        paths.push_back(save_image(app, payload, app.StillStream(), filename));
        update_latest_link(filename, options);
        if (options->Get().raw)
        {
                filename = filename.substr(0, filename.rfind('.')) + ".dng";
                paths.push_back(save_image(app, payload, app.RawStream(), filename));
        }
        options->Set().framestart++;
        if (options->Get().wrap)
                options->Set().framestart %= options->Get().wrap;
        return paths;
}

static int get_colourspace_flags(const std::string &codec)
{
        if (codec == "mjpeg" || codec == "yuv420")
                return RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE;
        return RPiCamEncoder::FLAG_VIDEO_NONE;
}

class CameraWorker : public QObject
{
        Q_OBJECT

public:
        CameraWorker()
        {
                still_options_ = still_app_.GetOptions();
                still_options_->Set().qt_preview = true;
                still_options_->Set().nopreview = false;
                still_options_->Set().encoding = "jpg";
                still_options_->Set().codec = "h264";
                still_options_->Set().bitrate.set("10mbps");
        }

        ~CameraWorker()
        {
                shutdown();
        }

public slots:
        void initialize() {}

        void shutdown()
        {
                if (video_thread_.joinable())
                {
                        video_stop_requested_ = true;
                        RPiCamApp::MsgType t = RPiCamApp::MsgType::Quit;
                        RPiCamApp::MsgPayload payload;
                        if (encoder_)
                                encoder_->PostMessage(t, payload);
                        video_thread_.join();
                }
                if (encoder_)
                {
                        encoder_->StopCamera();
                        encoder_->StopEncoder();
                        encoder_->Teardown();
                        encoder_->CloseCamera();
                        encoder_.reset();
                        video_output_.reset();
                }
                if (preview_active_)
                {
                        still_app_.StopCamera();
                        still_app_.Teardown();
                        preview_active_ = false;
                }
                if (camera_open_)
                {
                        still_app_.CloseCamera();
                        camera_open_ = false;
                }
        }

        void startPreview()
        {
                try
                {
                        if (!camera_open_)
                        {
                                still_app_.OpenCamera();
                                camera_open_ = true;
                        }
                        still_app_.ConfigureViewfinder();
                        still_app_.StartCamera();
                        preview_active_ = true;
                        emit previewStarted();
                        emit statusMessage("Preview running");
                }
                catch (std::exception const &e)
                {
                        emit errorOccurred(QString::fromStdString(e.what()));
                }
        }

        void stopPreview()
        {
                try
                {
                        if (preview_active_)
                        {
                                still_app_.StopCamera();
                                still_app_.Teardown();
                                preview_active_ = false;
                                emit previewStopped();
                                emit statusMessage("Preview stopped");
                        }
                }
                catch (std::exception const &e)
                {
                        emit errorOccurred(QString::fromStdString(e.what()));
                }
        }

        void updateResolution(int width, int height)
        {
                still_options_->Set().width = width;
                still_options_->Set().height = height;
        }

        void updateFramerate(double fps)
        {
                if (fps <= 0.0)
                        still_options_->Set().framerate.reset();
                else
                        still_options_->Set().framerate = static_cast<float>(fps);
        }

        void updateStillEncoding(const QString &encoding)
        {
                still_options_->Set().encoding = encoding.toStdString();
        }

        void updateVideoCodec(const QString &codec)
        {
                still_options_->Set().codec = codec.toStdString();
        }

        void updateVideoBitrate(int kbps)
        {
                still_options_->Set().bitrate.set(std::to_string(kbps) + "kbps");
        }

        void setStillOutput(const QString &path)
        {
                still_options_->Set().output = path.toStdString();
        }

        void setVideoOutput(const QString &path)
        {
                video_output_path_ = path.toStdString();
        }

        void captureStill()
        {
                try
                {
                        unsigned int flags = RPiCamApp::FLAG_STILL_NONE;
                        std::string encoding = still_options_->Get().encoding;
                        if (encoding == "rgb24" || encoding == "png")
                                flags |= RPiCamApp::FLAG_STILL_BGR;
                        if (encoding == "rgb48")
                                flags |= RPiCamApp::FLAG_STILL_BGR48;
                        else if (encoding == "bmp")
                                flags |= RPiCamApp::FLAG_STILL_RGB;
                        if (still_options_->Get().raw)
                                flags |= RPiCamApp::FLAG_STILL_RAW;

                        bool resume_preview = preview_active_;
                        if (preview_active_)
                        {
                                still_app_.StopCamera();
                                still_app_.Teardown();
                                preview_active_ = false;
                        }

                        still_app_.ConfigureStill(flags);
                        still_app_.StartCamera();

                        std::vector<std::string> captured_paths;
                        while (true)
                        {
                                RPiCamApp::Msg msg = still_app_.Wait();
                                if (msg.type == RPiCamApp::MsgType::Timeout)
                                {
                                        still_app_.StopCamera();
                                        still_app_.StartCamera();
                                        continue;
                                }
                                if (msg.type == RPiCamApp::MsgType::Quit)
                                        throw std::runtime_error("Preview window closed");
                                if (msg.type == RPiCamApp::MsgType::RequestComplete)
                                {
                                        CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);
                                        captured_paths = save_images(still_app_, payload);
                                        break;
                                }
                        }

                        still_app_.StopCamera();
                        still_app_.Teardown();

                        if (resume_preview)
                        {
                                still_app_.ConfigureViewfinder();
                                still_app_.StartCamera();
                                preview_active_ = true;
                                emit previewStarted();
                        }

                        if (!captured_paths.empty())
                        {
                                emit stillCaptured(QString::fromStdString(captured_paths.front()));
                                emit statusMessage(QStringLiteral("Still captured: %1").arg(QString::fromStdString(captured_paths.front())));
                        }
                }
                catch (std::exception const &e)
                {
                        emit errorOccurred(QString::fromStdString(e.what()));
                }
        }

        void startVideo()
        {
                try
                {
                        if (video_thread_.joinable())
                                return;

                        if (preview_active_)
                        {
                                still_app_.StopCamera();
                                still_app_.Teardown();
                                preview_active_ = false;
                        }

                        encoder_ = std::make_unique<RPiCamEncoder>();
                        VideoOptions *video_opts = encoder_->GetOptions();
                        video_opts->Set() = still_options_->Get();
                        video_opts->Set().output = video_output_path_;
                        if (video_opts->Get().output.empty())
                                video_opts->Set().output = "video.h264";

                        video_output_ = std::unique_ptr<Output>(Output::Create(video_opts));
                        encoder_->SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, video_output_.get(), _1, _2, _3, _4));
                        encoder_->SetMetadataReadyCallback(std::bind(&Output::MetadataReady, video_output_.get(), _1));

                        encoder_->OpenCamera();
                        encoder_->ConfigureVideo(get_colourspace_flags(video_opts->Get().codec));
                        encoder_->StartEncoder();
                        encoder_->StartCamera();

                        video_stop_requested_ = false;
                        video_thread_ = std::thread(&CameraWorker::videoLoop, this);
                        emit videoStarted(QString::fromStdString(video_opts->Get().output));
                        emit statusMessage(QStringLiteral("Recording video to %1").arg(QString::fromStdString(video_opts->Get().output)));
                }
                catch (std::exception const &e)
                {
                        emit errorOccurred(QString::fromStdString(e.what()));
                }
        }

        void stopVideo()
        {
                if (!encoder_)
                        return;

                video_stop_requested_ = true;
                RPiCamApp::MsgType t = RPiCamApp::MsgType::Quit;
                RPiCamApp::MsgPayload payload;
                encoder_->PostMessage(t, payload);
                if (video_thread_.joinable())
                        video_thread_.join();

                encoder_->StopCamera();
                encoder_->StopEncoder();
                encoder_->Teardown();
                encoder_->CloseCamera();
                emit videoStopped(QString::fromStdString(encoder_->GetOptions()->Get().output));
                emit statusMessage("Video recording stopped");

                encoder_.reset();
                video_output_.reset();
        }

signals:
        void previewStarted();
        void previewStopped();
        void stillCaptured(const QString &path);
        void videoStarted(const QString &path);
        void videoStopped(const QString &path);
        void statusMessage(const QString &message);
        void errorOccurred(const QString &message);

private:
        void videoLoop()
        {
                Stream *video_stream = encoder_->VideoStream();
                while (!video_stop_requested_)
                {
                        RPiCamEncoder::Msg msg = encoder_->Wait();
                        if (video_stop_requested_)
                                break;
                        if (msg.type == RPiCamApp::MsgType::Timeout)
                        {
                                encoder_->StopCamera();
                                encoder_->StartCamera();
                                continue;
                        }
                        if (msg.type == RPiCamApp::MsgType::Quit)
                                break;
                        if (msg.type != RPiCamApp::MsgType::RequestComplete)
                                continue;
                        CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);
                        encoder_->EncodeBuffer(payload, video_stream);
                }
        }

        RPiCamStillApp still_app_;
        StillOptions *still_options_ = nullptr;
        bool camera_open_ = false;
        bool preview_active_ = false;
        std::unique_ptr<RPiCamEncoder> encoder_;
        std::unique_ptr<Output> video_output_;
        std::thread video_thread_;
        std::atomic<bool> video_stop_requested_ { false };
        std::string video_output_path_;
};

class MainWindow : public QMainWindow
{
        Q_OBJECT

public:
        MainWindow()
        {
                worker_ = new CameraWorker();
                worker_->moveToThread(&worker_thread_);
                connect(&worker_thread_, &QThread::finished, worker_, &QObject::deleteLater);
                worker_thread_.start();
                QMetaObject::invokeMethod(worker_, "initialize", Qt::QueuedConnection);

                createUi();
                connectSignals();
        }

        ~MainWindow()
        {
                worker_thread_.quit();
                worker_thread_.wait();
        }

protected:
        void closeEvent(QCloseEvent *event) override
        {
                QMetaObject::invokeMethod(worker_, "shutdown", Qt::BlockingQueuedConnection);
                worker_thread_.quit();
                worker_thread_.wait();
                QMainWindow::closeEvent(event);
        }

private:
        void createUi()
        {
                QWidget *central = new QWidget(this);
                setCentralWidget(central);

                auto *main_layout = new QHBoxLayout(central);
                preview_container_ = new QWidget(central);
                preview_container_->setObjectName("previewContainer");
                preview_container_->setMinimumSize(640, 480);
                preview_container_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
                main_layout->addWidget(preview_container_, 3);

                auto *control_layout = new QVBoxLayout();
                main_layout->addLayout(control_layout, 2);

                auto *capture_group = new QGroupBox(tr("Capture"), central);
                auto *capture_layout = new QVBoxLayout(capture_group);

                start_preview_button_ = new QPushButton(tr("Start Preview"), capture_group);
                stop_preview_button_ = new QPushButton(tr("Stop Preview"), capture_group);
                capture_button_ = new QPushButton(tr("Capture Photo"), capture_group);
                start_video_button_ = new QPushButton(tr("Start Video"), capture_group);
                stop_video_button_ = new QPushButton(tr("Stop Video"), capture_group);
                stop_preview_button_->setEnabled(false);
                stop_video_button_->setEnabled(false);

                auto *still_output_layout = new QHBoxLayout();
                still_output_edit_ = new QLineEdit(capture_group);
                still_output_edit_->setPlaceholderText(tr("Filename or pattern (e.g. image%04u.jpg)"));
                auto *still_browse = new QPushButton(tr("Browse"), capture_group);
                still_output_layout->addWidget(still_output_edit_);
                still_output_layout->addWidget(still_browse);

                auto *video_output_layout = new QHBoxLayout();
                video_output_edit_ = new QLineEdit(capture_group);
                video_output_edit_->setPlaceholderText(tr("Video file (e.g. clip.h264)"));
                auto *video_browse = new QPushButton(tr("Browse"), capture_group);
                video_output_layout->addWidget(video_output_edit_);
                video_output_layout->addWidget(video_browse);

                capture_layout->addWidget(start_preview_button_);
                capture_layout->addWidget(stop_preview_button_);
                capture_layout->addWidget(capture_button_);
                capture_layout->addWidget(start_video_button_);
                capture_layout->addWidget(stop_video_button_);
                capture_layout->addWidget(new QLabel(tr("Still output"), capture_group));
                capture_layout->addLayout(still_output_layout);
                capture_layout->addWidget(new QLabel(tr("Video output"), capture_group));
                capture_layout->addLayout(video_output_layout);

                control_layout->addWidget(capture_group);

                auto *settings_group = new QGroupBox(tr("Settings"), central);
                auto *settings_form = new QFormLayout(settings_group);

                width_spin_ = new QSpinBox(settings_group);
                width_spin_->setRange(0, 8000);
                width_spin_->setValue(0);
                height_spin_ = new QSpinBox(settings_group);
                height_spin_->setRange(0, 8000);
                height_spin_->setValue(0);
                framerate_spin_ = new QDoubleSpinBox(settings_group);
                framerate_spin_->setRange(0.0, 120.0);
                framerate_spin_->setDecimals(2);
                framerate_spin_->setSingleStep(1.0);
                framerate_spin_->setValue(30.0);
                encoding_combo_ = new QComboBox(settings_group);
                encoding_combo_->addItems({ "jpg", "png", "bmp", "yuv420" });
                codec_combo_ = new QComboBox(settings_group);
                codec_combo_->addItems({ "h264", "mjpeg", "yuv420" });
                bitrate_combo_ = new QComboBox(settings_group);
                bitrate_combo_->addItem("5 Mbps", 5000);
                bitrate_combo_->addItem("10 Mbps", 10000);
                bitrate_combo_->addItem("20 Mbps", 20000);
                bitrate_combo_->setCurrentIndex(1);

                settings_form->addRow(tr("Width"), width_spin_);
                settings_form->addRow(tr("Height"), height_spin_);
                settings_form->addRow(tr("Framerate"), framerate_spin_);
                settings_form->addRow(tr("Still encoding"), encoding_combo_);
                settings_form->addRow(tr("Video codec"), codec_combo_);
                settings_form->addRow(tr("Video bitrate"), bitrate_combo_);

                control_layout->addWidget(settings_group);
                control_layout->addStretch(1);

                status_label_ = new QLabel(tr("Ready"), central);
                statusBar()->addPermanentWidget(status_label_, 1);

                connect(still_browse, &QPushButton::clicked, this, &MainWindow::chooseStillOutput);
                connect(video_browse, &QPushButton::clicked, this, &MainWindow::chooseVideoOutput);
        }

        void connectSignals()
        {
                connect(start_preview_button_, &QPushButton::clicked, this, &MainWindow::startPreview);
                connect(stop_preview_button_, &QPushButton::clicked, this, &MainWindow::stopPreview);
                connect(capture_button_, &QPushButton::clicked, this, &MainWindow::captureStill);
                connect(start_video_button_, &QPushButton::clicked, this, &MainWindow::startVideo);
                connect(stop_video_button_, &QPushButton::clicked, this, &MainWindow::stopVideo);

                connect(width_spin_, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::updateResolution);
                connect(height_spin_, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::updateResolution);
                connect(framerate_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateFramerate);
                connect(encoding_combo_, &QComboBox::currentTextChanged, this, &MainWindow::updateStillEncoding);
                connect(codec_combo_, &QComboBox::currentTextChanged, this, &MainWindow::updateVideoCodec);
                connect(bitrate_combo_, &QComboBox::currentIndexChanged, this, &MainWindow::updateBitrate);

                connect(worker_, &CameraWorker::previewStarted, this, &MainWindow::onPreviewStarted);
                connect(worker_, &CameraWorker::previewStopped, this, &MainWindow::onPreviewStopped);
                connect(worker_, &CameraWorker::stillCaptured, this, &MainWindow::onStillCaptured);
                connect(worker_, &CameraWorker::videoStarted, this, &MainWindow::onVideoStarted);
                connect(worker_, &CameraWorker::videoStopped, this, &MainWindow::onVideoStopped);
                connect(worker_, &CameraWorker::statusMessage, this, &MainWindow::onStatusMessage);
                connect(worker_, &CameraWorker::errorOccurred, this, &MainWindow::onError);

                updateResolution();
                updateFramerate(framerate_spin_->value());
                updateStillEncoding(encoding_combo_->currentText());
                updateVideoCodec(codec_combo_->currentText());
                updateBitrate(bitrate_combo_->currentIndex());
        }

        void startPreview()
        {
                QtPreview::RegisterPreviewParent(preview_container_);
                QMetaObject::invokeMethod(worker_, "startPreview", Qt::QueuedConnection);
                start_preview_button_->setEnabled(false);
                stop_preview_button_->setEnabled(true);
        }

        void stopPreview()
        {
                QMetaObject::invokeMethod(worker_, "stopPreview", Qt::QueuedConnection);
        }

        void captureStill()
        {
                QMetaObject::invokeMethod(worker_, "setStillOutput", Qt::QueuedConnection,
                                          Q_ARG(QString, still_output_edit_->text()));
                QMetaObject::invokeMethod(worker_, "captureStill", Qt::QueuedConnection);
        }

        void startVideo()
        {
                QMetaObject::invokeMethod(worker_, "setVideoOutput", Qt::QueuedConnection,
                                          Q_ARG(QString, video_output_edit_->text()));
                QMetaObject::invokeMethod(worker_, "startVideo", Qt::QueuedConnection);
                start_video_button_->setEnabled(false);
                stop_video_button_->setEnabled(true);
                start_preview_button_->setEnabled(false);
                stop_preview_button_->setEnabled(false);
        }

        void stopVideo()
        {
                QMetaObject::invokeMethod(worker_, "stopVideo", Qt::QueuedConnection);
        }

        void chooseStillOutput()
        {
                QString path = QFileDialog::getSaveFileName(this, tr("Select still output"), still_output_edit_->text());
                if (!path.isEmpty())
                        still_output_edit_->setText(path);
        }

        void chooseVideoOutput()
        {
                QString path = QFileDialog::getSaveFileName(this, tr("Select video output"), video_output_edit_->text());
                if (!path.isEmpty())
                        video_output_edit_->setText(path);
        }

        void updateResolution()
        {
                QMetaObject::invokeMethod(worker_, "updateResolution", Qt::QueuedConnection,
                                          Q_ARG(int, width_spin_->value()), Q_ARG(int, height_spin_->value()));
        }

        void updateFramerate(double value)
        {
                QMetaObject::invokeMethod(worker_, "updateFramerate", Qt::QueuedConnection, Q_ARG(double, value));
        }

        void updateStillEncoding(const QString &encoding)
        {
                QMetaObject::invokeMethod(worker_, "updateStillEncoding", Qt::QueuedConnection, Q_ARG(QString, encoding));
        }

        void updateVideoCodec(const QString &codec)
        {
                QMetaObject::invokeMethod(worker_, "updateVideoCodec", Qt::QueuedConnection, Q_ARG(QString, codec));
        }

        void updateBitrate(int index)
        {
                int kbps = bitrate_combo_->itemData(index).toInt();
                QMetaObject::invokeMethod(worker_, "updateVideoBitrate", Qt::QueuedConnection, Q_ARG(int, kbps));
        }

        void onPreviewStarted()
        {
                start_preview_button_->setEnabled(false);
                stop_preview_button_->setEnabled(true);
        }

        void onPreviewStopped()
        {
                start_preview_button_->setEnabled(true);
                stop_preview_button_->setEnabled(false);
        }

        void onStillCaptured(const QString &path)
        {
                QMessageBox::information(this, tr("Capture complete"), tr("Saved still to %1").arg(path));
        }

        void onVideoStarted(const QString &path)
        {
                QMessageBox::information(this, tr("Video recording"), tr("Recording to %1").arg(path));
        }

        void onVideoStopped(const QString &path)
        {
                start_video_button_->setEnabled(true);
                stop_video_button_->setEnabled(false);
                start_preview_button_->setEnabled(true);
                stop_preview_button_->setEnabled(false);
                QMessageBox::information(this, tr("Recording stopped"), tr("Saved video to %1").arg(path));
        }

        void onStatusMessage(const QString &message)
        {
                status_label_->setText(message);
        }

        void onError(const QString &message)
        {
                QMessageBox::critical(this, tr("Camera error"), message);
                status_label_->setText(message);
                start_preview_button_->setEnabled(true);
                stop_preview_button_->setEnabled(false);
                start_video_button_->setEnabled(true);
                stop_video_button_->setEnabled(false);
        }

        CameraWorker *worker_ = nullptr;
        QThread worker_thread_;
        QWidget *preview_container_ = nullptr;
        QPushButton *start_preview_button_ = nullptr;
        QPushButton *stop_preview_button_ = nullptr;
        QPushButton *capture_button_ = nullptr;
        QPushButton *start_video_button_ = nullptr;
        QPushButton *stop_video_button_ = nullptr;
        QLineEdit *still_output_edit_ = nullptr;
        QLineEdit *video_output_edit_ = nullptr;
        QSpinBox *width_spin_ = nullptr;
        QSpinBox *height_spin_ = nullptr;
        QDoubleSpinBox *framerate_spin_ = nullptr;
        QComboBox *encoding_combo_ = nullptr;
        QComboBox *codec_combo_ = nullptr;
        QComboBox *bitrate_combo_ = nullptr;
        QLabel *status_label_ = nullptr;
};

int main(int argc, char **argv)
{
        QApplication app(argc, argv);
        MainWindow window;
        window.setWindowTitle(QObject::tr("RPi Camera GUI"));
        window.resize(1280, 720);
        window.show();
        return app.exec();
}

