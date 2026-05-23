/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, Kletternaut
 *
 * rpicam-ctrl — Qt runtime control panel for rpicam-vid.
 *
 * Connects to /tmp/rpicam-vid.sock and sends parameter updates via
 * Unix Domain Socket. Requires rpicam-vid built with the ControlSocket patch.
 *
 * Build:
 *   meson configure build -Denable_rpicam_ctrl=enabled
 *   ninja -C build
 */

#include <array>
#include <cmath>

#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QFile>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLocalSocket>
#include <QMainWindow>
#include <QMap>
#include <QProcess>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSettings>
#include <QShortcut>
#include <QSlider>
#include <QStandardItemModel>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

static constexpr int RECONN_MS = 2000;
static constexpr int DEBOUNCE_MS = 60;

// ---------------------------------------------------------------------------
// Camera capability probe (via rpicam-hello --list-cameras)
// ---------------------------------------------------------------------------

struct CameraInfo
{
	bool present = false;
	bool hasAf = false;
	int maxFps = 120; // populated from --list-cameras output
	QString model;
};

// Known cameras that support autofocus.
static const QSet<QString> kAfCameraModels = {
	"imx708", // Raspberry Pi Camera Module 3
	"arducam_64mp", // Arducam 64 MP
	"arducam_imx519", // Arducam 16 MP
	"arducam_imx708", // Arducam imx708 variant
};

// Runs "rpicam-hello --list-cameras", parses output and returns info for
// cameras 0 and 1. If the probe fails (command not found, timeout, …) both
// entries remain at their zero-initialised defaults so the GUI falls back to
// showing all controls without restriction.
static std::array<CameraInfo, 2> probeCameras()
{
	std::array<CameraInfo, 2> info;

	QProcess proc;
	proc.setProcessChannelMode(QProcess::MergedChannels);
	proc.start("rpicam-hello", { "--list-cameras" });
	if (!proc.waitForFinished(4000))
	{
		proc.kill();
		return info;
	}

	const QString out = proc.readAllStandardOutput();

	// Lines of interest look like:  "0 : imx708 [4608x2592 …]"
	QRegularExpression re(R"(^\s*(\d+)\s*:\s*(\S+)\s*\[)");
	// Mode lines look like: "         2028x1520 [40.01 fps - …]"
	QRegularExpression reFps(R"(\[(\d+\.?\d*)\s+fps)");
	int parsedCamIdx = -1;
	for (const QString &line : out.split('\n'))
	{
		const auto m = re.match(line);
		if (m.hasMatch())
		{
			parsedCamIdx = m.captured(1).toInt();
			if (parsedCamIdx < 0 || parsedCamIdx >= 2)
			{
				parsedCamIdx = -1;
				continue;
			}
			const QString model = m.captured(2).toLower();
			info[parsedCamIdx].present = true;
			info[parsedCamIdx].model = model;
			info[parsedCamIdx].hasAf = kAfCameraModels.contains(model);
			info[parsedCamIdx].maxFps = 0; // filled in by mode lines below
		}
		else if (parsedCamIdx >= 0)
		{
			const auto mFps = reFps.match(line);
			if (mFps.hasMatch())
			{
				const int fps = static_cast<int>(mFps.captured(1).toDouble());
				info[parsedCamIdx].maxFps = std::max(info[parsedCamIdx].maxFps, fps);
			}
		}
	}
	// Ensure a sensible default for cameras where no fps line was found.
	for (auto &ci : info)
		if (ci.present && ci.maxFps == 0)
			ci.maxFps = 120;
	return info;
}

// ---------------------------------------------------------------------------
// Framerate slider: value 0 = auto, 1..N = fps directly (1 fps integer steps).
// ---------------------------------------------------------------------------
static constexpr int FPS_SLIDER_MAX_DEFAULT = 120; // overridden by caps/probe

// Returns the display label for a framerate slider value.
static QString framerateLabel(int fps)
{
	return fps <= 0 ? "auto" : QString("%1 fps").arg(fps);
}

// ---------------------------------------------------------------------------
// Shutter slider: position 0 = auto; 1..SHUTTER_STEPS maps logarithmically
// to 100 µs .. maxShutterUs(fpsIdx).  The upper bound is dynamic: it equals
// 1 000 000 / fps so the shutter can never exceed one frame period.
// ---------------------------------------------------------------------------
static constexpr int SHUTTER_STEPS = 200;

// Returns the maximum meaningful shutter in µs for the given fps slider index.
// fallbackMaxFps is used when fpsIdx == 0 (auto); it comes from the caps message.
static int maxShutterUs(int fpsSlidVal, int fallbackMaxFps)
{
	double fps = static_cast<double>(fpsSlidVal);
	if (fps <= 0.0)
		fps = (fallbackMaxFps > 0) ? static_cast<double>(fallbackMaxFps) : 30.0;
	if (fps < 1.0)
		fps = 1.0; // floor at 1 fps
	return static_cast<int>(1'000'000.0 / fps);
}

static int shutterMicros(int sliderVal, int maxUs)
{
	if (sliderVal <= 0)
		return 0;
	const double lo = 100.0;
	const double hi = static_cast<double>(maxUs);
	const double t = static_cast<double>(sliderVal - 1) / (SHUTTER_STEPS - 1);
	return static_cast<int>(std::round(lo * std::pow(hi / lo, t)));
}

// Inverse of shutterMicros: find the closest slider position for targetUs.
static int shutterSliderForMicros(int targetUs, int maxUs)
{
	if (targetUs <= 0)
		return 0;
	const double lo = 100.0;
	const double hi = static_cast<double>(maxUs);
	if (targetUs <= static_cast<int>(lo))
		return 1;
	if (targetUs >= maxUs)
		return SHUTTER_STEPS;
	const double t = std::log(static_cast<double>(targetUs) / lo) / std::log(hi / lo);
	return std::clamp(static_cast<int>(std::round(t * (SHUTTER_STEPS - 1))) + 1, 1, SHUTTER_STEPS);
}

static QString shutterLabel(int sliderVal, int maxUs)
{
	if (sliderVal <= 0)
		return "auto";
	const int us = shutterMicros(sliderVal, maxUs);
	if (us < 1000)
		return QString("%1 µs").arg(us);
	if (us < 1'000'000)
		return QString("%1 ms").arg(us / 1000.0, 0, 'f', 1);
	return QString("%1 s").arg(us / 1'000'000.0, 0, 'f', 2);
}

// ---------------------------------------------------------------------------
struct CameraState
{
	int brightness = 0;
	int ev = 0;
	int contrast = 100;
	int saturation = 100;
	int zoom = 10;
	int gain = 10;
	int sharpness = 10;
	int awbGainR = 150;
	int awbGainB = 120;
	int awbIdx = 0;
	int meteringIdx = 0;
	int exposureIdx = 0;
	int denoiseIdx = 0;
	int hdrIdx = 0;
	int shutter = 0; // slider position 0..SHUTTER_STEPS; 0=auto, log-scale -> shutterMicros()
	int framerateIdx = 0; // fps directly: 0=auto, 1..N fps
	int afIdx = 0; // 0=auto, 1=continuous, 2=manual
	int lens = 0; // 0..100; val/10.0 = LensPosition
};

// ---------------------------------------------------------------------------

class ControlWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit ControlWindow(QWidget *parent = nullptr) : QMainWindow(parent)
	{
		setWindowTitle("rpicam-ctrl");
		setMinimumWidth(520);

		sock_ = new QLocalSocket(this);
		connect(sock_, &QLocalSocket::stateChanged, this,
				[this](QLocalSocket::LocalSocketState s)
				{
					if (s == QLocalSocket::ConnectedState)
						onConnected();
					else if (s == QLocalSocket::UnconnectedState)
						onDisconnected();
				});
		connect(sock_, &QLocalSocket::readyRead, this, &ControlWindow::onSocketData);

		reconnTimer_ = new QTimer(this);
		reconnTimer_->setInterval(RECONN_MS);
		connect(reconnTimer_, &QTimer::timeout, this, &ControlWindow::tryConnect);

		debounceTimer_ = new QTimer(this);
		debounceTimer_->setSingleShot(true);
		debounceTimer_->setInterval(DEBOUNCE_MS);
		connect(debounceTimer_, &QTimer::timeout, this, &ControlWindow::flush);

		// Probe available cameras before building the UI so that buildUi()
		// can populate the camera combo and set AF controls accordingly.
		camInfo_ = probeCameras();

		// Auto-select cam1 if only cam1 has a live rpicam-vid socket.
		// This handles the case where cam0 is listed by libcamera but is not
		// actually running (e.g. hardware fault), while cam1 is active.
		// Socket presence is a more reliable signal than probe output.
		// Must happen before buildUi() so the combo is initialized correctly.
		const bool sock0 = QFile::exists("/tmp/rpicam-vid0.sock");
		const bool sock1 = QFile::exists("/tmp/rpicam-vid1.sock");
		if (!sock0 && sock1)
		{
			currentCameraIdx_ = 1;
			currentSockPath_ = "/tmp/rpicam-vid1.sock";
		}

		buildUi();
		loadAllStatesFromFiles();
		tryConnect();

		// Keyboard shortcuts (mirrors TUI)
		connect(new QShortcut(Qt::Key_R, this), &QShortcut::activated, this, &ControlWindow::onReset);
		connect(new QShortcut(Qt::Key_Q, this), &QShortcut::activated, this, &QMainWindow::close);
		connect(new QShortcut(Qt::Key_C, this), &QShortcut::activated, this,
				[this]()
				{
					int next = (cameraCombo_->currentIndex() + 1) % cameraCombo_->count();
					cameraCombo_->setCurrentIndex(next);
				});

		// Restore last window position (but let height adjust to content).
		QSettings settings;
		if (settings.contains("window/geometry"))
		{
			restoreGeometry(settings.value("window/geometry").toByteArray());
			adjustSize();
		}

		// Status bar text slightly smaller than the application default.
		QFont sbFont = statusBar()->font();
		sbFont.setPointSizeF(sbFont.pointSizeF() * 0.85);
		statusBar()->setFont(sbFont);
	}

protected:
	void closeEvent(QCloseEvent *event) override
	{
		// Disconnect socket signals before closing to avoid callbacks into
		// partially-destroyed widgets during Qt's teardown sequence.
		reconnTimer_->stop();
		debounceTimer_->stop();
		sock_->disconnect();
		sock_->abort();

		QSettings settings;
		settings.setValue("window/geometry", saveGeometry());
		saveCameraState(); // capture any unsaved changes for the active camera
		for (int i = 0; i < 2; ++i)
			saveCameraStateToFile(i, cameraStates_[i]);
		QMainWindow::closeEvent(event);
	}

private slots:
	void tryConnect()
	{
		if (sock_->state() != QLocalSocket::UnconnectedState)
			return;
		sock_->connectToServer(currentSockPath_);
	}

	void onConnected()
	{
		reconnTimer_->stop();
		statusBar()->showMessage("  \u25cf  Connected  \u2014  " + currentSockPath_);
		statusBar()->setStyleSheet("QStatusBar { color: green; }");
		// Reload state from file on every connect. If rpicam-vid restarted
		// it deleted the state file → loadCameraStateFromFile() returns
		// defaults, so we never push stale in-memory values to a fresh camera
		// session. If the GUI restarted while the camera was already running,
		// the file still exists and we restore exactly what was set before.
		cameraStates_[currentCameraIdx_] = loadCameraStateFromFile(currentCameraIdx_);
		applyUiState(cameraStates_[currentCameraIdx_]);
		// Defer by one event-loop cycle so the socket is fully open for
		// writing (stateChanged fires before isOpen() is true).
		QTimer::singleShot(0, this, [this] { sendCameraState(cameraStates_[currentCameraIdx_]); });
	}

	void onDisconnected()
	{
		statusBar()->showMessage("  \u25cb  Disconnected  \u2014  retrying every 2 s\u2026");
		statusBar()->setStyleSheet("QStatusBar { color: #c0392b; }");
		if (!reconnTimer_->isActive())
			reconnTimer_->start();
	}

	// Receives capability information pushed by rpicam-vid on connect.
	// Format: "caps:maxfps=<n>,hasaf=<0|1>\n"
	void onSocketData()
	{
		while (sock_->canReadLine())
		{
			const QString line = QString::fromUtf8(sock_->readLine()).trimmed();
			if (line.startsWith("caps:"))
				parseCaps(line.mid(5));
		}
	}

	void parseCaps(const QString &caps)
	{
		for (const QString &kv : caps.split(','))
		{
			const QStringList parts = kv.split('=');
			if (parts.size() != 2)
				continue;
			const QString key = parts[0].trimmed();
			const QString val = parts[1].trimmed();
			if (key == "maxfps")
			{
				const int fps = val.toInt();
				if (fps > 0 && currentCameraIdx_ >= 0 && currentCameraIdx_ < 2)
				{
					// Store the authoritative max fps from the active sensor mode.
					// Used by maxShutterUs() when framerate is set to auto.
					camInfo_[currentCameraIdx_].maxFps = fps;
					// Clamp the framerate slider to the hardware max.
					QSignalBlocker b(framerateSlider_);
					framerateSlider_->setMaximum(fps);
					if (framerateSlider_->value() > fps)
					{
						framerateSlider_->setValue(0);
						framerateVal_->setText("auto");
					}
				}
			}
			else if (key == "hasaf")
			{
				if (currentCameraIdx_ >= 0 && currentCameraIdx_ < 2)
					camInfo_[currentCameraIdx_].hasAf = (val == "1");
				applyCameraCapabilities(currentCameraIdx_);
			}
		}
	}

	// Queue a key:value command and (re)start the debounce window.
	void queue(const QString &key, const QString &value)
	{
		pending_[key] = value;
		debounceTimer_->start();
	}

	// Send all queued commands in a single write.
	void flush()
	{
		if (sock_->state() != QLocalSocket::ConnectedState || pending_.isEmpty())
			return;
		QString batch;
		for (auto it = pending_.cbegin(); it != pending_.cend(); ++it)
			batch += it.key() + ":" + it.value() + "\n";
		pending_.clear();
		sock_->write(batch.toUtf8());
		sock_->flush();
	}

	// Send a command immediately (discrete controls, reset).
	void sendNow(const QString &payload)
	{
		if (!sock_->isOpen() || sock_->state() != QLocalSocket::ConnectedState)
			return;
		sock_->write((payload + "\n").toUtf8());
		sock_->flush();
	}

	// ------------------------------------------------------------------
	// Slider / combo slots
	// ------------------------------------------------------------------

	void onBrightnessChanged(int v)
	{
		double val = v / 100.0;
		brightnessVal_->setText(QString::asprintf("%+.2f", val));
		queue("brightness", QString::number(val, 'f', 2));
	}

	void onEvChanged(int v)
	{
		double val = v / 10.0;
		evVal_->setText(QString::asprintf("%+.1f", val));
		queue("ev", QString::number(val, 'f', 1));
	}

	void onContrastChanged(int v)
	{
		double val = v / 100.0;
		contrastVal_->setText(QString::number(val, 'f', 2));
		queue("contrast", QString::number(val, 'f', 2));
	}

	void onSaturationChanged(int v)
	{
		double val = v / 100.0;
		saturationVal_->setText(QString::number(val, 'f', 2));
		queue("saturation", QString::number(val, 'f', 2));
	}

	// Zoom: slider 10…100  →  1.0x … 10.0x centred ROI
	void onZoomChanged(int v)
	{
		double factor = v / 10.0;
		double w = 1.0 / factor;
		double h = 1.0 / factor;
		double x = (1.0 - w) / 2.0;
		double y = (1.0 - h) / 2.0;
		zoomVal_->setText(QString::number(factor, 'f', 1) + "\xc3\x97"); // ×
		queue("roi", QString("%1,%2,%3,%4").arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(w, 0, 'f', 4).arg(h, 0, 'f', 4));
	}

	void onGainChanged(int v)
	{
		double val = v / 10.0;
		gainVal_->setText(QString::number(val, 'f', 1) + "x");
		queue("gain", QString::number(val, 'f', 1));
	}

	void onSharpnessChanged(int v)
	{
		double val = v / 10.0;
		sharpnessVal_->setText(QString::number(val, 'f', 1));
		queue("sharpness", QString::number(val, 'f', 1));
	}

	void onShutterChanged(int v)
	{
		const int maxUs = maxShutterUs(framerateSlider_->value(), camInfo_[currentCameraIdx_].maxFps);
		shutterVal_->setText(shutterLabel(v, maxUs));
		queue("shutter", QString::number(shutterMicros(v, maxUs)));
	}

	void onFramerateChanged(int fps)
	{
		framerateVal_->setText(framerateLabel(fps));
		// Send 0 for auto, otherwise the integer fps value.
		queue("framerate", fps <= 0 ? "0" : QString::number(fps));

		// Rescale the shutter slider: preserve the current µs value as closely
		// as possible, but clamp to the new maximum (= one frame period).
		const int newMaxUs = maxShutterUs(fps, camInfo_[currentCameraIdx_].maxFps);
		const int currentUs = shutterMicros(shutterSlider_->value(), newMaxUs);
		const int clampedUs = std::min(currentUs, newMaxUs);
		{
			QSignalBlocker b(shutterSlider_);
			shutterSlider_->setValue(shutterSliderForMicros(clampedUs, newMaxUs));
		}
		shutterVal_->setText(shutterLabel(shutterSlider_->value(), newMaxUs));
		// Re-send shutter with the recalculated µs so the camera reflects the
		// new one-frame-period maximum. Skip when shutter is on auto (pos == 0).
		if (shutterSlider_->value() > 0)
			queue("shutter", QString::number(shutterMicros(shutterSlider_->value(), newMaxUs)));
	}

	void onAfChanged(int idx)
	{
		static const QStringList modes = { "auto", "continuous", "manual" };
		lensSlider_->setEnabled(idx == 2);
		if (idx >= 0 && idx < modes.size())
			sendNow("af:" + modes[idx]);
	}

	void onLensChanged(int v)
	{
		lensVal_->setText(QString::number(v / 10.0, 'f', 1));
		queue("lens", QString::number(v / 10.0, 'f', 1));
	}

	void onAwbChanged(int idx)
	{
		static const QStringList modes = { "auto",	 "incandescent", "tungsten", "fluorescent",
										   "indoor", "daylight",	 "cloudy",	 "manual" };
		bool isManual = (idx == modes.size() - 1);
		if (!isManual)
			sendNow("awb:" + modes[idx]);
		else
		{
			// Send current slider values as manual gains
			sendNow(QString("awbgains:%1,%2")
						.arg(awbGainRSlider_->value() / 100.0, 0, 'f', 2)
						.arg(awbGainBSlider_->value() / 100.0, 0, 'f', 2));
		}
	}

	void onAwbGainRChanged(int v)
	{
		double val = v / 100.0;
		awbGainRVal_->setText(QString::number(val, 'f', 2));
		setAwbComboToManual();
		queue("awbgains", QString("%1,%2").arg(val, 0, 'f', 2).arg(awbGainBSlider_->value() / 100.0, 0, 'f', 2));
	}

	void onAwbGainBChanged(int v)
	{
		double val = v / 100.0;
		awbGainBVal_->setText(QString::number(val, 'f', 2));
		setAwbComboToManual();
		queue("awbgains", QString("%1,%2").arg(awbGainRSlider_->value() / 100.0, 0, 'f', 2).arg(val, 0, 'f', 2));
	}

	void setAwbComboToManual()
	{
		static const QStringList modes = { "auto",	 "incandescent", "tungsten", "fluorescent",
										   "indoor", "daylight",	 "cloudy",	 "manual" };
		int manualIdx = modes.size() - 1;
		if (awbCombo_->currentIndex() != manualIdx)
		{
			QSignalBlocker b(awbCombo_);
			awbCombo_->setCurrentIndex(manualIdx);
		}
	}

	void onHdrChanged(int idx)
	{
		static const QStringList modes = { "off", "auto", "sensor", "single-exp" };
		if (idx >= 0 && idx < modes.size())
			sendNow("hdr:" + modes[idx]);
	}

	void onMeteringChanged(int idx)
	{
		static const QStringList modes = { "centre", "spot", "average", "custom" };
		if (idx >= 0 && idx < modes.size())
			sendNow("metering:" + modes[idx]);
	}

	void onExposureChanged(int idx)
	{
		static const QStringList modes = { "normal", "sport" };
		if (idx >= 0 && idx < modes.size())
			sendNow("exposure:" + modes[idx]);
	}

	void onDenoiseChanged(int idx)
	{
		static const QStringList modes = { "auto", "off", "cdn_off", "cdn_fast", "cdn_hq" };
		if (idx >= 0 && idx < modes.size())
			sendNow("denoise:" + modes[idx]);
	}

	// ------------------------------------------------------------------
	// Per-camera state helpers
	// ------------------------------------------------------------------

	// State files: /tmp/rpicam-vid{N}.state (JSON) - shared with rpicam-ctrl-cli.

	static QString stateFilePath(int camIdx)
	{
		return QString("/tmp/rpicam-vid%1.state").arg(camIdx);
	}

	static void saveCameraStateToFile(int camIdx, const CameraState &c)
	{
		QJsonObject o;
		o["brightness"] = c.brightness;
		o["ev"] = c.ev;
		o["contrast"] = c.contrast;
		o["saturation"] = c.saturation;
		o["zoom"] = c.zoom;
		o["gain"] = c.gain;
		o["sharpness"] = c.sharpness;
		o["awbGainR"] = c.awbGainR;
		o["awbGainB"] = c.awbGainB;
		o["awbIdx"] = c.awbIdx;
		o["meteringIdx"] = c.meteringIdx;
		o["exposureIdx"] = c.exposureIdx;
		o["denoiseIdx"] = c.denoiseIdx;
		o["hdrIdx"] = c.hdrIdx;
		o["shutter"] = c.shutter;
		o["framerateIdx"] = c.framerateIdx; // fps directly (0 = auto)
		o["afIdx"] = c.afIdx;
		o["lens"] = c.lens;
		// Write atomically: QSaveFile writes to a temp file and renames on
		// commit, overwriting any existing target file.
		QSaveFile f(stateFilePath(camIdx));
		if (f.open(QIODevice::WriteOnly))
		{
			f.write(QJsonDocument(o).toJson());
			f.commit();
		}
	}

	static CameraState loadCameraStateFromFile(int camIdx)
	{
		QFile f(stateFilePath(camIdx));
		if (!f.open(QIODevice::ReadOnly))
			return CameraState {}; // file absent -> use defaults
		const QJsonObject o = QJsonDocument::fromJson(f.readAll()).object();
		if (o.isEmpty())
			return CameraState {};
		CameraState c;
		c.brightness = o.value("brightness").toInt(c.brightness);
		c.ev = o.value("ev").toInt(c.ev);
		c.contrast = o.value("contrast").toInt(c.contrast);
		c.saturation = o.value("saturation").toInt(c.saturation);
		c.zoom = o.value("zoom").toInt(c.zoom);
		c.gain = o.value("gain").toInt(c.gain);
		c.sharpness = o.value("sharpness").toInt(c.sharpness);
		c.awbGainR = o.value("awbGainR").toInt(c.awbGainR);
		c.awbGainB = o.value("awbGainB").toInt(c.awbGainB);
		c.awbIdx = o.value("awbIdx").toInt(c.awbIdx);
		c.meteringIdx = o.value("meteringIdx").toInt(c.meteringIdx);
		c.exposureIdx = o.value("exposureIdx").toInt(c.exposureIdx);
		c.denoiseIdx = o.value("denoiseIdx").toInt(c.denoiseIdx);
		c.hdrIdx = o.value("hdrIdx").toInt(c.hdrIdx);
		c.shutter = std::min(o.value("shutter").toInt(c.shutter), SHUTTER_STEPS);
		// framerateIdx is the fps directly (0=auto, 1..N fps).
		// Accept legacy "framerate" float key for backwards compat.
		if (o.contains("framerateIdx"))
			c.framerateIdx = std::clamp(o.value("framerateIdx").toInt(0), 0, 240);
		else
			c.framerateIdx = std::clamp(static_cast<int>(std::round(o.value("framerate").toDouble(0.0))), 0, 240);
		c.afIdx = o.value("afIdx").toInt(c.afIdx);
		c.lens = o.value("lens").toInt(c.lens);
		return c;
	}

	void loadAllStatesFromFiles()
	{
		for (int i = 0; i < 2; ++i)
			cameraStates_[i] = loadCameraStateFromFile(i);
		// Apply the active camera's state to the UI (widgets already built).
		applyUiState(cameraStates_[currentCameraIdx_]);
		// Hardware will be synced in onConnected() via sendCameraState().
	}

	void saveCameraState()
	{
		CameraState &s = cameraStates_[currentCameraIdx_];
		s.brightness = brightnessSlider_->value();
		s.ev = evSlider_->value();
		s.contrast = contrastSlider_->value();
		s.saturation = saturationSlider_->value();
		s.zoom = zoomSlider_->value();
		s.gain = gainSlider_->value();
		s.sharpness = sharpnessSlider_->value();
		s.awbGainR = awbGainRSlider_->value();
		s.awbGainB = awbGainBSlider_->value();
		s.awbIdx = awbCombo_->currentIndex();
		s.meteringIdx = meteringCombo_->currentIndex();
		s.exposureIdx = exposureCombo_->currentIndex();
		s.denoiseIdx = denoiseCombo_->currentIndex();
		s.hdrIdx = hdrCombo_->currentIndex();
		s.shutter = shutterSlider_->value();
		s.framerateIdx = framerateSlider_->value();
		s.afIdx = afCombo_->currentIndex();
		s.lens = lensSlider_->value();
	}

	void applyUiState(const CameraState &s)
	{
		QSignalBlocker b1(brightnessSlider_), b2(evSlider_), b3(contrastSlider_), b4(saturationSlider_);
		QSignalBlocker b5(zoomSlider_), b6(gainSlider_), b7(sharpnessSlider_);
		QSignalBlocker b8(awbGainRSlider_), b9(awbGainBSlider_), b10(awbCombo_);
		QSignalBlocker b11(meteringCombo_), b12(exposureCombo_), b13(denoiseCombo_), b14(hdrCombo_);
		QSignalBlocker b15(shutterSlider_), b16(framerateSlider_), b17(afCombo_), b18(lensSlider_);

		brightnessSlider_->setValue(s.brightness);
		evSlider_->setValue(s.ev);
		contrastSlider_->setValue(s.contrast);
		saturationSlider_->setValue(s.saturation);
		zoomSlider_->setValue(s.zoom);
		gainSlider_->setValue(s.gain);
		sharpnessSlider_->setValue(s.sharpness);
		awbGainRSlider_->setValue(s.awbGainR);
		awbGainBSlider_->setValue(s.awbGainB);
		awbCombo_->setCurrentIndex(s.awbIdx);
		meteringCombo_->setCurrentIndex(s.meteringIdx);
		exposureCombo_->setCurrentIndex(s.exposureIdx);
		denoiseCombo_->setCurrentIndex(s.denoiseIdx);
		hdrCombo_->setCurrentIndex(s.hdrIdx);
		shutterSlider_->setValue(s.shutter);
		framerateSlider_->setValue(s.framerateIdx);
		afCombo_->setCurrentIndex(s.afIdx);
		lensSlider_->setValue(s.lens);
		lensSlider_->setEnabled(s.afIdx == 2);

		brightnessVal_->setText(QString::asprintf("%+.2f", s.brightness / 100.0));
		evVal_->setText(QString::asprintf("%+.1f", s.ev / 10.0));
		contrastVal_->setText(QString::number(s.contrast / 100.0, 'f', 2));
		saturationVal_->setText(QString::number(s.saturation / 100.0, 'f', 2));
		zoomVal_->setText(QString::number(s.zoom / 10.0, 'f', 1) + "\xc3\x97");
		gainVal_->setText(QString::number(s.gain / 10.0, 'f', 1) + "x");
		sharpnessVal_->setText(QString::number(s.sharpness / 10.0, 'f', 1));

		static constexpr int MANUAL_AWB_IDX = 7;
		if (s.awbIdx == MANUAL_AWB_IDX)
		{
			awbGainRVal_->setText(QString::number(s.awbGainR / 100.0, 'f', 2));
			awbGainBVal_->setText(QString::number(s.awbGainB / 100.0, 'f', 2));
		}
		else
		{
			awbGainRVal_->setText("auto");
			awbGainBVal_->setText("auto");
		}

		const int maxUs = maxShutterUs(s.framerateIdx, camInfo_[currentCameraIdx_].maxFps);
		shutterVal_->setText(shutterLabel(s.shutter, maxUs));
		framerateVal_->setText(framerateLabel(s.framerateIdx));
		lensVal_->setText(QString::number(s.lens / 10.0, 'f', 1));
	}

	void sendCameraState(const CameraState &s)
	{
		static const QStringList awbModes = { "auto",	"incandescent", "tungsten", "fluorescent",
											  "indoor", "daylight",		"cloudy",	"manual" };
		static const QStringList meteringModes = { "centre", "spot", "average", "custom" };
		static const QStringList exposureModes = { "normal", "sport" };
		static const QStringList denoiseModes = { "auto", "off", "cdn_off", "cdn_fast", "cdn_hq" };
		static const QStringList hdrModes = { "off", "auto", "sensor", "single-exp" };

		double zoomFactor = s.zoom / 10.0;
		double w = 1.0 / zoomFactor;
		double h = 1.0 / zoomFactor;
		double x = (1.0 - w) / 2.0;
		double y = (1.0 - h) / 2.0;

		QString batch;
		batch += QString("brightness:%1\n").arg(s.brightness / 100.0, 0, 'f', 2);
		batch += QString("ev:%1\n").arg(s.ev / 10.0, 0, 'f', 1);
		batch += QString("contrast:%1\n").arg(s.contrast / 100.0, 0, 'f', 2);
		batch += QString("saturation:%1\n").arg(s.saturation / 100.0, 0, 'f', 2);
		// Only send roi/gain if non-default; otherwise we'd override --roi /
		// let AE manage gain freely (gain=10 means 1.0× = AE default).
		if (s.zoom != 10)
			batch +=
				QString("roi:%1,%2,%3,%4\n").arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(w, 0, 'f', 4).arg(h, 0, 'f', 4);
		if (s.gain != 10)
			batch += QString("gain:%1\n").arg(s.gain / 10.0, 0, 'f', 1);
		batch += QString("sharpness:%1\n").arg(s.sharpness / 10.0, 0, 'f', 1);

		static constexpr int MANUAL_AWB_IDX = 7;
		if (s.awbIdx == MANUAL_AWB_IDX)
			batch += QString("awbgains:%1,%2\n").arg(s.awbGainR / 100.0, 0, 'f', 2).arg(s.awbGainB / 100.0, 0, 'f', 2);
		else
			batch += "awb:" + awbModes[s.awbIdx] + "\n";

		batch += "metering:" + meteringModes[s.meteringIdx] + "\n";
		batch += "exposure:" + exposureModes[s.exposureIdx] + "\n";
		batch += "denoise:" + denoiseModes[s.denoiseIdx] + "\n";
		batch += "hdr:" + hdrModes[s.hdrIdx] + "\n";

		if (s.shutter != 0)
		{
			const int maxUs = maxShutterUs(s.framerateIdx, camInfo_[currentCameraIdx_].maxFps);
			batch += QString("shutter:%1\n").arg(shutterMicros(s.shutter, maxUs));
		}
		if (s.framerateIdx != 0)
			batch += QString("framerate:%1\n").arg(s.framerateIdx);

		sendNow(batch.trimmed());
	}

	void onCameraChanged(int idx)
	{
		static const QStringList paths = {
			"/tmp/rpicam-vid0.sock",
			"/tmp/rpicam-vid1.sock",
		};
		if (idx < 0 || idx >= paths.size())
			return;
		// Discard any debounced commands for the old camera so they cannot
		// leak into the new camera's socket after the switch.
		debounceTimer_->stop();
		pending_.clear();
		// Persist the current camera's UI values before switching.
		saveCameraState();
		currentCameraIdx_ = idx;
		currentSockPath_ = paths[idx];
		reconnTimer_->stop();
		sock_->abort();
		// Restore the target camera's UI immediately (no flickering wait).
		applyUiState(cameraStates_[currentCameraIdx_]);
		// Show / hide AF controls based on the new camera's capabilities.
		applyCameraCapabilities(currentCameraIdx_);
		// Immediately attempt connection to the new socket; the reconnect
		// timer serves only as a fallback if this first attempt fails.
		tryConnect();
	}

	void onReset()
	{
		// Cancel any pending debounced command so it cannot overwrite the reset.
		debounceTimer_->stop();
		pending_.clear();

		QSignalBlocker b1(brightnessSlider_), b2(evSlider_), b3(contrastSlider_), b4(saturationSlider_);
		QSignalBlocker b5(zoomSlider_), b6(awbCombo_), b7(gainSlider_), b8(sharpnessSlider_);
		QSignalBlocker b9(meteringCombo_), b10(exposureCombo_), b11(denoiseCombo_), b12(hdrCombo_);
		QSignalBlocker b13(awbGainRSlider_), b14(awbGainBSlider_);
		QSignalBlocker b15(shutterSlider_), b16(framerateSlider_), b17(afCombo_), b18(lensSlider_);

		brightnessSlider_->setValue(0);
		evSlider_->setValue(0);
		contrastSlider_->setValue(100);
		saturationSlider_->setValue(100);
		zoomSlider_->setValue(10);
		awbCombo_->setCurrentIndex(0);
		gainSlider_->setValue(10);
		sharpnessSlider_->setValue(10);
		meteringCombo_->setCurrentIndex(0);
		exposureCombo_->setCurrentIndex(0);
		denoiseCombo_->setCurrentIndex(0);
		hdrCombo_->setCurrentIndex(0);
		awbGainRSlider_->setValue(150);
		awbGainBSlider_->setValue(120);
		shutterSlider_->setValue(0);
		framerateSlider_->setValue(0); // index 0 = auto
		afCombo_->setCurrentIndex(0);
		lensSlider_->setValue(0);
		lensSlider_->setEnabled(false);

		brightnessVal_->setText("+0.00");
		evVal_->setText("+0.0");
		contrastVal_->setText("1.00");
		saturationVal_->setText("1.00");
		zoomVal_->setText("1.0\xc3\x97");
		gainVal_->setText("1.0x");
		sharpnessVal_->setText("1.0");
		awbGainRVal_->setText("auto");
		awbGainBVal_->setText("auto");
		shutterVal_->setText("auto");
		framerateVal_->setText("auto"); // framerateIdx reset to 0
		lensVal_->setText("0.0");

		sendNow("brightness:0.00\n"
				"ev:0.0\n"
				"contrast:1.00\n"
				"saturation:1.00\n"
				"awb:auto\n"
				"roi:0.0,0.0,1.0,1.0\n"
				"gain:0\n"
				"sharpness:1.0\n"
				"metering:centre\n"
				"exposure:normal\n"
				"denoise:auto\n"
				"hdr:off\n"
				"shutter:0\n"
				"framerate:0"); // 0 restores AE-managed framerate

		// Keep the stored state in sync with the reset UI.
		cameraStates_[currentCameraIdx_] = CameraState {}; // framerateIdx=0 = auto
	}

private:
	// ------------------------------------------------------------------
	// AF visibility helper
	// ------------------------------------------------------------------

	// Returns true if the camera at camIdx supports AF, or if no probe data
	// is available (show controls rather than hide them in that case).
	bool cameraHasAf(int camIdx) const
	{
		const bool anyFound = camInfo_[0].present || camInfo_[1].present;
		if (!anyFound)
			return true; // no probe data → don't hide anything
		if (camIdx < 0 || camIdx >= 2)
			return false;
		return camInfo_[camIdx].hasAf;
	}

	void applyCameraCapabilities(int camIdx)
	{
		// Set the framerate slider range based on the probe data.
		// Slider value = fps directly (0=auto, 1..maxFps).
		const bool anyFound = camInfo_[0].present || camInfo_[1].present;
		if (anyFound && camIdx >= 0 && camIdx < 2 && camInfo_[camIdx].maxFps > 0)
		{
			const int maxFps = camInfo_[camIdx].maxFps;
			QSignalBlocker b(framerateSlider_);
			framerateSlider_->setMaximum(maxFps);
			if (framerateSlider_->value() > maxFps)
			{
				framerateSlider_->setValue(0);
				framerateVal_->setText("auto");
			}
		}
	}

	// ------------------------------------------------------------------
	// UI construction
	// ------------------------------------------------------------------

	void buildUi()
	{
		auto *central = new QWidget(this);
		setCentralWidget(central);

		auto *root = new QVBoxLayout(central);
		root->setSpacing(2);
		root->setContentsMargins(8, 8, 8, 4);

		// Camera selector — only list cameras that are detected; if the probe
		// found nothing at all (command not available etc.) fall back to "0"/"1".
		auto *camBar = new QHBoxLayout;
		camBar->addWidget(new QLabel("Camera:"));
		cameraCombo_ = new QComboBox;

		const bool anyFound = camInfo_[0].present || camInfo_[1].present;
		for (int i = 0; i < 2; ++i)
		{
			QString label = QString("Cam %1").arg(i);
			if (camInfo_[i].present && !camInfo_[i].model.isEmpty())
				label += "  ·  " + camInfo_[i].model;
			else if (!camInfo_[i].present && anyFound)
				label += "  (—)";
			cameraCombo_->addItem(label);

			// Grey out / disable unavailable entries.
			if (!camInfo_[i].present && anyFound)
			{
				auto *m = qobject_cast<QStandardItemModel *>(cameraCombo_->model());
				if (m)
					m->item(i)->setEnabled(false);
			}
		}

		cameraCombo_->setMaximumWidth(200);
		// Initialize to the pre-selected camera (set before buildUi() runs).
		// Must happen before connecting the signal so no spurious onCameraChanged fires.
		cameraCombo_->setCurrentIndex(currentCameraIdx_);
		connect(cameraCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				&ControlWindow::onCameraChanged);
		camBar->addWidget(cameraCombo_);
		camBar->addStretch();
		root->addLayout(camBar);

		root->addWidget(buildExposureGroup());
		root->addWidget(buildShutterGroup());
		root->addWidget(buildImageGroup());
		root->addWidget(buildCameraGroup());
		root->addWidget(buildAwbGroup());
		root->addWidget(buildZoomGroup());
		focusGroup_ = buildFocusGroup(); // built but not added to layout — AF hidden for now
		root->addWidget(buildAdvancedGroup());

		applyCameraCapabilities(currentCameraIdx_);

		auto *resetBtn = new QPushButton("Reset all to defaults");
		resetBtn->setFixedHeight(34);
		connect(resetBtn, &QPushButton::clicked, this, &ControlWindow::onReset);
		root->addWidget(resetBtn);

		statusBar()->showMessage("  \u25cb  Connecting\u2026");
		statusBar()->setStyleSheet("QStatusBar { color: gray; }");
	}

	QWidget *buildExposureGroup()
	{
		auto *group = new QFrame;
		group->setFrameShape(QFrame::StyledPanel);
		auto *grid = new QGridLayout(group);
		grid->setSpacing(3);
		grid->setContentsMargins(6, 4, 6, 4);
		grid->setColumnStretch(1, 1);

		brightnessSlider_ = addRow(grid, 0, "Brightness", -100, 100, 0, &brightnessVal_);
		brightnessVal_->setText("+0.00");
		connect(brightnessSlider_, &QSlider::valueChanged, this, &ControlWindow::onBrightnessChanged);

		evSlider_ = addRow(grid, 1, "EV", -30, 30, 0, &evVal_);
		evVal_->setText("+0.0");
		connect(evSlider_, &QSlider::valueChanged, this, &ControlWindow::onEvChanged);

		return group;
	}

	QWidget *buildImageGroup()
	{
		auto *group = new QFrame;
		group->setFrameShape(QFrame::StyledPanel);
		auto *grid = new QGridLayout(group);
		grid->setSpacing(3);
		grid->setContentsMargins(6, 4, 6, 4);
		grid->setColumnStretch(1, 1);

		contrastSlider_ = addRow(grid, 0, "Contrast", 0, 300, 100, &contrastVal_);
		contrastVal_->setText("1.00");
		connect(contrastSlider_, &QSlider::valueChanged, this, &ControlWindow::onContrastChanged);

		saturationSlider_ = addRow(grid, 1, "Saturation", 0, 300, 100, &saturationVal_);
		saturationVal_->setText("1.00");
		connect(saturationSlider_, &QSlider::valueChanged, this, &ControlWindow::onSaturationChanged);

		return group;
	}

	QWidget *buildCameraGroup()
	{
		auto *group = new QFrame;
		group->setFrameShape(QFrame::StyledPanel);
		auto *grid = new QGridLayout(group);
		grid->setSpacing(3);
		grid->setContentsMargins(6, 4, 6, 4);
		grid->setColumnStretch(1, 1);

		gainSlider_ = addRow(grid, 0, "Gain", 10, 160, 10, &gainVal_);
		gainVal_->setText("1.0x");
		connect(gainSlider_, &QSlider::valueChanged, this, &ControlWindow::onGainChanged);

		sharpnessSlider_ = addRow(grid, 1, "Sharpness", 0, 160, 10, &sharpnessVal_);
		sharpnessVal_->setText("1.0");
		connect(sharpnessSlider_, &QSlider::valueChanged, this, &ControlWindow::onSharpnessChanged);

		return group;
	}

	QWidget *buildAwbGroup()
	{
		auto *group = new QFrame;
		group->setFrameShape(QFrame::StyledPanel);
		auto *grid = new QGridLayout(group);
		grid->setSpacing(3);
		grid->setContentsMargins(6, 4, 6, 4);
		grid->setColumnStretch(1, 1);

		// AWB mode
		grid->addWidget(new QLabel("AWB mode:"), 0, 0);
		awbCombo_ = new QComboBox;
		awbCombo_->addItems(
			{ "auto", "incandescent", "tungsten", "fluorescent", "indoor", "daylight", "cloudy", "manual" });
		connect(awbCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ControlWindow::onAwbChanged);
		awbCombo_->setMaximumWidth(120);
		grid->addWidget(awbCombo_, 0, 1);

		// AWB gains — always enabled; moving them auto-switches AWB combo to manual
		awbGainRSlider_ = addRow(grid, 1, "Gain R", 10, 400, 150, &awbGainRVal_);
		awbGainRVal_->setText("auto");
		connect(awbGainRSlider_, &QSlider::valueChanged, this, &ControlWindow::onAwbGainRChanged);

		awbGainBSlider_ = addRow(grid, 2, "Gain B", 10, 400, 120, &awbGainBVal_);
		awbGainBVal_->setText("auto");
		connect(awbGainBSlider_, &QSlider::valueChanged, this, &ControlWindow::onAwbGainBChanged);

		return group;
	}

	QWidget *buildAdvancedGroup()
	{
		auto *group = new QFrame;
		group->setFrameShape(QFrame::StyledPanel);
		auto *grid = new QGridLayout(group);
		grid->setSpacing(3);
		grid->setContentsMargins(6, 4, 6, 4);

		auto *meteringLbl = new QLabel("Metering:");
		meteringLbl->setFixedWidth(90);
		grid->addWidget(meteringLbl, 0, 0);
		meteringCombo_ = new QComboBox;
		meteringCombo_->addItems({ "centre", "spot", "average", "custom" });
		connect(meteringCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				&ControlWindow::onMeteringChanged);
		grid->addWidget(meteringCombo_, 0, 1);

		grid->addWidget(new QLabel("Exposure:"), 0, 2);
		exposureCombo_ = new QComboBox;
		exposureCombo_->addItems({ "normal", "sport" });
		connect(exposureCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				&ControlWindow::onExposureChanged);
		grid->addWidget(exposureCombo_, 0, 3);

		auto *denoiseLbl = new QLabel("Denoise:");
		denoiseLbl->setFixedWidth(90);
		grid->addWidget(denoiseLbl, 1, 0);
		denoiseCombo_ = new QComboBox;
		denoiseCombo_->addItems({ "auto", "off", "cdn_off", "cdn_fast", "cdn_hq" });
		connect(denoiseCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				&ControlWindow::onDenoiseChanged);
		grid->addWidget(denoiseCombo_, 1, 1);

		grid->addWidget(new QLabel("HDR:"), 1, 2);
		hdrCombo_ = new QComboBox;
		hdrCombo_->addItems({ "off", "auto", "sensor", "single-exp" });
		connect(hdrCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ControlWindow::onHdrChanged);
		grid->addWidget(hdrCombo_, 1, 3);

		return group;
	}

	QWidget *buildShutterGroup()
	{
		auto *group = new QFrame;
		group->setFrameShape(QFrame::StyledPanel);
		auto *grid = new QGridLayout(group);
		grid->setSpacing(3);
		grid->setContentsMargins(6, 4, 6, 4);
		grid->setColumnStretch(1, 1);

		shutterSlider_ = addRow(grid, 0, "Shutter", 0, SHUTTER_STEPS, 0, &shutterVal_);
		shutterVal_->setText("auto");
		connect(shutterSlider_, &QSlider::valueChanged, this, &ControlWindow::onShutterChanged);

		// Framerate slider range: 0=auto, 1..FPS_STEP_COUNT; max clamped by caps/probe later.
		framerateSlider_ = addRow(grid, 1, "Framerate", 0, FPS_SLIDER_MAX_DEFAULT, 0, &framerateVal_);
		framerateVal_->setText("auto");
		connect(framerateSlider_, &QSlider::valueChanged, this, &ControlWindow::onFramerateChanged);

		return group;
	}

	QWidget *buildFocusGroup()
	{
		auto *group = new QFrame;
		group->setFrameShape(QFrame::StyledPanel);
		auto *grid = new QGridLayout(group);
		grid->setSpacing(3);
		grid->setContentsMargins(6, 4, 6, 4);
		grid->setColumnStretch(1, 1);

		auto *afLbl = new QLabel("AF mode:");
		afLbl->setFixedWidth(90);
		grid->addWidget(afLbl, 0, 0);
		afCombo_ = new QComboBox;
		afCombo_->addItems({ "auto", "continuous", "manual" });
		afCombo_->setMaximumWidth(120);
		connect(afCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ControlWindow::onAfChanged);
		grid->addWidget(afCombo_, 0, 1);

		auto *trigBtn = new QPushButton("Trigger AF");
		trigBtn->setMaximumWidth(100);
		connect(trigBtn, &QPushButton::clicked, this, [this] { sendNow("af:trigger"); });
		grid->addWidget(trigBtn, 0, 2);

		lensSlider_ = addRow(grid, 1, "Lens pos.", 0, 100, 0, &lensVal_);
		lensVal_->setText("0.0");
		lensSlider_->setEnabled(false);
		connect(lensSlider_, &QSlider::valueChanged, this, &ControlWindow::onLensChanged);

		return group;
	}

	QWidget *buildZoomGroup()
	{
		auto *group = new QFrame;
		group->setFrameShape(QFrame::StyledPanel);
		auto *grid = new QGridLayout(group);
		grid->setSpacing(3);
		grid->setContentsMargins(6, 4, 6, 4);
		grid->setColumnStretch(1, 1);

		zoomSlider_ = addRow(grid, 0, "Zoom", 10, 100, 10, &zoomVal_);
		zoomVal_->setText("1.0\xc3\x97");
		connect(zoomSlider_, &QSlider::valueChanged, this, &ControlWindow::onZoomChanged);

		return group;
	}

	// Add one labelled slider row to a QGridLayout.
	// Columns: 0=label  1=slider  2=value
	QSlider *addRow(QGridLayout *g, int row, const QString &label, int minV, int maxV, int initV, QLabel **valueOut)
	{
		auto *lbl = new QLabel(label + ":");
		lbl->setFixedWidth(90);
		g->addWidget(lbl, row, 0);

		auto *slider = new QSlider(Qt::Horizontal);
		slider->setRange(minV, maxV);
		slider->setValue(initV);
		slider->setTickPosition(QSlider::TicksBelow);
		slider->setTickInterval((maxV - minV) / 10);
		g->addWidget(slider, row, 1);

		auto *vl = new QLabel;
		vl->setFixedWidth(62);
		vl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
		g->addWidget(vl, row, 2);
		*valueOut = vl;

		return slider;
	}

	// ------------------------------------------------------------------
	// State
	// ------------------------------------------------------------------

	QLocalSocket *sock_;
	QTimer *reconnTimer_;
	QTimer *debounceTimer_;
	QMap<QString, QString> pending_;
	QString currentSockPath_ = "/tmp/rpicam-vid0.sock";
	int currentCameraIdx_ = 0;
	CameraState cameraStates_[2];
	std::array<CameraInfo, 2> camInfo_;
	QComboBox *cameraCombo_;

	QSlider *brightnessSlider_;
	QSlider *evSlider_;
	QSlider *contrastSlider_;
	QSlider *saturationSlider_;
	QSlider *zoomSlider_;
	QSlider *gainSlider_;
	QSlider *sharpnessSlider_;
	QSlider *awbGainRSlider_;
	QSlider *awbGainBSlider_;
	QLabel *brightnessVal_;
	QLabel *evVal_;
	QLabel *contrastVal_;
	QLabel *saturationVal_;
	QLabel *zoomVal_;
	QLabel *gainVal_;
	QLabel *sharpnessVal_;
	QLabel *awbGainRVal_;
	QLabel *awbGainBVal_;
	QComboBox *awbCombo_;
	QComboBox *meteringCombo_;
	QComboBox *exposureCombo_;
	QComboBox *denoiseCombo_;
	QComboBox *hdrCombo_;
	QSlider *shutterSlider_;
	QSlider *framerateSlider_;
	QSlider *lensSlider_;
	QLabel *shutterVal_;
	QLabel *framerateVal_;
	QLabel *lensVal_;
	QComboBox *afCombo_;
	QWidget *focusGroup_ = nullptr;
};

#include "main.moc"

// ---------------------------------------------------------------------------

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	app.setApplicationName("rpicam-ctrl");
	app.setApplicationDisplayName("rpicam-ctrl");
	app.setWindowIcon(QIcon::fromTheme("rpicam-ctrl", QIcon(":/rpicam-ctrl.svg")));

	ControlWindow win;
	win.show();
	return app.exec();
}
