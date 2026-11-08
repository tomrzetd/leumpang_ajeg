#include "pose_capture.h"
#include "motion_editor.h"
#include "robot_link.h"
#include "twin_model.h"

#include <QCameraDevice>
#include <QCoreApplication>
#include <QImage>
#include <QMediaDevices>
#include <QThreadPool>
#include <QVideoFrame>
#include <cstring>

#ifdef LA_HAS_ORT
#include "pose_onnx.h"
#endif

using namespace la;
using namespace la::studio;

PoseCapture::PoseCapture(RobotLink* link, TwinModel* twin, MotionEditor* motion, QObject* parent)
    : QObject(parent), link_(link), twin_(twin), motion_(motion) {
    rt_.opts.mirror = true;
    connect(new QMediaDevices(this), &QMediaDevices::videoInputsChanged, this, &PoseCapture::devicesChanged);
#ifdef LA_HAS_ORT
    net_ = std::make_shared<PoseOnnx>();
    say(tr("pilih model ONNX MoveNet SinglePose untuk deteksi pose"));
#else
    say(tr("studio dibangun tanpa ONNX Runtime: hanya pratinjau kamera"));
#endif
    frameClock_.start();
    streamClock_.start();
}

PoseCapture::~PoseCapture() {
    stop();
    for (int i = 0; i < 100 && busy_->load(); ++i) QThreadPool::globalInstance()->waitForDone(20);
}

bool PoseCapture::available() const {
#ifdef LA_HAS_ORT
    return true;
#else
    return false;
#endif
}
bool PoseCapture::modelLoaded() const {
#ifdef LA_HAS_ORT
    return net_ && net_->loaded();
#else
    return false;
#endif
}

QStringList PoseCapture::devices() const {
    QStringList out;
    for (const QCameraDevice& d : QMediaDevices::videoInputs()) out << d.description();
    return out;
}

void PoseCapture::setMirror(bool v) { if (v != rt_.opts.mirror) { rt_.opts.mirror = v; filter_.reset(); emit optionsChanged(); } }
void PoseCapture::setDriveTwin(bool v) {
    if (v == driveTwin_) return;
    driveTwin_ = v;
    if (!v) twin_->clearPreview(QStringLiteral("kamera"));
    emit optionsChanged();
}
void PoseCapture::setStreamToRobot(bool v) {
    if (v == stream_) return;
    stream_ = v;
    if (!v && link_->connected()) {                             // selesai: robot kembali HOME
        QVariantList src = twin_->homeSrc();
        link_->poseTarget(src, 0, 0);
    }
    emit optionsChanged();
}

void PoseCapture::setVideoSink(QObject* s) {
    QVideoSink* vs = qobject_cast<QVideoSink*>(s);
    if (vs == sink_) return;
    if (sink_) disconnect(sink_, nullptr, this, nullptr);
    sink_ = vs;
    session_.setVideoSink(vs);
    if (vs) connect(vs, &QVideoSink::videoFrameChanged, this, &PoseCapture::onFrame);
    emit videoSinkChanged();
}

void PoseCapture::start(int idx) {
    const QList<QCameraDevice> devs = QMediaDevices::videoInputs();
    if (devs.isEmpty()) { say(tr("tidak ada kamera")); return; }
    const QCameraDevice dev = devs.value(qBound(0, idx, (int)devs.size() - 1));
    camera_.reset(new QCamera(dev));
    connect(camera_.get(), &QCamera::activeChanged, this, &PoseCapture::runningChanged);
    connect(camera_.get(), &QCamera::errorOccurred, this, [this](QCamera::Error, const QString& e) { say(tr("kamera: %1").arg(e)); });
    session_.setCamera(camera_.get());
    camera_->start();
    filter_.reset();
    say(tr("kamera: %1").arg(dev.description()));
}

void PoseCapture::stop() {
    if (camera_) { camera_->stop(); session_.setCamera(nullptr); camera_.reset(); }
    if (twin_) twin_->clearPreview(QStringLiteral("kamera"));
    emit runningChanged();
}

bool PoseCapture::loadModel(const QUrl& url) {
#ifdef LA_HAS_ORT
    const QString path = url.isLocalFile() ? url.toLocalFile() : url.toString();
    std::string err;
    auto n = std::make_shared<PoseOnnx>();
    if (!n->load(path.toStdString(), &err)) { say(tr("model gagal dimuat: %1").arg(QString::fromStdString(err))); return false; }
    for (int i = 0; i < 100 && busy_->load(); ++i) QThreadPool::globalInstance()->waitForDone(20);
    net_ = n;
    modelInfo_ = QString::fromStdString(n->describe());
    emit modelChanged();
    say(tr("model siap: %1").arg(modelInfo_));
    return true;
#else
    Q_UNUSED(url);
    say(tr("studio dibangun tanpa ONNX Runtime"));
    return false;
#endif
}

void PoseCapture::onFrame(const QVideoFrame& frame) {
    const double dt = frameClock_.restart() / 1000.0;
    if (dt > 0) fps_ = fps_ * 0.9 + 0.1 / dt;
#ifdef LA_HAS_ORT
    if (!net_ || !net_->loaded() || busy_->load() || !frame.isValid()) return;
    QImage img = frame.toImage();
    if (img.isNull()) return;
    if (img.width() > 640) img = img.scaledToWidth(640, Qt::FastTransformation);
    img = img.convertToFormat(QImage::Format_RGB888);
    busy_->store(true);
    std::shared_ptr<PoseOnnx> net = net_;
    std::shared_ptr<std::atomic<bool>> busy = busy_;
    QPointer<PoseCapture> self(this);
    QThreadPool::globalInstance()->start([self, net, busy, img] {
        QElapsedTimer t; t.start();
        Keypoint kp[KP_COUNT];
        std::string err;
        const bool ok = net->run(img.constBits(), img.width(), img.height(), (int)img.bytesPerLine(), kp, &err);
        const double ms = (double)t.nsecsElapsed() / 1e6;
        std::vector<Keypoint> v(kp, kp + KP_COUNT);
        const int w = img.width(), h = img.height();
        busy->store(false);
        QMetaObject::invokeMethod(QCoreApplication::instance(), [self, v, w, h, ms, ok, err] {
            if (!self) return;                                   // dievaluasi di thread utama
            if (!ok) { self->say(QString::fromStdString(err)); return; }
            self->onResult(v.data(), w, h, ms);
        }, Qt::QueuedConnection);
    });
#else
    Q_UNUSED(frame);
#endif
}

void PoseCapture::onResult(const Keypoint* kp, int w, int h, double ms) {
    inferMs_ = inferMs_ * 0.8 + ms * 0.2;
    memcpy(last_, kp, sizeof(last_));
    kpOut_.clear();
    for (int i = 0; i < KP_COUNT; ++i) {
        QVariantMap m;
        m[QStringLiteral("x")] = kp[i].x / (float)w; m[QStringLiteral("y")] = kp[i].y / (float)h; m[QStringLiteral("score")] = kp[i].score;
        kpOut_ << m;
    }
    JointSrc base; homeSrc(base);
    RetargetResult r = rt_.solve(kp, base);
    filter_.apply(r.src, r.mask, 1.0f / 15.0f);
    QVariantList src;
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) src << (int)r.src[b][s];
    if (driveTwin_ && r.mask) twin_->setPreview(src, QStringLiteral("kamera"));
    if (motion_->recording() && r.mask) motion_->addSample(src);
    if (stream_ && r.mask && link_->connected() && streamClock_.elapsed() >= 50) {   // <= 20 Hz
        streamClock_.restart();
        link_->poseTarget(src, r.mask, 400);
    }
    emit poseChanged();
}

bool PoseCapture::calibrateTPose() {
    const bool ok = rt_.calibrateTPose(last_);
    say(ok ? tr("kalibrasi T-pose: lengan atas %1, lengan bawah %2 x lebar bahu").arg(rt_.opts.upperRatio, 0, 'f', 2).arg(rt_.opts.foreRatio, 0, 'f', 2)
           : tr("kalibrasi gagal: rentangkan kedua lengan ke samping, seluruh tubuh atas terlihat"));
    return ok;
}
