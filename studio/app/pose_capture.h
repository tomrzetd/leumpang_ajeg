// PoseCapture: kamera PC -> MoveNet (ONNX Runtime, thread pekerja) -> retarget -> twin / robot / rekaman.
#pragma once
#include "retarget.h"

#include <QCamera>
#include <QElapsedTimer>
#include <QMediaCaptureSession>
#include <QObject>
#include <QPointer>
#include <QStringList>
#include <QUrl>
#include <QVariantList>
#include <QVideoSink>
#include <atomic>
#include <memory>

class RobotLink;
class TwinModel;
class MotionEditor;
namespace la::studio { class PoseOnnx; }

class PoseCapture : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool available READ available CONSTANT)              // dibangun dengan ONNX Runtime?
    Q_PROPERTY(QStringList devices READ devices NOTIFY devicesChanged)
    Q_PROPERTY(bool running READ running NOTIFY runningChanged)
    Q_PROPERTY(bool modelLoaded READ modelLoaded NOTIFY modelChanged)
    Q_PROPERTY(QString modelInfo READ modelInfo NOTIFY modelChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(QVariantList keypoints READ keypoints NOTIFY poseChanged)    // [{x,y,score}] ternormalisasi 0..1
    Q_PROPERTY(double fps READ fps NOTIFY poseChanged)
    Q_PROPERTY(double inferMs READ inferMs NOTIFY poseChanged)
    Q_PROPERTY(bool mirror READ mirror WRITE setMirror NOTIFY optionsChanged)
    Q_PROPERTY(bool driveTwin READ driveTwin WRITE setDriveTwin NOTIFY optionsChanged)
    Q_PROPERTY(bool streamToRobot READ streamToRobot WRITE setStreamToRobot NOTIFY optionsChanged)
    Q_PROPERTY(QObject* videoSink READ videoSink WRITE setVideoSink NOTIFY videoSinkChanged)

public:
    PoseCapture(RobotLink* link, TwinModel* twin, MotionEditor* motion, QObject* parent = nullptr);
    ~PoseCapture() override;

    bool available() const;
    QStringList devices() const;
    bool running() const { return camera_ && camera_->isActive(); }
    bool modelLoaded() const;
    QString modelInfo() const { return modelInfo_; }
    QString status() const { return status_; }
    QVariantList keypoints() const { return kpOut_; }
    double fps() const { return fps_; }
    double inferMs() const { return inferMs_; }
    bool mirror() const { return rt_.opts.mirror; }
    void setMirror(bool v);
    bool driveTwin() const { return driveTwin_; }
    void setDriveTwin(bool v);
    bool streamToRobot() const { return stream_; }
    void setStreamToRobot(bool v);
    QObject* videoSink() const { return sink_; }
    void setVideoSink(QObject* s);

    Q_INVOKABLE void start(int deviceIndex);
    Q_INVOKABLE void stop();
    Q_INVOKABLE bool loadModel(const QUrl& url);
    Q_INVOKABLE bool calibrateTPose();
    Q_INVOKABLE void refreshDevices() { emit devicesChanged(); }

signals:
    void devicesChanged();
    void runningChanged();
    void modelChanged();
    void statusChanged();
    void poseChanged();
    void optionsChanged();
    void videoSinkChanged();

private:
    void onFrame(const QVideoFrame& f);
    void onResult(const la::studio::Keypoint* kp, int w, int h, double ms);
    void say(const QString& s) { status_ = s; emit statusChanged(); }

    RobotLink* link_;
    TwinModel* twin_;
    MotionEditor* motion_;
    std::unique_ptr<QCamera> camera_;
    QMediaCaptureSession session_;
    QPointer<QVideoSink> sink_;
    std::shared_ptr<la::studio::PoseOnnx> net_;
    std::shared_ptr<std::atomic<bool>> busy_ = std::make_shared<std::atomic<bool>>(false);   // dibagi dgn pekerja
    la::studio::Retargeter rt_;
    la::studio::JointFilter filter_;
    la::studio::Keypoint last_[la::studio::KP_COUNT]{};
    QVariantList kpOut_;
    QString status_, modelInfo_;
    QElapsedTimer frameClock_, streamClock_;
    double fps_ = 0, inferMs_ = 0;
    bool driveTwin_ = true, stream_ = false;
};
