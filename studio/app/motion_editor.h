// MotionEditor: editor keyframe (format sama dengan tools/lam.py JSON dan LAM1 firmware).
#pragma once
#include "motion_doc.h"

#include <QElapsedTimer>
#include <QObject>
#include <QTimer>
#include <QUrl>
#include <QVariantList>

class RobotLink;
class TwinModel;

class MotionEditor : public QObject {
    Q_OBJECT
    Q_PROPERTY(QString name READ name WRITE setName NOTIFY docChanged)
    Q_PROPERTY(bool returnHome READ returnHome WRITE setReturnHome NOTIFY docChanged)
    Q_PROPERTY(QVariantList frames READ frames NOTIFY docChanged)
    Q_PROPERTY(int totalMs READ totalMs NOTIFY docChanged)
    Q_PROPERTY(QString problem READ problem NOTIFY docChanged)
    Q_PROPERTY(bool previewing READ previewing NOTIFY previewChanged)
    Q_PROPERTY(int previewMs READ previewMs NOTIFY previewChanged)
    Q_PROPERTY(bool recording READ recording NOTIFY recordChanged)
    Q_PROPERTY(int recordSamples READ recordSamples NOTIFY recordChanged)
    Q_PROPERTY(QString message READ message NOTIFY messageChanged)

public:
    MotionEditor(RobotLink* link, TwinModel* twin, QObject* parent = nullptr);

    QString name() const { return QString::fromStdString(doc_.name); }
    void setName(const QString& n);
    bool returnHome() const { return doc_.returnHome; }
    void setReturnHome(bool v);
    QVariantList frames() const;
    int totalMs() const { return (int)doc_.totalMs(); }
    QString problem() const;
    bool previewing() const { return previewTimer_.isActive(); }
    int previewMs() const { return previewMs_; }
    bool recording() const { return recording_; }
    int recordSamples() const { return (int)rec_.size(); }
    QString message() const { return message_; }

    Q_INVOKABLE void clear();
    Q_INVOKABLE void addFrame(const QVariantList& src22, int ms, int insertAt = -1);
    Q_INVOKABLE void setFramePose(int i, const QVariantList& src22);
    Q_INVOKABLE QVariantList framePose(int i) const;
    Q_INVOKABLE void setFrameMs(int i, int ms);
    Q_INVOKABLE void setFrameInterp(int i, int interp);
    Q_INVOKABLE void removeFrame(int i);
    Q_INVOKABLE void moveFrame(int i, int delta);
    Q_INVOKABLE void duplicateFrame(int i);
    Q_INVOKABLE void showFrame(int i);                   // tampilkan keyframe di twin

    Q_INVOKABLE void preview(bool on);
    Q_INVOKABLE void seek(int ms);

    Q_INVOKABLE bool loadJson(const QUrl& url);
    Q_INVOKABLE bool saveJson(const QUrl& url);
    Q_INVOKABLE bool importLam(const QUrl& url);
    Q_INVOKABLE bool exportLam(const QUrl& url);
    Q_INVOKABLE bool loadExample();
    Q_INVOKABLE bool upload(int slot, bool saveToSd);

    // rekaman (kamera / robot): sampel diambil pemanggil, disederhanakan jadi keyframe
    Q_INVOKABLE void startRecording();
    Q_INVOKABLE void addSample(const QVariantList& src22);
    Q_INVOKABLE int  stopRecording(int tolCounts);       // jumlah keyframe yang ditambahkan

signals:
    void docChanged();
    void previewChanged();
    void recordChanged();
    void messageChanged();

private:
    bool fromJson(const QByteArray& j, QString* err);
    QByteArray toJson() const;
    void say(const QString& m) { message_ = m; emit messageChanged(); }
    void showAt(int ms);

    RobotLink* link_;
    TwinModel* twin_;
    la::studio::MotionDoc doc_;
    QTimer previewTimer_;
    QElapsedTimer previewClock_;
    int previewMs_ = 0, previewOffset_ = 0;
    bool recording_ = false;
    QElapsedTimer recClock_;
    std::vector<la::studio::PoseSample> rec_;
    QString message_;
};
