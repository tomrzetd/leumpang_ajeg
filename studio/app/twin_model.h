// TwinModel: pose digital twin untuk QML (FK dari studio/lib, frame robot -> frame Qt Quick 3D).
//  Robot: X depan, Y kiri, Z atas (mm).  Qt Quick 3D: x kanan-layar, y atas, z ke penonton.
//  Pemetaan (permutasi siklik, tetap tangan-kanan): qt = (Y, Z, X).
#pragma once
#include "kinematics.h"

#include <QObject>
#include <QQuaternion>
#include <QVariantList>
#include <QVector3D>

class TwinModel : public QObject {
    Q_OBJECT
    Q_PROPERTY(QVariantList meshes READ meshes CONSTANT)
    Q_PROPERTY(QVariantList jointInfo READ jointInfo CONSTANT)
    Q_PROPERTY(QVariantList homeSrc READ homeSrc CONSTANT)
    Q_PROPERTY(int phoneLink READ phoneLink CONSTANT)
    Q_PROPERTY(QVariantList positions READ positions NOTIFY poseChanged)
    Q_PROPERTY(QVariantList rotations READ rotations NOTIFY poseChanged)
    Q_PROPERTY(QVariantList ghostPositions READ ghostPositions NOTIFY ghostChanged)
    Q_PROPERTY(QVariantList ghostRotations READ ghostRotations NOTIFY ghostChanged)
    Q_PROPERTY(QVariantList currentSrc READ currentSrc NOTIFY poseChanged)
    Q_PROPERTY(QString previewOwner READ previewOwner NOTIFY sourceChanged)
    Q_PROPERTY(bool ghostVisible READ ghostVisible WRITE setGhostVisible NOTIFY ghostChanged)
    Q_PROPERTY(bool showPhone READ showPhone WRITE setShowPhone NOTIFY optionsChanged)
    Q_PROPERTY(bool imuTilt READ imuTilt WRITE setImuTilt NOTIFY optionsChanged)
    Q_PROPERTY(int highlightLink READ highlightLink WRITE setHighlightLink NOTIFY optionsChanged)

public:
    explicit TwinModel(QObject* parent = nullptr);

    QVariantList meshes() const { return meshes_; }
    QVariantList jointInfo() const { return jointInfo_; }
    QVariantList homeSrc() const;
    int phoneLink() const { return phoneLink_; }
    QVariantList positions() const { return pos_; }
    QVariantList rotations() const { return rot_; }
    QVariantList ghostPositions() const { return gpos_; }
    QVariantList ghostRotations() const { return grot_; }
    QVariantList currentSrc() const;
    QString previewOwner() const { return owner_; }
    bool ghostVisible() const { return ghostVisible_; }
    bool showPhone() const { return showPhone_; }
    bool imuTilt() const { return imuTilt_; }
    int highlightLink() const { return highlight_; }
    void setGhostVisible(bool v) { if (v != ghostVisible_) { ghostVisible_ = v; emit ghostChanged(); } }
    void setShowPhone(bool v) { if (v != showPhone_) { showPhone_ = v; emit optionsChanged(); } }
    void setImuTilt(bool v) { if (v != imuTilt_) { imuTilt_ = v; recompute(); emit optionsChanged(); } }
    void setHighlightLink(int v) { if (v != highlight_) { highlight_ = v; emit optionsChanged(); } }

    // Pratinjau (servo kerja, editor gerak, kamera) menimpa pose robot sampai clearPreview().
    Q_INVOKABLE void setPreview(const QVariantList& src22, const QString& owner);
    Q_INVOKABLE void clearPreview(const QString& owner);
    // Dari RobotLink
    Q_INVOKABLE void setRobotPose(const QVariantList& kin22);
    Q_INVOKABLE void setRobotActual(const QVariantList& fb22);
    Q_INVOKABLE void setRobotTilt(double rollDeg, double pitchDeg);
    Q_INVOKABLE int linkForJoint(int index) const;

signals:
    void poseChanged();
    void ghostChanged();
    void sourceChanged();
    void optionsChanged();

private:
    static bool toSrc(const QVariantList& v, la::studio::JointSrc out);
    void recompute();
    void solve(const la::studio::JointSrc src, QVariantList& pos, QVariantList& rot) const;

    QVariantList meshes_, jointInfo_;
    QVariantList pos_, rot_, gpos_, grot_;
    la::studio::JointSrc robot_{}, actual_{}, preview_{};
    bool haveRobot_ = false, haveActual_ = false;
    QString owner_;
    bool ghostVisible_ = true, showPhone_ = true, imuTilt_ = true;
    int highlight_ = -1, phoneLink_ = -1;
    float roll_ = 0, pitch_ = 0;
};
