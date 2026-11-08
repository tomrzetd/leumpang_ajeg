#include "twin_model.h"
#include "leumpang/controller.h"

#include <QColor>
#include <QVariantMap>
#include <cstring>

using namespace la;
using namespace la::studio;

static QVector3D toQt(const Vec3& v) { return QVector3D(v.y, v.z, v.x); }
static QQuaternion toQt(const Quat& q) { return QQuaternion(q.w, q.y, q.z, q.x); }

TwinModel::TwinModel(QObject* parent) : QObject(parent) {
    phoneLink_ = linkIndex("phone");
    for (const gen::MeshDef& m : gen::MESHES) {
        const gen::MaterialDef& mat = gen::MATERIALS[m.material];
        QVariantMap e;
        e[QStringLiteral("link")] = (int)m.link;
        e[QStringLiteral("source")] = QStringLiteral("qrc:/meshes/") + QString::fromLatin1(m.file);
        e[QStringLiteral("material")] = QString::fromLatin1(mat.name);
        e[QStringLiteral("color")] = QColor::fromRgbF(mat.r, mat.g, mat.b);
        e[QStringLiteral("metalness")] = mat.metalness;
        e[QStringLiteral("roughness")] = mat.roughness;
        meshes_ << e;
    }
    JointSrc home; la::studio::homeSrc(home);
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
        const JointDef& j = JOINTS[b][s];
        const int a = rawToSrc(b, s, j.lo, 0), c = rawToSrc(b, s, j.hi, 0);
        QVariantMap e;
        e[QStringLiteral("index")] = b * SERVO_SLOTS + s;
        e[QStringLiteral("bus")] = b; e[QStringLiteral("slot")] = s; e[QStringLiteral("id")] = j.id;
        e[QStringLiteral("name")] = QString::fromLatin1(JOINT_NAMES[b][s]);
        e[QStringLiteral("min")] = qMin(a, c); e[QStringLiteral("max")] = qMax(a, c);
        e[QStringLiteral("home")] = (int)home[b][s];
        e[QStringLiteral("link")] = linkForJoint(b * SERVO_SLOTS + s);
        jointInfo_ << e;
    }
    memcpy(robot_, home, sizeof(robot_));
    recompute();
}

QVariantList TwinModel::homeSrc() const {
    JointSrc h; la::studio::homeSrc(h);
    QVariantList v;
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) v << (int)h[b][s];
    return v;
}

int TwinModel::linkForJoint(int index) const {
    const int b = index / SERVO_SLOTS, s = index % SERVO_SLOTS;
    for (int i = 0; i < gen::LINK_COUNT; ++i) if (gen::LINKS[i].bus == b && gen::LINKS[i].slot == s) return i;
    return -1;
}

bool TwinModel::toSrc(const QVariantList& v, JointSrc out) {
    if (v.size() != 2 * SERVO_SLOTS) return false;
    for (int i = 0; i < 2 * SERVO_SLOTS; ++i) out[i / SERVO_SLOTS][i % SERVO_SLOTS] = (int16_t)v[i].toInt();
    return true;
}

void TwinModel::solve(const JointSrc src, QVariantList& pos, QVariantList& rot) const {
    Pose P[gen::LINK_COUNT];
    WorldOpts o;
    if (imuTilt_) { o.rollRad = roll_ / kRad2Deg; o.pitchRad = pitch_ / kRad2Deg; }
    world(src, o, P);
    pos.clear(); rot.clear();
    for (int i = 0; i < gen::LINK_COUNT; ++i) { pos << toQt(P[i].p); rot << toQt(P[i].q); }
}

void TwinModel::recompute() {
    solve(owner_.isEmpty() ? robot_ : preview_, pos_, rot_);
    emit poseChanged();
    // hantu: pratinjau aktif -> pose robot; tanpa pratinjau -> umpan balik servo aktual
    if (!owner_.isEmpty()) solve(robot_, gpos_, grot_);
    else if (haveActual_) solve(actual_, gpos_, grot_);
    else { gpos_ = pos_; grot_ = rot_; }
    emit ghostChanged();
}

QVariantList TwinModel::currentSrc() const {
    QVariantList v;
    const JointSrc& s = owner_.isEmpty() ? robot_ : preview_;
    for (int b = 0; b < 2; ++b) for (int k = 0; k < SERVO_SLOTS; ++k) v << (int)s[b][k];
    return v;
}

void TwinModel::setPreview(const QVariantList& src, const QString& owner) {
    if (!toSrc(src, preview_)) return;
    const bool changed = owner_ != owner;
    owner_ = owner.isEmpty() ? QStringLiteral("pratinjau") : owner;
    if (changed) emit sourceChanged();
    recompute();
}
void TwinModel::clearPreview(const QString& owner) {
    if (owner_.isEmpty() || (!owner.isEmpty() && owner != owner_)) return;
    owner_.clear();
    emit sourceChanged();
    recompute();
}
void TwinModel::setRobotPose(const QVariantList& kin) {
    if (!toSrc(kin, robot_)) return;
    haveRobot_ = true;
    recompute();
}
void TwinModel::setRobotActual(const QVariantList& fb) {
    if (!toSrc(fb, actual_)) return;
    haveActual_ = true;
}
void TwinModel::setRobotTilt(double r, double p) { roll_ = (float)r; pitch_ = (float)p; }
