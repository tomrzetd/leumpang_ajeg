#include "motion_editor.h"
#include "robot_link.h"
#include "twin_model.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <cstring>

using namespace la;
using namespace la::studio;

static QString pathOf(const QUrl& u) { return u.isLocalFile() ? u.toLocalFile() : (u.scheme() == QLatin1String("qrc") ? QLatin1Char(':') + u.path() : u.toString()); }
static bool toSrc(const QVariantList& v, int16_t out[SERVO_BUSES][SERVO_SLOTS]) {
    if (v.size() != 2 * SERVO_SLOTS) return false;
    for (int i = 0; i < 2 * SERVO_SLOTS; ++i) out[i / SERVO_SLOTS][i % SERVO_SLOTS] = (int16_t)v[i].toInt();
    return true;
}
static QVariantList fromSrc(const int16_t s[SERVO_BUSES][SERVO_SLOTS]) {
    QVariantList v;
    for (int b = 0; b < 2; ++b) for (int k = 0; k < SERVO_SLOTS; ++k) v << (int)s[b][k];
    return v;
}

MotionEditor::MotionEditor(RobotLink* link, TwinModel* twin, QObject* parent)
    : QObject(parent), link_(link), twin_(twin) {
    previewTimer_.setInterval(20);
    connect(&previewTimer_, &QTimer::timeout, this, [this] {
        const int t = previewOffset_ + (int)previewClock_.elapsed();
        if (t > totalMs()) { preview(false); showAt(totalMs()); return; }
        showAt(t);
    });
}

void MotionEditor::setName(const QString& n) {
    const std::string s = n.left(16).toStdString();
    if (s == doc_.name) return;
    doc_.name = s; emit docChanged();
}
void MotionEditor::setReturnHome(bool v) { if (v != doc_.returnHome) { doc_.returnHome = v; emit docChanged(); } }

QVariantList MotionEditor::frames() const {
    QVariantList out;
    uint32_t t = 0;
    for (const Keyframe& k : doc_.frames) {
        QVariantMap m;
        m[QStringLiteral("ms")] = k.ms; m[QStringLiteral("interp")] = k.interp; m[QStringLiteral("startMs")] = (int)t;
        out << m;
        t += k.ms;
    }
    return out;
}

QString MotionEditor::problem() const {
    std::string err;
    int16_t trim[2][SERVO_SLOTS] = {};
    if (link_ && link_->connected()) {
        const QVariantList tv = link_->trim();
        if (tv.size() == 2 * SERVO_SLOTS) toSrc(tv, trim);
    }
    return doc_.validate(trim, &err) ? QString() : QString::fromStdString(err);
}

void MotionEditor::clear() { preview(false); doc_ = MotionDoc(); emit docChanged(); }

void MotionEditor::addFrame(const QVariantList& src, int ms, int at) {
    Keyframe k;
    if (!toSrc(src, k.j)) return;
    k.ms = (uint16_t)qBound((int)MOTION_MIN_FRAME_MS, ms, (int)MOTION_MAX_FRAME_MS);
    k.interp = MI_COSINE;
    if (at < 0 || at > (int)doc_.frames.size()) doc_.frames.push_back(k);
    else doc_.frames.insert(doc_.frames.begin() + at, k);
    emit docChanged();
}
void MotionEditor::setFramePose(int i, const QVariantList& src) {
    if (i < 0 || i >= (int)doc_.frames.size()) return;
    if (toSrc(src, doc_.frames[(size_t)i].j)) emit docChanged();
}
QVariantList MotionEditor::framePose(int i) const {
    if (i < 0 || i >= (int)doc_.frames.size()) return {};
    return fromSrc(doc_.frames[(size_t)i].j);
}
void MotionEditor::setFrameMs(int i, int ms) {
    if (i < 0 || i >= (int)doc_.frames.size()) return;
    doc_.frames[(size_t)i].ms = (uint16_t)qBound((int)MOTION_MIN_FRAME_MS, ms, (int)MOTION_MAX_FRAME_MS);
    emit docChanged();
}
void MotionEditor::setFrameInterp(int i, int ip) {
    if (i < 0 || i >= (int)doc_.frames.size() || ip < 0 || ip > 2) return;
    doc_.frames[(size_t)i].interp = (uint8_t)ip;
    emit docChanged();
}
void MotionEditor::removeFrame(int i) {
    if (i < 0 || i >= (int)doc_.frames.size()) return;
    doc_.frames.erase(doc_.frames.begin() + i);
    emit docChanged();
}
void MotionEditor::moveFrame(int i, int d) {
    const int j = i + d;
    if (i < 0 || j < 0 || i >= (int)doc_.frames.size() || j >= (int)doc_.frames.size()) return;
    std::swap(doc_.frames[(size_t)i], doc_.frames[(size_t)j]);
    emit docChanged();
}
void MotionEditor::duplicateFrame(int i) {
    if (i < 0 || i >= (int)doc_.frames.size()) return;
    doc_.frames.insert(doc_.frames.begin() + i + 1, doc_.frames[(size_t)i]);
    emit docChanged();
}
void MotionEditor::showFrame(int i) {
    if (i < 0 || i >= (int)doc_.frames.size()) return;
    twin_->setPreview(fromSrc(doc_.frames[(size_t)i].j), QStringLiteral("gerak"));
}

void MotionEditor::showAt(int ms) {
    previewMs_ = ms;
    JointSrc home, out; homeSrc(home);
    doc_.sample((uint32_t)qMax(0, ms), home, out);
    twin_->setPreview(fromSrc(out), QStringLiteral("gerak"));
    emit previewChanged();
}
void MotionEditor::preview(bool on) {
    if (on && !doc_.frames.empty()) {
        previewOffset_ = previewMs_ >= totalMs() ? 0 : previewMs_;
        previewClock_.start();
        previewTimer_.start();
    } else {
        previewTimer_.stop();
    }
    emit previewChanged();
}
void MotionEditor::seek(int ms) { if (previewTimer_.isActive()) { previewOffset_ = ms; previewClock_.restart(); } showAt(ms); }

// ---- berkas -------------------------------------------------------------------
QByteArray MotionEditor::toJson() const {
    static const char* ip[] = {"cosine", "linear", "step"};
    QJsonObject root;
    root[QStringLiteral("name")] = QString::fromStdString(doc_.name);
    root[QStringLiteral("returnHome")] = doc_.returnHome;
    QJsonArray fr;
    for (const Keyframe& k : doc_.frames) {
        QJsonObject f, j;
        f[QStringLiteral("ms")] = k.ms;
        f[QStringLiteral("interp")] = QString::fromLatin1(ip[k.interp < 3 ? k.interp : 0]);
        for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
            j[QString::fromLatin1(jointName(b, s))] = k.j[b][s];
        f[QStringLiteral("joints")] = j;
        fr.append(f);
    }
    root[QStringLiteral("frames")] = fr;
    return QJsonDocument(root).toJson(QJsonDocument::Indented);
}

bool MotionEditor::fromJson(const QByteArray& data, QString* err) {
    QJsonParseError pe;
    const QJsonDocument jd = QJsonDocument::fromJson(data, &pe);
    if (!jd.isObject()) { *err = tr("JSON tidak valid: %1").arg(pe.errorString()); return false; }
    const QJsonObject root = jd.object();
    MotionDoc d;
    d.name = root.value(QStringLiteral("name")).toString(QStringLiteral("gerak")).left(16).toStdString();
    d.returnHome = root.value(QStringLiteral("returnHome")).toBool(true);
    JointSrc prev; homeSrc(prev);                                   // sama dengan lam.py: sendi kosong = frame sebelumnya
    for (const QJsonValue& fv : root.value(QStringLiteral("frames")).toArray()) {
        const QJsonObject f = fv.toObject();
        Keyframe k;
        memcpy(k.j, prev, sizeof(k.j));
        k.ms = (uint16_t)qBound((int)MOTION_MIN_FRAME_MS, f.value(QStringLiteral("ms")).toInt(500), (int)MOTION_MAX_FRAME_MS);
        const QString ip = f.value(QStringLiteral("interp")).toString(QStringLiteral("cosine"));
        k.interp = ip == QLatin1String("linear") ? MI_LINEAR : (ip == QLatin1String("step") ? MI_STEP : MI_COSINE);
        const QJsonObject j = f.value(QStringLiteral("joints")).toObject();
        for (auto it = j.begin(); it != j.end(); ++it) {
            uint8_t b, s;
            if (!jointByName(it.key().toLatin1().constData(), b, s)) { *err = tr("sendi tidak dikenal: %1").arg(it.key()); return false; }
            k.j[b][s] = (int16_t)it.value().toInt();
        }
        memcpy(prev, k.j, sizeof(prev));
        d.frames.push_back(k);
    }
    preview(false);
    doc_ = d;
    emit docChanged();
    return true;
}

bool MotionEditor::loadJson(const QUrl& url) {
    QFile f(pathOf(url));
    if (!f.open(QIODevice::ReadOnly)) { say(tr("gagal membuka %1").arg(f.fileName())); return false; }
    QString err;
    if (!fromJson(f.readAll(), &err)) { say(err); return false; }
    say(tr("dimuat: %1 (%2 keyframe)").arg(name()).arg(doc_.frames.size()));
    return true;
}
bool MotionEditor::loadExample() { return loadJson(QUrl(QStringLiteral("qrc:/motions/lambai.json"))); }
bool MotionEditor::saveJson(const QUrl& url) {
    QFile f(pathOf(url));
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) { say(tr("gagal menulis %1").arg(f.fileName())); return false; }
    f.write(toJson());
    say(tr("disimpan: %1").arg(f.fileName()));
    return true;
}
bool MotionEditor::importLam(const QUrl& url) {
    QFile f(pathOf(url));
    if (!f.open(QIODevice::ReadOnly)) { say(tr("gagal membuka %1").arg(f.fileName())); return false; }
    const QByteArray b = f.readAll();
    std::string err;
    MotionDoc d;
    if (!MotionDoc::fromLam((const uint8_t*)b.constData(), (size_t)b.size(), d, &err)) { say(QString::fromStdString(err)); return false; }
    preview(false); doc_ = d; emit docChanged();
    say(tr("LAM dimuat: %1").arg(name()));
    return true;
}
bool MotionEditor::exportLam(const QUrl& url) {
    std::string err;
    const std::vector<uint8_t> lam = doc_.toLam(&err);
    if (lam.empty()) { say(QString::fromStdString(err)); return false; }
    QFile f(pathOf(url));
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) { say(tr("gagal menulis %1").arg(f.fileName())); return false; }
    f.write((const char*)lam.data(), (qint64)lam.size());
    say(tr("LAM1 %1 B -> %2").arg(lam.size()).arg(f.fileName()));
    return true;
}
bool MotionEditor::upload(int slot, bool save) {
    const QString p = problem();
    if (!p.isEmpty()) { say(tr("tidak diunggah: %1").arg(p)); return false; }
    std::string err;
    const std::vector<uint8_t> lam = doc_.toLam(&err);
    if (lam.empty() || !link_->uploadMotion(QByteArray((const char*)lam.data(), (int)lam.size()), slot, save)) {
        say(tr("unggah gagal (robot belum terhubung?)"));
        return false;
    }
    say(tr("'%1' (%2 B) diunggah ke slot %3%4").arg(name()).arg(lam.size()).arg(slot).arg(save ? tr(" + SD") : QString()));
    return true;
}

// ---- rekaman -------------------------------------------------------------------
void MotionEditor::startRecording() { rec_.clear(); recording_ = true; recClock_.start(); emit recordChanged(); }
void MotionEditor::addSample(const QVariantList& src) {
    if (!recording_) return;
    PoseSample p{};
    if (!toSrc(src, p.j)) return;
    p.tMs = (uint32_t)recClock_.elapsed();
    if (!rec_.empty() && p.tMs <= rec_.back().tMs) return;
    rec_.push_back(p);
    if (rec_.size() % 10 == 0) emit recordChanged();
}
int MotionEditor::stopRecording(int tol) {
    recording_ = false;
    emit recordChanged();
    if (rec_.size() < 2) { say(tr("rekaman terlalu pendek")); return 0; }
    const int room = MOTION_MAX_FRAMES - (int)doc_.frames.size();
    if (room < 2) { say(tr("dokumen sudah penuh (%1 keyframe)").arg(MOTION_MAX_FRAMES)); return 0; }
    std::vector<Keyframe> k = simplify(rec_, tol, room);
    doc_.frames.insert(doc_.frames.end(), k.begin(), k.end());
    emit docChanged();
    say(tr("rekaman %1 sampel / %2 s -> %3 keyframe").arg(rec_.size()).arg(rec_.back().tMs / 1000.0, 0, 'f', 1).arg(k.size()));
    return (int)k.size();
}
