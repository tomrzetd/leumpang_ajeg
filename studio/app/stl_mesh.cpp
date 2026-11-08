#include "stl_mesh.h"

#include <QFile>
#include <QHash>
#include <QVector3D>
#include <QtEndian>
#include <cmath>
#include <cstring>
#include <vector>

StlMesh::StlMesh(QQuick3DObject* parent) : QQuick3DGeometry(parent) {}

void StlMesh::setSource(const QUrl& u) {
    if (u == source_) return;
    source_ = u;
    load();
    emit sourceChanged();
}

static QString localPath(const QUrl& u) {
    if (u.scheme() == QLatin1String("qrc")) return QLatin1Char(':') + u.path();
    if (u.isLocalFile()) return u.toLocalFile();
    return u.toString();
}

void StlMesh::load() {
    clear();
    tris_ = 0;
    QFile f(localPath(source_));
    if (!f.open(QIODevice::ReadOnly)) { qWarning("StlMesh: tidak bisa membuka %s", qPrintable(source_.toString())); update(); return; }
    const QByteArray d = f.readAll();
    if (d.size() < 84) { update(); return; }
    const quint32 n = qFromLittleEndian<quint32>(d.constData() + 80);
    if ((qint64)d.size() < 84 + (qint64)n * 50) { qWarning("StlMesh: berkas terpotong %s", qPrintable(source_.toString())); update(); return; }

    // robot (x,y,z) -> Qt (y,z,x)
    std::vector<QVector3D> pos(n * 3), fn(n);
    for (quint32 i = 0; i < n; ++i) {
        const char* r = d.constData() + 84 + i * 50;
        float v[12];
        memcpy(v, r, sizeof v);
        for (int k = 0; k < 3; ++k) pos[i * 3 + k] = QVector3D(v[3 + k * 3 + 1], v[3 + k * 3 + 2], v[3 + k * 3 + 0]);
        QVector3D nn = QVector3D::crossProduct(pos[i * 3 + 1] - pos[i * 3], pos[i * 3 + 2] - pos[i * 3]);
        fn[i] = nn.normalized();
    }
    // normal halus per posisi terkuantisasi, hanya dari segitiga dengan sudut < 50 derajat
    struct Key { qint32 x, y, z; bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; } };
    auto key = [](const QVector3D& p) { return Key{(qint32)std::lround(p.x() * 100), (qint32)std::lround(p.y() * 100), (qint32)std::lround(p.z() * 100)}; };
    struct KeyHash { size_t operator()(const Key& k) const { return (size_t)k.x * 73856093u ^ (size_t)k.y * 19349663u ^ (size_t)k.z * 83492791u; } };
    QHash<quint64, std::vector<quint32>> shared;
    auto hk = [&](const QVector3D& p) { const Key k = key(p); return (quint64)KeyHash{}(k); };
    for (quint32 i = 0; i < n * 3; ++i) shared[hk(pos[i])].push_back(i / 3);
    const float cosCrease = std::cos(50.0f * 3.14159265f / 180.0f);

    QByteArray vb;
    vb.resize((qsizetype)n * 3 * 6 * (qsizetype)sizeof(float));
    float* out = reinterpret_cast<float*>(vb.data());
    QVector3D mn(1e9f, 1e9f, 1e9f), mx(-1e9f, -1e9f, -1e9f);
    for (quint32 i = 0; i < n * 3; ++i) {
        const QVector3D& p = pos[i];
        const QVector3D& own = fn[i / 3];
        QVector3D acc;
        for (quint32 t : shared[hk(p)]) if (QVector3D::dotProduct(fn[t], own) >= cosCrease) acc += fn[t];
        const QVector3D nrm = acc.lengthSquared() > 1e-12f ? acc.normalized() : own;
        *out++ = p.x(); *out++ = p.y(); *out++ = p.z();
        *out++ = nrm.x(); *out++ = nrm.y(); *out++ = nrm.z();
        mn = QVector3D(qMin(mn.x(), p.x()), qMin(mn.y(), p.y()), qMin(mn.z(), p.z()));
        mx = QVector3D(qMax(mx.x(), p.x()), qMax(mx.y(), p.y()), qMax(mx.z(), p.z()));
    }
    setVertexData(vb);
    setStride(6 * sizeof(float));
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
    addAttribute(QQuick3DGeometry::Attribute::PositionSemantic, 0, QQuick3DGeometry::Attribute::F32Type);
    addAttribute(QQuick3DGeometry::Attribute::NormalSemantic, 3 * sizeof(float), QQuick3DGeometry::Attribute::F32Type);
    setBounds(mn, mx);
    tris_ = (int)n;
    update();
}
