// StlMesh: geometri Qt Quick 3D dari STL biner (frame robot -> frame Qt), normal halus
// dengan sudut lipat 50 derajat agar kotak membulat tampak mulus tapi tepi tajam tetap tegas.
#pragma once
#include <QQuick3DGeometry>
#include <QUrl>
#include <QtQml/qqmlregistration.h>

class StlMesh : public QQuick3DGeometry {
    Q_OBJECT
    QML_NAMED_ELEMENT(StlMesh)
    Q_PROPERTY(QUrl source READ source WRITE setSource NOTIFY sourceChanged)
    Q_PROPERTY(int triangles READ triangles NOTIFY sourceChanged)

public:
    explicit StlMesh(QQuick3DObject* parent = nullptr);
    QUrl source() const { return source_; }
    void setSource(const QUrl& u);
    int triangles() const { return tris_; }

signals:
    void sourceChanged();

private:
    void load();
    QUrl source_;
    int tris_ = 0;
};
