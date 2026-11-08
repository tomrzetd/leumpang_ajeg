// Leumpang Studio - aplikasi desktop (Qt 6): koneksi robot, digital twin 3D, servo, gerak, kamera.
//
//   leumpang_studio                     jalankan normal
//   leumpang_studio --selftest DIR      uji otomatis: simulator, unggah & putar gerak, live pose,
//                                       tangkapan layar tiap tab ke DIR; exit 1 bila ada warning QML.
#include "motion_editor.h"
#include "pose_capture.h"
#include "robot_link.h"
#include "twin_model.h"

#include <QDir>
#include <QGuiApplication>
#include <QImage>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QTimer>
#include <QtQuick3D/qquick3d.h>
#include <cstdio>
#include <functional>

static QStringList g_qmlProblems;
static QtMessageHandler g_prevHandler = nullptr;

static void messageHandler(QtMsgType t, const QMessageLogContext& c, const QString& m) {
    const bool fromQml = c.file && (QString::fromLatin1(c.file).endsWith(QLatin1String(".qml")) || QString::fromLatin1(c.file).startsWith(QLatin1String("qrc:")));
    if ((t == QtWarningMsg || t == QtCriticalMsg || t == QtFatalMsg) &&
        (fromQml || m.contains(QLatin1String(".qml")) || m.contains(QLatin1String("StlMesh"))))
        g_qmlProblems << m;
    if (g_prevHandler) g_prevHandler(t, c, m);
}

int main(int argc, char** argv) {
    QGuiApplication::setApplicationName(QStringLiteral("Leumpang Studio"));
    QGuiApplication::setOrganizationName(QStringLiteral("leumpang-ajeg"));
    QSurfaceFormat::setDefaultFormat(QQuick3D::idealSurfaceFormat(4));
    QGuiApplication app(argc, argv);
    QQuickStyle::setStyle(QStringLiteral("Material"));

    QString selftestDir;
    const QStringList args = app.arguments();
    const int st = args.indexOf(QStringLiteral("--selftest"));
    if (st >= 0) selftestDir = args.value(st + 1, QStringLiteral("selftest"));
    g_prevHandler = qInstallMessageHandler(messageHandler);

    RobotLink link;
    TwinModel twin;
    MotionEditor motion(&link, &twin);
    PoseCapture camera(&link, &twin, &motion);

    QObject::connect(&link, &RobotLink::gaitChanged, &twin, [&] { twin.setRobotPose(link.kinSrc()); });
    QObject::connect(&link, &RobotLink::servoChanged, &twin, [&] { twin.setRobotActual(link.fbSrc()); });
    QObject::connect(&link, &RobotLink::telemetryChanged, &twin, [&] { twin.setRobotTilt(link.roll(), link.pitch()); });
    QObject::connect(&link, &RobotLink::servoChanged, &motion, &MotionEditor::docChanged);   // trim -> validasi ulang

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty(QStringLiteral("robot"), &link);
    engine.rootContext()->setContextProperty(QStringLiteral("twin"), &twin);
    engine.rootContext()->setContextProperty(QStringLiteral("motion"), &motion);
    engine.rootContext()->setContextProperty(QStringLiteral("poseCam"), &camera);
    engine.rootContext()->setContextProperty(QStringLiteral("selftest"), !selftestDir.isEmpty());
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed, &app, [] { QCoreApplication::exit(2); }, Qt::QueuedConnection);
    engine.loadFromModule("Leumpang", "Main");
    if (engine.rootObjects().isEmpty()) return 2;
    if (selftestDir.isEmpty()) return app.exec();

    // ------------------------------------------------------------------ selftest
    QDir().mkpath(selftestDir);
    QQuickWindow* win = qobject_cast<QQuickWindow*>(engine.rootObjects().first());
    int failures = 0;
    auto check = [&](bool ok, const char* what) { std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what); std::fflush(stdout); if (!ok) ++failures; };
    auto shot = [&](const QString& name) {
        const QImage img = win->grabWindow();
        const bool ok = !img.isNull() && img.save(selftestDir + QLatin1Char('/') + name);
        std::printf("      layar -> %s/%s (%dx%d)\n", qPrintable(selftestDir), qPrintable(name), img.width(), img.height());
        return ok;
    };
    auto tab = [&](int i) { win->setProperty("currentTab", i); };

    struct Step { int atMs; std::function<void()> fn; };
    QList<Step> steps = {
        {300, [&] { link.connectSim(); }},
        {2800, [&] {
            check(link.alive() && link.mode() == 720, "simulator terhubung, mode READY");
            check(link.kinSrc().size() == 22 && link.fbSrc().size() == 22, "GAIT_STATE & SERVO_STATE diterima");
            check(twin.positions().size() == la::studio::gen::LINK_COUNT, "twin punya pose semua link");
            tab(0); shot(QStringLiteral("01_servo.png"));
        }},
        {3000, [&] { check(motion.loadExample() && motion.frames().size() == 6, "contoh gerak 'lambai' dimuat"); tab(1); }},
        {3200, [&] { check(motion.upload(0, false), "unggah gerak ke slot 0"); }},
        {3600, [&] {
            const QVariantList s = link.motionSlots();
            check(!s.isEmpty() && s[0].toMap().value(QStringLiteral("state")).toInt() == 2, "slot 0 siap di robot");
            link.playMotion(0);
        }},
        {4700, [&] { check(link.mode() == 810, "robot memutar gerak (mode 810)"); shot(QStringLiteral("02_gerak_diputar.png")); }},
        {8500, [&] { check(link.mode() == 720, "gerak selesai -> READY"); motion.seek(1400); shot(QStringLiteral("03_pratinjau_gerak.png")); }},
        {8800, [&] {
            twin.clearPreview(QString());
            QVariantList p = twin.homeSrc(); p[4] = -1000; p[11 + 2] = 1800;
            link.poseTarget(p, (1u << 4) | (1u << 13), 600);
        }},
        {9300, [&] { check(link.mode() == 820, "live pose (mode 820)"); tab(2); shot(QStringLiteral("04_kamera.png")); }},
        {9600, [&] { tab(3); shot(QStringLiteral("05_jalan.png")); }},
        {9900, [&] { tab(4); shot(QStringLiteral("06_parameter.png")); }},
        {10200, [&] { tab(5); shot(QStringLiteral("07_log.png")); check(!link.logLines().isEmpty(), "log robot tampil"); }},
        {10600, [&] {
            check(g_qmlProblems.isEmpty(), "tidak ada warning/galat QML");
            for (const QString& p : g_qmlProblems) std::printf("      QML: %s\n", qPrintable(p));
            std::printf("== selftest studio: %s\n", failures || !g_qmlProblems.isEmpty() ? "GAGAL" : "lulus");
            QCoreApplication::exit(failures || !g_qmlProblems.isEmpty() ? 1 : 0);
        }},
    };
    for (const Step& s : steps) QTimer::singleShot(s.atMs, &app, s.fn);
    QTimer::singleShot(30000, &app, [] { std::printf("selftest: timeout\n"); QCoreApplication::exit(3); });
    return app.exec();
}
