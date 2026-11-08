// RobotLink: koneksi studio ke robot (TCP Wi-Fi, serial USB, atau simulator di dalam proses).
#pragma once
#include "lap_client.h"
#include "sim_robot.h"

#include <QObject>
#include <QStringList>
#include <QTimer>
#include <QVariantList>
#include <memory>

class QTcpSocket;
class QSerialPort;

class RobotLink : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    Q_PROPERTY(bool alive READ alive NOTIFY telemetryChanged)
    Q_PROPERTY(bool isSim READ isSim NOTIFY connectedChanged)
    Q_PROPERTY(QString linkLabel READ linkLabel NOTIFY connectedChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(QString robotName READ robotName NOTIFY helloChanged)
    Q_PROPERTY(QString boardName READ boardName NOTIFY helloChanged)
    Q_PROPERTY(QString fwVersion READ fwVersion NOTIFY helloChanged)
    Q_PROPERTY(uint caps READ caps NOTIFY helloChanged)
    Q_PROPERTY(int mode READ mode NOTIFY telemetryChanged)
    Q_PROPERTY(QString modeName READ modeName NOTIFY telemetryChanged)
    Q_PROPERTY(double roll READ roll NOTIFY telemetryChanged)
    Q_PROPERTY(double pitch READ pitch NOTIFY telemetryChanged)
    Q_PROPERTY(int imuHz READ imuHz NOTIFY telemetryChanged)
    Q_PROPERTY(int loopAvgUs READ loopAvgUs NOTIFY telemetryChanged)
    Q_PROPERTY(int loopMaxUs READ loopMaxUs NOTIFY telemetryChanged)
    Q_PROPERTY(int servoPhaseUs READ servoPhaseUs NOTIFY telemetryChanged)
    Q_PROPERTY(int fbOk READ fbOk NOTIFY telemetryChanged)
    Q_PROPERTY(double autoH READ autoH NOTIFY telemetryChanged)
    Q_PROPERTY(int flags READ flags NOTIFY telemetryChanged)
    Q_PROPERTY(QVariantList cmdSrc READ cmdSrc NOTIFY servoChanged)
    Q_PROPERTY(QVariantList fbSrc READ fbSrc NOTIFY servoChanged)
    Q_PROPERTY(QVariantList trim READ trim NOTIFY servoChanged)
    Q_PROPERTY(QVariantList valid READ valid NOTIFY servoChanged)
    Q_PROPERTY(QVariantList kinSrc READ kinSrc NOTIFY gaitChanged)
    Q_PROPERTY(QVariantList params READ params NOTIFY configChanged)
    Q_PROPERTY(QVariantList motionSlots READ motionSlots NOTIFY motionsChanged)
    Q_PROPERTY(QStringList logLines READ logLines NOTIFY logChanged)

public:
    explicit RobotLink(QObject* parent = nullptr);
    ~RobotLink() override;

    Q_INVOKABLE QStringList serialPorts() const;
    Q_INVOKABLE void connectTcp(const QString& host, int port = 5760);
    Q_INVOKABLE void connectSerial(const QString& port, int baud = 115200);
    Q_INVOKABLE void connectSim();
    Q_INVOKABLE void disconnectLink();

    Q_INVOKABLE void sendCmd(int id);                       // 0 reset, 1 go, 2 stop, 3 kalibrasi
    Q_INVOKABLE void setParam(int id, double value);
    Q_INVOKABLE void setTrim(int bus, int slot, int value);
    Q_INVOKABLE void gaitIntent(double vx, double yawDeg, int ttlMs);
    Q_INVOKABLE void poseTarget(const QVariantList& src22, uint mask, int ttlMs);
    Q_INVOKABLE bool uploadMotion(const QByteArray& lam, int slot, bool save);
    Q_INVOKABLE void playMotion(int slot);
    Q_INVOKABLE void stopMotion(int slot);
    Q_INVOKABLE void simImu(double rollDeg, double pitchDeg);
    Q_INVOKABLE void clearLog();

    bool connected() const { return kind_ != None; }
    bool alive() const;
    bool isSim() const { return kind_ == Sim; }
    QString linkLabel() const { return label_; }
    QString status() const { return status_; }
    QString robotName() const;
    QString boardName() const;
    QString fwVersion() const;
    uint caps() const { return cli_.haveHello ? cli_.hello.caps : 0; }
    int mode() const { return cli_.haveTele ? cli_.tele.mode : 0; }
    QString modeName() const;
    double roll() const { return cli_.tele.rollDeg; }
    double pitch() const { return cli_.tele.pitchDeg; }
    int imuHz() const { return cli_.tele.imuHz; }
    int loopAvgUs() const { return cli_.tele.loopAvgUs; }
    int loopMaxUs() const { return cli_.tele.loopMaxUs; }
    int servoPhaseUs() const { return cli_.tele.servoPhaseUs; }
    int fbOk() const { return cli_.tele.fbOk; }
    double autoH() const { return cli_.tele.autoH; }
    int flags() const { return cli_.tele.flags; }
    QVariantList cmdSrc() const;
    QVariantList fbSrc() const;
    QVariantList trim() const;
    QVariantList valid() const;
    QVariantList kinSrc() const;
    QVariantList params() const;
    QVariantList motionSlots() const;
    QStringList logLines() const { return log_; }

signals:
    void connectedChanged();
    void statusChanged();
    void helloChanged();
    void telemetryChanged();
    void servoChanged();
    void gaitChanged();
    void configChanged();
    void motionsChanged();
    void logChanged();
    void ackReceived(int type, int status);

private:
    enum Kind { None, Tcp, Serial, Sim };
    void setupClient();
    void attached(Kind k, const QString& label);
    void setStatus(const QString& s);
    void appendLog(const QString& line);
    void onBytes(const QByteArray& b);

    Kind kind_ = None;
    QString label_, status_;
    la::studio::LapClient cli_;
    QTcpSocket* tcp_ = nullptr;
    QSerialPort* serial_ = nullptr;
    std::unique_ptr<la::studio::SimRobot> sim_;
    QByteArray simRx_;                 // host -> sim (diproses per siklus sim)
    QTimer simTimer_, pingTimer_;
    qint64 lastRxMs_ = 0;
    QStringList log_;
    QString logPartial_;
};
