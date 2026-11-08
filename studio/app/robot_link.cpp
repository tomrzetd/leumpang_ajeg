#include "robot_link.h"
#include "kinematics.h"

#include <QDateTime>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTcpSocket>

using namespace la;
using namespace la::studio;

static qint64 nowMs() { return QDateTime::currentMSecsSinceEpoch(); }

RobotLink::RobotLink(QObject* parent) : QObject(parent) {
    setupClient();
    simTimer_.setInterval((int)CONTROL_PERIOD_MS);
    simTimer_.setTimerType(Qt::PreciseTimer);
    connect(&simTimer_, &QTimer::timeout, this, [this] {
        if (!sim_) return;
        if (!simRx_.isEmpty()) {
            const QByteArray b = simRx_; simRx_.clear();
            sim_->fromHost((const uint8_t*)b.constData(), (size_t)b.size());
        }
        sim_->advance(1);
    });
    pingTimer_.setInterval(1000);
    connect(&pingTimer_, &QTimer::timeout, this, [this] {
        if (kind_ == None) return;
        cli_.ping();
        if (cli_.haveTele && !alive()) setStatus(tr("tidak ada data dari robot > 3 s"));
        emit telemetryChanged();               // segarkan 'alive'
    });
    setStatus(tr("belum terhubung"));
}

RobotLink::~RobotLink() { disconnectLink(); }

void RobotLink::setupClient() {
    cli_.write = [this](const uint8_t* d, size_t n) {
        const QByteArray b((const char*)d, (int)n);
        switch (kind_) {
            case Tcp:    if (tcp_) tcp_->write(b); break;
            case Serial: if (serial_) serial_->write(b); break;
            case Sim:    simRx_.append(b); break;
            default: break;
        }
    };
    cli_.onLog = [this](const std::string& s) { appendLog(QString::fromUtf8(s.data(), (int)s.size())); };
    cli_.onPacket = [this](uint8_t type) {
        using namespace la::lap;
        lastRxMs_ = nowMs();
        switch (type) {
            case HELLO:
                emit helloChanged();
                setStatus(tr("terhubung: %1 (%2, fw %3)").arg(robotName(), boardName(), fwVersion()));
                break;
            case TELEMETRY:     emit telemetryChanged(); break;
            case SERVO_STATE:   emit servoChanged(); break;
            case GAIT_STATE:    emit gaitChanged(); break;
            case CONFIG:        emit configChanged(); break;
            case MOTION_STATUS: emit motionsChanged(); break;
            case ACK:
                emit ackReceived(cli_.lastAck.ackType, cli_.lastAck.status);
                if (cli_.lastAck.status != OK) {
                    static const char* names[] = {"OK", "BAD_LEN", "UNSUPPORTED", "REJECTED_STATE", "BUSY", "BAD_VALUE"};
                    appendLog(QStringLiteral("[studio] ACK 0x%1: %2").arg(cli_.lastAck.ackType, 2, 16, QLatin1Char('0'))
                                  .arg(QString::fromLatin1(cli_.lastAck.status < 6 ? names[cli_.lastAck.status] : "?")));
                }
                break;
            default: break;
        }
    };
}

QStringList RobotLink::serialPorts() const {
    QStringList out;
    for (const QSerialPortInfo& p : QSerialPortInfo::availablePorts()) {
        QString d = p.description();
        out << (d.isEmpty() ? p.portName() : p.portName() + QStringLiteral(" - ") + d);
    }
    return out;
}

void RobotLink::attached(Kind k, const QString& label) {
    kind_ = k; label_ = label;
    cli_.reset();
    lastRxMs_ = nowMs();
    LapClient::Rates r;
    cli_.begin(r);
    pingTimer_.start();
    emit connectedChanged();
}

void RobotLink::connectTcp(const QString& host, int port) {
    disconnectLink();
    tcp_ = new QTcpSocket(this);
    tcp_->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    connect(tcp_, &QTcpSocket::readyRead, this, [this] { onBytes(tcp_->readAll()); });
    connect(tcp_, &QTcpSocket::connected, this, [this, host, port] {
        attached(Tcp, QStringLiteral("TCP %1:%2").arg(host).arg(port));
    });
    connect(tcp_, &QTcpSocket::errorOccurred, this, [this](QAbstractSocket::SocketError) {
        setStatus(tr("TCP: %1").arg(tcp_ ? tcp_->errorString() : QString()));
        disconnectLink();
    });
    setStatus(tr("menghubungkan %1:%2 ...").arg(host).arg(port));
    tcp_->connectToHost(host, (quint16)port);
}

void RobotLink::connectSerial(const QString& portLabel, int baud) {
    disconnectLink();
    const QString port = portLabel.section(QLatin1Char(' '), 0, 0);
    serial_ = new QSerialPort(port, this);
    serial_->setBaudRate(baud);
    if (!serial_->open(QIODevice::ReadWrite)) {
        setStatus(tr("serial %1: %2").arg(port, serial_->errorString()));
        serial_->deleteLater(); serial_ = nullptr;
        return;
    }
    serial_->setDataTerminalReady(true);
    connect(serial_, &QSerialPort::readyRead, this, [this] { onBytes(serial_->readAll()); });
    connect(serial_, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError e) {
        if (e == QSerialPort::ResourceError) { setStatus(tr("serial terputus")); disconnectLink(); }
    });
    attached(Serial, QStringLiteral("USB %1").arg(port));
    setStatus(tr("serial %1 terbuka, menunggu HELLO ...").arg(port));
}

void RobotLink::connectSim() {
    disconnectLink();
    sim_.reset(new SimRobot);
    sim_->start([this](const uint8_t* d, size_t n) { onBytes(QByteArray((const char*)d, (int)n)); });
    attached(Sim, tr("Simulator (di dalam studio)"));
    simTimer_.start();
}

void RobotLink::disconnectLink() {
    const bool was = kind_ != None;
    kind_ = None;
    simTimer_.stop(); pingTimer_.stop();
    if (tcp_) { tcp_->disconnect(this); tcp_->abort(); tcp_->deleteLater(); tcp_ = nullptr; }
    if (serial_) { serial_->disconnect(this); serial_->close(); serial_->deleteLater(); serial_ = nullptr; }
    sim_.reset();
    simRx_.clear();
    cli_.reset();
    if (was) {
        label_.clear();
        setStatus(tr("terputus"));
        emit connectedChanged(); emit helloChanged(); emit telemetryChanged();
    }
}

void RobotLink::onBytes(const QByteArray& b) { cli_.feed((const uint8_t*)b.constData(), (size_t)b.size()); }

bool RobotLink::alive() const { return kind_ != None && cli_.haveTele && nowMs() - lastRxMs_ < 3000; }

void RobotLink::setStatus(const QString& s) {
    if (s == status_) return;
    status_ = s;
    emit statusChanged();
}

void RobotLink::appendLog(const QString& line) {
    logPartial_ += line;
    int nl;
    while ((nl = logPartial_.indexOf(QLatin1Char('\n'))) >= 0) {
        QString l = logPartial_.left(nl).trimmed();
        logPartial_.remove(0, nl + 1);
        if (!l.isEmpty()) log_ << QTime::currentTime().toString(QStringLiteral("HH:mm:ss.zzz  ")) + l;
    }
    if (logPartial_.size() > 400) { log_ << logPartial_; logPartial_.clear(); }
    while (log_.size() > 600) log_.removeFirst();
    emit logChanged();
}
void RobotLink::clearLog() { log_.clear(); emit logChanged(); }

// ---- perintah ---------------------------------------------------------------
void RobotLink::sendCmd(int id) { if (connected()) cli_.cmd((uint8_t)id); }
void RobotLink::setParam(int id, double v) { if (connected()) cli_.setParam((uint8_t)id, (float)v); }
void RobotLink::setTrim(int b, int s, int v) { if (connected()) cli_.setTrim((uint8_t)b, (uint8_t)s, (int16_t)v); }
void RobotLink::gaitIntent(double vx, double yaw, int ttl) { if (connected()) cli_.gaitIntent((float)vx, (float)yaw, (uint16_t)qBound(0, ttl, 1000)); }
void RobotLink::poseTarget(const QVariantList& src, uint mask, int ttl) {
    if (!connected() || src.size() != 2 * SERVO_SLOTS) return;
    int16_t j[2][SERVO_SLOTS];
    for (int i = 0; i < 2 * SERVO_SLOTS; ++i) j[i / SERVO_SLOTS][i % SERVO_SLOTS] = (int16_t)src[i].toInt();
    cli_.poseTarget(j, mask, (uint16_t)qBound(0, ttl, 1000));
}
bool RobotLink::uploadMotion(const QByteArray& lam, int slot, bool save) {
    if (!connected() || slot < 0 || slot >= MOTION_SLOTS) return false;
    return cli_.motionUpload((uint8_t)slot, (const uint8_t*)lam.constData(), (size_t)lam.size(), save);
}
void RobotLink::playMotion(int slot) { if (connected()) cli_.motionPlay((uint8_t)slot, true); }
void RobotLink::stopMotion(int slot) { if (connected()) cli_.motionPlay((uint8_t)slot, false); }
void RobotLink::simImu(double r, double p) { if (connected()) cli_.simImu((float)r, (float)p); }

// ---- state ------------------------------------------------------------------
QString RobotLink::robotName() const { return cli_.haveHello ? QString::fromLatin1(cli_.hello.name) : QString(); }
QString RobotLink::boardName() const {
    if (!cli_.haveHello) return {};
    switch (cli_.hello.board) {
        case lap::BOARD_ESP32S3: return QStringLiteral("ESP32-S3");
        case lap::BOARD_ESP32P4: return QStringLiteral("ESP32-P4");
        case lap::BOARD_STM32H562: return QStringLiteral("STM32H562");
        case lap::BOARD_SIM: return QStringLiteral("simulator");
        default: return QStringLiteral("?");
    }
}
QString RobotLink::fwVersion() const {
    return cli_.haveHello ? QStringLiteral("%1.%2.%3").arg(cli_.hello.fwMajor).arg(cli_.hello.fwMinor).arg(cli_.hello.fwPatch) : QString();
}
QString RobotLink::modeName() const {
    if (!cli_.haveTele) return connected() ? tr("menunggu") : tr("offline");
    return QString::fromLatin1(la::modeShortName((Mode)cli_.tele.mode));
}

QVariantList RobotLink::cmdSrc() const {
    QVariantList v;
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
        v << (int)rawToSrc(b, s, cli_.servo.cmd[b][s], cli_.servo.trim[b][s]);
    return v;
}
QVariantList RobotLink::fbSrc() const {
    QVariantList v;
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
        v << (int)rawToSrc(b, s, cli_.servo.fb[b][s], cli_.servo.trim[b][s]);
    return v;
}
QVariantList RobotLink::trim() const {
    QVariantList v;
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) v << (int)cli_.servo.trim[b][s];
    return v;
}
QVariantList RobotLink::valid() const {
    QVariantList v;
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) v << (cli_.haveServo && cli_.servo.valid[b][s] != 0);
    return v;
}
QVariantList RobotLink::kinSrc() const {
    QVariantList v;
    if (!cli_.haveGait) return v;
    int16_t j[2][SERVO_SLOTS];
    cli_.gaitSrc(j);
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) v << (int)j[b][s];
    return v;
}
QVariantList RobotLink::params() const {
    QVariantList v;
    if (!cli_.haveConfig) return v;
    for (int i = 0; i < (int)ParamId::COUNT_; ++i) v << (double)cli_.config.v[i];
    return v;
}
QVariantList RobotLink::motionSlots() const {
    QVariantList v;
    static const char* st[] = {"kosong", "menulis", "siap", "diputar"};
    for (int i = 0; i < MOTION_SLOTS; ++i) {
        QVariantMap m;
        m[QStringLiteral("state")] = cli_.motions.state[i];
        m[QStringLiteral("stateName")] = QString::fromLatin1(cli_.motions.state[i] < 4 ? st[cli_.motions.state[i]] : "?");
        m[QStringLiteral("frames")] = cli_.motions.nFrames[i];
        m[QStringLiteral("name")] = QString::fromLatin1(cli_.motions.name[i]);
        v << m;
    }
    return v;
}
