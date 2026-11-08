import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

ApplicationWindow {
    id: win
    width: 1600
    height: 920
    minimumWidth: 1100
    minimumHeight: 680
    visible: true
    title: "Leumpang Studio" + (robot.connected ? "  -  " + robot.linkLabel : "")

    Material.theme: Material.Dark
    Material.accent: "#E86A1C"
    Material.primary: "#262b31"
    Material.background: "#1d2126"

    property alias currentTab: tabs.currentIndex
    property var workPose: twin.homeSrc          // pose kerja bersama (panel servo, editor gerak)
    readonly property int maskArmsHead: (0x1E) | (0x1E << 11) | (1 << 11)
    readonly property int maskAll: (1 << 22) - 1

    function modeColor(m) {
        if (m === 705) return "#d9534f"
        if (m === 720) return "#3fa34d"
        if (m === 730) return "#2d7dd2"
        if (m >= 740 && m <= 800) return "#e8a21c"
        if (m === 810) return "#9b59b6"
        if (m === 820) return "#17a2b8"
        return "#5a6370"
    }

    header: ToolBar {
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10
            anchors.rightMargin: 10
            spacing: 8
            Label { text: "Leumpang Studio"; font.bold: true; font.pixelSize: 17; color: "#E86A1C" }
            ToolSeparator {}
            ComboBox {
                id: linkKind
                model: ["Simulator", "Wi-Fi (TCP)", "USB serial"]
                implicitWidth: 150
                enabled: !robot.connected
                onActivated: if (currentIndex === 2) portBox.model = robot.serialPorts()
            }
            TextField {
                id: hostField
                visible: linkKind.currentIndex === 1
                text: "192.168.4.1"
                implicitWidth: 140
                enabled: !robot.connected
                placeholderText: "IP robot"
            }
            ComboBox {
                id: portBox
                visible: linkKind.currentIndex === 2
                implicitWidth: 230
                enabled: !robot.connected
                model: []
                Component.onCompleted: model = robot.serialPorts()
            }
            ToolButton {
                visible: linkKind.currentIndex === 2 && !robot.connected
                text: "↻"
                onClicked: portBox.model = robot.serialPorts()
                ToolTip.visible: hovered
                ToolTip.text: "Segarkan daftar port"
            }
            Button {
                text: robot.connected ? "Putuskan" : "Hubungkan"
                highlighted: !robot.connected
                onClicked: {
                    if (robot.connected) { robot.disconnectLink(); return }
                    if (linkKind.currentIndex === 0) robot.connectSim()
                    else if (linkKind.currentIndex === 1) robot.connectTcp(hostField.text, 5760)
                    else if (portBox.currentText.length > 0) robot.connectSerial(portBox.currentText, 115200)
                }
            }
            ToolSeparator {}
            Button { text: "Reset / HOME"; enabled: robot.connected; onClicked: robot.sendCmd(0) }
            Button { text: "Seimbang (GO)"; enabled: robot.connected; onClicked: robot.sendCmd(1) }
            Button { text: "Kalibrasi IMU"; enabled: robot.connected; onClicked: robot.sendCmd(3) }
            Button {
                text: "LEPAS SERVO"
                enabled: robot.connected
                Material.background: "#8b2e2b"
                onClicked: robot.sendCmd(2)
                ToolTip.visible: hovered
                ToolTip.text: "STOP: semua servo free (robot akan jatuh bila berdiri)"
            }
            Item { Layout.fillWidth: true }
            Label { text: robot.status; elide: Text.ElideRight; Layout.maximumWidth: 420; opacity: 0.8 }
        }
    }

    footer: ToolBar {
        implicitHeight: 34
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10
            anchors.rightMargin: 10
            spacing: 14
            Pill { text: robot.modeName + (robot.mode > 0 ? "  " + robot.mode : ""); color: win.modeColor(robot.mode) }
            Label { text: "roll " + robot.roll.toFixed(1) + "°   pitch " + robot.pitch.toFixed(1) + "°" }
            Label { text: "IMU " + robot.imuHz + " Hz" }
            Label { text: "loop " + robot.loopAvgUs + " / " + robot.loopMaxUs + " µs   ICS " + robot.servoPhaseUs + " µs" }
            Label { text: "servo OK " + robot.fbOk + "/22"; color: robot.connected && robot.fbOk < 22 ? "#f0ad4e" : "#d8dde3" }
            Label { text: "autoH " + robot.autoH.toFixed(1) + " mm" }
            Item { Layout.fillWidth: true }
            Label { text: robot.connected ? (robot.robotName + "  " + robot.boardName + "  fw " + robot.fwVersion) : "offline"; opacity: 0.7 }
            Rectangle { width: 10; height: 10; radius: 5; color: robot.alive ? "#3fa34d" : (robot.connected ? "#f0ad4e" : "#5a6370") }
        }
    }

    SplitView {
        anchors.fill: parent
        orientation: Qt.Horizontal

        Twin3D {
            SplitView.fillWidth: true
            SplitView.minimumWidth: 420
        }

        Pane {
            SplitView.preferredWidth: 700
            SplitView.minimumWidth: 520
            padding: 0
            ColumnLayout {
                anchors.fill: parent
                spacing: 0
                TabBar {
                    id: tabs
                    Layout.fillWidth: true
                    TabButton { text: "Servo" }
                    TabButton { text: "Gerak" }
                    TabButton { text: "Kamera" }
                    TabButton { text: "Jalan" }
                    TabButton { text: "Parameter" }
                    TabButton { text: "Log" }
                }
                StackLayout {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    currentIndex: tabs.currentIndex
                    ServoPage { studio: win }
                    MotionPage { studio: win }
                    CameraPage { studio: win }
                    GaitPage { studio: win }
                    ParamPage { studio: win }
                    LogPage { studio: win }
                }
            }
        }
    }
}
