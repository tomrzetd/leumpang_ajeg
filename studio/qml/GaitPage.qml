import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

// Jalan kontinu (mode 800, eksperimental) + simulasi kemiringan IMU untuk simulator.
Item {
    id: page
    property var studio
    readonly property bool gaitOn: robot.params.length > 12 && robot.params[12] > 0.5

    Timer {
        id: walkTimer
        interval: 200
        repeat: true
        running: walkBtn.pressed && robot.connected
        triggeredOnStart: true
        onTriggered: robot.gaitIntent(vx.value, yaw.value, 500)
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 10

        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            text: "Gait kontinu (GAIT_INTENT): robot harus di BALANCE (tombol Seimbang). Tahan tombol Jalan; lepas = robot menutup langkah lalu berdiri. Uji dulu di simulator, lalu di robot dengan tali pengaman."
            opacity: 0.8
        }
        RowLayout {
            Switch {
                text: "gaitEnable"
                checked: page.gaitOn
                enabled: robot.connected
                onToggled: robot.setParam(12, checked ? 1 : 0)
            }
            Label { text: page.gaitOn ? "aktif" : "mati (default firmware)"; color: page.gaitOn ? "#3fa34d" : "#f0ad4e" }
        }
        GridLayout {
            columns: 3
            Layout.fillWidth: true
            Label { text: "maju / mundur" }
            Slider { id: vx; from: -15; to: 15; value: 8; stepSize: 0.5; Layout.fillWidth: true }
            Label { text: vx.value.toFixed(1) + " mm/langkah"; Layout.preferredWidth: 120 }
            Label { text: "belok" }
            Slider { id: yaw; from: -8; to: 8; value: 0; stepSize: 0.5; Layout.fillWidth: true }
            Label { text: yaw.value.toFixed(1) + " °/langkah"; Layout.preferredWidth: 120 }
        }
        RowLayout {
            Button { text: "Lurus"; onClicked: yaw.value = 0 }
            Button { text: "Diam"; onClicked: { vx.value = 0; yaw.value = 0 } }
            Item { Layout.fillWidth: true }
            Button {
                id: walkBtn
                text: pressed ? "BERJALAN..." : "Tahan untuk jalan"
                highlighted: true
                enabled: robot.connected && page.gaitOn
                implicitWidth: 220
                implicitHeight: 64
                font.pixelSize: 16
            }
        }
        Label {
            text: "mode " + robot.mode + "  ·  " + ((robot.flags & 32) !== 0 ? "niat aktif" : "tanpa niat")
            opacity: 0.8
        }

        GroupBox {
            title: "Simulator: kemiringan IMU"
            visible: robot.isSim
            Layout.fillWidth: true
            GridLayout {
                anchors.fill: parent
                columns: 3
                Label { text: "roll" }
                Slider { id: sr; from: -30; to: 30; value: 0; Layout.fillWidth: true; onMoved: robot.simImu(sr.value, sp.value) }
                Label { text: sr.value.toFixed(1) + "°"; Layout.preferredWidth: 60 }
                Label { text: "pitch" }
                Slider { id: sp; from: -30; to: 30; value: 0; Layout.fillWidth: true; onMoved: robot.simImu(sr.value, sp.value) }
                Label { text: sp.value.toFixed(1) + "°"; Layout.preferredWidth: 60 }
                Button { text: "Tegak"; onClicked: { sr.value = 0; sp.value = 0; robot.simImu(0, 0) } }
                Button { text: "Dorong (pitch 25°)"; onClicked: { sp.value = 25; robot.simImu(sr.value, 25) } }
            }
        }
        Item { Layout.fillHeight: true }
    }
}
