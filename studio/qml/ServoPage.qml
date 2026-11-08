import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

// Kontrol per servo: pose kerja (slider), pratinjau twin, live ke robot (mode 820), trim.
Item {
    id: page
    property var studio
    property bool previewOn: false
    property bool liveOn: false

    function setJoint(i, v) {
        const w = studio.workPose.slice()
        w[i] = Math.round(v)
        studio.workPose = w
    }
    function liveMask() { return maskBox.currentIndex === 0 ? studio.maskArmsHead : studio.maskAll }

    Connections {
        target: page.studio
        function onWorkPoseChanged() { if (page.previewOn) twin.setPreview(page.studio.workPose, "servo") }
    }
    Timer {
        interval: 50
        repeat: true
        running: page.liveOn && robot.connected
        onTriggered: robot.poseTarget(page.studio.workPose, page.liveMask(), 300)
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 6

        Flow {
            Layout.fillWidth: true
            spacing: 6
            Button {
                text: "Ambil dari robot"
                enabled: robot.fbSrc.length === 22
                onClicked: page.studio.workPose = robot.fbSrc
                ToolTip.visible: hovered
                ToolTip.text: "Pose kerja = posisi servo aktual (bisa dipakai dengan servo dilepas: 'teach by hand')"
            }
            Button { text: "HOME"; onClicked: page.studio.workPose = twin.homeSrc }
            Button { text: "Cermin kiri↔kanan"; onClicked: {
                const w = page.studio.workPose.slice()
                for (let s = 1; s <= 10; ++s) { const r = w[s]; w[s] = w[11 + s]; w[11 + s] = r }
                page.studio.workPose = w
            } }
            Button {
                text: "+ Keyframe"
                highlighted: true
                onClicked: motion.addFrame(page.studio.workPose, 600, -1)
                ToolTip.visible: hovered
                ToolTip.text: "Tambahkan pose kerja ke editor gerak (600 ms)"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            Switch {
                text: "Pratinjau di twin"
                checked: page.previewOn
                onToggled: {
                    page.previewOn = checked
                    if (checked) twin.setPreview(page.studio.workPose, "servo"); else twin.clearPreview("servo")
                }
            }
            Switch {
                id: liveSwitch
                text: "Live ke robot"
                checked: page.liveOn
                enabled: robot.connected && (robot.caps & 128) !== 0
                onToggled: {
                    page.liveOn = checked
                    if (!checked) robot.poseTarget(page.studio.workPose, 0, 0)     // selesai -> HOME
                }
            }
            ComboBox {
                id: maskBox
                model: ["lengan + kepala", "SEMUA sendi (kaki ikut!)"]
                implicitWidth: 210
            }
            Item { Layout.fillWidth: true }
        }
        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            font.pixelSize: 12
            opacity: 0.7
            text: page.liveOn ? "Live aktif: 20 Hz, ttl 300 ms. Robot hanya menerima dari READY; batas sendi & kecepatan (servoStep) tetap dijaga firmware. Matikan = kembali HOME."
                              : "Geser slider untuk membentuk pose kerja. Arahkan kursor ke baris untuk menyorot sendi di twin."
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.leftMargin: 4
            Label { text: "sendi"; Layout.preferredWidth: 140; opacity: 0.6 }
            Label { text: "pose kerja"; Layout.fillWidth: true; opacity: 0.6 }
            Label { text: "kerja"; Layout.preferredWidth: 52; horizontalAlignment: Text.AlignRight; opacity: 0.6 }
            Label { text: "perintah"; Layout.preferredWidth: 58; horizontalAlignment: Text.AlignRight; opacity: 0.6 }
            Label { text: "aktual"; Layout.preferredWidth: 58; horizontalAlignment: Text.AlignRight; opacity: 0.6 }
            Label { text: "trim"; Layout.preferredWidth: 112; horizontalAlignment: Text.AlignHCenter; opacity: 0.6 }
        }

        ListView {
            id: list
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            model: twin.jointInfo
            ScrollBar.vertical: ScrollBar {}
            delegate: Rectangle {
                id: row
                required property var modelData
                required property int index
                width: ListView.view.width
                height: 44
                color: hover.hovered ? "#2c333b" : (index % 2 ? "#20252b" : "transparent")
                readonly property int cmdV: robot.kinSrc.length === 22 ? robot.kinSrc[index] : 0
                readonly property int fbV: robot.fbSrc.length === 22 ? robot.fbSrc[index] : 0
                readonly property bool ok: robot.valid.length === 22 && robot.valid[index]
                HoverHandler {
                    id: hover
                    onHoveredChanged: twin.highlightLink = hovered ? row.modelData.link : (twin.highlightLink === row.modelData.link ? -1 : twin.highlightLink)
                }
                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 4
                    anchors.rightMargin: 4
                    spacing: 6
                    Rectangle { width: 8; height: 8; radius: 4; color: !robot.connected ? "#5a6370" : (row.ok ? "#3fa34d" : "#d9534f") }
                    Label { text: row.modelData.name; Layout.preferredWidth: 124; elide: Text.ElideRight; font.bold: true; font.pixelSize: 13 }
                    Slider {
                        id: slider
                        Layout.fillWidth: true
                        from: row.modelData.min
                        to: row.modelData.max
                        stepSize: 10
                        onMoved: page.setJoint(row.index, value)
                        Binding on value {
                            when: !slider.pressed
                            value: page.studio.workPose[row.index]
                            restoreMode: Binding.RestoreNone
                        }
                    }
                    Label { text: page.studio.workPose[row.index]; Layout.preferredWidth: 52; horizontalAlignment: Text.AlignRight }
                    Label { text: robot.connected ? row.cmdV : "-"; Layout.preferredWidth: 58; horizontalAlignment: Text.AlignRight; opacity: 0.8 }
                    Label {
                        text: robot.connected ? row.fbV : "-"
                        Layout.preferredWidth: 58
                        horizontalAlignment: Text.AlignRight
                        color: robot.connected && Math.abs(row.fbV - row.cmdV) > 150 ? "#f0ad4e" : "#d8dde3"
                    }
                    RowLayout {
                        Layout.preferredWidth: 112
                        spacing: 0
                        readonly property int trimV: robot.trim.length === 22 ? robot.trim[row.index] : 0
                        ToolButton {
                            text: "\u2212"
                            enabled: robot.connected
                            implicitWidth: 34
                            onClicked: robot.setTrim(row.modelData.bus, row.modelData.slot, Math.max(-1000, parent.trimV - 5))
                            autoRepeat: true
                        }
                        Label {
                            text: robot.connected ? parent.trimV : "-"
                            Layout.preferredWidth: 40
                            horizontalAlignment: Text.AlignHCenter
                            color: parent.trimV !== 0 ? "#E86A1C" : "#d8dde3"
                        }
                        ToolButton {
                            text: "+"
                            enabled: robot.connected
                            implicitWidth: 34
                            onClicked: robot.setTrim(row.modelData.bus, row.modelData.slot, Math.min(1000, parent.trimV + 5))
                            autoRepeat: true
                        }
                    }
                }
            }
        }
    }
}
