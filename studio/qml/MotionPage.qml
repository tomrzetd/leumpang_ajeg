import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Dialogs
import QtQuick.Layouts

// Editor gerak: keyframe, pratinjau twin (identik dengan pemutar firmware), berkas, unggah ke robot.
Item {
    id: page
    property var studio
    property int selected: -1

    FileDialog { id: openJson; nameFilters: ["Gerak JSON (*.json)"]; onAccepted: motion.loadJson(selectedFile) }
    FileDialog { id: saveJson; fileMode: FileDialog.SaveFile; defaultSuffix: "json"; nameFilters: ["Gerak JSON (*.json)"]; onAccepted: motion.saveJson(selectedFile) }
    FileDialog { id: openLam; nameFilters: ["Template LAM1 (*.lam)"]; onAccepted: motion.importLam(selectedFile) }
    FileDialog { id: saveLam; fileMode: FileDialog.SaveFile; defaultSuffix: "lam"; nameFilters: ["Template LAM1 (*.lam)"]; onAccepted: motion.exportLam(selectedFile) }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 6

        RowLayout {
            Layout.fillWidth: true
            TextField {
                Layout.preferredWidth: 180
                placeholderText: "nama (maks 16)"
                maximumLength: 16
                text: motion.name
                onEditingFinished: motion.name = text
            }
            CheckBox { text: "Kembali ke HOME"; checked: motion.returnHome; onToggled: motion.returnHome = checked }
            Item { Layout.fillWidth: true }
            Label { text: motion.frames.length + "/64 keyframe  ·  " + (motion.totalMs / 1000).toFixed(2) + " s" }
        }
        Flow {
            Layout.fillWidth: true
            spacing: 6
            Button {
                text: "Berkas \u25be"
                onClicked: fileMenu.open()
                Menu {
                    id: fileMenu
                    MenuItem { text: "Baru"; onTriggered: { motion.clear(); page.selected = -1 } }
                    MenuItem { text: "Buka JSON..."; onTriggered: openJson.open() }
                    MenuItem { text: "Simpan JSON..."; onTriggered: saveJson.open() }
                    MenuSeparator {}
                    MenuItem { text: "Impor .lam..."; onTriggered: openLam.open() }
                    MenuItem { text: "Ekspor .lam..."; onTriggered: saveLam.open() }
                    MenuSeparator {}
                    MenuItem { text: "Contoh: lambai"; onTriggered: motion.loadExample() }
                }
            }
            Button { text: "+ pose kerja"; highlighted: true; onClicked: motion.addFrame(page.studio.workPose, 600, page.selected >= 0 ? page.selected + 1 : -1) }
            Button {
                text: "+ robot aktual"
                enabled: robot.fbSrc.length === 22
                onClicked: motion.addFrame(robot.fbSrc, 600, page.selected >= 0 ? page.selected + 1 : -1)
                ToolTip.visible: hovered
                ToolTip.text: "Teach-by-hand: lepas servo, bentuk pose dengan tangan, lalu tambahkan"
            }
            Button {
                text: "Timpa frame"
                enabled: page.selected >= 0
                onClicked: motion.setFramePose(page.selected, page.studio.workPose)
                ToolTip.visible: hovered
                ToolTip.text: "Ganti pose frame terpilih dengan pose kerja (tab Servo)"
            }
            Button {
                text: "Frame \u2192 kerja"
                enabled: page.selected >= 0
                onClicked: page.studio.workPose = motion.framePose(page.selected)
                ToolTip.visible: hovered
                ToolTip.text: "Salin pose frame terpilih ke pose kerja untuk diedit di tab Servo"
            }
        }

        // linimasa
        RowLayout {
            Layout.fillWidth: true
            Button {
                text: motion.previewing ? "⏸ Jeda" : "▶ Pratinjau"
                enabled: motion.frames.length > 0
                onClicked: motion.preview(!motion.previewing)
            }
            Slider {
                id: timeline
                Layout.fillWidth: true
                from: 0
                to: Math.max(1, motion.totalMs)
                onMoved: motion.seek(value)
                Binding on value { when: !timeline.pressed; value: motion.previewMs; restoreMode: Binding.RestoreNone }
            }
            Label { text: (motion.previewMs / 1000).toFixed(2) + " s"; Layout.preferredWidth: 60 }
            Button { text: "Selesai"; flat: true; onClicked: { motion.preview(false); twin.clearPreview("gerak") } }
        }

        ListView {
            id: frames
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            model: motion.frames
            ScrollBar.vertical: ScrollBar {}
            delegate: Rectangle {
                id: fr
                required property var modelData
                required property int index
                width: ListView.view.width
                height: 50
                color: page.selected === index ? "#3a2a1c" : (index % 2 ? "#20252b" : "transparent")
                border.color: page.selected === index ? "#E86A1C" : "transparent"
                MouseArea {
                    anchors.fill: parent
                    onClicked: { page.selected = fr.index; motion.showFrame(fr.index) }
                }
                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 8
                    anchors.rightMargin: 4
                    spacing: 6
                    Label { text: "#" + (fr.index + 1); Layout.preferredWidth: 34; font.bold: true }
                    Label { text: (fr.modelData.startMs / 1000).toFixed(2) + " s"; Layout.preferredWidth: 56; opacity: 0.6 }
                    TextField {
                        text: fr.modelData.ms
                        Layout.preferredWidth: 76
                        horizontalAlignment: Text.AlignRight
                        validator: IntValidator { bottom: 20; top: 10000 }
                        onEditingFinished: if (acceptableInput && Number(text) !== fr.modelData.ms) motion.setFrameMs(fr.index, Number(text))
                    }
                    Label { text: "ms"; opacity: 0.6 }
                    ComboBox {
                        model: ["cosine", "linear", "step"]
                        currentIndex: fr.modelData.interp
                        Layout.preferredWidth: 110
                        onActivated: motion.setFrameInterp(fr.index, currentIndex)
                    }
                    Item { Layout.fillWidth: true }
                    ToolButton { text: "↑"; onClicked: { motion.moveFrame(fr.index, -1); page.selected = Math.max(0, fr.index - 1) } }
                    ToolButton { text: "↓"; onClicked: { motion.moveFrame(fr.index, 1); page.selected = Math.min(motion.frames.length - 1, fr.index + 1) } }
                    ToolButton { text: "⧉"; onClicked: motion.duplicateFrame(fr.index); ToolTip.visible: hovered; ToolTip.text: "Gandakan" }
                    ToolButton { text: "✕"; onClicked: { motion.removeFrame(fr.index); page.selected = -1 } }
                }
            }
            Label {
                anchors.centerIn: parent
                visible: motion.frames.length === 0
                opacity: 0.5
                horizontalAlignment: Text.AlignHCenter
                text: "Belum ada keyframe.\nBentuk pose di tab Servo, rekam dari Kamera,\natau buka contoh 'lambai'."
            }
        }

        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            color: motion.problem.length > 0 ? "#f0ad4e" : "#3fa34d"
            text: motion.frames.length === 0 ? "" : (motion.problem.length > 0 ? "⚠ " + motion.problem : "✓ valid untuk firmware (batas sendi + trim robot)")
        }

        // robot
        GroupBox {
            title: "Robot"
            Layout.fillWidth: true
            ColumnLayout {
                anchors.fill: parent
                RowLayout {
                    Label { text: "slot" }
                    ComboBox { id: slotBox; model: ["0", "1", "2", "3"]; implicitWidth: 80 }
                    CheckBox { id: toSd; text: "simpan ke SD"; checked: true }
                    Button { text: "Unggah"; highlighted: true; enabled: robot.connected && motion.frames.length > 0; onClicked: motion.upload(slotBox.currentIndex, toSd.checked) }
                    Button { text: "▶ Putar"; enabled: robot.connected; onClicked: robot.playMotion(slotBox.currentIndex) }
                    Button { text: "■ Stop"; enabled: robot.connected; onClicked: robot.stopMotion(slotBox.currentIndex) }
                }
                Flow {
                    Layout.fillWidth: true
                    spacing: 6
                    Repeater {
                        model: robot.motionSlots
                        delegate: Pill {
                            required property var modelData
                            required property int index
                            text: index + ": " + (modelData.name.length > 0 ? modelData.name : "-") + " (" + modelData.stateName + ")"
                            color: modelData.state === 3 ? "#9b59b6" : (modelData.state === 2 ? "#2d7dd2" : "#3a414a")
                        }
                    }
                }
                Label { text: motion.message; opacity: 0.8; Layout.fillWidth: true; elide: Text.ElideRight }
            }
        }
    }
}
