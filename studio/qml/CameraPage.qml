import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Dialogs
import QtQuick.Layouts
import QtMultimedia

// Pindai pose dari kamera PC: MoveNet -> retarget lengan + kepala -> twin / robot (live) / rekaman gerak.
Item {
    id: page
    property var studio
    readonly property var bones: [[5, 6], [5, 7], [7, 9], [6, 8], [8, 10], [5, 11], [6, 12], [11, 12],
                                  [11, 13], [13, 15], [12, 14], [14, 16], [0, 1], [0, 2], [1, 3], [2, 4]]

    FileDialog { id: modelDlg; nameFilters: ["Model ONNX (*.onnx)"]; onAccepted: poseCam.loadModel(selectedFile) }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 6

        RowLayout {
            Layout.fillWidth: true
            ComboBox {
                id: camBox
                Layout.fillWidth: true
                model: poseCam.devices
                displayText: count === 0 ? "tidak ada kamera" : currentText
            }
            ToolButton { text: "↻"; onClicked: poseCam.refreshDevices() }
            Button {
                text: poseCam.running ? "Stop kamera" : "Mulai kamera"
                highlighted: !poseCam.running
                enabled: camBox.count > 0 || poseCam.running
                onClicked: poseCam.running ? poseCam.stop() : poseCam.start(camBox.currentIndex)
            }
        }
        RowLayout {
            Layout.fillWidth: true
            Button { text: "Pilih model ONNX"; enabled: poseCam.available; onClicked: modelDlg.open() }
            Label {
                Layout.fillWidth: true
                elide: Text.ElideRight
                text: poseCam.modelLoaded ? poseCam.modelInfo : (poseCam.available ? "belum ada model (MoveNet SinglePose .onnx)" : "build tanpa ONNX Runtime")
                opacity: 0.8
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#111418"
            radius: 4
            VideoOutput {
                id: video
                anchors.fill: parent
                fillMode: VideoOutput.PreserveAspectFit
                transform: Scale { origin.x: video.width / 2; xScale: poseCam.mirror ? -1 : 1 }
                Component.onCompleted: poseCam.videoSink = video.videoSink
            }
            Canvas {
                id: overlay
                anchors.fill: parent
                transform: Scale { origin.x: overlay.width / 2; xScale: poseCam.mirror ? -1 : 1 }
                onPaint: {
                    const ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)
                    const kp = poseCam.keypoints
                    if (!kp || kp.length !== 17) return
                    const r = video.contentRect
                    const px = (k) => r.x + kp[k].x * r.width
                    const py = (k) => r.y + kp[k].y * r.height
                    ctx.lineWidth = 3
                    ctx.strokeStyle = "#E86A1C"
                    for (const b of page.bones) {
                        if (kp[b[0]].score < 0.3 || kp[b[1]].score < 0.3) continue
                        ctx.beginPath(); ctx.moveTo(px(b[0]), py(b[0])); ctx.lineTo(px(b[1]), py(b[1])); ctx.stroke()
                    }
                    ctx.fillStyle = "#4fc3f7"
                    for (let i = 0; i < 17; ++i) {
                        if (kp[i].score < 0.3) continue
                        ctx.beginPath(); ctx.arc(px(i), py(i), 4, 0, 2 * Math.PI); ctx.fill()
                    }
                }
                Connections { target: poseCam; function onPoseChanged() { overlay.requestPaint() } }
            }
            Label {
                anchors.centerIn: parent
                visible: !poseCam.running
                opacity: 0.5
                horizontalAlignment: Text.AlignHCenter
                text: "Kamera mati.\nBerdiri menghadap kamera, tubuh atas terlihat penuh."
            }
            Label {
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.margins: 8
                visible: poseCam.running
                text: poseCam.fps.toFixed(0) + " fps kamera  ·  inferensi " + poseCam.inferMs.toFixed(1) + " ms"
                color: "#cfd8e3"
            }
        }

        Flow {
            Layout.fillWidth: true
            spacing: 6
            Switch { text: "Cermin"; checked: poseCam.mirror; onToggled: poseCam.mirror = checked }
            Switch { text: "Gerakkan twin"; checked: poseCam.driveTwin; onToggled: poseCam.driveTwin = checked }
            Switch {
                text: "Live ke robot"
                checked: poseCam.streamToRobot
                enabled: robot.connected && poseCam.modelLoaded
                onToggled: poseCam.streamToRobot = checked
                ToolTip.visible: hovered
                ToolTip.text: "Lengan + kepala robot mengikuti Anda (mode 820, hanya dari READY). Kaki tidak pernah dikendalikan kamera."
            }
            Button { text: "Kalibrasi T-pose"; enabled: poseCam.modelLoaded; onClicked: poseCam.calibrateTPose() }
        }
        RowLayout {
            Layout.fillWidth: true
            Button {
                text: motion.recording ? "■ Stop rekam" : "● Rekam ke editor gerak"
                enabled: poseCam.modelLoaded || motion.recording
                Material.background: motion.recording ? "#8b2e2b" : "#3a414a"
                onClicked: motion.recording ? motion.stopRecording(toleransi.value) : motion.startRecording()
            }
            Label { text: "toleransi" }
            SpinBox { id: toleransi; from: 5; to: 300; value: 40; stepSize: 5; editable: true }
            Label { text: motion.recording ? motion.recordSamples + " sampel" : ""; Layout.fillWidth: true }
        }
        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            text: poseCam.status + (motion.message.length > 0 && !motion.recording ? "  ·  " + motion.message : "")
            opacity: 0.8
        }
    }
}
