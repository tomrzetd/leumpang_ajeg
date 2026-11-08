import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

// Parameter runtime robot (SET_PARAM). Nilai dibaca dari pesan CONFIG.
Item {
    id: page
    property var studio
    readonly property var defs: [
        { id: 0,  name: "Sudut jatuh",          unit: "°" },
        { id: 1,  name: "Sudut re-arm",          unit: "°" },
        { id: 2,  name: "IMU basi setelah",      unit: "ms" },
        { id: 3,  name: "Langkah servo maks",    unit: "hit/siklus" },
        { id: 4,  name: "Pemicu langkah roll",   unit: "°" },
        { id: 5,  name: "Pemicu langkah pitch",  unit: "°" },
        { id: 6,  name: "Angkat kaki maks",      unit: "mm" },
        { id: 7,  name: "Sway dasar",            unit: "mm" },
        { id: 8,  name: "Sway gain",             unit: "" },
        { id: 9,  name: "Tanda roll IMU",        unit: "±1" },
        { id: 10, name: "Tanda pitch IMU",       unit: "±1" },
        { id: 11, name: "Tukar roll/pitch IMU",  unit: "0/1" },
        { id: 12, name: "Gait kontinu aktif",    unit: "0/1" },
        { id: 13, name: "Langkah gait maks",     unit: "mm" },
        { id: 14, name: "Belok gait maks",       unit: "°/langkah" },
        { id: 15, name: "Henti rintangan (HP)",  unit: "mm, 0 = mati" }
    ]

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 4
        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            opacity: 0.75
            text: robot.params.length > 0 ? "Nilai dari robot. Ubah lalu Enter / Set. Firmware menyimpan ke NVS/SD saat robot diam (READY/FALLEN)."
                                          : "Hubungkan robot untuk membaca parameter."
        }
        ListView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            model: page.defs
            ScrollBar.vertical: ScrollBar {}
            delegate: Rectangle {
                id: prow
                required property var modelData
                required property int index
                width: ListView.view.width
                height: 46
                color: index % 2 ? "#20252b" : "transparent"
                readonly property var current: robot.params.length > modelData.id ? robot.params[modelData.id] : undefined
                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 8
                    anchors.rightMargin: 8
                    Label { text: prow.modelData.name; Layout.preferredWidth: 200 }
                    Label {
                        text: prow.current === undefined ? "-" : Number(prow.current).toFixed(2)
                        Layout.preferredWidth: 80
                        horizontalAlignment: Text.AlignRight
                        font.bold: true
                    }
                    Label { text: prow.modelData.unit; Layout.preferredWidth: 90; opacity: 0.6 }
                    TextField {
                        id: edit
                        Layout.fillWidth: true
                        placeholderText: "nilai baru"
                        enabled: robot.connected
                        validator: DoubleValidator {}
                        onAccepted: setBtn.clicked()
                    }
                    Button {
                        id: setBtn
                        text: "Set"
                        enabled: robot.connected && edit.text.length > 0
                        onClicked: { robot.setParam(prow.modelData.id, Number(edit.text.replace(",", "."))); edit.text = "" }
                    }
                }
            }
        }
    }
}
