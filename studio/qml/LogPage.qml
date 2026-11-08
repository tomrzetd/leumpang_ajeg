import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

Item {
    id: page
    property var studio

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        RowLayout {
            Label { text: robot.logLines.length + " baris"; opacity: 0.7 }
            Item { Layout.fillWidth: true }
            CheckBox { id: follow; text: "ikuti"; checked: true }
            Button { text: "Bersihkan"; onClicked: robot.clearLog() }
        }
        ListView {
            id: list
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            model: robot.logLines
            ScrollBar.vertical: ScrollBar {}
            onCountChanged: if (follow.checked) positionViewAtEnd()
            delegate: Label {
                required property string modelData
                width: ListView.view.width
                text: modelData
                font.family: "monospace"
                font.pixelSize: 12
                wrapMode: Text.WrapAnywhere
                color: modelData.indexOf("FALLEN") >= 0 || modelData.indexOf("ditolak") >= 0 || modelData.indexOf("ACK") >= 0 ? "#f0ad4e" : palette.text
            }
        }
    }
}
