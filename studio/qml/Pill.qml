import QtQuick
import QtQuick.Controls

Rectangle {
    id: pill
    property alias text: lbl.text
    radius: height / 2
    implicitHeight: 22
    implicitWidth: lbl.implicitWidth + 20
    color: "#5a6370"
    Label {
        id: lbl
        anchors.centerIn: parent
        color: "white"
        font.bold: true
        font.pixelSize: 12
    }
}
