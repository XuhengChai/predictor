import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 2.3

Switch {
    id: refreshSwitch
    text: qsTr("Refresh")
    checked: true
    font.pointSize: 14
    font.family: "Arial"
    property var offText:"Suspend"
    contentItem: Text {
        width: 140
        height: 20
        color: refreshSwitch.checked ? "#000000":"#888888"
        text: refreshSwitch.checked ? refreshSwitch.text: refreshSwitch.offText
        horizontalAlignment: Text.AlignHCenter
        font: refreshSwitch.font
        verticalAlignment: Text.AlignVCenter
        leftPadding: refreshSwitch.indicator.width + refreshSwitch.spacing
    }
}
