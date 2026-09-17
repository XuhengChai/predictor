import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.3

Window {
    id: dataStateTable
    visible: true
//    anchors.fill: parent
    height: 200
    width:400
    property int dataStatus: 7
    property var statusList: ["init.png","ready.png","ready.png","ready.png","error.png", "error.png", "error.png","error.png"]
    function updateSysStatus() {
        canvasSys.requestPaint()
    }
    Row {
        id: status
        visible: true
        width: dataStateTable.width
        height: dataStateTable.height
        //        anchors.fill: parent
        spacing: 20
        Column {
            id: sysStatus
            spacing: 0
            height: parent.height
            width: (status.width - status.spacing) *0.3
            Label {
                id: label
                width: parent.width
                height: 0.2 * parent.height
                text: qsTr("System")
                font.pointSize: 10
                //                font.family: "Times New Roman"
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
            Canvas {
                id: canvasSys
                width: parent.width
                height: 0.8 * parent.height
                visible: true
                function drawImage(imgPath) {
                    var ctx = getContext("2d")
                    var ox = width/2 - 50
                    var oy = height/2 - 50
                    var h = Math.min(100, height)
                    var w = Math.min(100, width)
                    ctx.reset()
                    ctx.clearRect(ox, oy, w, h)
                    ctx.drawImage(imgPath, ox, oy, w, h)
                    ctx.globalCompositeOperation = 'source-over'
                    //                    ctx.globalAlpha = 0.5
                    ctx.stroke()
                    ctx.restore()
                }
                function clearImage() {
                    var ctx = getContext("2d")
                    ctx.clearRect(0, 0, 100, 100)
                    ctx.stroke()
                }
                onPaint: {
                    var bsdPath = 'assets/' + statusList[dataStateTable.dataStatus]
                    drawImage(bsdPath)
                }
                Component.onCompleted: {
                    loadImage("assets/init.png")
                    loadImage("assets/ready.png")
                    loadImage("assets/error.png")
                }
            }
        }
        Column
        {
            id: nodesStatus
            spacing: 0
            height: parent.height
            width: (status.width - status.spacing) *0.7
            Label{
                id:labelTimeStamp;
//                text:"Watchdog | node state"
                text:"Time: 0.0"
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                font.pointSize: 10
                height: parent.height* 0.2;
//                font.pointSize: 10
                leftPadding: 10
                //                anchors.verticalCenter: parent.verticalCenter
            }
            TextArea{
                id:txtNodes;
                width: parent.width;
                height: parent.height* 0.6;
                font.weight: Font.Black
                font.family: "Courier New"
                font.pointSize: 10
                Layout.fillHeight: true
                Layout.fillWidth: true
                text:"Debug status info"
                anchors.horizontalCenter: parent.horizontalCenter
            }
            CheckBox {
                id: checkBoxSaveImg
                objectName: "checkBoxSaveImg"
                width: parent.width;
                height: parent.height* 0.2;
                text: qsTr("Save Origin Image")
            }
        }
    }
    Component.onCompleted: {
        bsdViewSingal.timeStampRevised.connect(onTimeStampRevised);
        bsdViewSingal.icaStatusRevised.connect(onIcaStatusRevised);
        bsdViewSingal.nodesStatusRevised.connect(onNodesStatusRevised);
    }
    function onTimeStampRevised(tvalue) {
//        console.log('onTimeStampRevised', tvalue)
        labelTimeStamp.text = tvalue
    }
    function onNodesStatusRevised(val) {
        txtNodes.text = val[0]
    }
    function onIcaStatusRevised(tvalue) {
        dataStateTable.dataStatus = tvalue
        console.log('onIcaStatusRevised', tvalue)
        dataStateTable.updateSysStatus()
    }
}