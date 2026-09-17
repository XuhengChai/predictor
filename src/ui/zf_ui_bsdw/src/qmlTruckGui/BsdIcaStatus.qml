import QtQuick 2.12
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Extras 1.4

Item {
    id: bsdIcaStatus
    //    anchors.fill: parent
    //    anchors.margins: 2
    visible: true
    height: 125
    width: 220
    property int bsdStatus: 2
    property int icaStatus: 4
//    uint8 INITIALIZATION = 10
//    uint8 READY = 20
//    uint8 DOWNGRADED_READY = 25
//    uint8 ENABLE = 30
//    uint8 DOWNGRADED_ENABLE = 35
//    uint8 ACTIVE = 40
//    uint8 DOWNGRADED_ACTIVE = 45
//    uint8 ERROR = 60
    // length: 8
    property var statusList: ["init.png","ready.png","ready_d.png","enable.png","enable_d.png", "warn.png", "warn_d.png","error.png"]
    property bool flashing: false
    property bool abStatus: true

    function updateIcaStatus() {
        canvasIca.requestPaint()
    }
    function updateBsdStatus() {
        canvasBsd.requestPaint()
    }



    Canvas {
        id: canvasEb
        width: 0.32 * parent.height
        height: 0.32 * parent.height
        anchors.horizontalCenter: bsdIcaStatus.horizontalCenter
        visible:false
        onPaint: {
            var ctx = getContext("2d")
            var ox = width/2 - 20
            var oy = height/2 - 20
            var h = Math.min(40, height)
            var w = Math.min(40, width)
            ctx.reset()
            ctx.clearRect(ox, oy, w, h)
            ctx.drawImage('assets/ab_err.png', ox, oy, w, h)
            ctx.globalCompositeOperation = 'source-over'
            //                    ctx.globalAlpha = 0.5
            //                    console.log("----------", width)
            ctx.stroke()
            ctx.restore()
        }
        Component.onCompleted: {
            loadImage("assets/ab_err.png")
        }
    }

//    Timer {
//        id: flashTimer
//        interval: 500;
//        running:true;
//        repeat: true;
//        onTriggered: {
//            bsdIcaStatus.flashing = !bsdIcaStatus.flashing
//            updateIcaStatus()
//            updateBsdStatus()
//        }
//    }
    Row {
        id: rowBsdIcaStatus
        visible: true
        width: bsdIcaStatus.width
        height: bsdIcaStatus.height
        //        anchors.fill: parent
        spacing: 20
        Column {
            id: bsdStatus
            spacing: 0
            height: parent.height
            width: (rowBsdIcaStatus.width - rowBsdIcaStatus.spacing) / 2
            Label {
                id: labelBsd
                width: parent.width
                height: 0.2 * parent.height
                text: qsTr("BSDW")
                font.pointSize: 10
                //                font.family: "Times New Roman"
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
            Canvas {
                id: canvasBsd
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
                    //                    console.log("----------", width)
                    ctx.stroke()
                    ctx.restore()
                }
                function clearImage() {
                    var ctx = getContext("2d")
                    ctx.clearRect(0, 0, 100, 100)
                    ctx.stroke()
                }
                onPaint: {
                    var bsdPath = 'assets/' + statusList[bsdIcaStatus.bsdStatus]
                    if (bsdIcaStatus.flashing) {
                        drawImage('assets/ready.png')
                    } else {
                        drawImage(bsdPath)
                    }
                }
                Component.onCompleted: {
//                    loadImage("assets/bsd_normal.png")
//                    loadImage("assets/bsd_downgrade.png")
//                    loadImage("assets/bsd_error.png")
                    loadImage("assets/init.png")
                    loadImage("assets/ready.png")
                    loadImage("assets/ready_d.png")
                    loadImage("assets/enable.png")
                    loadImage("assets/enable_d.png")
                    loadImage("assets/warn.png")
                    loadImage("assets/warn_d.png")
                    loadImage("assets/error.png")
                }
            }

//            Label {
//                id: textArea
//                width: 220
//                height: 30
//                text: "ATA DISALLOWED"
//                font.pointSize: 16
//                horizontalAlignment: Text.AlignHCenter
//                verticalAlignment: Text.AlignVCenter
//                visible:abStatus
//            }

        }
        Column {
            id: icaStatus
            spacing: 0
            height: parent.height
            width: (rowBsdIcaStatus.width - rowBsdIcaStatus.spacing) / 2
            Label {
                id: labelIca
                width: parent.width
                height: 0.2 * parent.height
                text: qsTr("ICA")
                font.pointSize: 10
                //                font.family: "Times New Roman"
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
            Canvas {
                id: canvasIca
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
                onPaint: {
                    var icaPath = 'assets/' + statusList[bsdIcaStatus.icaStatus]
                    if (bsdIcaStatus.flashing) {
                        drawImage('assets/ready.png')
                    } else {
                        drawImage(icaPath)
                    }
                }
                Component.onCompleted: {
//                    loadImage("assets/ica_normal.png")
//                    loadImage("assets/ica_downgrade.png")
//                    loadImage("assets/ica_error.png")
                    loadImage("assets/init.png")
                    loadImage("assets/ready.png")
                    loadImage("assets/ready_d.png")
                    loadImage("assets/enable.png")
                    loadImage("assets/enable_d.png")
                    loadImage("assets/warn.png")
                    loadImage("assets/warn_d.png")
                    loadImage("assets/error.png")

                }
            }
        }
    }
    Canvas {
        id: canvasAtaNo
        y: parent.height
        width: 100
        height: 100
        anchors.horizontalCenterOffset: 0
        anchors.horizontalCenter: bsdIcaStatus.horizontalCenter
        visible:abStatus
        onPaint: {
            var ctx = getContext("2d")
            var ox = width/2 - 50
            var oy = height/2 - 50
            var h = Math.min(100, height)
            var w = Math.min(100, width)
            ctx.reset()
            ctx.clearRect(ox, oy, w, h)
            ctx.drawImage('assets/ata_no.png', ox, oy, w, h)
            ctx.globalCompositeOperation = 'source-over'
            //                    ctx.globalAlpha = 0.5
            //                    console.log("----------", width)
            ctx.stroke()
            ctx.restore()
        }
        Component.onCompleted: {
            loadImage("assets/ata_no.png")
        }
    }
}
