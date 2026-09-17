import QtQuick 2.12
//import QtMultimedia 5.12
import QtQuick.Window 2.0
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.3

//import QtQuick.Controls.Styles 1.4
import QtQuick.Controls 1.4 as QC14
import QtQuick.Extras 1.4 as QE14
import QtQuick.Shapes 1.12
import QtGraphicalEffects 1.12

//import QtQuick.Controls 1.4
//import QtQuick.Shapes
//import QtQuick.Shapes 1.12
//import dashboard.dashboard 1.0
//import dashboard.TurnIndicator 1.0
import "dashboard/" as Dashboard

//import QtQuick.Controls.Material 2.0
//import PerceptionSceneUnderQML 1.0
import Qt.labs.platform 1.1
import "custom"

Window {
    id: bsdWindow
    width: 1820
    height: 1000
    visible: true
    property int pointSize: 10
    property var plotRange: [-40, 40, -20, 20] //xlim ylim
    signal lrImgChanged(var lr)

    FileDialog {
        id: fileDialog
        objectName: "fileDialog"
        title: "Please choose a file"
        folder: shortcuts.home
        onAccepted: {
//            console.log("You chose: " + fileDialog.file)
        }
        onRejected: {
//            console.log("Canceled")
        }
//        Component.onCompleted: visible = true
    }
    FolderDialog {
        id: folderDialog
        objectName: "folderDialog"
        currentFolder: viewer.folder
        folder: StandardPaths.standardLocations(StandardPaths.PicturesLocation)[0]
    }

    QC14.SplitView {
        id: mainRowLayout
        anchors.fill: parent
        anchors.margins: 5
        QC14.SplitView {
            id: splitView
            orientation: Qt.Vertical
            Layout.maximumWidth: 0.52*bsdWindow.width
            Layout.minimumWidth: 0.2*bsdWindow.width
            width: 0.45*bsdWindow.width
            Rectangle {
                id: rectInputVideo

                Layout.fillHeight: true
                Layout.fillWidth: false
                Layout.preferredHeight: 200
                Layout.preferredWidth: rectTruckInfo.width
                color: "#ffffff"
                border.color: "#bdbebf"
                border.width: 1
                radius: 15
                Item{
                    id: mapItemArea
                    anchors.fill: parent
                    anchors.centerIn: parent
                    clip: true
                    Image {
                        id: mapImg
                        //这里使图片居中显示
                        x: mapItemArea.width/2-mapImg.width/2
                        y: mapItemArea.height/2-mapImg.height/2
//                        anchors.fill: parent
//                        anchors.centerIn: parent;
                        fillMode: Image.PreserveAspectFit;//填充模式——等比拉伸
//                        scale:0.18
                        scale:0.43
                        cache:false
                        property bool counter: false
                        function reload()
                        {
                            counter = !counter;
                            mapImg.source = "image://MemoryImg?id=" + counter;
                        }
                        //source: "file:///D:/DL/data/20240312/1501/data_img/1710226498.024736536.jpg"   
                        //source: "file:///D:/DL/data/20240312/1501/data_img/1710226498.024736536.jpg"                        //图像异步加载，只对本地图像有用
//                        asynchronous: true // will flicker
                    }
                    MouseArea {
                        id: mapDragArea
                        anchors.fill: mapImg
                        drag.target: mapImg
                        //这里使图片不管是比显示框大还是比显示框小都不会被拖拽出显示区域
                        drag.minimumX: (mapImg.width > mapItemArea.width) ? (mapItemArea.width - mapImg.width) : 0
                        drag.minimumY: (mapImg.height > mapItemArea.height) ? (mapItemArea.height - mapImg.height) : 0
                        drag.maximumX: (mapImg.width > mapItemArea.width) ? 0 : (mapItemArea.width - mapImg.width)
                        drag.maximumY: (mapImg.height > mapItemArea.height) ? 0 : (mapItemArea.height - mapImg.height)

                        //使用鼠标滚轮缩放
                        onWheel: {
                            //每次滚动都是120的倍数
                            var datla = wheel.angleDelta.y/120;
                            if(datla > 0)
                            {
                                mapImg.scale = mapImg.scale/0.9
                            }
                            else
                            {
                                mapImg.scale = mapImg.scale*0.9
                            }
                        }
                    }
                }

//                ColorButton {
//                    id: colorBtnCapture
//                    visible: false
//                    x: 41
//                    y: 94
//                    width: 73
//                    height: 30
//                    text: "Capture"
//                }
            }

            Rectangle {
                id: rectTruckInfo
                Layout.minimumHeight: 0.3*bsdWindow.height
                Layout.maximumHeight: 0.5*bsdWindow.height

                height: 0.5*bsdWindow.height

                Layout.fillHeight: true
                Layout.preferredHeight: 300
                //                Layout.preferredWidth: sliderSpinBoxEgoAngle.width + 10
//                Layout.preferredWidth: 300
                Layout.preferredWidth: 0.2*bsdWindow.width


                color: "#ffffff"
                border.color: "#bdbebf"
                border.width: 1
                radius: 15
                Column {
                    id: gridLayoutTruckInfo
                    anchors.fill: parent
                    anchors.margins: 5
                    spacing: 20
                    Row {
                        id: rowRadioBtns
                        property int radioBtnsCheckedIndex: 1
                        anchors.left: parent.left
                        anchors.leftMargin: 0
                        width:rectTruckInfo.width
                        spacing: 5
                        RadioButton {
                            id: radioBtnLeftSignal
                            width:0.25*rowRadioBtns.width
                            text: qsTr("front")
                            font.pointSize: 8
                            onCheckedChanged: {
                                if (radioBtnLeftSignal.checked) {
                                    rowRadioBtns.radioBtnsCheckedIndex = 0
                                    lrImgChanged(0)
                                    //                                    console.log('radioBtnLeftSignal',
                                    //                                                rowRadioBtns.radioBtnsCheckedIndex)
                                    updateAlert()
                                }
                            }
                        }

                        RadioButton {
                            id: radioBtnRightSignal
                            width:0.25*rowRadioBtns.width
                            text: qsTr("right")
                            font.pointSize: 8
                            //checked: true
                            onCheckedChanged: {
                                if (radioBtnRightSignal.checked) {
                                    rowRadioBtns.radioBtnsCheckedIndex = 1
                                    lrImgChanged(1)
                                    //                                    console.log('radioBtnRightSignal',
                                    //                                                rowRadioBtns.radioBtnsCheckedIndex)
                                    updateAlert()
                                }
                            }
                        }
                        TextField {
                            anchors.verticalCenter: rowRadioBtns.verticalCenter
                            id: textCount
                            width: 0.1 * rowRadioBtns.width
                            text: "0"
                            visible: false //type,posx,posy,w,l,heading]
                            padding: 0
                            topPadding: 6
                            leftPadding: 10
                            font.pointSize: 9
                            font.family: "Times New Roman"
                            selectByMouse: true
                            horizontalAlignment: Text.AlignHCenter
                        }
                        TextField {
                            anchors.verticalCenter: rowRadioBtns.verticalCenter
                            id: textTimeStamp
                            width: 0.4 * rowRadioBtns.width
                            text: "0.0" //type,posx,posy,w,l,heading]
                            padding: 0
                            topPadding: 6
                            leftPadding: 10
                            font.pointSize: 9
                            font.family: "Times New Roman"
                            selectByMouse: true
                            horizontalAlignment: Text.AlignHCenter
                        }

                    }

                    SliderSpinBox {
                        anchors.horizontalCenter: gridLayoutTruckInfo.horizontalCenter
                        id: sliderSpinBoxEgoAngle
                        anchors.left: parent.left
                        anchors.leftMargin: 5
                        value: 10.0
                        name: qsTr("Ego angle/°")
                        stepSize: 10
                        decimals: 1
                        onValueChanged: {
                            updateRender()
                        }
                    }
                    Row {
                        id: rowDangerScale
                        visible: false
                        anchors.left: parent.left
                        anchors.leftMargin: 5
                        spacing: 20
                        Label {
                            anchors.verticalCenter: rowDangerScale.verticalCenter
                            id: labelDangerScale

                            text: qsTr("Danger scale")
                            font.pointSize: bsdWindow.pointSize
                            //                            font.family: "Times New Roman"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }

                        ComboBox {
                            id: comboBoxDangerLevel
                            currentIndex: 3
                            anchors.verticalCenter: rowDangerScale.verticalCenter
                            font.pointSize: bsdWindow.pointSize
                            font.family: "Arial"
                            wheelEnabled: true
                            //                        displayText: currentText
                            textRole: "text"
                            model: ListModel {
                                id: dangerScaleItems
                                ListElement {
                                    text: "LEVEL1"
                                    color: "Red"
                                }
                                ListElement {
                                    text: "LEVEL2"
                                    color: "Black"
                                }
                                ListElement {
                                    text: "LEVEL3"
                                    color: "Gray"
                                }
                                ListElement {
                                    text: "LEVEL-1"
                                    color: "Red"
                                }
                            }
                            contentItem: Text {
                                leftPadding: 10
                                rightPadding: comboBoxDangerLevel.indicator.width
                                              + comboBoxDangerLevel.spacing
                                text: comboBoxDangerLevel.displayText
                                font: comboBoxDangerLevel.font
                                color: dangerScaleItems.get(
                                           comboBoxDangerLevel.currentIndex).color
                                verticalAlignment: Text.AlignVCenter
                                elide: Text.ElideRight
                            }
                            // value from 1 to 3
                            indicator: Canvas {
                                width: 12
                                height: 8
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.right: parent.right
                                anchors.rightMargin: 5
                                contextType: "2d"
                                onPaint: {
                                    context.reset()
                                    context.moveTo(0, 0)
                                    context.lineTo(width, 0)
                                    context.lineTo(width / 2, height)
                                    context.closePath()
                                    context.fill()
                                }
                            }
//                            background: Rectangle {

//                                //                                implicitWidth: comboBoxDangerLevel.width
//                                //                                implicitHeight: comboBoxDangerLevel.height
//                                border.color: comboBoxDangerLevel.pressed ? "#17a81a" : "Gray"
//                                border.width: 1
//                                radius: 5
//                            }
                            onCurrentIndexChanged: {
                                canvasTrcukAlert.requestPaint()
                            }

                            //                        onActivated: {console.log(currentIndex,currentText) }
                        }

                        Button {
                            id: btnClearAgents
                            text: qsTr("Clear agents")
                            onClicked: {

                                tableWidget.model.clear()
                            }
                        }
                    }
                    Row {
                        id: rowAgentType
                        visible: false
                        anchors.left: parent.left
                        anchors.leftMargin: 5
                        spacing: 20
                        Label {
                            anchors.verticalCenter: rowAgentType.verticalCenter
                            id: labelAgentType
                            text: qsTr("Agent type")
                            font.pointSize: bsdWindow.pointSize
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }

                        ComboBox {
                            id: comboBoxAgentType
                            anchors.verticalCenter: rowAgentType.verticalCenter
                            //                        width: 250
                            //                        height: 40
                            font.pointSize: bsdWindow.pointSize
                            font.family: "Arial"
                            wheelEnabled: true
                            //                        displayText: currentText
                            textRole: "text"
                            model: ListModel {
                                id: agentTypeItems
                                ListElement {
                                    text: "Vehicle"
                                    color: "Black"
                                }
                                ListElement {
                                    text: "Pedestrian"
                                    color: "Black"
                                }
                                ListElement {
                                    text: "Cyclist"
                                    color: "Black"
                                }
                                ListElement {
                                    text: "Motorcycle"
                                    color: "Black"
                                }
                                ListElement {
                                    text: "trcuk"
                                    color: "Black"
                                }
                                ListElement {
                                    text: "bus"
                                    color: "Black"
                                }
                                ListElement {
                                    text: "unknown"
                                    color: "Black"
                                }
                                ListElement {
                                    text: "MutiAgents"
                                    color: "Black"
                                }
                                //                                ListElement {
                                //                                    text: "Unknow"
                                //                                    color: "Black"
                                //                                }
                            }
                            contentItem: Text {
                                leftPadding: 10
                                rightPadding: comboBoxAgentType.indicator.width
                                              + comboBoxAgentType.spacing
                                text: comboBoxAgentType.displayText
                                font: comboBoxAgentType.font
                                color: agentTypeItems.get(
                                           comboBoxAgentType.currentIndex).color
                                verticalAlignment: Text.AlignVCenter
                                elide: Text.ElideRight
                            }
                            // value from 1 to 3
                            indicator: Canvas {
                                width: 12
                                height: 8
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.right: parent.right
                                anchors.rightMargin: 5
                                contextType: "2d"
                                onPaint: {
                                    context.reset()
                                    context.moveTo(0, 0)
                                    context.lineTo(width, 0)
                                    context.lineTo(width / 2, height)
                                    context.closePath()
                                    context.fill()
                                }
                            }
//                            background: Rectangle {
//                                //                                implicitWidth: comboBoxAgentType.width
//                                //                                implicitHeight: comboBoxAgentType.height
//                                border.color: comboBoxAgentType.pressed ? "#17a81a" : "Gray"
//                                border.width: 1
//                                radius: 5
//                            }
                            onCurrentIndexChanged: {
                                canvasTrcukAlert.requestPaint()
                            }
                        }
                    }

                    Row {
                        anchors.horizontalCenter: gridLayoutTruckInfo.horizontalCenter
                        id: rowAgentString
                        visible: false
                        anchors.left: parent.left
                        anchors.leftMargin: 5
                        spacing: 5
                        Label {
                            anchors.verticalCenter: rowAgentString.verticalCenter
                            id: labelAgentString
                            width: 0.15 * rowAgentString.width
                            color: "#000000"
                            text: qsTr("Agent:")
                            font.pointSize: bsdWindow.pointSize
                            font.family: "Times New Roman"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                        TextField {
                            anchors.verticalCenter: rowAgentString.verticalCenter
                            id: textAgentString
                            width: 0.6 * rowAgentString.width
                            //                            function getTextAgent() {
                            //                                var sum = []
                            //                                sum.push(dangerScaleItems.get(
                            //                                             comboBoxDangerLevel.currentIndex).text)
                            //                                sum.push(agentTypeItems.get(
                            //                                             comboBoxAgentType.currentIndex).text)
                            //                                var i
                            //                                for (i = 0; i < xyCoord.count; ++i) {
                            //                                    sum.push(xyCoord.itemAt(i).value)
                            //                                }
                            //                                return sum.join(", ")
                            //                            }
                            text: "1, 1.27, 2.3, 0.7, 0.7, 20" //type,posx,posy,w,l,heading]
                            padding: 0
                            topPadding: 6
                            leftPadding: 0
                            font.pointSize: 9
                            font.family: "Times New Roman"
                            selectByMouse: true
                            horizontalAlignment: Text.AlignHCenter
                            //                            onTextChanged: {
                            //                                updateRender()
                            //                            }
                        }
                        Button {
                            id: button
                            width: 0.25 * rowAgentString.width
                            text: qsTr("Add agent")
                            onClicked: {
                                var strData = textAgentString.text
                                var dataList = strData.split(",")
                                console.log('strData is', strData)
                                // [name,level,status,posx,posy,w,l,vx,vy]
                                //                                var keys = tableWidget.colNames
                                var data = {

                                }
                                var i = 0
                                data[tableWidget.colNames[i++]]
                                        = tableWidget.agentType[Number(
                                                                    dataList[0])]
                                data[tableWidget.colNames[i++]] = Number(dataList[0])
                                data[tableWidget.colNames[i++]] = Number(dataList[0])
                                data[tableWidget.colNames[i++]] = Number(
                                            dataList[1])
                                data[tableWidget.colNames[i++]] = Number(
                                            dataList[2])
                                data[tableWidget.colNames[i++]] = Number(
                                            dataList[3])
                                data[tableWidget.colNames[i++]] = Number(
                                            dataList[4])
                                data[tableWidget.colNames[i++]] = Number(
                                            dataList[5])
                                data[tableWidget.colNames[i++]] = 0.0
                                data[tableWidget.colNames[i++]] = 0.0
                                data[tableWidget.colNames[i++]] = "unknown"
                                data[tableWidget.colNames[i++]] = Number(
                                            dataList[0])
                                tableWidget.model.append(data)
                                updateRender()
                                //                                console.log('canvasTrcukAlert', tableWidget.model.count)
                                //                                var data = {'time': new Date().toTimeString()};
                                //                                msg.model.append(data);
                            }
                        }
                    }
                    Rectangle {
                        id: tableWidgetRect
                        width: gridLayoutTruckInfo.width
                        height: gridLayoutTruckInfo.height * 0.4
//                        height: gridLayoutTruckInfo.height /2
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.leftMargin: 5
                        TableWidget {
                            id: tableWidget
                            //                            anchors.leftMargin: 5
                            //                            anchors.horizontalCenter: gridLayoutTruckInfo.horizontalCenter
                        }
                    }


                    Grid{
                            id: gridLayoutStatus
                            columns: 2;
                            rows:2;
                            width: gridLayoutTruckInfo.width
                            height: gridLayoutTruckInfo.height * 0.35
                            anchors.margins: 5;
                            columnSpacing: 5;
                            rowSpacing: 5;
                            property var childHeight: 0.2 * gridLayoutStatus.height
                            property var childWidth: (gridLayoutStatus.width - 5)

//                            Rectangle{
//                                id:rect00;
//                                width: gridLayoutStatus.childWidth * 0.3;
//                                height: gridLayoutStatus.childHeight
//                                Layout.row:0;
//                                Layout.column: 0;
//                            }
                            Rectangle{
                                id:rect01;
                                width: gridLayoutStatus.childWidth * 0.35;
                                height: gridLayoutStatus.childHeight
                                Label{
                                    id:label01;
                                    text:"DCU hardware\nresource monitor"
                                    leftPadding: 10
//                                    anchors.horizontalCenter: parent.horizontalCenter
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Layout.row:0;
                                Layout.column: 0;
                            }
                            Rectangle{
                                id:rect02;
                                width: gridLayoutStatus.childWidth * 0.65;
                                height: gridLayoutStatus.childHeight
                                Label{
                                    id:label02;
                                    text:"Watchdog | node state"
                                    leftPadding: 10
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                Layout.row:0;
                                Layout.column: 1;
                            }


//                            TextArea{
//                                id:txtBsdIca;
//                                width: gridLayoutStatus.childWidth * 0.3;
//                                height: gridLayoutStatus.height - gridLayoutStatus.childHeight - 5;
//                                font.pointSize: 12
//                                placeholderText: "Bsdw Ica"
//                                Layout.fillWidth: true
//                                Layout.fillHeight: true
//                                Layout.row:1;
//                                Layout.column:0;
//                            }
                            TextArea{
                                id:txtDcu;
                                width: gridLayoutStatus.childWidth * 0.35;
                                height: gridLayoutStatus.height - gridLayoutStatus.childHeight - 5;
                                font.weight: Font.Black
                                font.family: "Courier New"
                                font.pointSize: 10
                                placeholderText: "DCU"
                                Layout.fillHeight: true
                                Layout.fillWidth: true

                                Layout.row:1;
                                Layout.column: 0;
                            }
                            TextArea{
                                id:txtNodes;
                                width: gridLayoutStatus.childWidth * 0.65;
                                height: gridLayoutStatus.height - gridLayoutStatus.childHeight - 5;
                                font.weight: Font.Black
                                font.family: "Courier New"
                                font.pointSize: 10
                                placeholderText: "Nodes status"
                                Layout.fillHeight: true
                                Layout.fillWidth: true
                                Layout.row:1;
                                Layout.column: 1;
                            }
                        }

//                    BsdIcaStatus
//                    {
//                        id:bsdIcaStatus
//                        width: gridLayoutTruckInfo.width
////                        height: 50

//                    }
//                    Rectangle {
//                        id: nodeWidgetRect
//                        width: gridLayoutTruckInfo.width
//                        height: gridLayoutTruckInfo.height*0.1
////                        height: gridLayoutTruckInfo.height /2
//                        anchors.horizontalCenter: parent.horizontalCenter
//                        anchors.leftMargin: 5
//                        NodesWidget {
//                            id: nodeWidget
//                            //                            anchors.leftMargin: 5
//                            //                            anchors.horizontalCenter: gridLayoutTruckInfo.horizontalCenter
//                        }
//                    }
                }
            }
        }

        Rectangle {
            id: rectTrcukRender
            color: "#ffffff"
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredHeight: 200
            Layout.preferredWidth: 600
            width:0.8*bsdWindow.width
            border.color: "#bdbebf"
            border.width: 1
            radius: 15
            property int renderLorR: rowRadioBtns.radioBtnsCheckedIndex
//            property var agentsImgPath: ["assets/vehicle.png", "assets/pedestrian.png", "assets/cyclist.png", "assets/motorcycle.png", "assets/mutiAgents.png"]
            property variant agentImgPath: 'assets/' + tableWidget.agentType[comboBoxAgentType.currentIndex] + ".png"
            property variant levelColor: ["Red.png", "Yellow.png", "Gray.png"]

            //            property var agents: getAgents()
            property var agentCol: ['posx', //0
                'posy', //1
                'w', //2
                'l', //3
                'heading', //4
                'itype']

            ColumnLayout {
                id: rowLayout
                anchors.fill: parent
                anchors.margins: 15
                spacing: 5
                Canvas {
                    id: canvasTrcukAlert
                    //                    anchors.fill: parent
                    Layout.fillHeight: true
                    Layout.preferredHeight: 30
                    Layout.preferredWidth: rowLayout.width
                    property real centerX: canvasTrcukAlert.width / 2
                    property real centerY: canvasTrcukAlert.height / 2

                    function drawImage(imgPath,x1,x2) {

                        var ctx = getContext("2d")
                        ctx.reset()
                        ctx.translate(x1, canvasTrcukAlert.height/2 - 50)
                        ctx.drawImage(imgPath, 0, 0)
                        //                    ctx.drawImage('assets/mutiAgents.png', 0, 0)
                        ctx.globalCompositeOperation = canvasTrcukRender.operation[0]
                        //                    ctx.globalAlpha = 0.5
//                        console.log("----------", rectTrcukRender.agentImgPath)
//                        ctx.drawImage(rectTrcukRender.agentImgPath, x2, 0,100,100)
                        ctx.drawImage(rectTrcukRender.agentImgPath, x2, 0)
                        ctx.stroke()
                        // restore previous setup
                        ctx.restore()
                    }
                    function clearImage(imgPath,x1,x2) {
                        var ctx = getContext("2d")
                        ctx.reset()
                        ctx.translate(x1, 10)
                        ctx.clearRect(0, 0, 100, 100)
                        ctx.clearRect(x2, 0,100,100)
                        ctx.stroke()
                        // restore previous setup
                        ctx.restore()
                    }
                    onPaint: {

                        var xyPath = 'assets/alertLGray.png'
                        var x1 = centerX * 1.5 - 50
                        var x2 = 100
                        if (rectTrcukRender.renderLorR == 0) {
                            //left
                            x1 = centerX * 0.4
                            x2 = -100
                            switch (comboBoxDangerLevel.currentIndex) {
                            case 0:
                                xyPath = 'assets/ICARed.png'
                                break
                            case 1:
                                xyPath = 'assets/ICAYellow.png'
                                break
                            case 2:
                                xyPath = 'assets/ICAGray.png'
                                break
                            }
                        } else if (rectTrcukRender.renderLorR == 1) {

                            switch (comboBoxDangerLevel.currentIndex) {
                            case 0:
                                xyPath = 'assets/ICARed.png'
                                break
                            case 1:
                                xyPath = 'assets/ICAYellow.png'
                                break
                            case 2:
                                xyPath = 'assets/ICAGray.png'
                                break
                            }
                        }
//                        console.log('canvasTrcukAlert',
//                                    comboBoxDangerLevel.currentIndex, "is",
//                                    rectTrcukRender.renderLorR, xyPath, x2)
                        if (comboBoxDangerLevel.currentIndex === 3)
                        {
                            clearImage(xyPath, x1, x2)
                        }
                        else
                        {
                            drawImage(xyPath, x1, x2)
                        }
                    }
                    TextArea {
                        id: icaInfoText
                        text: qsTr("")
                        font.pointSize: 18
                        placeholderText: "ICA Info"
                        height: canvasTrcukAlert.height
                        anchors.right: dashboardInfo.left
                        anchors.rightMargin: 0
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Rectangle
                    {
                        id: dashboardInfo
                        anchors.bottom: parent.bottom
                        anchors.bottomMargin: 0
                        anchors.left: parent.mid
                        anchors.leftMargin: 0
                        width: 200
                        height: canvasTrcukAlert.height
                        anchors.horizontalCenter: parent.horizontalCenter
//                        color: "#161616"
                        Dashboard.ValueSource {
                            id: valueSource
                        }

                        // Dashboards are typically in a landscape orientation, so we need to ensure
                        // our height is never greater than our width.
                        Item {
                            id: container
                            width: dashboardInfo.width
                            height: Math.min(dashboardInfo.width, dashboardInfo.height)
                            anchors.centerIn: parent

                            Row {
                                id: gaugeRow
                                spacing: container.width * 0.02
                                anchors.centerIn: parent

                                Dashboard.TurnIndicator {
                                    id: leftIndicator
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: height
                                    height: container.height * 0.2 - gaugeRow.spacing

                                    direction: Qt.LeftArrow
                                    on: valueSource.turnSignal == Qt.LeftArrow
                                }

                                QE14.CircularGauge {
                                    objectName: "speedometer"
                                    id: speedometer
                                    property real gauge_value: 0
                                    value: gauge_value
                                    anchors.verticalCenter: parent.verticalCenter
                                    maximumValue: 180
                                    // We set the width to the height, because the height will always be
                                    // the more limited factor. Also, all circular controls letterbox
                                    // their contents to ensure that they remain circular. However, we
                                    // don't want to extra space on the left and right of our gauges,
                                    // because they're laid out horizontally, and that would create
                                    // large horizontal gaps between gauges on wide screens.
                                    width: height
                                    height: container.height
                                    Behavior on value {
                                              NumberAnimation {
                                                  duration: 100
                                              }
                                    }

                                    style: Dashboard.DashboardGaugeStyle {}
                                }

                                Dashboard.TurnIndicator {
                                    id: rightIndicator
                                    anchors.verticalCenter: parent.verticalCenter
                                    width: height
                                    height: container.height * 0.2 - gaugeRow.spacing

                                    direction: Qt.RightArrow
                                    on: valueSource.turnSignal == Qt.RightArrow
                                }

                            }
                        }
                    } // Rectangle--end
                    TextArea {
                        id: pncInfoText
                        text: qsTr("")
                        font.pointSize: 20
                        placeholderText: "PNC Info"
                        height: canvasTrcukAlert.height
                        anchors.left: dashboardInfo.right
                        anchors.leftMargin: 0
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }

                Rectangle {
                    id: truckSegLine
                    Layout.preferredHeight: 2
                    Layout.preferredWidth: rowLayout.width
                    //                    color: "white"
                    border.color: "black"
                    border.width: 2
//                    Component.onCompleted:{
//                        console.log("rectTrcukRender.width", rectTrcukRender.width)
//                        console.log("rowLayout.width", rowLayout.width)

//                        console.log("truckSegLine.width", truckSegLine.width)

//                    }
                }

                Canvas {
                    id: canvasTrcukRender
                    //                anchors.fill: parent
                    Layout.preferredHeight: 150
                    Layout.preferredWidth: rowLayout.width
                    property real proportionw2p: 8
                    property real centerX: canvasTrcukRender.width / 2
                    property real centerY: canvasTrcukRender.height / 2
                    //                    property real truckHeight: 100
                    property real imageSize: 50
                    //                    property real truckWidth: -1.339
                    property variant truckHead: [5.37, 1.337, 2.676, 7.18, 0]
                    property variant truckTail: [1.27, 1.337, 2.676, 13, sliderSpinBoxEgoAngle.value]
                    property real disRed: 1.5
                    property real disYellow: 4.5
                    property real disGray: 6
                    property variant truckHeadRect: world2Pixel(truckHead)
                    property variant truckLineL: world2PixelLR(truckHead, [0, 0.5])
                    property variant truckLineR: world2PixelLR(truckHead, [ -1.5 - truckHead[2], -0.5 - truckHead[2]])

                    property variant truckHeadRedL: world2PixelLR(truckHead, [0, disRed])
                    property variant truckHeadYellowL: world2PixelLR(truckHead, [disRed, disYellow])
                    property variant truckHeadGrayL: world2PixelLR(truckHead, [disYellow, disGray])
                    property variant truckTailRect: world2Pixel(truckTail)
                    property variant truckTailRedL: world2PixelLR(truckTail, [0, disRed])
                    property variant truckTailYellowL: world2PixelLR(truckTail, [disRed, disYellow])
                    property variant truckTailGrayL: world2PixelLR(truckTail, [disYellow, disGray])

                    property variant truckHeadRedR: world2PixelLR(truckHead, [ -disRed - truckHead[2], - truckHead[2]])
                    property variant truckHeadYellowR: world2PixelLR(truckHead, [-disYellow - truckHead[2], -disRed - truckHead[2]])
                    property variant truckHeadGrayR: world2PixelLR(truckHead, [-disGray - truckHead[2], -disYellow - truckHead[2]])
                    property variant truckTailRedR: world2PixelLR(truckTail, [ -disRed - truckTail[2], - truckTail[2]])
                    property variant truckTailYellowR: world2PixelLR(truckTail, [-disYellow - truckTail[2], -disRed - truckTail[2]])
                    property variant truckTailGrayR: world2PixelLR(truckTail, [-disGray - truckTail[2], -disYellow - truckTail[2]])

                    property var operation: ['source-over', 'source-in', 'source-over', 'source-atop', 'destination-over', 'destination-in', 'destination-out', 'destination-atop', 'lighter', 'copy', 'xor', 'qt-clear', 'qt-destination', 'qt-multiply', 'qt-screen', 'qt-overlay', 'qt-darken', 'qt-lighten', 'qt-color-dodge', 'qt-color-burn', 'qt-hard-light', 'qt-soft-light', 'qt-difference', 'qt-exclusion']
                    property var alertColor: ['Red', 'Yellow', 'Gray']
                    property var sourceColor: ['red','darkred', 'cyan','darkcyan', 'green', 'darkgreen']
                    property var sourceAlpha: [0.5, 0.75, 0.5, 0.75, 0.5, 0.75]
                    Layout.fillHeight: true

                    function drawRotImage(xyimgPath,rect) {
                        var ctx = getContext("2d")
                        ctx.save()
                        ctx.translate(xyimgPath[0], xyimgPath[1])
                        //                        ctx.scale(0.5, 0.5)
                        ctx.rotate(rect[4])
                        if (rect.length === 6)
                        {
                            ctx.globalAlpha = sourceAlpha[rect[5]]
                        }
                        ctx.drawImage(xyimgPath[2], rect[0], rect[1],
                                      rect[2], rect[3])
                        if (showBoxSwitch.checked)
                        {
//                           ctx.strokeStyle = "#09f";
                           ctx.strokeStyle = "blue";
                           if (rect.length === 6)
                           {
                               ctx.strokeStyle = sourceColor[rect[5]]
                           }
                           ctx.strokeRect(rect[0], rect[1], rect[2], rect[3])
                        }
                        //                    ctx.drawImage('assets/mutiAgents.png', 0, 0)
                        ctx.globalCompositeOperation = canvasTrcukRender.operation[0]

                        //                    ctx.globalAlpha = 0.5
                        //                    ctx.drawImage(xyimgPath[2], 0, 0)
                        //                        ctx.drawImage(rectTrcukRender.agentImgPath, 0, 100)
                        ctx.stroke()
                        // restore previous setup
                        ctx.restore()
                    }

                    function drawRotCenRect(xy, rect) {
                        var ctx = getContext("2d")
                        ctx.save()
                        ctx.strokeStyle = xy[2] // 画笔颜色（边框颜色）
                        ctx.translate(xy[0], xy[1])
                        ctx.rotate(rect[4])
                        //                        console.log("drawRotCenRect", rect)
//                        ctx.strokeRect(rect[0], rect[1], rect[2], rect[3])
                        ctx.globalAlpha = 0.2
                        ctx.fillStyle = xy[2] // 画刷颜色
                        ctx.fillRect(rect[0], rect[1],
                                     rect[2],
                                     rect[3]) // 绘制矩形 begin point and width height
                        //                    ctx.globalCompositeOperation = canvasTrcukRender.operation[0]
                        //                    ctx.drawImage(xyimgPath[2], 0, 0)
                        // restore previous setup
                        ctx.restore()
                    }

                    function drawRotLevel(xy, rect, vxylevel)
                    {
                        var ctx = getContext("2d")
                        ctx.save()
                        ctx.translate(xy[0], xy[1])
                        //                        ctx.scale(0.5, 0.5)
                        ctx.rotate(rect[4])
                        ctx.globalCompositeOperation = canvasTrcukRender.operation[0]
                        ctx.globalAlpha = 0.9
                        var bsdLevel = vxylevel[2]
                        var icaStatus = vxylevel[3]
                        var wl = Math.min(rect[2]/2, rect[3]/2)


//                        ctx.drawImage(imgPath, rect[0], rect[1],
//                                      rect[2], rect[3])
//                        ctx.stroke()
                        // notice direction is reversed from world to pixel
                        var recvxy = world2Pixel([vxylevel[0], vxylevel[1], 0, 0, 0])
                        if (showSpeedSwitch.checked){
                            ctx.rotate(-rect[4])
                            drawArrow(ctx, 0, 0, recvxy[0], recvxy[1], "#09f",30,5,1) //"#f36"
                            ctx.rotate(rect[4])
                        }

                        if (bsdLevel !== -1 || icaStatus !== -1)
                        {
//                            ctx.rotate(-rect[4])
//                            ctx.rotate(rect[4])
                            if (icaStatus !== -1)
                            {
//                                ctx.drawImage("assets/ICARed.png", rect[2]/2-wl, rect[3]/2-wl,
//                                              wl, wl)
                                var wlm = Math.min(rect[2]*2/3, rect[3]*2/3)
                                var imgPaths = ["assets/ICA", rectTrcukRender.levelColor[bsdLevel]]
                                if (wlm < 15)
                                {
                                    wlm = 15
                                }
//                                ctx.translate(wlm/2 + rect[0], wlm/2 + rect[1])
                                ctx.rotate(-rect[4])
                                ctx.drawImage(imgPaths.join(''), 0, 0,
                                              wlm, wlm)
                            }
                            else {
                                var imgPath = "assets/alertR" + alertColor[bsdLevel] + ".png"
                                if (wl < 15)
                                {
                                    wl = 15
                                }
                                ctx.translate(rect[0], rect[1])
                                ctx.rotate(-rect[4])
                                ctx.drawImage(imgPath, -wl/2, -wl/2,
                                              wl, wl)
                            }
                        }

                        // restore previous setup
                        ctx.restore()

                    }

                    function drawArrowSource(xy, rect, vxylevel)
                    {
                        if (showSpeedSwitch.checked){
                            var ctx = getContext("2d")
                            ctx.save()
                            ctx.translate(xy[0], xy[1])
                            //                        ctx.scale(0.5, 0.5)
                            ctx.globalCompositeOperation = canvasTrcukRender.operation[0]
                            ctx.globalAlpha = 0.9
                            var bsdLevel = vxylevel[2]
                            // notice direction is reversed from world to pixel
                            var recvxy = world2Pixel([vxylevel[0], vxylevel[1], 0, 0, 0])
                            drawArrow(ctx, 0, 0, recvxy[0], recvxy[1], "#09f",30,5,1) //"#f36"
                            // restore previous setup
                            ctx.restore()
                        }
                    }

                    function drawArrow(ctx, fromX, fromY, toX, toY,color,theta,headlen,width) {
                        if (toX === 0 && toY === 0)
                        {
                            return;
                        }
                        // 计算各角度和对应的P2,P3坐标
                        var angle = Math.atan2(fromY - toY, fromX - toX) * 180 / Math.PI,
                            angle1 = (angle + theta) * Math.PI / 180,
                            angle2 = (angle - theta) * Math.PI / 180,
                            topX = headlen * Math.cos(angle1),
                            topY = headlen * Math.sin(angle1),
                            botX = headlen * Math.cos(angle2),
                            botY = headlen * Math.sin(angle2);
                        ctx.strokeStyle = color;
//                        ctx.strokeStyle = 'rgba(253, 150, 38, 0.41)';
                        ctx.lineWidth = width;
                        ctx.stroke();
                        ctx.beginPath();

                        var arrowX = fromX - topX,
                            arrowY = fromY - topY;

                        ctx.moveTo(arrowX, arrowY);
                        ctx.moveTo(fromX, fromY);
                        ctx.lineTo(toX, toY);
                        arrowX = toX + topX;
                        arrowY = toY + topY;
                        ctx.moveTo(arrowX, arrowY);
                        ctx.lineTo(toX, toY);
                        arrowX = toX + botX;
                        arrowY = toY + botY;
                        ctx.lineTo(arrowX, arrowY);
                        ctx.strokeStyle = color;
//                        ctx.globalAlpha = 0.5;
                        ctx.lineWidth = width;
                        ctx.stroke();

                    }

                    function drawAgents() {
                        var i
//                        console.log("drawAgents", tableWidget.model.rowCount())
                        var t_index = 3
                        for (i = 0; i < tableWidget.model.rowCount(); ++i) {
                            var posx = tableWidget.model.get(i).posx
                            var posy = tableWidget.model.get(i).posy
                            var w = tableWidget.model.get(i).width
                            var l = tableWidget.model.get(i).length
                            var heading = tableWidget.model.get(i).heading
                            var itype = tableWidget.model.get(i).itype
                            var rectAgent = world2Pixel(
                                        [posx, posy, w, l, heading])
                            var rectImg = [ - rectAgent[2]/2, - rectAgent[3]/2,
                                    rectAgent[2], rectAgent[3], rectAgent[4]]

                            var imgPath = ["assets", tableWidget.agentType[itype] + ".png"]
                            //console.log(rectAgent, imgPath.join("/"))

                            var level = tableWidget.model.get(i).bsdlevel
                            var icaStatus = tableWidget.model.get(i).icastatus
                            var vx = tableWidget.model.get(i).velx
                            var vy = tableWidget.model.get(i).vely
                            if (level > 9) // use level-10 to represent the source such as sensor/pp
                            {
                                var iSource = level - 10
                                rectImg.push(iSource)
                                drawArrowSource([centerX + rectAgent[0], centerY + rectAgent[1]], rectImg, [vx, vy, level, icaStatus])
                                drawRotImage([centerX + rectAgent[0], centerY + rectAgent[1], imgPath.join("/")],
                                             rectImg)
                                continue
                            }
                            drawRotImage([centerX + rectAgent[0], centerY + rectAgent[1], imgPath.join("/")],
                                         rectImg)
                            if (icaStatus !== -1 && icaStatus !== "-1")
                            {
                                if (level !== -1)
                                {
                                    t_index = level
                                }
                                comboBoxAgentType.currentIndex = itype
                                //console.log("t_index", t_index, i, icaStatus, (icaStatus + 1) ,comboBoxDangerLevel.currentIndex, comboBoxAgentType.currentIndex)
                            }
                            drawRotLevel([centerX + rectAgent[0], centerY + rectAgent[1]], rectImg, [vx, vy, level, icaStatus])
                            if (t_index !== comboBoxDangerLevel.currentIndex)
                            {
                                comboBoxDangerLevel.currentIndex = t_index;
                            }

                        }
                    }
//                    Timer{
//                            id:timerSpeed
//                            property int count: 0
//                            property int tspeed: 0

//                            function timeChanged() {
//                                //随机数模拟数据来源
//                                if (!playerStopBtn.checked)
//                                {
//                                    return;
//                                }
//                                count++;
//                                timerSpeed.tspeed = speedometer.gauge_value *0.1 * count
//                                if(count > 14){
//                                    count = 0;
//                                }
//                            }
//                            interval: 60;running:true;repeat: true;
//                            onTriggered: timeChanged();
//                        }
//                    Shape {
//                        id:shapeLane

//                        ShapePath {
//                            id: shapePathL
//                            strokeColor: "black"
//                            strokeStyle: ShapePath.DashLine
//                            dashPattern: [6, 12]
//                            fillColor: "transparent"
//                            startX:canvasTrcukRender.truckLineL[0] + canvasTrcukRender.centerX
//                            startY:timerSpeed.tspeed
////                            fillGradient: LinearGradient {
////                                x1: canvasTrcukRender.truckLineL[0] + canvasTrcukRender.centerX
////                                y1: timerSpeed.tspeed
////                                x2: shapePathL.fieldWidth
////                                y2: shapePathL.fieldHeight
////                                GradientStop { position: 0.0; color: "white" }
//////                                GradientStop { position: 0.5; color: "green" }
////                                GradientStop { position: 1.0; color: "black" }
////                            }
//                            PathLine {
//                                x: canvasTrcukRender.truckLineL[0] + canvasTrcukRender.centerX
//                                y: canvasTrcukRender.truckLineL[1] + canvasTrcukRender.centerY + canvasTrcukRender.truckLineL[3]
//                            }
//                        }
//                        ShapePath {
//                            id: shapePathLTop
//                            strokeColor: "black"
//                            strokeStyle: ShapePath.DashLine
//                            dashPattern: [6, 12]
//                            fillColor: "transparent"
//                            startX:canvasTrcukRender.truckLineL[0] + canvasTrcukRender.centerX
//                            startY:timerSpeed.tspeed
//                            PathLine {
//                                x: canvasTrcukRender.truckLineL[0] + canvasTrcukRender.centerX
//                                y: 0
//                            }
//                        }

//                        ShapePath {
//                            id: shapePathR
//                            strokeColor: "black"
//                            strokeStyle: ShapePath.DashLine
//                            dashPattern: [6, 12]
//                            fillColor: "transparent"
//                            startX:canvasTrcukRender.truckLineR[0] + canvasTrcukRender.centerX
//                            startY:timerSpeed.tspeed


//                            PathLine {
//                                x: canvasTrcukRender.truckLineR[0] + canvasTrcukRender.centerX
//                                y: canvasTrcukRender.truckLineR[1] + canvasTrcukRender.centerY + canvasTrcukRender.truckLineR[3]
//                            }
//                        }
//                        ShapePath {
//                            id: shapePathRTop
//                            strokeColor: "black"
//                            strokeStyle: ShapePath.DashLine
//                            dashPattern: [6, 12]
//                            fillColor: "transparent"
//                            startX:canvasTrcukRender.truckLineR[0] + canvasTrcukRender.centerX
//                            startY:timerSpeed.tspeed
//                            PathLine {
//                                x: canvasTrcukRender.truckLineR[0] + canvasTrcukRender.centerX
//                                y: 0
//                            }
//                        }
//                    }


                    onPaint: {
//                        console.log('centerX', "centery", centerX, centerY)

                        var ctx = getContext("2d")
                        ctx.reset()
                        ctx.lineWidth = 1 // 画笔宽度

                        ctx.strokeStyle = "#888" // 画笔颜色（边框颜色）
                        ctx.fillStyle = "blue" // 画刷颜色
                        //                    ctx.beginPath()
                        //                    ctx.rect(truckHeadRect[0], truckHeadRect[1],
                        //                             truckHeadRect[2],
                        //                             truckHeadRect[3]) // 绘制矩形 begin point and width height
                        //                    //                  console.log(truckHeadRect[0])
                        //                    ctx.fill()
                        //                    ctx.fillRect(truckTailRect[0], truckTailRect[1],
                        //                                 truckTailRect[2],
                        //                                 truckTailRect[3]) // 绘制矩形 begin point and width height
                        drawRotImage([centerX, centerY, "assets/truck_head.png"],
                                     truckHeadRect)
                        drawRotImage([centerX, centerY, "assets/truck_tail.png"],
                                     truckTailRect)



                        //                        ctx.strokeStyle = "red" // 画笔颜色（边框颜色）
                        //drawRotCenRect([centerX, centerY, "black"], truckHeadRect)

//                        drawRotCenRect([centerX, centerY, "red"], truckHeadRedL)
                        drawRotCenRect([centerX, centerY, "gray"], truckHeadYellowL)
                        drawRotCenRect([centerX, centerY, "lightgray"], truckHeadGrayL)
//                        drawRotCenRect([centerX, centerY, "blue"], truckTailRect)
//                        drawRotCenRect([centerX, centerY, "red"], truckTailRedL)
                        drawRotCenRect([centerX, centerY, "gray"], truckTailYellowL)
                        drawRotCenRect([centerX, centerY, "lightgray"], truckTailGrayL)

//                        drawRotCenRect([centerX, centerY, "red"], truckHeadRedR)
                        drawRotCenRect([centerX, centerY, "gray"], truckHeadYellowR)
                        drawRotCenRect([centerX, centerY, "lightgray"], truckHeadGrayR)
//                        drawRotCenRect([centerX, centerY, "red"], truckTailRedR)
                        drawRotCenRect([centerX, centerY, "gray"], truckTailYellowR)
                        drawRotCenRect([centerX, centerY, "lightgray"], truckTailGrayR)

                        drawAgents()

                        //                      ctx.strokeStyle = "gray" // 画笔颜色（边框颜色）
                        //                        var xyPath = [truckRedRectLeft[0] + truckRedRectLeft[2]
                        //                                      / 2 - canvasTrcukRender.imageSize
                        //                                      / 2, truckRedRectLeft[1] + truckRedRectLeft[3] / 2
                        //                                      - canvasTrcukRender.imageSize / 2, 'assets/alertLYellow.png']


                        //                    ctx.rotate(180 * Math.PI / 180)
                        //                    ctx.clip()  // create clip from triangle path
                        // draw image with clip applied
                        //                            ctx.stroke()
                    }

                    Component.onCompleted: {
                        loadImage("assets/alertLGray.png")
                        loadImage("assets/alertLYellow.png")
                        loadImage("assets/alertLRed.png")
                        loadImage("assets/alertRGray.png")
                        loadImage("assets/alertRYellow.png")
                        loadImage("assets/alertRRed.png")
                        loadImage("assets/ICARed.png")
                        loadImage("assets/ICAYellow.png")
                        loadImage("assets/ICAGray.png")


                        loadImage("assets/truck_head.png")
                        loadImage("assets/truck_tail.png")
                        loadImage("assets/vehicle.png")
                        loadImage("assets/pedestrian.png")
                        loadImage("assets/cyclist.png")
                        loadImage("assets/motorcycle.png")
                        loadImage("assets/truck.png")
                        loadImage("assets/bus.png")
                        loadImage("assets/unknown.png")
                        loadImage("assets/mutiAgents.png")

                    }
                    MouseArea {
                        id: mouseArea
                        anchors.fill: parent
                        property real lastX: 0
                        property real lastY: 0

                        //                        acceptedButtons: Qt.LeftButton | Qt.RightButton
                        //                        onClicked: {
                        //                            if (mouse.button === Qt.RightButton)
                        //                                parent.color = 'blue';
                        //                            else
                        //                                parent.color = 'red';
                        //                        }
                        acceptedButtons: Qt.LeftButton
                        onPressed: {
//                            if (mouse.button === Qt.MiddleButton) {
                              if (mouse.button === Qt.LeftButton) {
                                mouse.accepted = true;  // Enable mouse events
                                lastX = mouse.x
                                lastY = mouse.y
                            }
                        }
                        onPositionChanged: {
//                            if (mouse.buttons === Qt.MiddleButton && mouse.accepted) {
                              if (mouse.buttons === Qt.LeftButton && mouse.accepted) {
                                var mouseX = mouse.x - lastX
                                var mouseY = mouse.y - lastY
                                lastX = mouse.x
                                lastY = mouse.y
//                                console.log("mouseX is", mouseX, mouseY)
                                canvasTrcukRender.centerX += mouseX
                                canvasTrcukRender.centerY += mouseY
                                updateRender()
                            }
                        }
                        onWheel: {
                            //if (wheel.modifiers & Qt.ControlModifier) {
                            if (true) {
                                if (wheel.angleDelta.y > 0) {
                                    canvasTrcukRender.proportionw2p += 1
                                    if (canvasTrcukRender.proportionw2p > 40) {
                                        canvasTrcukRender.proportionw2p = 40
                                    }
                                } else {
                                    canvasTrcukRender.proportionw2p -= 1
                                    if (canvasTrcukRender.proportionw2p < 5) {
                                        canvasTrcukRender.proportionw2p = 5
                                    }
                                }
                                updateRender()
                            }
                        }
                        Keys.onLeftPressed:
                        {
                            if (!playerStopBtn.checked)
                            {
                                sliderSpinBoxPlayer.value -= 1
                            }
                        }
                        Keys.onRightPressed:
                        {
                            if (!playerStopBtn.checked)
                            {
                                sliderSpinBoxPlayer.value += 1
                            }
                        }
                        BsdIcaStatus
                        {
                            id:bsdIcaStatus
                            anchors.top: parent.top
                            anchors.left: parent.left

                        }
                        RowLayout
                        {
                            id: rowOpenPath
                            width: rowPlayer.width
                            anchors.left: rowPlayer.left
                            anchors.leftMargin: 0
                            anchors.bottomMargin: 6
                            anchors.bottom: rowPlayer.top
                            spacing: 10
                            property bool isFile: true;

                            //                            spacing: 10
                            Button{
                                id: btnChooseFilePath
                                objectName: "btnChooseFilePath"
                                width: (rowOpenPath.width - 2* rowOpenPath.spacing)*0.15
                                text: "Open File"
                                Layout.fillWidth: false
                                onClicked: {
                                    rowOpenPath.isFile = true
                                    fileDialog.open();
                                }
                            }
                            Button {
                                id: btnChooseFolderPath
                                width: (rowOpenPath.width - 2* rowOpenPath.spacing)*0.15
                                text: "Open Folder"
                                Layout.fillWidth: false
                                objectName: "btnChooseFolderPath"
                                onClicked:{
                                    rowOpenPath.isFile = false
                                    folderDialog.open()
                                }
                            }
                            TextField {
                                id: textFilePath
//                                Layout.alignment: Qt.AlignRight | Qt.AlignVCenter
                                width: (rowOpenPath.width - 2* rowOpenPath.spacing)*0.7
                                Layout.fillWidth: true
                                visible: rowOpenPath.isFile
                                objectName: "textFilePath"
                                text: fileDialog.file
                                //                                anchors.left: parent.left + btnChooseFolderPath.width
                            }
                            TextField {
                                id: textFolderPath
//                                Layout.alignment: Qt.AlignRight | Qt.AlignVCenter
                                width: (rowOpenPath.width - 2* rowOpenPath.spacing)*0.7
                                visible: rowOpenPath.isFile == false
                                Layout.fillWidth: true
                                objectName: "textFolderPath"
                                text: folderDialog.folder
                                //                                anchors.left: parent.left + btnChooseFolderPath.width
                            }
                        }
                        RowLayout
                        {
                            id: rowPlayer
                            anchors.bottom: parent.bottom
                            anchors.horizontalCenter: parent.horizontalCenter
                            spacing: 10
                            Button{
                                id: playerStopBtn
                                property string iconName: "assets/stop.png"
                                objectName: "playerStopBtn"

                                width: 20
                                checkable: true
                                checked: true // continue
                                background: parent.transparent
                                icon.color: "transparent"
                                icon.source: iconName
                                onCheckedChanged: {
                                    if (playerStopBtn.checked)
                                    {
                                        playerStopBtn.iconName = "assets/stop.png"
                                    }
                                    else
                                    {
                                        playerStopBtn.iconName = "assets/play.png"
                                    }
                                }

                            }
                            SliderSpinBoxTwo {
                                id: sliderSpinBoxPlayer
                                objectName: "sliderSpinBoxPlayer"
                                from:0
                                stepSize: 1
                                value: 0
                                decimals: 0
                                anchors.left: parent.left + playerStopBtn.width
                                onValueChanged:
                                {
                                    if (sliderSpinBoxPlayer.value === sliderSpinBoxPlayer.to)
                                    {
                                        playerStopBtn.checked = false
                                    }

                                }
                            }
                        }



                    }
                    ColumnLayout
                    {
                        id:switchColTruck
                        anchors.bottom: parent.bottom
                        anchors.bottomMargin: 0
                        anchors.left: parent.left
                        anchors.leftMargin: 0
                        spacing: 5
                        SensorWidget{
                            id: sensorWgt
                            objectName: "sensorWidget"
                            Layout.preferredHeight: 120
                            Layout.preferredWidth: switchColTruck.width
//                            height: 120
//                            anchors.bottom: showBoxSwitch.top
                        }
                        RefreshSwitch {
                            id: showBoxSwitch
                            height: 30
                            text: "Hide box"
                            font.pointSize: 10
                            checked: true
                            offText:"Show box"
                            onCheckedChanged:
                            {
                                updateRender();
                            }
                        }
                        RefreshSwitch {
                            id: showSpeedSwitch
                            height: 30
                            text: "Hide speed"
                            font.pointSize: 10
                            checked: true
                            offText:"Show speed"
                            onCheckedChanged:
                            {
                                updateRender();
                            }
                        }
                    }
                    ColorButton {
                        id: roundButtonReset
                        anchors.bottom: parent.bottom
                        anchors.bottomMargin: 0
                        anchors.right: parent.right
                        anchors.rightMargin: 0
                        text: "Reset"
                        focusPolicy: Qt.WheelFocus
                        onClicked: {
                            canvasTrcukRender.centerX = canvasTrcukRender.width / 2
                            canvasTrcukRender.centerY =  canvasTrcukRender.height / 2
                            canvasTrcukRender.proportionw2p =  8
                            updateRender()
                        }
                    }
                }
            }
        }
    }
    function updateRender() {
        canvasTrcukRender.requestPaint()
    }
    function updateAlert() {
        canvasTrcukAlert.requestPaint()
    }

    function getAgents() {
        console.log(" getAgents data is", tableWidget.model.rowCount())
        var agents = []
        var i
        for (i = 0; i < tableWidget.model.rowCount(); ++i) {
            var data = {

            }
            data[tableWidget.colNames[3]] = tableWidget.model.get(i).posx
            data[tableWidget.colNames[4]] = tableWidget.model.get(i).posy
            data[tableWidget.colNames[5]] = tableWidget.model.get(i).width
            data[tableWidget.colNames[6]] = tableWidget.model.get(i).length
            data[tableWidget.colNames[7]] = tableWidget.model.get(i).heading
            data[tableWidget.colNames[11]] = tableWidget.model.get(i).itype
            agents.push(data)
            console.log("data is", data[tableWidget.colNames[3]],
                        data[tableWidget.colNames[4]],
                        data[tableWidget.colNames[5]],
                        data[tableWidget.colNames[6]],
                        data[tableWidget.colNames[7]],
                        data[tableWidget.colNames[11]])
        }
        return agents
    }

    // uint: m
    function world2Pixel(rect5) {
        var yp = Math.round(-rect5[0] * canvasTrcukRender.proportionw2p)
        var xp = Math.round(-rect5[1] * canvasTrcukRender.proportionw2p)
        var wp = Math.round(rect5[2] * canvasTrcukRender.proportionw2p)
        var lp = Math.round(rect5[3] * canvasTrcukRender.proportionw2p)
        var a = -rect5[4] * Math.PI / 180
        return [xp, yp, wp, lp, a]
    }

    // dis [0, 0.5] [0.5, 4.5]
    function world2PixelLR(rect5, dis) {
        var rec = [...rect5]
        rec[1] += dis[1]
        rec[2] = Math.abs(dis[1] - dis[0])
//        console.log("rect5 is",rec, dis)
        return world2Pixel(rec)
    }
//    function onModelReset() {
//         console.log("onModelReset --is");
//    }

//    Connections {
//        target: nodeStatusModel
//        function onModelReset() {
//            nodeWidget.model.clear()
////            console.log("nodeStatusModel is")
//            var i
//            for (i = 0; i < fusionModel.rowCount(); ++i) {
//                var data = nodeStatusModel.get(i)
//                nodeWidget.model.append(data)
////                console.log("data is", data[tableWidget.colNames[3]],
////                            data[tableWidget.colNames[4]],
////                            data[tableWidget.colNames[5]],
////                            data[tableWidget.colNames[6]],
////                            data[tableWidget.colNames[7]],
////                            data[tableWidget.colNames[11]])
//            }
//        }
//    }
    Component.onCompleted: {
        bsdViewSingal.icaInfoRevised.connect(onIcaInfoRevised);
        bsdViewSingal.egoAngleRevised.connect(onEgoAngleRevised);
//        bsdViewSingal.levelRevised.connect(onLevelRevised);
//        bsdViewSingal.agentRevised.connect(onAgentRevised);
        bsdViewSingal.timeStampRevised.connect(onTimeStampRevised);
        bsdViewSingal.speedRevised.connect(onSpeedRevised);
        bsdViewSingal.maxSlidesRevised.connect(onMaxSlidesRevised);
        bsdViewSingal.slidesNumRevised.connect(onSlidesNumRevised);
        bsdViewSingal.icaStatusRevised.connect(onIcaStatusRevised);
        bsdViewSingal.pncInfoRevised.connect(onPncInfoRevised);
        bsdViewSingal.bsdStatusRevised.connect(onBsdStatusRevised);
        bsdViewSingal.abStatusRevised.connect(onAbStatusRevised);
        bsdViewSingal.nodesStatusRevised.connect(onNodesStatusRevised);
        bsdViewSingal.turnLRRevised.connect(onTurnLRRevised);
        bsdViewSingal.imageChanged.connect(onImageChanged);

        fusionModel.modelReset.connect(onModelReset);
    }
    function onModelReset() {
//         console.log("onModelReset --is", fusionModel.rowCount());
        tableWidget.model.clear()
        var i
        for (i = 0; i < fusionModel.rowCount(); ++i) {
            var data = fusionModel.get(i)
            tableWidget.model.append(data)
//            console.log("data is", data[tableWidget.colNames[3]],
//                        data[tableWidget.colNames[4]],
//                        data[tableWidget.colNames[5]],
//                        data[tableWidget.colNames[6]],
//                        data[tableWidget.colNames[7]],
//                        data[tableWidget.colNames[11]])
        }
        updateRender();
    }
    function onImageChanged(val)
    {
//        mapImg.source = "";
//        mapImg.source = "image://MemoryImg";
        mapImg.reload();
    }

    function onEgoAngleRevised(jvalue) {
//        console.log('onEgoAngleRevised', jvalue)
        sliderSpinBoxEgoAngle.value = jvalue
        if (jvalue > 5)
        {
            valueSource.turnSignal = Qt.RightArrow
        }
        else if (jvalue < -5)
        {
            valueSource.turnSignal = Qt.LeftArrow
        }
        else
        {
            valueSource.turnSignal = -1
        }
    }
//    function onLevelRevised(tvalue) {
////            console.log('onLevelRevised', tvalue)
//        comboBoxDangerLevel.currentIndex = tvalue
//    }
//    function onAgentRevised(tvalue) {
////            console.log('onAgentRevised', tvalue)
//        comboBoxAgentType.currentIndex = tvalue
//    }
    function onTimeStampRevised(tvalue) {
//        console.log('onTimeStampRevised', tvalue)
        textTimeStamp.text = tvalue
        //var vals = tvalue.split(',')
//        console.log('onTimeStampRevised', vals[0], vals[1])
        //textTimeStamp.text = vals[1]
        //textCount.text = vals[0]
        //mapImg.source = vals[2]

    }
    function onSpeedRevised(tvalue) {
//            console.log('onSpeedRevised', tvalue * 3.6)
//            valuesource.kph = 100
        speedometer.gauge_value = tvalue * 3.6
    }
    function onMaxSlidesRevised(tvalue) {
//            console.log('onMaxSlidesRevised', tvalue)
        sliderSpinBoxPlayer.to = tvalue
    }
    function onSlidesNumRevised(tvalue) {
//            console.log('onSlidesNumRevised', tvalue)
        sliderSpinBoxPlayer.value = tvalue
    }
    function onIcaStatusRevised(tvalue) {
        bsdIcaStatus.icaStatus = tvalue
        bsdIcaStatus.updateIcaStatus()
    }
    function onBsdStatusRevised(tvalue) {
        bsdIcaStatus.bsdStatus = tvalue
        bsdIcaStatus.updateBsdStatus()
    }
    function onAbStatusRevised(tvalue) {
        bsdIcaStatus.abStatus = tvalue
    }
    function onIcaInfoRevised(val) {
//        console.log('onIcaInfoRevised', val)
        icaInfoText.text = val
    }
    function onPncInfoRevised(val) {
        pncInfoText.text = val
    }
    function onNodesStatusRevised(val) {
//            console.log('onNodesStatusRevised', val)
        txtDcu.text = val[0]
        txtNodes.text = val[1]

    }
    function onTurnLRRevised(tvalue) {
        console.log('onTurnLRRevised', tvalue)
        if (tvalue === 0) {
            radioBtnLeftSignal.checked = 1
            radioBtnRightSignal.checked = 0
        } else if (tvalue === 1) {
            radioBtnLeftSignal.checked = 0
            radioBtnRightSignal.checked = 1
        } else {
            radioBtnLeftSignal.checked = 0
            radioBtnRightSignal.checked = 0
            rowRadioBtns.radioBtnsCheckedIndex = tvalue
        }
        //            console.log('onTurnLR',radioBtnLeftSignal.checked, radioBtnRightSignal.checked,rowRadioBtns.radioBtnsCheckedIndex)
    }
}