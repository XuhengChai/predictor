import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.3

Item {
    id: sensorWgt
    visible: true
    anchors.fill: parent
    height: 120
    width: 150
//    property var colNames: ["cam", "cam_pp", "srr", "srr_pp", "ipm", "ipm_pp","fusion"]
    property var colNames: ["cam", "cam_pp", "srr", "srr_pp", "ipm", "ipm_pp","fusion"]
    property var checkdList: [false, false, false, false, false, false, true]
    signal sensorCheckedChanged(var ckd, var id)
    Grid {
        id: checkboxGrid
        anchors.top: parent.top
        anchors.topMargin: 10
        anchors.left: parent.left
        anchors.leftMargin: 10
        //        anchors.horizontalCenter: sensorWgt.horizontalCenter
//        anchors.verticalCenter: sensorWgt.verticalCenter
        columns: 2
        rows: 4
        spacing: 10

        CheckBox{
            id:checkBoxCam;
            text:colNames[0]
            checked:checkdList[0]
            Layout.row:0;
            Layout.column: 0;
            onCheckedChanged:
            {
                //checkdList[0] = checkBoxCam.checked
                sensorWgt.sensorCheckedChanged(checkBoxCam.checked, 0)
            }
        }
        CheckBox{
            id:checkBoxCamPP;
            text:colNames[1]
            checked:checkdList[1]
            Layout.row:0;
            Layout.column: 1;
            onCheckedChanged:
            {
                //checkdList[1] = checkBoxCamPP.checked
                sensorWgt.sensorCheckedChanged(checkBoxCamPP.checked, 1)
            }
        }
        CheckBox{
            id:checkBoxSrr;
            text:colNames[2]
            checked:checkdList[2]
            Layout.row:1;
            Layout.column: 0;
            onCheckedChanged:
            {
                //checkdList[2] = checkBoxSrr.checked
                sensorWgt.sensorCheckedChanged(checkBoxSrr.checked, 2)
            }
        }
        CheckBox{
            id:checkBoxSrrPP;
            text:colNames[3]
            checked:checkdList[3]
            Layout.row:1;
            Layout.column: 1;
            onCheckedChanged:
            {
                //checkdList[3] = checkBoxSrrPP.checked
                sensorWgt.sensorCheckedChanged(checkBoxSrrPP.checked, 3)
            }
        }
        CheckBox{
            id:checkBoxIpm;
            text:colNames[4]
            checked:checkdList[4]
            Layout.row:2;
            Layout.column: 0;
            onCheckedChanged:
            {
                //checkdList[4] = checkBoxIpm.checked
                sensorWgt.sensorCheckedChanged(checkBoxIpm.checked, 4)
            }
        }
        CheckBox{
            id:checkBoxIpmPP;
            text:colNames[5]
            checked:checkdList[5]
            Layout.row:2;
            Layout.column: 1;
            onCheckedChanged:
            {
                //checkdList[5] = checkBoxIpmPP.checked
                sensorWgt.sensorCheckedChanged(checkBoxIpmPP.checked, 5)
            }
        }
        CheckBox{
            id:checkBoxFusion;
            text:colNames[6]
            checked:checkdList[6]
            Layout.row:2;
            Layout.column: 0;
            onCheckedChanged:
            {
                //checkdList[6] = checkBoxFusion.checked
                sensorWgt.sensorCheckedChanged(checkBoxFusion.checked, 6)
            }
        }
//        Repeater {
//            model:
//            CheckBox {
//                text: modelData
//                // Add necessary properties and behaviors to the checkboxes
//            }
//        }
    }
}