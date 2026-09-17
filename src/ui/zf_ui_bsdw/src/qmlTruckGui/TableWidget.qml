////import QtQuick.Window 2.12
////import QtQuick.Controls 1.4
//import QtQuick 2.12
//import QtQuick.Window 2.12
//import QtQuick.Controls 2.1
//import Qt.labs.qmlmodels 1.0

//Item {
//    id: frameTable
//    visible: true
//    anchors.fill: parent
//    height: 200
//    width:300
//    property var colNames: ['type',
//        'bsdlevel',
//        'icastatus',
//        'posx',
//        'posy',
//        'width',
//        'length',
//        'heading',
//        'velx',
//        'vely',
//        'name',
//        'itype'
//    ]
//    property var model: fusionModel
//    property var agentType: ["vehicle", "pedestrian", "cyclist", "motorcycle", "truck", "bus", "unknown", "mutiAgents"]
//    ListView {
//        id: agentTable
//        anchors.fill: parent
////        anchors.left: parent.left
////        anchors.top: parent.top
////        anchors.topMargin: 2
////        anchors.right: parent.right
////        anchors.bottom: parent.bottom
////        anchors.bottomMargin: 2
//        contentWidth: headerItem.width
//        flickableDirection: Flickable.HorizontalAndVerticalFlick

//        header: Row {
//            spacing: 1
//            function itemAt(index) { return repeater.itemAt(index) }
//            Repeater {
//                id: repeater
//                model: colNames
//                Label {
//                    text: modelData
//                    font.bold: true
//                    font.pixelSize: 12
//                    padding: 10
//                    background: Rectangle { color: "silver" }
//                }
//            }
//        }

////        ListModel {
////            id: agentModel
////            ListElement {
////                type: "veh"
////                level: "level 1"
////                status: "Noraml"
////                posx: 1.5
////                posy: 1.5
////                width: 2.3
////                length: 4.6
////                heading:30
////                velx: 0
////                vely: 0
////                name: "A Masterpiece"
////                itype: 0
////            }
////            ListElement {
////                type: "per"
////                level: "level 2"
////                status: "Noraml"
////                posx: -1.5
////                posy: 4.5
////                width: 0.7
////                length: 1.5
////                heading:-30
////                velx: 0
////                vely: 0
////                name: "Brilliance"
////                itype: 1
////            }
////        }
//        model: fusionModel
//        delegate: Column {
//            id: delegate
//            property int row: index
//            Row {
//                spacing: 1
//                Repeater {
//                    model: colNames
//                    ItemDelegate {
//                        property int column: index
//                        text: fusionModel.get(row)[modelData]
//                        width: agentTable.headerItem.itemAt(column).width
//                    }
////                    Component.onCompleted: {
////                        console.log(row, " fusionModel.get(row)", fusionModel.get(row).bsdLevel)
////                    }
//                }
//            }

//            Rectangle {
//                color: "silver"
//                width: parent.width
//                height: 1
//            }
//        }
//        ScrollIndicator.horizontal: ScrollIndicator { }
//        ScrollIndicator.vertical: ScrollIndicator { }
//        Connections {
//            target: fusionModel
//            // Connection definition
//            onModelReset: {
//                updateRender(); // Call the slot (function)
//            }
//        }
//    }
//}
import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 1.4

Item {
    id: frameTable
    visible: true
    anchors.fill: parent
    height: 200
    width:300
    property var colNames: ['type',
        'bsdlevel',
        'icastatus',
        'posx',
        'posy',
        'width',
        'length',
        'heading',
        'velx',
        'vely',
        'name',
        'itype'
    ]
    property var model: agentModel
    property var agentType: ["vehicle", "pedestrian", "cyclist", "motorcycle", "truck", "bus", "unknown", "mutiAgents"]

    TableView {
        id: agentTable
        anchors.fill: parent
        focus: true

        TableViewColumn { role: "type"; title: "Type"; width: 50; }
        TableViewColumn { role: "bsdlevel"; title: "Bsdlevel"; width: 30; }
        TableViewColumn { role: "icastatus"; title: "Icastatus"; width: 30; }
        TableViewColumn { role: "posx"; title: "Posx"; width: 60; }
        TableViewColumn { role: "posy"; title: "Posy"; width: 60; }
        TableViewColumn { role: "width"; title: "Width"; width: 100; }
        TableViewColumn { role: "length"; title: "Length"; width: 100; }
        TableViewColumn { role: "heading"; title: "Heading"; width: 100; }
        TableViewColumn { role: "velx"; title: "Velx"; width: 100; }
        TableViewColumn { role: "vely"; title: "Vely"; width: 100; }
        TableViewColumn { role: "name"; title: "Name"; width: 100; }
        TableViewColumn { role: "itype"; title: "Itype"; width: 100; }
//        model: fusionModel
//        Connections {
//            target: fusionModel
//            // Connection definition
//            onModelReset: {
//                agentTable.update()
//                updateRender(); // Call the slot (function)
//            }
//        }

        model: agentModel
        ListModel {
            id: agentModel
//            ListElement {
//                type: "veh"
//                bsdlevel: 0
//                icastatus: 1
//                posx: 1.5
//                posy: 1.5
//                width: 2.3
//                length: 4.6
//                heading:30
//                velx: 0
//                vely: 0
//                name: "A Masterpiece"
//                itype: 0
//            }
//            ListElement {
//                type: "per"
//                bsdlevel: 0
//                icastatus: 1
//                posx: -1.5
//                posy: 4.5
//                width: 0.7
//                length: 1.5
//                heading:-30
//                velx: 0
//                vely: 0
//                name: "Brilliance"
//                itype: 1
//            }
        }
    }
}
