# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

import sys
import os

from PyQt5.QtCore import QUrl,QObject
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtWidgets import QApplication, QLabel

from PyQt5.QtQuick import QQuickView, QQuickWindow, QSGRendererInterface
from PyQt5.QtQml import qmlRegisterType, QQmlApplicationEngine
from PyQt5.QtQml import QQmlComponent, QQmlEngine
import pickle

from ui.zf_bsd_view_model import PerceptionViewModel
import ui.zf_qml_qrc
from ui.zf_bsd_view import FusionObjList


def attach_online_predictor(view_model):
    try:
        from zf_traj_prediction.zf_common import EDatasource
        from zf_data_predictor_adapter import Predictor

        online_predictor = Predictor(data_source=EDatasource.OFFLINE_RELATIVE)
        view_model.set_online_prediction_provider(online_predictor.predict_agents_ui)
        return online_predictor
    except Exception as exc:
        print(f"Failed to attach online predictor: {exc}")
        return None

def view_load():
    app = QGuiApplication(sys.argv)
    # app = QApplication(sys.argv)
    app.setApplicationName('Collision Safety for Near-Field ADAS-Predictor V0.0.1')
    view = QQuickView()

    view_model = PerceptionViewModel()
    online_predictor = attach_online_predictor(view_model)
    view_singal = view_model.get_view()  # 这样在urgui里面定义的变量可以用self，否则只能定义全局变量
    view_obj_model = view_model.get_view_obj()
    node_status_model = view_model.get_view_node()


    view.rootContext().setContextProperty("bsdViewSingal", view_singal)
    view.rootContext().setContextProperty("fusionModel", view_obj_model)
    view.rootContext().setContextProperty("nodeStatusModel", node_status_model)

    # qmlRegisterType(FusionObjList, "FusionObjList", 1, 0, "FusionObjList")

    view.setResizeMode(QQuickView.SizeRootObjectToView)
#    qml_file = os.path.join(os.path.dirname(__file__), "qmlTruckGui/main.qml")
    #    qml_file = os.path.join(os.path.dirname(__file__), "qmlTruckGui/custom/Spinbox.qml")
    #    qml_file = os.path.join(os.path.dirname(__file__), "qmlTruckGui/RenderWidget.qml")
    ###########
    qml_file = "qrc:/qmlTruckGui/zf_bsd_display.qml"
    view.setSource(QUrl(qml_file))
    # qml_file = os.path.join(os.path.dirname(__file__), "qmlTruckGui/zf_bsd_display.qml")
    # view.setSource(QUrl.fromLocalFile(qml_file))
    print(qml_file)
    if view.status() == QQuickView.Error:
        print(view.status())
        sys.exit(-1)
    view.show()
    root_obj = view.rootObject()
    view_model.init_connect(root_obj)
    # view_model.test_data()
    # view_model.load_log()

    # gauge=view.findChild(QObject,'speedometer')
    # gauge.setProperty('gauge_value',100)

    sys.exit(app.exec())
    # Deleting the view before it goes out of scope is required to make
    # sure all child QML instances are destroyed in the correct order.
    del online_predictor
    del view

def engine_load():
    app = QGuiApplication(sys.argv)

    engine = QQmlApplicationEngine()
#    qml_file = os.path.join(os.path.dirname(__file__), "qmlTruckGui/zf_bsd_dishplay.qml")
    qml_file = os.path.join(os.path.dirname(__file__), "qmlTruckGui/main.qml")
    view_model = PerceptionViewModel()
    view_singal = view_model.get_view()  # 这样在urgui里面定义的变量可以用self，否则只能定义全局变量
    view_obj_model = view_model.get_view_obj()


    engine.rootContext().setContextProperty("bsdViewSingal", view_singal)
    engine.rootContext().setContextProperty("fusionModel", view_obj_model)

    # engine.setResizeMode(QQuickView.SizeRootObjectToView)
    url = QUrl.fromLocalFile(qml_file)
    engine = QQmlEngine()
    component = QQmlComponent(engine, url)
    Perception = component.create()
    # engine.load(qml_file)
    if not Perception:
        sys.exit(-1)
    sys.exit(app.exec())


def load_pkl():
    print("load pkl", os.getcwd())
    # fusion_path = "./Downloads/data_0312/ego_ags_hmi_0312_1336.pkl"
    # fusion_path = "./Downloads/data_0312/ego_ags_hmi_0312_1341.pkl"
    # fusion_path = "./Downloads/data_0312/ego_ags_hmi_0312_1444.pkl"
    fusion_path = r"C:\Users\z0221778\Downloads\OfflineHmiMsg.pickle"
    # fusion_path = "./Downloads/ego_ags_hmi_0312_1501_art_angle_0412.pkl"
    # fusion_path = "./Downloads/ego_ags_hmi_0312_1501_art_angle.pkl"
    # fusion_path = "./Downloads/new_ego_ags_hmi.pkl"
    # fusion_path = "./Downloads/ego_ags_hmi_0312_1501.pkl"
    with open(fusion_path, "rb") as f:
        _frames = pickle.load(f)
    # Mapping dictionary for itype

    path = r"D:\DL\data\20240312\1501\data_img"
    imgpathes = os.listdir(path)
    tm = float(imgpathes[0].replace(".jpg", ""))

    i = 0
    t = _frames[2]
    print(_frames[2])


if __name__ == "__main__":
    view_load()
    # load_pkl()
   # engine_load()
    # loadpkl()


#    QQuickWindow.setGraphicsApi(QSGRendererInterface.OpenGL)



#    qml_file = Path(__file__).parent / "qmlTruckGui/main.qml"
#    # qml_file = Path(__file__).parent / "qmlTruckGui/TabWidget.qml"
#    url = QUrl.fromLocalFile(qml_file)
#    engine = QQmlEngine()
#    component = QQmlComponent(engine, url)
#    Perception = component.create()
#    if Perception:
#        print(f"The person's name is ")
#    else:
#        print(component.errors())
#    del engine


