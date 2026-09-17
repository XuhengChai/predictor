#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickView>
#include <rclcpp/rclcpp.hpp>

#include "zf_ui_bsdw/zf_bsd_view_model.h"

// #include <iostream>
// #include <fstream>
// #include <string>

// std::string parseFromLine(const std::string &line)
// {
//     zf::ui::MsgUi msg;
//     std::stringstream data(line);
//     data >> msg.time_stamp >> msg.articulation_angle >> msg.yaw_rate >>
//     msg.speed >> msg.turn_light_lr >> msg.ica_action_guide >>
//     msg.ica_trigger_ag_id >> msg.ica_target_time >> msg.ica_target_speed;

//     size_t agentCount, agentDataSize;
//     data >> agentCount;
//     data >> agentDataSize;
//     std::vector<std::vector<std::string>> agents;
//     for (size_t i = 0; i < agentCount; ++i)
//     {
//         std::vector<std::string> agent;
//         for (size_t j = 0; j < agentDataSize; ++j)
//         {
//             std::string agentStr;
//             data >> agentStr;
//             agent.push_back(agentStr);
//         }
//         agents.push_back(agent);
//     }

//     std::stringstream ss;
//     ss << msg.time_stamp << " " << msg.articulation_angle << " "
//        << msg.yaw_rate << " " << msg.speed << " "
//        << msg.turn_light_lr << " " << msg.ica_action_guide << " "
//        << msg.ica_trigger_ag_id << " " << msg.ica_target_time << " "
//        << msg.ica_target_speed << " ";
//     auto agentCnt = agents.size();
//     ss << agents.size() << " ";
//     if (agentCnt)
//     {
//         ss << agents[0].size() << " ";
//     }
//     for (auto &agt : agents)
//     {
//         for (const auto &agentStr : agt)
//         {
//             ss << agentStr << " ";
//         }
//     }

//     return ss.str();
// }

// int main()
// {
//     std::ifstream file("/home/nvidia/Music/log/2024-07/2024-07-11
//     13:15:32/can/planning.log"); // Replace "filename.txt" with the actual
//     file path std::cout << " open the file." << std::endl;

//     if (file.is_open())
//     {
//         std::string line;
//         while (std::getline(file, line))
//         {
//             // Process each line here
//             // std::cout << line.c_str() << std::endl; // Print the line
//             auto new_line = parseFromLine(line);
//             zf::LOG_WARN() << new_line.compare(line) ;

//             // Do additional processing if needed
//         }
//         std::cout << " opened." << std::endl;

//         file.close();
//     }
//     else
//     {
//         std::cout << "Unable to open the file." << std::endl;
//     }

//     return 0;
// }

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // need Window
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QGuiApplication app(argc, argv);
  QQmlApplicationEngine engine;

  zf::ui::PerceptionViewModel *viewModel = new zf::ui::PerceptionViewModel;
  auto guiView = viewModel->GetView();
  auto guiViewObj = viewModel->GetViewObj();

  engine.rootContext()->setContextProperty("bsdViewSingal", guiView);
  engine.rootContext()->setContextProperty("fusionModel", guiViewObj);
  //    QObject::connect(test, &QAbstractItemModel::modelReset, &engine,
  //    [&engine]() {
  //        QMetaObject::invokeMethod(engine.rootObjects().first(),
  //        "onModelReset");
  //    });
  engine.addImageProvider("MemoryImg", guiView->ImgProvider());
  const QUrl url(QStringLiteral("qrc:///qmlTruckGui/zf_bsd_display.qml"));
  // const QUrl url(QStringLiteral("qrc:///qmlTruckGui/NodesWidget.qml"));
  QObject::connect(
      &engine, &QQmlApplicationEngine::objectCreated, &app,
      [url](QObject *obj, const QUrl &objUrl)
      {
        if (!obj && url == objUrl)
          QCoreApplication::exit(-1);
      },
      Qt::QueuedConnection);
  engine.load(url);
  if (engine.rootObjects().isEmpty())
    return -1;
  viewModel->InitConnect(engine.rootObjects().first());
  //    test->SetValueList(m_list);
  //    viewModel->SetIcaInfo("no_attention \n ");
  // viewModel->test_data();

  return app.exec();
}

// int main(int argc, char *argv[])
//{
//     QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
//     QGuiApplication app(argc, argv);
////    FusionObjList *test = new FusionObjList;
//    QQuickView view;
////    view.rootContext()->setContextProperty("fusionModel", test);
//    PerceptionViewModel *viewModel = new PerceptionViewModel;
//    auto guiView = viewModel->GetView();
//    auto guiViewObj = viewModel->GetViewObj();

//    view.rootContext()->setContextProperty("bsdViewSingal", guiView);
//    view.rootContext()->setContextProperty("fusionModel", guiViewObj);
////    auto engine = view.engine();

////    QObject::connect(test, &QAbstractItemModel::modelReset, &engine,
///[&engine]() { / QMetaObject::invokeMethod(engine.rootObjects().first(),
///"onModelReset"); /    });

//    const QUrl url(QStringLiteral("qrc:///qmlTruckGui/zf_bsd_display.qml"));
//    view.connect(view.engine(), &QQmlEngine::quit, &app,
//    &QCoreApplication::quit); view.setSource(url);

//    view.show();
////    test->SetValueList(m_list);
//    viewModel->test_data();

//    return app.exec();
//}
