// 使用真实窗口验证选择框和会话锁定，不确认网口，不启动 EtherCAT。
#include "monitor_window.h"
#include "diagnostic_axis.h"
#include <QApplication>
#include <QComboBox>
#include <QTimer>
#include <QPushButton>
#include <iostream>

extern monitor::SharedData sharedData;
int main(int argc, char** argv) {
    QApplication app(argc, argv);
    auto* window = new MonitorWindow(sharedData);
    window->show();
    auto* box = window->findChild<QComboBox*>("diagnosticAxisSelector");
    if (!box || box->count() != 16 || diagnostic::axis() != 0) return 1;
    for (int i = 1; i <= 15; ++i) {
        box->setCurrentIndex(i);
        if (diagnostic::axis() != i || box->currentData().toInt() != i) return 2;
    }
    if (!box->itemText(1).contains("Joint_Body") || !box->itemText(8).contains("Joint_l7") ||
        !box->itemText(15).contains("Joint_r7")) return 3;
    box->setCurrentIndex(0);
    if (diagnostic::axis() != 0 || diagnostic::lockSelection()) return 4;
    // 按下未选轴的确认网口按钮，必须保持尚未确认，不打开网口。
    bool clicked = false;
    for (auto* button : window->findChildren<QPushButton*>()) {
        if (button->text() == "Confirm connection") { button->click(); clicked = true; }
    }
    if (!clicked) return 8;
    if (sharedData.interfaceConfirmed.load()) return 5;
    box->setCurrentIndex(9);
    if (!diagnostic::lockSelection()) return 6;
    int result = 0;
    QTimer::singleShot(200, [&]() {
        if (box->isEnabled() || diagnostic::axis() != 9 || sharedData.interfaceConfirmed.load()) result = 7;
        // 生成可检查的界面图，显示所选轴、锁定状态和原有模式控件。
        window->grab().save("single_axis_gui_preview.png");
        window->close();
    });
    QObject::connect(window, &MonitorWindow::windowClosed, &app, &QApplication::quit);
    app.exec();
    delete window;
    if (!result) std::cout << "PASS: 15 joint labels, empty selection, locked session and clean no-bus shutdown\n";
    return result;
}
