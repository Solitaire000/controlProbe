#pragma execution_character_set("utf-8")
#pragma once
#include <QWidget>
#include <QPointer>
#include "ui_home.h"
#include "../AutoControlDll/controlProbe.h"

class Home : public QMainWindow
{
    Q_OBJECT

public:
    explicit Home(QWidget* parent = nullptr);
    ~Home();

    int saveImage(QString path);

private slots:
    void onOpenControlProbeClicked();
    void onEmbeddedControlProbeCloseRequested();

signals:
    void matchFinished();

private:
    void openAsDialog();
    void openAsEmbeddedTab();
    

    Ui::HomeClass ui;
    QPointer<controlProbe> m_embeddedControlProbe;  // QPointer：对象被delete后自动变nullptr，避免野指针
    bool m_useEmbeddedMode = false;  // 模式开关


};