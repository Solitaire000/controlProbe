#include "home.h"
#include <QDialog>
#include <QVBoxLayout>

Home::Home(QWidget* parent) : QMainWindow(parent)
{
    ui.setupUi(this);
    connect(ui.btnOpenControlProbe, &QPushButton::clicked,
        this, &Home::onOpenControlProbeClicked);
    connect(ui.tabWidget, &QTabWidget::tabCloseRequested, this, [this](int index) {
        QWidget* w = ui.tabWidget->widget(index);
        if (w == m_embeddedControlProbe)
        {
            ui.tabWidget->removeTab(index);
            m_embeddedControlProbe->deleteLater();
        }
        else
        {
            ui.tabWidget->removeTab(index);
            w->deleteLater();
        }
        });
    m_embeddedControlProbe = new controlProbe(ui.tabWidget);
    connect(m_embeddedControlProbe, &controlProbe::getImage, this, &Home::saveImage);
    connect(this, &Home::matchFinished, m_embeddedControlProbe, &controlProbe::showMatchResultImg);
    Mat resultImg;
    resultImg = imread("D:/files/universalPath/autocontrols/A_WB_PROGRAM/data/autocontrol/photos/needleMark 1/needleMark_2.bmp");
    m_embeddedControlProbe->resultImg = resultImg;
    emit matchFinished();
    
}

Home::~Home() {}

void Home::onOpenControlProbeClicked()
{
    if (m_useEmbeddedMode)
        openAsEmbeddedTab();
    else
        openAsDialog();
}

// ---------- 弹窗模式 ----------
void Home::openAsDialog()
{
    QDialog* dlg = new QDialog(this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);   // 关闭即自动delete，无需手动管理
    dlg->setWindowTitle(tr("控制探针"));

    auto* layout = new QVBoxLayout(dlg);
    layout->setContentsMargins(0, 0, 0, 0);
    controlProbe* cp = new controlProbe(dlg);
    layout->addWidget(cp);
    dlg->resize(cp->sizeHint());

    // controlProbe 内部点"关闭"按钮 -> 关闭对话框（触发 WA_DeleteOnClose 自动释放）
    connect(cp, &controlProbe::requestClose, dlg, &QDialog::close);

    dlg->show();   // 非模态；如果要模态用 dlg->exec()，但exec()是阻塞的，配合WA_DeleteOnClose时用show()更常见

}

// ---------- 嵌入 tabWidget 模式 ----------
void Home::openAsEmbeddedTab()
{
    if (m_embeddedControlProbe)
    {
        // 已存在，直接切换过去，不重复创建
        int idx = ui.tabWidget->indexOf(m_embeddedControlProbe);
        if (idx >= 0)
            ui.tabWidget->setCurrentIndex(idx);
        return;
    }

    // 不存在（第一次打开，或者上次关闭后），重新创建
    m_embeddedControlProbe = new controlProbe(ui.tabWidget);
    int idx = ui.tabWidget->addTab(m_embeddedControlProbe, tr("探针控制"));
    ui.tabWidget->setCurrentIndex(idx);

    connect(m_embeddedControlProbe, &controlProbe::requestClose,
        this, &Home::onEmbeddedControlProbeCloseRequested);
}

void Home::onEmbeddedControlProbeCloseRequested()
{
    if (!m_embeddedControlProbe)
        return;

    int idx = ui.tabWidget->indexOf(m_embeddedControlProbe);
    if (idx >= 0)
        ui.tabWidget->removeTab(idx);   // 先从tabWidget摘掉

    m_embeddedControlProbe->deleteLater();  // 再异步delete，安全，不在信号槽调用栈里直接delete自己
    // deleteLater执行完后，QPointer m_embeddedControlProbe 会自动变成 nullptr
    // 下次点击 openAsEmbeddedTab() 时 if(m_embeddedControlProbe) 判断为false，重新创建
}

int Home::saveImage(QString path) {
    // 更新m_cameraParas参数
    // m_embeddedControlProbe->m_cameraParas.width = 1080;
    // 如果路径为空，则只更新相机参数
    if (path == "") {
        return 0;
    }
    qDebug() << path << endl;
    const std::string path_str = path.toStdString();

    
    return 0;
}