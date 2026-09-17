#include "zf_ui_bsdw/zf_paint_item.h"
#include <QDebug>

QtImageProvider::QtImageProvider() : QQuickImageProvider(QQuickImageProvider::Image)
{
    QImage img(100, 100, QImage::Format_RGB888);
    img.fill(QColor(Qt::blue));
    m_img = img;
}

QImage QtImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    //    qDebug()<<"QtImageProvider requestImage"<<id<<size<<requestedSize;
    return m_img;
}

QPixmap QtImageProvider::requestPixmap(const QString &id, QSize *size, const QSize &requestedSize)
{
    //    qDebug()<<"QtImageProvider requestPixmap";
    return QPixmap::fromImage(m_img);
}

void QtImageProvider::SetImageRc(const QImage &image)
{
    m_img = image;
}

// QtPaintItem::QtPaintItem(QQuickItem *parent) : QQuickItem(parent)
// {
//     //这句不加会报错
//     setFlag(ItemHasContents, true);
//     //默认图片
//     // m_imageThumb = QImage(":/Image/background.png");
//     QImage img(100,100,QImage::Format_RGB888);
//     img.fill(QColor(Qt::blue));
//     m_imageThumb = img;
// }

// void QtPaintItem::updateImage(const QImage &image)
// {
//     m_imageThumb = image;
//     update();
// }

// QSGNode * QtPaintItem::updatePaintNode(QSGNode *oldNode, QQuickItem::UpdatePaintNodeData *)
// {
//     auto node = dynamic_cast<QSGSimpleTextureNode *>(oldNode);

//     if(!node){
//         node = new QSGSimpleTextureNode();
//     }

//     QSGTexture *m_texture = window()->createTextureFromImage(m_imageThumb, QQuickWindow::TextureIsOpaque);
//     node->setOwnsTexture(true);
//     node->setRect(boundingRect());
//     node->markDirty(QSGNode::DirtyForceUpdate);
//     node->setTexture(m_texture);

//     return node;
// }