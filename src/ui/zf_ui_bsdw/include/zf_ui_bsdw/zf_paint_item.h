#ifndef QT_PAINT_ITEM_H
#define QT_PAINT_ITEM_H

#include <QQuickImageProvider>
#include <QImage>
 
class QtImageProvider : public QQuickImageProvider
{
public:
    QtImageProvider();
 
    QImage requestImage(const QString &id, QSize *size, const QSize& requestedSize);
    QPixmap requestPixmap(const QString &id, QSize *size, const QSize& requestedSize);
 
    void SetImageRc(const QImage &image);
private:
    QImage m_img;
};

// #include <QQuickItem>
// #include <QSGNode>
// #include <QSGSimpleRectNode>
// #include <QSGSimpleTextureNode>
// #include <QQuickWindow>
// #include <QImage>
// class QtPaintItem : public QQuickItem
// {
//     Q_OBJECT
// public:
//     explicit QtPaintItem(QQuickItem *parent = nullptr);
 
// public slots:
//     void updateImage(const QImage &);
 
// protected:
//     QSGNode * updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *) override;
 
// private:
//     QImage m_imageThumb;
// };
#endif // QT_PAINT_ITEM_H