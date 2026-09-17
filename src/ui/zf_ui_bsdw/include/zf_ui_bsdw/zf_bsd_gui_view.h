#ifndef ZF_BSD_GUI_VIEW_H
#define ZF_BSD_GUI_VIEW_H
#include <QGuiApplication>
#include <QAbstractListModel>
#include <iostream>

#include "zf_global/in/zf_ui_global.h"
#include "zf_ui_bsdw/zf_paint_item.h"

BEGIN_NS_ZF_UI

enum ETurnLRType
{
    TURN_LEFT = 0,
    TURN_RIGHT = 1,
    STRAIGHT = 2
};

class BSDMainView : public QObject
{
    Q_OBJECT
    Q_PROPERTY(float egoAngle READ getEgoAngle WRITE setEgoAngle NOTIFY egoAngleRevised)
    Q_PROPERTY(float speed READ getSpeed WRITE setSpeed NOTIFY speedRevised)
    Q_PROPERTY(int turnLR READ getTurnLR WRITE setTurnLR NOTIFY turnLRRevised)
    //    Q_PROPERTY(int level READ getLevel WRITE setLevel NOTIFY levelRevised)
    //    Q_PROPERTY(int agent READ getAgent WRITE setAgent NOTIFY agentRevised)
    Q_PROPERTY(QString timeStamp READ getTimeStamp WRITE setTimeStamp NOTIFY timeStampRevised)
    Q_PROPERTY(QString icaInfo READ getIcaInfo WRITE setIcaInfo NOTIFY icaInfoRevised)
    Q_PROPERTY(QString pncInfo READ getIcaInfo WRITE setIcaInfo NOTIFY pncInfoRevised)
    Q_PROPERTY(int maxSlides READ getMaxSlides WRITE setMaxSlides NOTIFY maxSlidesRevised)
    Q_PROPERTY(int slidesNum READ getSlidesNum WRITE setSlidesNum NOTIFY slidesNumRevised)
    Q_PROPERTY(int bsdStatus READ getBsdStatus WRITE setBsdStatus NOTIFY bsdStatusRevised)
    Q_PROPERTY(int icaStatus READ getIcaStatus WRITE setIcaStatus NOTIFY icaStatusRevised)
    Q_PROPERTY(bool abStatus READ getAbStatus WRITE setAbStatus NOTIFY abStatusRevised)
    Q_PROPERTY(QVariantList nodesStatus READ getNodesStatus WRITE setNodesStatus NOTIFY nodesStatusRevised)
    // Q_PROPERTY(QImage image MEMBER m_image READ image WRITE setImage NOTIFY imageChanged)

public:
    BSDMainView() : QObject()
    {
        m_fEgoAngle = -999.0;
        m_iTurnLR = ETurnLRType::STRAIGHT;
        //        m_iLevel = EDangerLevel::LEVEL1;
        //        m_iAgent = EAgentType::AGENT_VEHICLE;
        m_sTimeStamp = "0.0";
        m_sIcaInfo = "";
        m_sPncInfo = "";
        m_fSpeed = 0.0;
        m_iMaxSlides = 9999;
        m_iSlideNum = 999;
        m_iBsdStatus = 999;
        m_iIcaStatus = 999;
        m_bAbStatus = true;
        m_lstNodesStatus = QVariantList();
        m_pImgProvider = new QtImageProvider();
    }

signals:
    void egoAngleRevised(float val);
    void speedRevised(float val);
    void turnLRRevised(int val);
    //    void levelRevised(int val);
    //    void agentRevised(int val);
    void timeStampRevised(const QString &val);
    void icaInfoRevised(const QString &val);
    void pncInfoRevised(const QString &val);
    void maxSlidesRevised(int val);
    void slidesNumRevised(float val);
    void bsdStatusRevised(int val);
    void icaStatusRevised(int val);
    void abStatusRevised(bool val);
    void nodesStatusRevised(const QVariantList &val);
    void imageChanged();
public slots:
    void setImage(const QImage &image)
    {
        // m_image = image;
        m_pImgProvider->SetImageRc(image);
        emit imageChanged();
    }
    // QImage image() const
    // {
    //     return m_image;
    // }
    QtImageProvider *ImgProvider() const
    {
        return m_pImgProvider;
    }
    void setEgoAngle(float val)
    {
        if (abs(m_fEgoAngle - val) > 0.01f)
        {
            m_fEgoAngle = val;
            emit egoAngleRevised(m_fEgoAngle);
        }
    }

    float getEgoAngle() const
    {
        return m_fEgoAngle;
    }

    void setSpeed(float val)
    {
        if (abs(m_fSpeed - val) > 0.01f)
        {
            m_fSpeed = val;
            emit speedRevised(m_fSpeed);
        }
    }

    float getSpeed() const
    {
        return m_fSpeed;
    }

    void setTurnLR(int val)
    {
        if (m_iTurnLR != val)
        {
            m_iTurnLR = val;
            emit turnLRRevised(m_iTurnLR);
        }
    }

    int getTurnLR() const
    {
        return m_iTurnLR;
    }

    //    void setLevel(int val)
    //    {
    //        if (m_iLevel != val)
    //        {
    //            m_iLevel = val;
    //            emit levelRevised(m_iLevel);
    //        }
    //    }

    //    int getLevel() const
    //    {
    //        return m_iLevel;
    //    }

    //    void setAgent(int val)
    //    {
    //        if (m_iAgent != val)
    //        {
    //            m_iAgent = val;
    //            emit agentRevised(m_iAgent);
    //        }
    //    }

    //    int getAgent() const
    //    {
    //        return m_iAgent;
    //    }

    void setTimeStamp(const QString &val)
    {
        if (m_sTimeStamp != val)
        {
            m_sTimeStamp = val;
            emit timeStampRevised(m_sTimeStamp);
        }
    }

    QString getTimeStamp() const
    {
        return m_sTimeStamp;
    }

    void setIcaInfo(const QString &val)
    {
        if (m_sIcaInfo != val)
        {
            m_sIcaInfo = val;
            emit icaInfoRevised(m_sIcaInfo);
            //            printf("emit icaInfoRevised");
        }
    }

    QString getIcaInfo() const
    {
        return m_sIcaInfo;
    }
    
    void setPncInfo(const QString &val)
    {
        if (m_sPncInfo != val)
        {
            m_sPncInfo = val;
            emit pncInfoRevised(m_sPncInfo);
        }
    }

    QString getPncInfo() const
    {
        return m_sPncInfo;
    }

    void setMaxSlides(int val)
    {
        if (m_iMaxSlides != val)
        {
            m_iMaxSlides = val;
            emit maxSlidesRevised(m_iMaxSlides);
        }
    }

    int getMaxSlides() const
    {
        return m_iMaxSlides;
    }

    void setSlidesNum(int val)
    {
        if (m_iSlideNum != val)
        {
            m_iSlideNum = val;
            emit slidesNumRevised(m_iSlideNum);
        }
    }

    float getSlidesNum() const
    {
        return m_iSlideNum;
    }

    void setBsdStatus(int val)
    {
        if (m_iBsdStatus != val)
        {
            m_iBsdStatus = val;
            emit bsdStatusRevised(m_iBsdStatus);
        }
    }

    int getBsdStatus() const
    {
        return m_iBsdStatus;
    }

    void setIcaStatus(int val)
    {
        if (m_iIcaStatus != val)
        {
            m_iIcaStatus = val;

            emit icaStatusRevised(m_iIcaStatus);
        }
    }

    int getIcaStatus() const
    {
        return m_iIcaStatus;
    }

    void setAbStatus(bool val)
    {
        if (m_bAbStatus != val)
        {
            m_bAbStatus = val;
            emit abStatusRevised(m_bAbStatus);
        }
    }

    bool getAbStatus() const
    {
        return m_bAbStatus;
    }

    void setNodesStatus(const QVariantList &val)
    {
        if (m_lstNodesStatus != val)
        {
            m_lstNodesStatus = val;
            emit nodesStatusRevised(m_lstNodesStatus);
        }
    }

    QVariantList getNodesStatus() const
    {
        return m_lstNodesStatus;
    }

private:
    float m_fEgoAngle;
    float m_fSpeed;
    int m_iTurnLR;
    int m_iLevel;
    int m_iAgent;
    QString m_sTimeStamp;
    QString m_sIcaInfo;
    QString m_sPncInfo;
    int m_iMaxSlides;
    int m_iSlideNum;
    int m_iBsdStatus;
    int m_iIcaStatus;
    bool m_bAbStatus;
    QVariantList m_lstNodesStatus;
    // QImage m_image;
    QtImageProvider *m_pImgProvider;
};

END_NS_ZF_UI

#endif // ZF_BSD_GUI_VIEW_H
