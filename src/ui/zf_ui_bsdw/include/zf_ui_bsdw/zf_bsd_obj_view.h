#ifndef ZF_BSD_OBJ_VIEW_H
#define ZF_BSD_OBJ_VIEW_H
#include <QGuiApplication>
#include <QAbstractListModel>
#include <iostream>

#include "zf_global/in/zf_ui_global.h"
#include "zf_ui_struct.h"

BEGIN_NS_ZF_UI

class FusionObjList : public QAbstractListModel
{
    Q_OBJECT
    Q_ENUMS(MyRoles)
public:
    enum FusionObjRoles {
        Type = Qt::UserRole + 1,
        Bsdlevel = Type + 1,
        Icastatus = Bsdlevel + 1,
        PosX = Icastatus + 1,
        PosY = PosX + 1,
        Width = PosY + 1,
        Length = Width + 1,
        Heading = Length + 1,
        VelX = Heading + 1,
        VelY = VelX + 1,
        Name = VelY + 1,
        IType = Name + 1
    };
    FusionObjList()
    {
//        m_list.push_back({"veh", 1, 0, 7.5, 1.5, 2.3, 4.6, 30, 0, 0, "A Masterpiece", 0});
//        m_list.push_back({"ped", 2, -1, 12.0, -13.0, 0.8, 0.8, 180, 0, 0, "John Doe", 1});
    }
//    ~FusionObjList()
//    {

//    }
    using QAbstractListModel::QAbstractListModel;
    void SetValueList(const std::vector<FusionObjListItem> &items)
    {
//        Q_ASSERT_SAME_THREAD;
        beginResetModel();
        m_list = items;
        endResetModel();
    }

    QHash<int, QByteArray> roleNames() const override {
        QHash<int, QByteArray> roles;
        roles[Type] = "type";
        roles[Bsdlevel] = "bsdlevel";
        roles[Icastatus] = "icastatus";
        roles[PosX] = "posx";
        roles[PosY] = "posy";
        roles[Width] = "width";
        roles[Length] = "length";
        roles[Heading] = "heading";
        roles[VelX] = "velx";
        roles[VelY] = "vely";
        roles[Name] = "name";
        roles[IType] = "itype";
        return roles;
    }

    int rowCount(const QModelIndex & parent = QModelIndex()) const override {
        if (parent.isValid())
            return 0;
        return static_cast<int>(m_list.size());
    }
//    Q_SIGNAL bool textChanged(const QString & text);

    Q_SLOT QMap<QString, QVariant> get(int row)
    {
        if (row >= 0 && rowCount())
        {
            FusionObjListItem item = m_list[std::size_t(row)];
            QMap<QString, QVariant> dict;
            dict["type"] = QVariant(item.type);
            dict["bsdlevel"] = QVariant(item.bsdlevel);
            dict["icastatus"] = QVariant(item.icastatus);
            dict["posx"] = QVariant(item.posx);
            dict["posy"] = QVariant(item.posy);
            dict["width"] = QVariant(item.width);
            dict["length"] = QVariant(item.length);
            dict["heading"] = QVariant(item.heading);
            dict["velx"] = QVariant(item.velx);
            dict["vely"] = QVariant(item.vely);
            dict["name"] = QVariant(item.name);
            dict["itype"] = QVariant(item.itype);
            return dict;
        }
        return {};
    }

    bool setData(const QModelIndex &index, const QVariant &value, int role) override {
        if (!hasIndex(index.row(), index.column(), index.parent()) || !value.isValid())
            return false;
        FusionObjListItem &item = m_list[std::size_t(index.row())];
        switch (role) {
            case FusionObjRoles::Type:
                item.type = value.toString();
                break;
            case FusionObjRoles::Bsdlevel:
                item.bsdlevel = value.toInt();
                break;
            case FusionObjRoles::Icastatus:
                item.icastatus = value.toInt();
                break;
            case FusionObjRoles::PosX:
                item.posx = value.toDouble();
                break;
            case FusionObjRoles::PosY:
                item.posy = value.toDouble();
                break;
            case FusionObjRoles::Width:
                item.width = value.toDouble();
                break;
            case FusionObjRoles::Length:
                item.length = value.toDouble();
                break;
            case FusionObjRoles::Heading:
                item.heading = value.toInt();
                break;
            case FusionObjRoles::VelX:
                item.velx = value.toInt();
                break;
            case FusionObjRoles::VelY:
                item.vely = value.toInt();
                break;
            case FusionObjRoles::Name:
                item.name = value.toString();
                break;
            case FusionObjRoles::IType:
                item.itype = value.toInt();
                break;
            default:
                return false;
        }

        emit dataChanged(index, index, { role });

        return true;
    }

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override {
        if (!hasIndex(index.row(), index.column(), index.parent()))
            return {};

        const FusionObjListItem &item = m_list.at(std::size_t(index.row()));
        switch (role) {
            case FusionObjRoles::Type:
                return item.type;
            case FusionObjRoles::Bsdlevel:
                return item.bsdlevel;
            case FusionObjRoles::Icastatus:
                return item.icastatus;
            case FusionObjRoles::PosX:
                return item.posx;
            case FusionObjRoles::PosY:
                return item.posy;
            case FusionObjRoles::Width:
                return item.width;
            case FusionObjRoles::Length:
                return item.length;
            case FusionObjRoles::Heading:
                return item.heading;
            case FusionObjRoles::VelX:
                return item.velx;
            case FusionObjRoles::VelY:
                return item.vely;
            case FusionObjRoles::Name:
                return item.name;
            case FusionObjRoles::IType:
                return item.itype;
            default:
                return {};
        }
    }

private:
    std::vector<FusionObjListItem> m_list;
};

END_NS_ZF_UI
#endif // ZF_BSD_OBJ_VIEW_H
