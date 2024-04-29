//    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero
//
//    This file is part of D-FaLL-System.
//    
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//    
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//    
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    Abstract class to represent our reimplementation of QGraphicsRectItem. Will be the base
//    for table and crazyflie zone.
//
//    ----------------------------------------------------------------------------------


#include "myGraphicsRectItem.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>
#include <QPen>
#include <QBrush>
#include <QApplication>
#include <QVariant>
#include <string.h>

myGraphicsRectItem::myGraphicsRectItem(const QRectF & rect,  QGraphicsItem * parent)
    : QGraphicsRectItem(rect, parent)
{
    unlock();
    this->setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
    pen = new QPen(Qt::red);
    brush = new QBrush(Qt::red);

    tmp_rect = 0;
    _grabbers_created = false;
    resize_mode = false;
}

void myGraphicsRectItem::lock()
{
    this->setFlag(QGraphicsItem::ItemIsSelectable, false);
    this->setFlag(QGraphicsItem::ItemIsMovable, false);
    locked = true;
}

void myGraphicsRectItem::unlock()
{
    this->setFlag(QGraphicsItem::ItemIsSelectable, true);
    this->setFlag(QGraphicsItem::ItemIsMovable, true);
    locked = false;
}

void myGraphicsRectItem::deleteGrabbers()
{
    _bottomLeft_corner->setParentItem(NULL);
    _topLeft_corner->setParentItem(NULL);
    _topRight_corner->setParentItem(NULL);
    _bottomRight_corner->setParentItem(NULL);

    delete _bottomLeft_corner;
    delete _topLeft_corner;
    delete _topRight_corner;
    delete _bottomRight_corner;
    _grabbers_created = false;
}

QVariant myGraphicsRectItem::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (change == QGraphicsItem::ItemSelectedChange)
    {
        if (value == true)
        {
            qDebug("selected!");
            createGrabbers();
        }
        else
        {
            qDebug("unselected!");
            deleteGrabbers();
        }
    }
    if (change == QGraphicsItem::ItemPositionChange && scene()) // with this, block movement of object when we are hovering through cornergrabbers
    {
        if(checkCornerGrabbers() == CornerGrabber::noCorner)
        {
            //qDebug("move now!");
            return QPointF(value.toPointF().x(), value.toPointF().y());
        }
        else
        {
            //qDebug("dont move now!");
            return QPointF(pos().x(), pos().y());
        }
    }


    return QGraphicsItem::itemChange(change, value);
}

bool myGraphicsRectItem::grabbersAreCreated()
{
    return _grabbers_created;
}

void myGraphicsRectItem::setCornerPositions() //need to call this function whenever we chnge the size of the rectangle
{
    QRectF rect = this->rect();
    _bottomLeft_corner->setPos(rect.bottomLeft().x(), rect.bottomLeft().y());
    _topLeft_corner->setPos(rect.topLeft().x(), rect.topLeft().y());
    _topRight_corner->setPos(rect.topRight().x(), rect.topRight().y());
    _bottomRight_corner->setPos(rect.bottomRight().x(), rect.bottomRight().y());
}

void myGraphicsRectItem::createGrabbers()
{
    if(!grabbersAreCreated())
    {
        _bottomLeft_corner = new CornerGrabber(this, CornerGrabber::bottomLeft);
        _topLeft_corner = new CornerGrabber(this, CornerGrabber::topLeft);
        _topRight_corner = new CornerGrabber(this, CornerGrabber::topRight);
        _bottomRight_corner = new CornerGrabber(this, CornerGrabber::bottomRight);


        _bottomLeft_corner->installSceneEventFilter(this);
        _topLeft_corner->installSceneEventFilter(this);
        _topRight_corner->installSceneEventFilter(this);
        _bottomRight_corner->installSceneEventFilter(this);

        _grabbers_created = true;
        setCornerPositions();
    }
}

int myGraphicsRectItem::checkCornerGrabbers()
{

    // we return the first active found. Hopefully we never have two at the same time.
    if(_bottomLeft_corner->isActive())
        return CornerGrabber::bottomLeft;
    else if(_topLeft_corner->isActive())
        return CornerGrabber::topLeft;
    else if (_topRight_corner->isActive())
        return CornerGrabber::topRight;
    else if( _bottomRight_corner->isActive())
        return CornerGrabber::bottomRight;
    else
        return CornerGrabber::noCorner;               //0 is none
}

bool myGraphicsRectItem::anyGrabber()
{
    if(checkCornerGrabbers() != CornerGrabber::noCorner)
        return true;
    else
        return false;
}

void myGraphicsRectItem::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() != Qt::LeftButton)
        return;

    if(!locked)
    {
        createGrabbers();           //This is just in case they have not been created by now. We have a creator guardian anyhow. Change this maybe?
        if(anyGrabber())
        {
            resize_mode = true;
        }
    }

    QGraphicsRectItem::mousePressEvent(mouseEvent);
}

void myGraphicsRectItem::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    this->prepareGeometryChange();
    if(resize_mode)
    {
        switch(checkCornerGrabbers())
        {
            case CornerGrabber::bottomLeft:
            {
                qDebug("bottomLeft");
                QRectF resize_rect = this->rect();
                resize_rect.setBottomLeft(mouseEvent->pos());
                this->setRect(resize_rect.normalized());
                break;
            }
            case CornerGrabber::topLeft:
            {
                qDebug("topLeft");
                QRectF resize_rect = this->rect();
                resize_rect.setTopLeft(mouseEvent->pos());
                this->setRect(resize_rect.normalized());
                break;
            }
            case CornerGrabber::topRight:
            {
                qDebug("topRight");
                QRectF resize_rect = this->rect();
                resize_rect.setTopRight(mouseEvent->pos());
                this->setRect(resize_rect.normalized());
                break;
            }
            case CornerGrabber::bottomRight:
            {
                qDebug("bottomRight");
                QRectF resize_rect = this->rect();
                resize_rect.setBottomRight(mouseEvent->pos());
                this->setRect(resize_rect.normalized());
                break;
            }
            case CornerGrabber::noCorner:
                qDebug("No corner. Should never enter here");
            default:
                break;
        }
        setCornerPositions();
        rectSizeChanged();
    }
    QGraphicsRectItem::mouseMoveEvent(mouseEvent);
}

void myGraphicsRectItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{

    if (mouseEvent->button() != Qt::LeftButton)
        return;
    // TODO: stop resize mode
    if(resize_mode)
    {
        resize_mode = false;
    }
    QGraphicsRectItem::mouseReleaseEvent(mouseEvent);
}
