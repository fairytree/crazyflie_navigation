//    Copyright (C) 2020, ETH Zurich, D-ITET, Paul Beuchat
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
//    A banner that indicates the status of a controller.
//    Possible status values are: { OFF , ACTIVE , COORD }
//
//    ----------------------------------------------------------------------------------





#ifndef CONTROLLERSTATUSBANNER_H
#define CONTROLLERSTATUSBANNER_H

#include <QWidget>
#include <QMutex>

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>
#else
#include <QTextStream>
#endif

// STATUS VALUES
#define CONTROLLER_STATUS_BANNER_OFF     0
#define CONTROLLER_STATUS_BANNER_ACTIVE  1
#define CONTROLLER_STATUS_BANNER_COORD   2


namespace Ui {
class ControllerStatusBanner;
}

class ControllerStatusBanner : public QWidget
{
    Q_OBJECT

public:
    explicit ControllerStatusBanner(QWidget *parent = 0);
    ~ControllerStatusBanner();

    // PUBLIC METHODS FOR SETTING PROPERTIES
    // > Set the state of the checkbox
    void setStatus(int new_status);

private:
    // --------------------------------------------------- //
    // PRIVATE VARIABLES
    Ui::ControllerStatusBanner *ui;

    // > For the current status
    int m_current_status;

    // MUTEX FOR HANDLING ACCESS
    // > For the status variable and UI elements
    QMutex m_status_mutex;

};

#endif // CONTROLLERSTATUSBANNER_H
