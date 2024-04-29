#include "controllerstatusbanner.h"
#include "ui_controllerstatusbanner.h"

ControllerStatusBanner::ControllerStatusBanner(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControllerStatusBanner)
{
    ui->setupUi(this);

    // HIDE ALL EXCEPT THE "OFF" BANNER
    ui->frame_isOff->show();
    ui->frame_isActive->hide();
    ui->frame_isCoord->hide();
}

ControllerStatusBanner::~ControllerStatusBanner()
{
    delete ui;
}





// PUBLIC METHODS FOR SETTING PROPERTIES

// > For making the "enable flight" and "disable flight" buttons unavailable
void ControllerStatusBanner::setStatus(int new_status)
{
    // LOCK THE MUTEX
    m_status_mutex.lock();

    // CHECK IF THE "new_status" IS DIFFERENT FROM THE "m_current_status"
    if (new_status!=m_current_status)
    {
        // UPDATE THE CLASS VARIABLE TO THE NEW STATUS
        m_current_status = new_status;

        // SWITCH BETWEEN THE POSSIBLE STATUS VALUES
        switch (new_status)
        {
            case CONTROLLER_STATUS_BANNER_OFF:
            {
                ui->frame_isOff->show();
                ui->frame_isActive->hide();
                ui->frame_isCoord->hide();
                break;
            }
            case CONTROLLER_STATUS_BANNER_ACTIVE:
            {
                ui->frame_isOff->hide();
                ui->frame_isActive->show();
                ui->frame_isCoord->hide();
                break;
            }
            case CONTROLLER_STATUS_BANNER_COORD:
            {
                ui->frame_isOff->hide();
                ui->frame_isActive->hide();
                ui->frame_isCoord->show();
                break;
            }
            default:
            {
                // INFORM THE USER OF THE ERROR
                #ifdef CATKIN_MAKE
                ROS_ERROR("[CONTROLLER STATUS BANNER] the new_status requested is not a valid status value.");
                #else
                // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
                QTextStream(stdout) << "[CONTROLLER STATUS BANNER] the new_status requested is not a valid status value.";
                #endif
                break;
            }
        }
    }

    // UNLOCK THE MUTEX
    m_status_mutex.unlock();
}
