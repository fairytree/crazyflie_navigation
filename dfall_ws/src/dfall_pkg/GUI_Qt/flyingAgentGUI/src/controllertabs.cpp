//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
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
//    The tab wdiget that contains and manages the individual tabs used
//    to interface with each separate controller.
//
//    ----------------------------------------------------------------------------------





#include "controllertabs.h"
#include "ui_controllertabs.h"

ControllerTabs::ControllerTabs(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControllerTabs)
{
    ui->setupUi(this);


    // Set all the banners to be the inactive colouring
    setAllBannersToControllerIsOff();

    // Initialise the sound effect for when the controller changes while flying
    //QSoundEffect m_soundEffect_controllerChanged;
    //m_soundEffect_controllerChanged.setSource(QUrl::fromLocalFile("engine.wav"));
    //m_soundEffect_controllerChanged.setLoopCount(1);
    //m_soundEffect_controllerChanged.setVolume(0.25f);
    //m_soundEffect_controllerChanged.play();


    // Initialise the object name as blank
    m_object_name_for_emitting_pose_data = "";


    // CONNECT THE "MEASURED POST" SIGNAL TO EACH OF
    // THE TABS
    // i.e., connect the "measured pose value changed"
    // signal to the "set measured pose" slots
    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->default_controller_tab_widget , &DefaultControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->student_controller_tab_widget , &StudentControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->picker_controller_tab_widget , &PickerControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->tuning_controller_tab_widget , &TuningControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->remote_controller_tab_widget , &RemoteControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->template_controller_tab_widget , &TemplateControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->csone_controller_tab_widget , &CsoneControllerTab::setMeasuredPose
        );

    QObject::connect(
            this , &ControllerTabs::measuredPoseValueChanged ,
            ui->tutorial_controller_tab_widget , &TutorialControllerTab::setMeasuredPose
        );



    // CONNECT THE "MEASUREMENTS UNAVAILABLE" SIGNAL TO
    // EACH OF THE TABS
    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->default_controller_tab_widget , &DefaultControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->student_controller_tab_widget , &StudentControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->picker_controller_tab_widget , &PickerControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->tuning_controller_tab_widget , &TuningControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->remote_controller_tab_widget , &RemoteControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->template_controller_tab_widget , &TemplateControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->csone_controller_tab_widget , &CsoneControllerTab::poseDataUnavailableSlot
        );

    QObject::connect(
            this , &ControllerTabs::poseDataUnavailableSignal ,
            ui->tutorial_controller_tab_widget , &TutorialControllerTab::poseDataUnavailableSlot
        );


    // CONNECT TO THE COORDINATOR SIGNAL TO BE ALWAYS UPDATED
    // WITH THE LIST OF AGENT IDs TO COORDINATE
    // This is passed from this "controller tabs widget" to
    // each of the controller tabs. The signal is simply
    // "passed through"
    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->default_controller_tab_widget , &DefaultControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->student_controller_tab_widget , &StudentControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->picker_controller_tab_widget , &PickerControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->tuning_controller_tab_widget , &TuningControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->remote_controller_tab_widget , &RemoteControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->template_controller_tab_widget , &TemplateControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->csone_controller_tab_widget , &CsoneControllerTab::setAgentIDsToCoordinate
        );

    QObject::connect(
            this , &ControllerTabs::agentIDsToCoordinateChanged ,
            ui->tutorial_controller_tab_widget , &TutorialControllerTab::setAgentIDsToCoordinate
        );

    



#ifdef CATKIN_MAKE

    //ros::init();

    // Get the namespace of this node
    std::string this_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[CONTROLLER TABS GUI] ros::this_node::getNamespace() =  " << this_namespace);

    // Get the type and ID of this flying agent GUI
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the node IDs are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[CONTROLLER TABS GUI] Node NOT FUNCTIONING :-)");
        ros::spin();
    }



    // GET THE "onload" PARAMETERS FOR WHAT DATA TO EMIT
    // > Create a "ros::NodeHandle" type local variable
    //   "nodeHandle" as the current node, the "~" indcates
    //   that "self" is the node handle assigned to this
    //   variable.
    ros::NodeHandle nodeHandle("~");
    // > For "poseDataMocap"
    if(!nodeHandle.getParam("onlaunch_shouldEmitPoseDataMocap", m_shouldEmitPoseDataMocap))
    {
        m_shouldEmitPoseDataMocap = false;
        ROS_ERROR("[CONTROLLER TABS GUI] missing parameter 'onlaunch_shouldEmitPoseDataMocap'");
    }
    // > For "poseDataOnboard"
    if(!nodeHandle.getParam("onlaunch_shouldEmitPoseDataOnboard", m_shouldEmitPoseDataOnboard))
    {
        m_shouldEmitPoseDataOnboard = false;
        ROS_ERROR("[CONTROLLER TABS GUI] missing parameter 'onlaunch_shouldEmitPoseDataOnboard'");
    }




    // CREATE A NODE HANDLE TO THIS GUI
    ros::NodeHandle nodeHandle_for_this_gui(this_namespace);

    // CREATE A NODE HANDLE TO THE D-FaLL ROOT
    ros::NodeHandle nodeHandle_dfall_root("/dfall");

    // CREATE THE SUBSCRIBER TO THE MOTION CAPTURE DATA
    m_poseDataMocapSubscriber = nodeHandle_dfall_root.subscribe("ViconDataPublisher/ViconData", 3, &ControllerTabs::poseDataMocapReceivedCallback, this);

    // CREATE THE SUBSCRIBER TO AGENT SPECIFIC TOPICS:
    // i.e., only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        // THE STATE ESTIMATE FROM ONBOARD THE FLYING AGENT
        m_poseDataOnboardSubscriber = nodeHandle_for_this_gui.subscribe("CrazyRadio/CFStateEstimate", 3, &ControllerTabs::poseDataOnboardReceivedCallback, this);
        // THE CONTROLLER THAT IS CURRENTLY OPERATING
        controllerUsedSubscriber = nodeHandle_for_this_gui.subscribe("FlyingAgentClient/ControllerUsed", 1, &ControllerTabs::controllerUsedChangedCallback, this);
    }

    // NOTE: FOR THE "poseDataMocap" AND "poseDataOnboard"
    // SUBSCRIBERS:
    // > The second argument is the queue_size, and the
    //   documentation states that:
    //   "If messages are arriving too fast and you are
    //    unable to keep up, roscpp will start throwing
    //    away messages."
    // > It is clearly stated that publisher queues throw
    //   away old messages first, and subscriber queues
    //   are likely the same.
    // > As we are only interested in latest measurement
    //   a short queue is used.



    // > Publisher for the emergency stop button
    //emergencyStopPublisher = nodeHandle_dfall_root.advertise<dfall_pkg::IntWithHeader>("EmergencyStop", 1);

    // > For changes in the database that defines {agentID,cfID,flying zone} links
    //databaseChangedSubscriber = nodeHandle_dfall_root.subscribe("CentralManagerService/DBChanged", 1, &TopBanner::databaseChangedCallback, this);;
    centralManagerDatabaseService = nodeHandle_dfall_root.serviceClient<dfall_pkg::CMQuery>("CentralManagerService/Query", false);


#endif

}

ControllerTabs::~ControllerTabs()
{
    delete ui;
}



void ControllerTabs::showHideController_toggle(QString qstr_label, QWidget * tab_widget_to_toggle)
{
    // Get the current index of the tab
    // > Note the this returns -1 if the tab is not found
    int current_index_of_tab = ui->controller_tabs_widget->indexOf(tab_widget_to_toggle);

    // Switch depending on whether the tab was found
    if (current_index_of_tab < 0)
    {
        // Insert the tab
        ui->controller_tabs_widget->addTab(tab_widget_to_toggle,qstr_label);
    }
    else
    {
        // Remove the tab
        ui->controller_tabs_widget->removeTab(current_index_of_tab);
    }
}



void ControllerTabs::showHideController_default_changed()
{
    showHideController_toggle("Default",ui->default_tab);
}

void ControllerTabs::showHideController_student_changed()
{
    showHideController_toggle("Student",ui->student_tab);
}

void ControllerTabs::showHideController_picker_changed()
{
    showHideController_toggle("Picker",ui->picker_tab);
}

void ControllerTabs::showHideController_tuning_changed()
{
    showHideController_toggle("Tuning",ui->tuning_tab);
}

void ControllerTabs::showHideController_remote_changed()
{
    showHideController_toggle("Remote",ui->remote_tab);
}

void ControllerTabs::showHideController_template_changed()
{
    showHideController_toggle("Template",ui->template_tab);
}

void ControllerTabs::showHideController_csone_changed()
{
    showHideController_toggle("CS1",ui->csone_tab);
}

void ControllerTabs::showHideController_tutorial_changed()
{
    showHideController_toggle("Tutorial",ui->tutorial_tab);
}




void ControllerTabs::setObjectNameForDisplayingPoseData( QString object_name )
{
    m_should_search_pose_data_for_object_name_mutex.lock();
    if (object_name.isEmpty())
    {
        // Set the class variable accordingly
        m_object_name_for_emitting_pose_data = "";
        // Update the flag accordingly
        m_should_search_pose_data_for_object_name = false;
        // Emit a signal to let the tabs know
        emit poseDataUnavailableSignal();
        // Inform the user
        #ifdef CATKIN_MAKE
            ROS_INFO("[CONTROLLER TABS GUI] No longer emitting pose data for any object.");
        #endif
    }
    else
    {
        // Set the class variable accordingly
        m_object_name_for_emitting_pose_data = object_name.toStdString();
        // Update the flag accordingly
        m_should_search_pose_data_for_object_name = true;
        // Inform the user
        #ifdef CATKIN_MAKE
            ROS_INFO_STREAM("[CONTROLLER TABS GUI] now emitting data for object named: " << m_object_name_for_emitting_pose_data << ", with ID = " << m_ID );
        #endif


        #ifdef CATKIN_MAKE
        // Get also the context
        dfall_pkg::CMQuery contextCall;
        contextCall.request.studentID = m_ID;

        centralManagerDatabaseService.waitForExistence(ros::Duration(-1));

        if(centralManagerDatabaseService.call(contextCall))
        {
            m_context = contextCall.response.crazyflieContext;
            m_area = m_context.localArea;
            ROS_INFO_STREAM("[CONTROLLER TABS GUI] AreaBounds:\n" << m_area);

            //qstr_crazyflie_name.append(QString::fromStdString(m_context.crazyflieName));

            //m_object_name_for_emitting_pose_data = QString::fromStdString(my_context.crazyflieName);

        }
        else
        {
            ROS_ERROR_STREAM("[CONTROLLER TABS GUI] Failed to load context for agentID = " << m_ID);
        }
        #endif

    }
    m_should_search_pose_data_for_object_name_mutex.unlock();
}


#ifdef CATKIN_MAKE
// > For the data from the motion capture system, received on
//   "m_poseDataMocapSubscriber"
void ControllerTabs::poseDataMocapReceivedCallback(const dfall_pkg::ViconData& viconData)
{
    m_should_search_pose_data_for_object_name_mutex.lock();
    if (m_should_search_pose_data_for_object_name)
    {
        for(std::vector<dfall_pkg::FlyingVehicleState>::const_iterator it = viconData.crazyflies.begin(); it != viconData.crazyflies.end(); ++it)
        {
            dfall_pkg::FlyingVehicleState pose_in_global_frame = *it;

            if(pose_in_global_frame.vehicleName == m_object_name_for_emitting_pose_data)
            {

                // Convert it into the local frame
                float originX = (m_area.xmin + m_area.xmax) / 2.0;
                float originY = (m_area.ymin + m_area.ymax) / 2.0;
                
                pose_in_global_frame.x -= originX;
                pose_in_global_frame.y -= originY;

                // change Z origin to zero, i.e., to the table height, zero of global coordinates, instead of middle of the box
                //float originZ = 0.0;
                // float originZ = (area.zmin + area.zmax) / 2.0;
                //pose_in_global_frame.z -= originZ;

                if (m_shouldEmitPoseDataMocap)
                {
                    emit measuredPoseValueChanged(
                            pose_in_global_frame.x,
                            pose_in_global_frame.y,
                            pose_in_global_frame.z,
                            pose_in_global_frame.roll,
                            pose_in_global_frame.pitch,
                            pose_in_global_frame.yaw,
                            pose_in_global_frame.isValid
                        );
                }
            }
        }
    }
    m_should_search_pose_data_for_object_name_mutex.unlock();
}
#endif





#ifdef CATKIN_MAKE
// > For the data from the motion capture system, received on
//   "m_poseDataOnboardSubscriber"
void ControllerTabs::poseDataOnboardReceivedCallback(const dfall_pkg::FlyingVehicleState& pose_in_local_frame)
{
    if (m_shouldEmitPoseDataOnboard)
    {
        emit measuredPoseValueChanged(
                pose_in_local_frame.x,
                pose_in_local_frame.y,
                pose_in_local_frame.z,
                pose_in_local_frame.roll,
                pose_in_local_frame.pitch,
                pose_in_local_frame.yaw,
                pose_in_local_frame.isValid
            );
    }
}
#endif





//    ----------------------------------------------------------------------------------
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
//    C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
//    C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
//    C      O   O  N  NN    T    R   R  O   O  L      L      E      R   R
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
//
//    EEEEE  N   N    A    BBBB   L      EEEEE  DDDD
//    E      NN  N   A A   B   B  L      E      D   D
//    EEE    N N N  A   A  BBBB   L      EEE    D   D
//    E      N  NN  AAAAA  B   B  L      E      D   D
//    EEEEE  N   N  A   A  BBBB   LLLLL  EEEEE  DDDD
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
// > For the controller currently operating, received on "controllerUsedSubscriber"
void ControllerTabs::controllerUsedChangedCallback(const std_msgs::Int32& msg)
{
    //ROS_INFO_STREAM("[COORDINATOR ROW GUI] Controller Used Changed Callback called with msg.data = " << msg.data);
    setControllerEnabled(msg.data);
}
#endif


void ControllerTabs::setControllerEnabled(int new_controller)
{
    // Define the tab text highlight colour locally
    const static QColor tab_text_colour_highlight = QColor(40,120,40);

    // First set everything back to normal colouring
    setAllTabLabelsToNormalColouring();
    setAllBannersToControllerIsOff();

    // Lock the mutex
    m_change_highlighted_controller_mutex.lock();

    // Now switch to highlight the tab corresponding to
    // the enable controller
    switch(new_controller)
    {
       case DEFAULT_CONTROLLER:
       {
           setTextColourOfTabLabel( tab_text_colour_highlight , ui->default_tab );
           ui->statusBanner_default->setStatus(CONTROLLER_STATUS_BANNER_ACTIVE);
           break;
       }
        case DEMO_CONTROLLER:
        {
            //ui->controller_enabled_label->setText("Demo");
            break;
        }
        case STUDENT_CONTROLLER:
        {
            setTextColourOfTabLabel( tab_text_colour_highlight , ui->student_tab );
            ui->statusBanner_student->setStatus(CONTROLLER_STATUS_BANNER_ACTIVE);
            break;
        }
        case MPC_CONTROLLER:
        {
            //...
            break;
        }
        case REMOTE_CONTROLLER:
        {
            setTextColourOfTabLabel( tab_text_colour_highlight , ui->remote_tab );
            ui->statusBanner_remote->setStatus(CONTROLLER_STATUS_BANNER_ACTIVE);
            break;
        }
        case TUNING_CONTROLLER:
        {
            setTextColourOfTabLabel( tab_text_colour_highlight , ui->tuning_tab );
            ui->statusBanner_tuning->setStatus(CONTROLLER_STATUS_BANNER_ACTIVE);
            break;
        }
        case PICKER_CONTROLLER:
        {
            setTextColourOfTabLabel( tab_text_colour_highlight , ui->picker_tab );
            ui->statusBanner_picker->setStatus(CONTROLLER_STATUS_BANNER_ACTIVE);
            break;
        }
        case TEMPLATE_CONTROLLER:
        {
            setTextColourOfTabLabel( tab_text_colour_highlight , ui->template_tab );
            ui->statusBanner_template->setStatus(CONTROLLER_STATUS_BANNER_ACTIVE);
            break;
        }
        case CSONE_CONTROLLER:
        {
            setTextColourOfTabLabel( tab_text_colour_highlight , ui->csone_tab );
            ui->statusBanner_csone->setStatus(CONTROLLER_STATUS_BANNER_ACTIVE);
            break;
        }
        case TUTORIAL_CONTROLLER:
        {
            setTextColourOfTabLabel( tab_text_colour_highlight , ui->tutorial_tab );
            ui->statusBanner_tutorial->setStatus(CONTROLLER_STATUS_BANNER_ACTIVE);
            break;
        }
        default:
        {
            //...
            break;
        }
    }

    // Unlock the mutex
    m_change_highlighted_controller_mutex.unlock();
}


void ControllerTabs::setTextColourOfTabLabel(QColor color , QWidget * tab_widget)
{
    // Get the current index of the tab
    int current_index_of_tab = ui->controller_tabs_widget->indexOf(tab_widget);
    // Only apply the colour if the tab is found
    if (current_index_of_tab >= 0)
    {
        ui->controller_tabs_widget->tabBar()->setTabTextColor(current_index_of_tab, color);
    }
}


void ControllerTabs::setAllTabLabelsToNormalColouring()
{
    // Define the tab text normal colour locally
    const static QColor tab_text_colour_normal = Qt::black;

    // Lock the mutex
    m_change_highlighted_controller_mutex.lock();

    // Set all the colours to normal
    setTextColourOfTabLabel( tab_text_colour_normal , ui->default_tab );
    setTextColourOfTabLabel( tab_text_colour_normal , ui->student_tab );
    setTextColourOfTabLabel( tab_text_colour_normal , ui->picker_tab );
    setTextColourOfTabLabel( tab_text_colour_normal , ui->tuning_tab );
    setTextColourOfTabLabel( tab_text_colour_normal , ui->remote_tab );
    setTextColourOfTabLabel( tab_text_colour_normal , ui->template_tab );
    setTextColourOfTabLabel( tab_text_colour_normal , ui->csone_tab );
    setTextColourOfTabLabel( tab_text_colour_normal , ui->tutorial_tab );

    // Unlock the mutex
    m_change_highlighted_controller_mutex.unlock();
}

void ControllerTabs::setAllBannersToControllerIsOff()
{
    // Lock the mutex
    m_change_highlighted_controller_mutex.lock();

    // Set all the banners to the off status
    ui->statusBanner_default ->setStatus(CONTROLLER_STATUS_BANNER_OFF);
    ui->statusBanner_student ->setStatus(CONTROLLER_STATUS_BANNER_OFF);
    ui->statusBanner_picker  ->setStatus(CONTROLLER_STATUS_BANNER_OFF);
    ui->statusBanner_tuning  ->setStatus(CONTROLLER_STATUS_BANNER_OFF);
    ui->statusBanner_remote  ->setStatus(CONTROLLER_STATUS_BANNER_OFF);
    ui->statusBanner_template->setStatus(CONTROLLER_STATUS_BANNER_OFF);
    ui->statusBanner_csone   ->setStatus(CONTROLLER_STATUS_BANNER_OFF);
    ui->statusBanner_tutorial->setStatus(CONTROLLER_STATUS_BANNER_OFF);

    // Unlock the mutex
    m_change_highlighted_controller_mutex.unlock();
}

void ControllerTabs::setAllBannersToControllerCoordinator()
{
    // Lock the mutex
    m_change_highlighted_controller_mutex.lock();

    // Set all the banners to the coordinator status
    ui->statusBanner_default ->setStatus(CONTROLLER_STATUS_BANNER_COORD);
    ui->statusBanner_student ->setStatus(CONTROLLER_STATUS_BANNER_COORD);
    ui->statusBanner_picker  ->setStatus(CONTROLLER_STATUS_BANNER_COORD);
    ui->statusBanner_tuning  ->setStatus(CONTROLLER_STATUS_BANNER_COORD);
    ui->statusBanner_remote  ->setStatus(CONTROLLER_STATUS_BANNER_COORD);
    ui->statusBanner_template->setStatus(CONTROLLER_STATUS_BANNER_COORD);
    ui->statusBanner_csone   ->setStatus(CONTROLLER_STATUS_BANNER_COORD);
    ui->statusBanner_tutorial->setStatus(CONTROLLER_STATUS_BANNER_COORD);

    // Unlock the mutex
    m_change_highlighted_controller_mutex.unlock();
}





//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void ControllerTabs::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
{
    // Pass on the signal to the tabs
    emit agentIDsToCoordinateChanged( agentIDs , shouldCoordinateAll );


    // Lock the mutex
    m_agentIDs_toCoordinate_mutex.lock();
    // Add the "coordinate all" flag
    m_shouldCoordinateAll = shouldCoordinateAll;
    // Clear the previous list of agent IDs
    m_vector_of_agentIDs_toCoordinate.clear();
    // Copy across the agent IDs, if necessary
    if (!shouldCoordinateAll)
    {
        for ( int irow = 0 ; irow < agentIDs.length() ; irow++ )
        {
            m_vector_of_agentIDs_toCoordinate.push_back( agentIDs[irow] );
        }
    }
    // Unlock the mutex
    m_agentIDs_toCoordinate_mutex.unlock();


#ifdef CATKIN_MAKE
    // If there is only one agent to coordinate,
    // then subscribe to the relevant data
    if (agentIDs.length() == 1)
    {

        // // > Create the appropriate node handle
        QString agent_base_namespace = "/dfall/agent" + QString::number(agentIDs[0]).rightJustified(3, '0');
        ros::NodeHandle agent_base_nodeHandle(agent_base_namespace.toStdString());

        
        // > Request the current instant controller
        ros::ServiceClient getInstantControllerService = agent_base_nodeHandle.serviceClient<dfall_pkg::IntIntService>("FlyingAgentClient/GetInstantController", false);
        dfall_pkg::IntIntService getInstantControllerCall;
        getInstantControllerCall.request.data = 0;
        getInstantControllerService.waitForExistence(ros::Duration(0.1));
        if(getInstantControllerService.call(getInstantControllerCall))
        {
            setControllerEnabled(getInstantControllerCall.response.data);
        }
        else
        {
            //setControllerEnabled(CONTROLLER_UNKNOWN);
        }

        // SUBSCRIBERS

        m_change_highlighted_controller_mutex.lock();
        // > For receiving the state estimate from onboard the fying agent
        m_poseDataOnboardSubscriber = agent_base_nodeHandle.subscribe("CrazyRadio/CFStateEstimate", 3, &ControllerTabs::poseDataOnboardReceivedCallback, this);
        // > For receiving messages that the instant controller was changed
        controllerUsedSubscriber = agent_base_nodeHandle.subscribe("FlyingAgentClient/ControllerUsed", 1, &ControllerTabs::controllerUsedChangedCallback, this);
        m_change_highlighted_controller_mutex.unlock();
    }
    else
    {
        // Unsubscribe
        m_change_highlighted_controller_mutex.lock();
        m_poseDataOnboardSubscriber.shutdown();
        controllerUsedSubscriber.shutdown();
        m_change_highlighted_controller_mutex.unlock();

        // Set all tabs to be normal colours
        setAllTabLabelsToNormalColouring();

        // Set all current controller banners to be in "coordinator mode"
        setAllBannersToControllerCoordinator();
    }
#endif

}





//    ----------------------------------------------------------------------------------
//    III  DDDD       &&&      TTTTT  Y   Y  PPPP   EEEEE
//     I   D   D     &           T     Y Y   P   P  E
//     I   D   D      &          T      Y    PPPP   EEE
//     I   D   D     & & &       T      Y    P      E
//    III  DDDD       &&&        T      Y    P      EEEEE
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
bool ControllerTabs::getTypeAndIDParameters()
{
    // Initialise the return variable as success
    bool return_was_successful = true;

    // Create a "ros::NodeHandle" type local variable "nodeHandle" as the current node,
    // the "~" indcates that "self" is the node handle assigned to this variable.
    ros::NodeHandle nodeHandle("~");

    // Get the value of the "type" parameter into a local string variable
    std::string type_string;
    if(!nodeHandle.getParam("type", type_string))
    {
        // Throw an error if the agent ID parameter could not be obtained
        ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] Failed to get type");
    }

    // Set the "m_type" class variable based on this string loaded
    if ((!type_string.compare("coordinator")))
    {
        m_type = TYPE_COORDINATOR;
    }
    else if ((!type_string.compare("agent")))
    {
        m_type = TYPE_AGENT;
    }
    else
    {
        // Set "m_type" to the value indicating that it is invlid
        m_type = TYPE_INVALID;
        return_was_successful = false;
        ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] The 'type' parameter retrieved was not recognised.");
    }


    // Construct the string to the namespace of this Paramater Service
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            // Get the value of the "agentID" parameter into the class variable "m_Id"
            if(!nodeHandle.getParam("agentID", m_ID))
            {
                // Throw an error if the agent ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[DEFAULT CONTROLLER TAB GUI] Is of type AGENT with ID = " << m_ID);
            }
            break;
        }

        // A COORDINATOR TYPE PARAMETER SERVICE IS REQUESTED FROM:
        // > The master GUI
        case TYPE_COORDINATOR:
        {
            // Get the value of the "coordID" parameter into the class variable "m_Id"
            if(!nodeHandle.getParam("coordID", m_ID))
            {
                // Throw an error if the coord ID parameter could not be obtained
                return_was_successful = false;
                ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[DEFAULT CONTROLLER TAB GUI] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[DEFAULT CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
