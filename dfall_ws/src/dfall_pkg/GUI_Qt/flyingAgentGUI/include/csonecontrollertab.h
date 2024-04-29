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
//    The GUI for a Csone Controller for students build from
//
//    ----------------------------------------------------------------------------------



// NOTE: Instructions for how to add a Chart are taken from this stack overflow answer:
// https://stackoverflow.com/questions/48362864/how-to-insert-qchartview-in-form-with-qt-designer
//
// Option 1: Promoted
// 1) first add QT += charts in the .pro
// 2) place the QGraphicsView to the design.
// 3) Right click on the QGraphicsView and select Promote to...
// 4) When doing the above, a menu appears, in the menu it should be set in QChartView in Promoted
//    Class Name, and QtCharts in Header file, then press the add button and finally press promote.
//
// Option 2: QtChart plugin
// This option was NOT used, see answer on stack overflow for further details




#ifndef CSONECONTROLLERTAB_H
#define CSONECONTROLLERTAB_H

#include <QWidget>
#include <QMutex>
#include <QVector>
#include <QLineEdit>
#include <QTextStream>

// For the chart:
#include <QChart>
#include <QLineSeries>
#include <QList>
#include <QPointF>
using namespace QtCharts;

#ifdef CATKIN_MAKE
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/package.h>

// Include the standard message types
//#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

// Include the DFALL message types
#include "dfall_pkg/IntWithHeader.h"
#include "dfall_pkg/SetpointWithHeader.h"
#include "dfall_pkg/CustomButtonWithHeader.h"

// Include the DFALL service types
#include "dfall_pkg/IntIntService.h"
#include "dfall_pkg/GetSetpointService.h"

// Include the shared definitions
#include "nodes/Constants.h"

// SPECIFY THE PACKAGE NAMESPACE
//using namespace dfall_pkg;

#else
// Include the shared definitions
#include "include/Constants_for_Qt_compile.h"

#endif





namespace Ui {
class CsoneControllerTab;
}

class CsoneControllerTab : public QWidget
{
    Q_OBJECT

public:
    explicit CsoneControllerTab(QWidget *parent = 0);
    ~CsoneControllerTab();



public slots:
    void setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll);
    void setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool isValid);
    void poseDataUnavailableSlot();



private slots:

    float validate_and_get_value_from_lineEdit(QLineEdit * lineEdit, float min, float max, int decimals, float default_value);

    void newDataForPerformingStepAndPlotting(float x, float pitch);
    void analyseStepResponse();
    void updateStepAnalysisLineEdit(float value, QLineEdit * lineEdit, int num_dec_places);


    void on_perform_step_button_clicked();
    void on_log_data_button_clicked();

    void on_simulate_step_response_button_clicked();
    void on_clear_simulation_button_clicked();

    void on_set_lead_compensator_parameters_button_clicked();

    void on_set_lag_compensator_parameters_button_clicked();

    void on_set_time_delay_button_clicked();

    void on_set_pitch_error_button_clicked();

    void on_toggle_integrator_button_clicked();
    void on_reset_integrator_button_clicked();

    void on_lineEdit_k_returnPressed();
    void on_lineEdit_T_returnPressed();
    void on_lineEdit_alpha_returnPressed();
    void on_lineEdit_T_lag_returnPressed();
    void on_lineEdit_alpha_lag_returnPressed();
    void on_lineEdit_kp_returnPressed();
    void on_lineEdit_kd_returnPressed();
    void on_lineEdit_time_delay_returnPressed();
    void on_lineEdit_pitch_error_returnPressed();
    void on_lineEdit_step_size_returnPressed();
    void on_lineEdit_step_duration_returnPressed();

    void on_lineEdit_k_editingFinished();
    void on_lineEdit_T_editingFinished();
    void on_lineEdit_alpha_editingFinished();
    void on_lineEdit_T_lag_editingFinished();
    void on_lineEdit_alpha_lag_editingFinished();

    void on_lineEdit_step_size_editingFinished();
    void on_lineEdit_step_duration_editingFinished();



    void simulate_step_response();


private:
    Ui::CsoneControllerTab *ui;

    // --------------------------------------------------- //
    // PRIVATE VARIABLES

    // The type of this node, i.e., agent or a coordinator,
    // specified as a parameter in the "*.launch" file
    int m_type = 0;

    // The ID  of this node
    int m_ID;

    // For coordinating multiple agents
    std::vector<int> m_vector_of_agentIDs_toCoordinate;
    bool m_shouldCoordinateAll = true;
    QMutex m_agentIDs_toCoordinate_mutex;

    // For the current setpoints
    float m_current_setpoint_x    = 0.0;
    float m_current_setpoint_y    = 0.0;
    float m_current_setpoint_z    = 0.4;
    float m_current_setpoint_yaw  = 0.0;

    // > Mutex for serialising acess to the controller variables
    QMutex m_controller_parameter_mutex;

    // > Mutex for serialising acess to the lag controller variables
    QMutex m_lag_controller_parameter_mutex;

    // FOR PLOTTING DATA ON THE CHART
    // > Mutex for serialising acess to any charting variable
    QMutex m_chart_mutex;
    // > Flag for whether to perform a step
    bool m_shouldPerformStep = false;
    // > Flag for whether to store data for plotting
    bool m_shouldStoreData_for_plotting = false;
    // > Flag for reseting the zoom level
    bool m_shouldResetZoom = false;
    // > Time (as a float) for the horizontal axis
    float m_time_for_step = 0.0f;
    // > The duration for which to record the step
    float m_step_response_data_recording_duration = 20.0f;
    // > Line Series for the x position data
    QLineSeries *m_lineSeries_for_setpoint_x;
    QLineSeries *m_lineSeries_for_measured_x;
    // > Line Series for the pitch angle data
    //QLineSeries *m_lineSeries_for_setpoint_pitch;
    QLineSeries *m_lineSeries_for_measured_pitch;


    // FOR SIMULATING THE CONTROLLER
    // > Mutex for serialising acess to any simulation variables
    QMutex m_simulation_mutex;

    // Flag for whether a simulation is under way
    bool m_simulationIsInProgress = false;

    // The state space matrices of lead compensator controller
    float m_lead_compensator_A=0.987578;
    float m_lead_compensator_B=0.00496888;
    float m_lead_compensator_C=-0.36;
    float m_lead_compensator_D=0.16;

    // The state of lead compensator controller
    float m_lead_compensator_state = 0.0;

    // The state space matrices of lag compensator controller
    float m_lag_compensator_A=0.0;
    float m_lag_compensator_B=0.0;
    float m_lag_compensator_C=0.0;
    float m_lag_compensator_D=1.0;

    // The state of lead compensator controller
    float m_lag_compensator_state = 0.0;

    // Line Series for plotting the simulation results
    QLineSeries *m_lineSeries_for_sim_setpoint_x;
    QLineSeries *m_lineSeries_for_sim_measured_x;
    QLineSeries *m_lineSeries_for_sim_measured_pitch;



#ifdef CATKIN_MAKE
    // PUBLISHER
    // > For requesting the setpoint to be changed
    ros::Publisher requestSetpointChangePublisher;

    // > For requesting the controller parameters to be changed
    ros::Publisher requestControllerParameterChangePublisher;

    // > For requesting the lag controller parameters to be changed
    ros::Publisher requestLagControllerParameterChangePublisher;

    // > For requesting a change in the integrator state
    ros::Publisher requestIntegratorStateChangePublisher;

    // > For requesting the time delay to be changed
    ros::Publisher requestTimeDelayChangePublisher;

    // > For requesting the pitch error to be changed
    ros::Publisher requestPitchErrorChangePublisher;

    // SUBSCRIBER
    // > For being notified when the setpoint is changed
    ros::Subscriber setpointChangedSubscriber;

    // > For being notified when the integrator state
    ros::Subscriber integratorStateChangedSubscriber;

    // PUBLISHER
    // > For notifying that a custom button is pressed
    ros::Publisher customButtonPublisher;
#endif



    // --------------------------------------------------- //
    // PRIVATE FUNCTIONS

#ifdef CATKIN_MAKE
    // For receiving message that the setpoint was changed
    void setpointChangedCallback(const dfall_pkg::SetpointWithHeader& newSetpoint);

    // For receiving message that the integrator state changes
    void integratorStateChangedCallback(const dfall_pkg::IntWithHeader& newIntegratorState);

    // Fill the header for a message
    void fillIntMessageHeader( dfall_pkg::IntWithHeader & msg );
    void fillSetpointMessageHeader( dfall_pkg::SetpointWithHeader & msg );
    void fillCustomButtonMessageHeader( dfall_pkg::CustomButtonWithHeader & msg );

    // Get the paramters that specify the type and ID
    bool getTypeAndIDParameters();
#endif

    void publishSetpoint(float x, float y, float z, float yaw_degrees);

    void publishControllerParameters(float k, float T, float alpha);

    void publishLagControllerParameters(float T, float alpha);

    void publishRequestForIntegratorStateChange(int flag_to_publish);

    void publishRequestForTimeDelayChange(int time_delay_to_publish);

    void publishRequestForPitchErrorChange(float pitch_error_to_publish);

    void convertIntoDiscreteTimeParameters(float k, float T, float alpha);

    void convertLagIntoDiscreteTimeParameters(float T, float alpha);

};

#endif // CSONECONTROLLERTAB_H
