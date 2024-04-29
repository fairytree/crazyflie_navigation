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





#include "csonecontrollertab.h"
#include "ui_csonecontrollertab.h"

CsoneControllerTab::CsoneControllerTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CsoneControllerTab)
{
    ui->setupUi(this);

    // Hide the two red frames that are used to indcated
    // when pose data is NOT valid
    ui->red_frame_position_left->setVisible(false);
    ui->red_frame_position_right->setVisible(false);

    // ------------------------------------------------------------- //
    // HIDE THE PD CONTROLLER FIELDS
    ui->label_pid_controller->hide();
    ui->label_kp->hide();
    ui->label_kd->hide();
    ui->lineEdit_kp->hide();
    ui->lineEdit_kd->hide();
    ui->set_pd_controller_parameters_button->hide();

    // ------------------------------------------------------------- //
    // HIDE THE INTEGRATOR FIELDS
    ui->label_integrator->hide();
    ui->label_integrator_state->hide();
    ui->toggle_integrator_button->hide();
    ui->reset_integrator_button->hide();


    // ------------------------------------------------------------- //
    // HIDE THE SIMULATION BUTTONS
    //ui->label_simulation->hide();
    //ui->label_simulation_blank->hide();
    //ui->simulate_step_response_button->hide();
    //ui->clear_simulation_button->hide();


    // ------------------------------------------------------------- //
    // SET VALIDATORS FOR THE LINE EDITS
    // > NOT USED BECAUSE DO NOT AUTOMATICALLY CLIP TO THE MIN AND MAX
    //ui->lineEdit_k->setValidator(new QDoubleValidator(-1.0,1.0,2,ui->lineEdit_k) );


    // ------------------------------------------------------------- //
    // SETUP THE CHART VIEW FOR THE X POSITION
    //
    // Set the sizing policy for the chart
    // > This needs to be set here because the sizing policy needs to be
    //   set for the chart, but only the sizing policy for the
    //   ui->chartView_for_x object is accessible through the graphical
    //   designer for the .ui file
    // > Syntax hint:
    //   void QWidget::setSizePolicy(QSizePolicy::Policy horizontal, QSizePolicy::Policy vertical)
    ui->chartView_for_x->setSizePolicy(QSizePolicy::MinimumExpanding,QSizePolicy::Expanding);
    ui->chartView_for_x->chart()->setMinimumHeight(600);

    // Hide the legend of the chart
    ui->chartView_for_x->chart()->legend()->hide();

    // Set the chart to have no title
    ui->chartView_for_x->chart()->setTitle("");

    // Set the theme of the chart
    ui->chartView_for_x->chart()->setTheme(QChart::ChartThemeLight);

    // Initialise the line series to be used for plotting
    m_lineSeries_for_setpoint_x = new QLineSeries();
    m_lineSeries_for_measured_x = new QLineSeries();

    m_lineSeries_for_sim_setpoint_x = new QLineSeries();
    m_lineSeries_for_sim_measured_x = new QLineSeries();

    // Add the line series to the chart
    ui->chartView_for_x->chart()->addSeries(m_lineSeries_for_setpoint_x);
    ui->chartView_for_x->chart()->addSeries(m_lineSeries_for_measured_x);

    ui->chartView_for_x->chart()->addSeries(m_lineSeries_for_sim_setpoint_x);
    ui->chartView_for_x->chart()->addSeries(m_lineSeries_for_sim_measured_x);

    // Set the style of the series
    m_lineSeries_for_setpoint_x->setPen(QPen( QBrush("blue") , 5.0 ));
    m_lineSeries_for_measured_x->setPen(QPen( QBrush("red")  , 5.0 ));

    m_lineSeries_for_sim_setpoint_x->setPen(QPen( QBrush("blue") , 5.0 , Qt::DotLine ));
    m_lineSeries_for_sim_measured_x->setPen(QPen( QBrush("red")  , 5.0 , Qt::DotLine ));

    // OPTIONS FOR THE LINE STYLE:
    // { Qt::SolidLine , Qt::DashLine , Qt::DotLine , Qt::DashDotLine , Qt::DashDotDotLine }

    // Set the initial axes limits
    ui->chartView_for_x->chart()->createDefaultAxes();
    ui->chartView_for_x->chart()->axisX()->setMin(-1.0);
    ui->chartView_for_x->chart()->axisX()->setMax(m_step_response_data_recording_duration);
    ui->chartView_for_x->chart()->axisY()->setMin(-0.6);
    ui->chartView_for_x->chart()->axisY()->setMax( 0.6);


    // ------------------------------------------------------------- //
    // SETUP THE CHART VIEW FOR THE PITCH ANGLE
    //
    // Set the sizing policy for the chart
    ui->chartView_for_pitch->setSizePolicy(QSizePolicy::MinimumExpanding,QSizePolicy::Expanding);
    ui->chartView_for_pitch->chart()->setMinimumHeight(600);

    // Hide the legend of the chart
    ui->chartView_for_pitch->chart()->legend()->hide();

    // Set the chart to have no title
    ui->chartView_for_pitch->chart()->setTitle("");

    // Set the theme of the chart
    ui->chartView_for_pitch->chart()->setTheme(QChart::ChartThemeLight);

    // Initialise the line series to be used for plotting
    //m_lineSeries_for_setpoint_pitch = new QLineSeries();
    m_lineSeries_for_measured_pitch = new QLineSeries();
    m_lineSeries_for_sim_measured_pitch = new QLineSeries();

    // Add the line series to the chart
    //ui->chartView_for_pitch->chart()->addSeries(m_lineSeries_for_setpoint_pitch);
    ui->chartView_for_pitch->chart()->addSeries(m_lineSeries_for_measured_pitch);
    ui->chartView_for_pitch->chart()->addSeries(m_lineSeries_for_sim_measured_pitch);

    // Set the style of the series
    //m_lineSeries_for_setpoint_pitch->setPen(QPen( QBrush("blue") , 5.0 ));
    m_lineSeries_for_measured_pitch->setPen(QPen( QBrush("red")  , 5.0 ));
    m_lineSeries_for_sim_measured_pitch->setPen(QPen( QBrush("red")  , 5.0 , Qt::DotLine  ));

    // Set the initial axes limits
    ui->chartView_for_pitch->chart()->createDefaultAxes();
    ui->chartView_for_pitch->chart()->axisX()->setMin(-1.0);
    ui->chartView_for_pitch->chart()->axisX()->setMax(m_step_response_data_recording_duration);
    ui->chartView_for_pitch->chart()->axisY()->setMin(-20.0);
    ui->chartView_for_pitch->chart()->axisY()->setMax(20.0);






#ifdef CATKIN_MAKE

    //ros::init();

    // Get the namespace of this node
    std::string this_namespace = ros::this_node::getNamespace();
    ROS_INFO_STREAM("[CSONE CONTROLLER TAB GUI] ros::this_node::getNamespace() =  " << this_namespace);

    // Get the type and ID of this flying agent GUI
    bool isValid_type_and_ID = getTypeAndIDParameters();

    // Stall if the node IDs are not valid
    if ( !isValid_type_and_ID )
    {
        ROS_ERROR("[CSONE CONTROLLER TAB GUI] Node NOT FUNCTIONING :-)");
        ros::spin();
    }


    // CREATE A NODE HANDLE TO THIS GUI
    ros::NodeHandle nodeHandle_for_this_gui(this_namespace);

    // CREATE THE REQUEST SETPOINT CHANGE PUBLISHER
    requestSetpointChangePublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::SetpointWithHeader>("CsoneControllerService/RequestSetpointChange", 1);

    // CREATE THE REQUEST CONTROLLER PARAMETER CHANGE PUBLISHER
    requestControllerParameterChangePublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::SetpointWithHeader>("CsoneControllerService/RequestControllerParametersChange", 1);

    // CREATE THE REQUEST LAG CONTROLLER PARAMETER CHANGE PUBLISHER
    requestLagControllerParameterChangePublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::SetpointWithHeader>("CsoneControllerService/RequestLagControllerParametersChange", 1);

    // CREATE THE REQUEST TIME DELAY CHANGE PUBLISHER
    requestTimeDelayChangePublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::IntWithHeader>("CsoneControllerService/RequestTimeDelayChange", 1);

    // CREATE THE REQUEST PITCH ERROR CHANGE PUBLISHER
    requestPitchErrorChangePublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::SetpointWithHeader>("CsoneControllerService/RequestPitchErrorChange", 1);

    // CREATE THE REQUEST INTEGRATOR STATE CHANGE PUBLISHER
    requestIntegratorStateChangePublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::IntWithHeader>("CsoneControllerService/RequestIntegratorStateChange", 1);

    // SUBSCRIBE TO SETPOINT CHANGES AND INTEGRATOR STATE CHANGES
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        setpointChangedSubscriber        = nodeHandle_for_this_gui.subscribe("CsoneControllerService/SetpointChanged"       , 1, &CsoneControllerTab::setpointChangedCallback       , this);
        integratorStateChangedSubscriber = nodeHandle_for_this_gui.subscribe("CsoneControllerService/IntegratorStateChanged", 1, &CsoneControllerTab::integratorStateChangedCallback, this);
    }

    // CREATE THE CUSTOM BUTTON PRESSED PUBLISHER
    customButtonPublisher = nodeHandle_for_this_gui.advertise<dfall_pkg::CustomButtonWithHeader>("CsoneControllerService/CustomButtonPressed", 1);

    // GET THE CURRENT SETPOINT AND CURRENT INTEGRATOR STATE
    // Only if this is an agent GUI
    if (m_type == TYPE_AGENT)
    {
        // > Request the current setpoint
        ros::ServiceClient getCurrentSetpointServiceClient = nodeHandle_for_this_gui.serviceClient<dfall_pkg::GetSetpointService>("CsoneControllerService/GetCurrentSetpoint", false);
        dfall_pkg::GetSetpointService getSetpointCall;
        getSetpointCall.request.data = 0;
        getCurrentSetpointServiceClient.waitForExistence(ros::Duration(0.1));
        if(getCurrentSetpointServiceClient.call(getSetpointCall))
        {
            setpointChangedCallback(getSetpointCall.response.setpointWithHeader);
        }
        else
        {
            // Inform the user
            ROS_INFO("[CSONE CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
        }

        // > Request the current integrator state
        ros::ServiceClient getCurrentIntegatorStateServiceClient = nodeHandle_for_this_gui.serviceClient<dfall_pkg::IntIntService>("CsoneControllerService/GetCurrentIntegratorState", false);
        dfall_pkg::IntIntService getIntegrtorStateCall;
        getIntegrtorStateCall.request.data = 0;
        getCurrentIntegatorStateServiceClient.waitForExistence(ros::Duration(0.1));
        if(getCurrentIntegatorStateServiceClient.call(getIntegrtorStateCall))
        {
             dfall_pkg::IntWithHeader temp_msg;
             temp_msg.data = getIntegrtorStateCall.response.data;
            integratorStateChangedCallback(temp_msg);
        }
        else
        {
            // Inform the user
            ROS_INFO("[CSONE CONTROLLER GUI] Failed to get current integrator state from controller using the \"GetCurrentIntegratorState\" service");
        }
    }

#endif

}

CsoneControllerTab::~CsoneControllerTab()
{
    delete ui;
}







float CsoneControllerTab::validate_and_get_value_from_lineEdit(QLineEdit * lineEdit, float min, float max, int decimals, float default_value)
{
    // Initialise the value to the default
    float return_value = default_value;

    // Update the duration from the field
    if(!lineEdit->text().isEmpty())
    {
        return_value = (lineEdit->text()).toFloat();
        // Ensure that it is in the range [min,max]
        if (return_value < min)
            return_value = min;
        else if (return_value > max)
            return_value = max;
    }

    // Clip the value to the specified decimal places


    // Put the value back into the line edit
    lineEdit->setText(QString::number( return_value, 'f', decimals));

    // Return the value
    return return_value;
}




//    ----------------------------------------------------------------------------------
//    BBBB   U   U  TTTTT  TTTTT   OOO   N   N   SSSS
//    B   B  U   U    T      T    O   O  NN  N  S
//    BBBB   U   U    T      T    O   O  N N N   SSS
//    B   B  U   U    T      T    O   O  N  NN      S
//    BBBB    UUU     T      T     OOO   N   N  SSSS
//    ----------------------------------------------------------------------------------



void CsoneControllerTab::on_perform_step_button_clicked()
{
    // Lock the mutex
    m_chart_mutex.lock();

    // Set the flag that a step should be performed
    m_shouldPerformStep = true;

    // Set the flag that the zoom levels should be reset
    m_shouldResetZoom = true;

    // Set the time back to be less than zero
    m_time_for_step = -1.0;

    // Set the minimum of the x-axis to agree with this
    ui->chartView_for_x    ->chart()->axisX()->setMin(m_time_for_step);
    ui->chartView_for_pitch->chart()->axisX()->setMin(m_time_for_step);

    // Update the duration from the field
    m_step_response_data_recording_duration = validate_and_get_value_from_lineEdit(ui->lineEdit_step_duration,1.0,100.0,0,20.0);

    // Set the minimum of the x-axis to agree with the duration
    ui->chartView_for_x    ->chart()->axisX()->setMax(m_step_response_data_recording_duration);
    ui->chartView_for_pitch->chart()->axisX()->setMax(m_step_response_data_recording_duration);


    // Clear any data from the line series
    // > For x position
    m_lineSeries_for_setpoint_x->removePoints(0,m_lineSeries_for_setpoint_x->count());
    m_lineSeries_for_measured_x->removePoints(0,m_lineSeries_for_measured_x->count());
    // > For pitch angles
    //m_lineSeries_for_setpoint_pitch->removePoints(0,m_lineSeries_for_setpoint_pitch->count());
    m_lineSeries_for_measured_pitch->removePoints(0,m_lineSeries_for_measured_pitch->count());

    // Set all the fields to be blank
    ui->lineEdit_step_start        ->setText("---");
    ui->lineEdit_step_end          ->setText("---");
    ui->lineEdit_overshoot_value   ->setText("---");
    ui->lineEdit_overshoot_percent ->setText("---");
    ui->lineEdit_rise_time         ->setText("---");
    ui->lineEdit_settling_time     ->setText("---");

    // Set the flag that should store data
    m_shouldStoreData_for_plotting = true;

    // Unlock the mutex
    m_chart_mutex.unlock();

    // Inform the user about the change
#ifdef CATKIN_MAKE
    ROS_INFO_STREAM("[CSONE CONTROLLER TAB GUI] Perform step started");
#endif
}

void CsoneControllerTab::on_log_data_button_clicked()
{
    // Lock the mutex
    m_chart_mutex.lock();

    // Set the flag that a step should be performed
    m_shouldPerformStep = false;

    // Set the flag that the zoom levels should be reset
    m_shouldResetZoom = true;

    // Set the time back to zero
    m_time_for_step = 0.0;

    // Set the minimum of the x-axis to agree with this
    ui->chartView_for_x    ->chart()->axisX()->setMin(m_time_for_step);
    ui->chartView_for_pitch->chart()->axisX()->setMin(m_time_for_step);

    // Update the duration from the field
    m_step_response_data_recording_duration = validate_and_get_value_from_lineEdit(ui->lineEdit_step_duration,1.0,100.0,0,20.0);

    // Set the minimum of the x-axis to agree with the duration
    ui->chartView_for_x    ->chart()->axisX()->setMax(m_step_response_data_recording_duration);
    ui->chartView_for_pitch->chart()->axisX()->setMax(m_step_response_data_recording_duration);

    // Clear any data from the line series
    // > For x position
    m_lineSeries_for_setpoint_x->removePoints(0,m_lineSeries_for_setpoint_x->count());
    m_lineSeries_for_measured_x->removePoints(0,m_lineSeries_for_measured_x->count());
    // > For pitch angles
    //m_lineSeries_for_setpoint_pitch->removePoints(0,m_lineSeries_for_setpoint_pitch->count());
    m_lineSeries_for_measured_pitch->removePoints(0,m_lineSeries_for_measured_pitch->count());

    // Set all the fields to be blank
    ui->lineEdit_step_start        ->setText("---");
    ui->lineEdit_step_end          ->setText("---");
    ui->lineEdit_overshoot_value   ->setText("---");
    ui->lineEdit_overshoot_percent ->setText("---");
    ui->lineEdit_rise_time         ->setText("---");
    ui->lineEdit_settling_time     ->setText("---");

    // Set the flag that should store data
    m_shouldStoreData_for_plotting = true;

    // Unlock the mutex
    m_chart_mutex.unlock();

    // Inform the user about the change
#ifdef CATKIN_MAKE
    ROS_INFO_STREAM("[CSONE CONTROLLER TAB GUI] Perform step started");
#endif
}




void CsoneControllerTab::on_simulate_step_response_button_clicked()
{
    // Initialise a flag for whether to start a simulation or not
    bool shouldStartSimulation = true;

    // Set the flag that a simulation is being performed
    m_simulation_mutex.lock();
    if (m_simulationIsInProgress)
    {
        shouldStartSimulation = false;
    }
    else
    {
        m_simulationIsInProgress = true;
        // Set the button to not be avaialble
        ui->simulate_step_response_button->setEnabled(false);
        ui->clear_simulation_button->setEnabled(false);
    }
    m_simulation_mutex.unlock();

    // Call the function that performs the simulation
    if (shouldStartSimulation)
    {
        // Lock the mutex
        m_chart_mutex.lock();

        // Clear the line series
        m_lineSeries_for_sim_setpoint_x->removePoints(0,m_lineSeries_for_sim_setpoint_x->count());
        m_lineSeries_for_sim_measured_x->removePoints(0,m_lineSeries_for_sim_measured_x->count());
        m_lineSeries_for_sim_measured_pitch->removePoints(0,m_lineSeries_for_sim_measured_pitch->count());

        // > For x position
        m_lineSeries_for_setpoint_x->removePoints(0,m_lineSeries_for_setpoint_x->count());
        m_lineSeries_for_measured_x->removePoints(0,m_lineSeries_for_measured_x->count());
        // > For pitch angles
        //m_lineSeries_for_setpoint_pitch->removePoints(0,m_lineSeries_for_setpoint_pitch->count());
        m_lineSeries_for_measured_pitch->removePoints(0,m_lineSeries_for_measured_pitch->count());

        // Unlock the mutex
        m_chart_mutex.unlock();

        simulate_step_response();
        //QTimer::singleShot(50, this, SLOT(simulate_step_response()));
    }
}


void CsoneControllerTab::on_clear_simulation_button_clicked()
{
    // Lock the mutex
    m_chart_mutex.lock();

    // First clear the line series
    m_lineSeries_for_sim_setpoint_x->removePoints(0,m_lineSeries_for_sim_setpoint_x->count());
    m_lineSeries_for_sim_measured_x->removePoints(0,m_lineSeries_for_sim_measured_x->count());
    m_lineSeries_for_sim_measured_pitch->removePoints(0,m_lineSeries_for_sim_measured_pitch->count());

    // > For x position
    m_lineSeries_for_setpoint_x->removePoints(0,m_lineSeries_for_setpoint_x->count());
    m_lineSeries_for_measured_x->removePoints(0,m_lineSeries_for_measured_x->count());
    // > For pitch angles
    //m_lineSeries_for_setpoint_pitch->removePoints(0,m_lineSeries_for_setpoint_pitch->count());
    m_lineSeries_for_measured_pitch->removePoints(0,m_lineSeries_for_measured_pitch->count());

    // Unlock the mutex
    m_chart_mutex.unlock();
}




void CsoneControllerTab::on_set_lead_compensator_parameters_button_clicked()
{
    // Initialise local variable for each of (x,y,z,yaw)
    float k = 1.0f, T = 1.0f, alpha = 1.0f;

    // Lock the mutex
    m_controller_parameter_mutex.lock();

    // Take the new value if available, otherwise use default
    // > For k
    k = validate_and_get_value_from_lineEdit(ui->lineEdit_k,-1000.0,1000.0,4,0.1);

    // > For T
    T = validate_and_get_value_from_lineEdit(ui->lineEdit_T,0.0,1000.0,2,1.0);

    // > For alpha
    alpha = validate_and_get_value_from_lineEdit(ui->lineEdit_alpha,0.0,1.0,2,0.5);

    // Unlock the mutex
    m_controller_parameter_mutex.unlock();

    // Call the function to publish the controller parameters
    publishControllerParameters(k,T,alpha);
}

void CsoneControllerTab::on_set_lag_compensator_parameters_button_clicked()
{
    // Initialise local variable for each of (x,y,z,yaw)
    float T = 0.0f, alpha = 1.0f;

    // Lock the mutex
    m_lag_controller_parameter_mutex.lock();

    // > For T
    T = validate_and_get_value_from_lineEdit(ui->lineEdit_T_lag,0.0,1000.0,2,0.0);

    // > For alpha
    alpha = validate_and_get_value_from_lineEdit(ui->lineEdit_alpha_lag,0.0,1000.0,2,1.0);

    // Unlock the mutex
    m_lag_controller_parameter_mutex.unlock();

    // Call the function to publish the controller parameters
    publishLagControllerParameters(T,alpha);
}

void CsoneControllerTab::on_set_time_delay_button_clicked()
{
    // Take the new value if available, otherwise use default
    float time_delay_float = validate_and_get_value_from_lineEdit(ui->lineEdit_time_delay,0.0,10000.0,0,0.0);
    float time_delay_int = int(time_delay_float);

    // Call the function to publish the time delay
    publishRequestForTimeDelayChange(time_delay_int);
}

void CsoneControllerTab::on_set_pitch_error_button_clicked()
{
    // Take the new value if available, otherwise use default
    float pitch_error = validate_and_get_value_from_lineEdit(ui->lineEdit_pitch_error,-180.0,180.0,3,0.0);

    // Call the function to publish the pitch error
    publishRequestForPitchErrorChange(pitch_error);
}

void CsoneControllerTab::on_toggle_integrator_button_clicked()
{
    // Call the function to publish the request for a change in the integrator state
    publishRequestForIntegratorStateChange(INTEGRATOR_FLAG_TOGGLE);
}

void CsoneControllerTab::on_reset_integrator_button_clicked()
{
    // Call the function to publish the request for a change in the integrator state
    publishRequestForIntegratorStateChange(INTEGRATOR_FLAG_RESET);
}





void CsoneControllerTab::on_lineEdit_k_returnPressed()
{
    ui->set_lead_compensator_parameters_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_T_returnPressed()
{
    ui->set_lead_compensator_parameters_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_alpha_returnPressed()
{
    ui->set_lead_compensator_parameters_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_T_lag_returnPressed()
{
    ui->set_lag_compensator_parameters_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_alpha_lag_returnPressed()
{
    ui->set_lag_compensator_parameters_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_kp_returnPressed()
{
    ui->set_pd_controller_parameters_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_kd_returnPressed()
{
    ui->set_pd_controller_parameters_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_time_delay_returnPressed()
{
    ui->set_time_delay_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_pitch_error_returnPressed()
{
    ui->set_pitch_error_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_step_size_returnPressed()
{
    ui->perform_step_button->animateClick();
}

void CsoneControllerTab::on_lineEdit_step_duration_returnPressed()
{
    ui->log_data_button->animateClick();
}




void CsoneControllerTab::on_lineEdit_k_editingFinished()
{
    m_controller_parameter_mutex.lock();
    validate_and_get_value_from_lineEdit(ui->lineEdit_k,-1000.0,1000.0,4,0.1);
    m_controller_parameter_mutex.unlock();
}

void CsoneControllerTab::on_lineEdit_T_editingFinished()
{
    m_controller_parameter_mutex.lock();
    validate_and_get_value_from_lineEdit(ui->lineEdit_T,0.0,1000.0,2,1.0);
    m_controller_parameter_mutex.unlock();
}

void CsoneControllerTab::on_lineEdit_alpha_editingFinished()
{
    m_controller_parameter_mutex.lock();
    validate_and_get_value_from_lineEdit(ui->lineEdit_alpha,0.0,1.0,2,0.5);
    m_controller_parameter_mutex.unlock();
}

void CsoneControllerTab::on_lineEdit_T_lag_editingFinished()
{
    m_lag_controller_parameter_mutex.lock();
    validate_and_get_value_from_lineEdit(ui->lineEdit_T_lag,0.0,1000.0,2,0.0);
    m_lag_controller_parameter_mutex.unlock();
}

void CsoneControllerTab::on_lineEdit_alpha_lag_editingFinished()
{
    m_lag_controller_parameter_mutex.lock();
    validate_and_get_value_from_lineEdit(ui->lineEdit_alpha_lag,0.0,1000.0,2,1.0);
    m_lag_controller_parameter_mutex.unlock();
}

void CsoneControllerTab::on_lineEdit_step_size_editingFinished()
{
    m_chart_mutex.lock();
    validate_and_get_value_from_lineEdit(ui->lineEdit_step_size,-2.0,2.0,2,0.5);
    m_chart_mutex.unlock();
}

void CsoneControllerTab::on_lineEdit_step_duration_editingFinished()
{
    m_chart_mutex.lock();
    m_step_response_data_recording_duration = validate_and_get_value_from_lineEdit(ui->lineEdit_step_duration,1.0,100.0,0,20.0);
    m_chart_mutex.unlock();
}




//    ----------------------------------------------------------------------------------
//    PPPP    OOO    SSSS  EEEEE     DDDD     A    TTTTT    A
//    P   P  O   O  S      E         D   D   A A     T     A A
//    PPPP   O   O   SSS   EEE       D   D  A   A    T    A   A
//    P      O   O      S  E         D   D  AAAAA    T    AAAAA
//    P       OOO   SSSS   EEEEE     DDDD   A   A    T    A   A
//    ----------------------------------------------------------------------------------


void CsoneControllerTab::setMeasuredPose(float x , float y , float z , float roll , float pitch , float yaw , bool isValid)
{
    if (isValid)
    {
        // Ensure the red frames are not visible
        if ( ui->red_frame_position_left->isVisible() )
            ui->red_frame_position_left->setVisible(false);
        if ( ui->red_frame_position_right->isVisible() )
            ui->red_frame_position_right->setVisible(false);
    }
    else
    {
        // Make visible the red frames to indicate that
        // measurement is NOT valid
        if ( !(ui->red_frame_position_left->isVisible()) )
            ui->red_frame_position_left->setVisible(true);
        if ( !(ui->red_frame_position_right->isVisible()) )
            ui->red_frame_position_right->setVisible(true);
    }

    // Pass the data through to the plotting function
    if (isValid)
    {
        newDataForPerformingStepAndPlotting(x,pitch*RAD2DEG);
    }
    else
    {
        newDataForPerformingStepAndPlotting(0.0,0.0);
    }
}


void CsoneControllerTab::poseDataUnavailableSlot()
{
//    ui->lineEdit_measured_x->setText("xx.xx");
//    ui->lineEdit_measured_y->setText("xx.xx");
//    ui->lineEdit_measured_z->setText("xx.xx");

//    ui->lineEdit_measured_roll->setText("xx.xx");
//    ui->lineEdit_measured_pitch->setText("xx.xx");
//    ui->lineEdit_measured_yaw->setText("xx.xx");
}



void CsoneControllerTab::newDataForPerformingStepAndPlotting(float x, float pitch)
{
    // Static variables for the min and max of the vertical axis
    static float min_x_value_plotted     = -0.01;
    static float max_x_value_plotted     =  0.01;
    static float min_pitch_value_plotted = -0.01;
    static float max_pitch_value_plotted =  0.01;
    static float next_rezoom_at_time     =  0.50;

    // Initialise a bool for whether the step response should be analsysed
    bool shouldAnalyseStepResponse = false;

    // Lock the mutex
    m_chart_mutex.lock();

    // Reset the min, max, and rezoom time if requested
    if (m_shouldResetZoom)
    {
        min_x_value_plotted     = -0.01;
        max_x_value_plotted     =  0.01;
        min_pitch_value_plotted = -0.01;
        max_pitch_value_plotted =  0.01;
        next_rezoom_at_time     =  0.50;
        // Set the flag to false
        m_shouldResetZoom = false;
    }

    // Only do something if the flag indicates to do so
    if (m_shouldStoreData_for_plotting)
    {
        // Add the data to the line series
        // > For the x position
        //float temp_x_setpoint = (ui->lineEdit_setpoint_current_x->text()).toFloat();
        float temp_x_setpoint = m_current_setpoint_x;
        m_lineSeries_for_setpoint_x->append(m_time_for_step, temp_x_setpoint );
        m_lineSeries_for_measured_x->append(m_time_for_step, x );
        // > For the pitch angle
        //m_lineSeries_for_setpoint_pitch->append(m_time_for_step, ... );
        m_lineSeries_for_measured_pitch->append(m_time_for_step, pitch );

        // Update the min and max values
        min_x_value_plotted     = std::min( min_x_value_plotted     , std::min(temp_x_setpoint,x) );
        max_x_value_plotted     = std::max( max_x_value_plotted     , std::max(temp_x_setpoint,x) );
        min_pitch_value_plotted = std::min( min_pitch_value_plotted , pitch );
        max_pitch_value_plotted = std::max( max_pitch_value_plotted , pitch );

        // Increment the time
        m_time_for_step += 0.005;

        // Change the setpoint if the time has been incremented to zero
        // > Note: use 2 milliseconds as the threshold for zero
        if (m_shouldPerformStep)
        {
            if (m_time_for_step >= -0.002)
            {
                // Set the flag so that we do not enter this loop again
                m_shouldPerformStep = false;

                // Extract the current step size
                float step_size = validate_and_get_value_from_lineEdit(ui->lineEdit_step_size,-2.0,2.0,2,0.5);

                // Determine the new x setpoint
                float new_x = 0.0;
                if (x < 0)
                    new_x =  0.5 * step_size;
                else
                    new_x = -0.5 * step_size;

                // Specify the default setpoints for (y,z,yaw)
                float default_y   = m_current_setpoint_y;
                float default_z   = m_current_setpoint_z;
                float default_yaw = m_current_setpoint_yaw;

                // Publish the new setpoint
                publishSetpoint(new_x,default_y,default_z,default_yaw);
            }
        }

        // Rezoom when the next time is reached
        if ( m_time_for_step >= (next_rezoom_at_time+0.002) )
        {
            // Rezoom the x position chart
            float diff_of_x_values_plotted = max_x_value_plotted - min_x_value_plotted;
            ui->chartView_for_x->chart()->axisY()->setMin( min_x_value_plotted - 0.1*diff_of_x_values_plotted );
            ui->chartView_for_x->chart()->axisY()->setMax( max_x_value_plotted + 0.1*diff_of_x_values_plotted );
            // Rezoom the pitch angles chart
            float diff_of_pitch_values_plotted = max_pitch_value_plotted - min_pitch_value_plotted;
            ui->chartView_for_pitch->chart()->axisY()->setMin( min_pitch_value_plotted - 0.1*diff_of_pitch_values_plotted );
            ui->chartView_for_pitch->chart()->axisY()->setMax( max_pitch_value_plotted + 0.1*diff_of_pitch_values_plotted );
            // Increment the next rezoom time
            next_rezoom_at_time += 0.5;
        }

        // Stop data collection and analyse the results if the time has
        // passed "m_step_response_data_recording_duration" seconds
        if (m_time_for_step >= (m_step_response_data_recording_duration+0.002))
        {
            // Set the flag to stop data collection
            m_shouldStoreData_for_plotting = false;
            // Rezoom the x position chart
            float diff_of_x_values_plotted = max_x_value_plotted - min_x_value_plotted;
            ui->chartView_for_x->chart()->axisY()->setMin( min_x_value_plotted - 0.1*diff_of_x_values_plotted );
            ui->chartView_for_x->chart()->axisY()->setMax( max_x_value_plotted + 0.1*diff_of_x_values_plotted );
            // Rezoom the pitch angle chart
            float diff_of_pitch_values_plotted = max_pitch_value_plotted - min_pitch_value_plotted;
            ui->chartView_for_pitch->chart()->axisY()->setMin( min_pitch_value_plotted - 0.1*diff_of_pitch_values_plotted );
            ui->chartView_for_pitch->chart()->axisY()->setMax( max_pitch_value_plotted + 0.1*diff_of_pitch_values_plotted );
            // Set the flag that the step response should be analysed
            shouldAnalyseStepResponse = true;
            // Inform the user that the step is finished
            #ifdef CATKIN_MAKE
            ROS_INFO("[CSONE CONTROLLER GUI] Step finished.");
            #endif
        }
    }

    // Unlock the mutex
    m_chart_mutex.unlock();

    // Perform analysis of the step response if required
    if (shouldAnalyseStepResponse)
    {
        analyseStepResponse();
    }

}

void CsoneControllerTab::analyseStepResponse()
{
    // Initialise variable for the quantities to be determined
    float setpoint_start    = -999.9;
    float setpoint_end      = -999.9;
    float rise_time         = -999.9;
    float overshoot_value   = -999.9;
    float settling_time     = -999.9;
    float overshoot_percent = -999.9;

    // RETIEVE THE NECESSARY DATA INTO LOCAL VARIABLES

    // Lock the mutex
    m_chart_mutex.lock();

    // Get the length of the line series
    int count_of_lineSeries_for_setpoint_x = m_lineSeries_for_setpoint_x->count();
    int count_of_lineSeries_for_measured_x = m_lineSeries_for_measured_x->count();

    // Get all the points
    QVector<QPointF> vector_of_points_for_setpoint_x = m_lineSeries_for_setpoint_x->pointsVector();
    QVector<QPointF> vector_of_points_for_measured_x = m_lineSeries_for_measured_x->pointsVector();

    // Unlock the mutex
    m_chart_mutex.unlock();

    // PERFORM THE ANALYSIS

    // Only do something if there is actually data to process
    if ( (count_of_lineSeries_for_setpoint_x<1) && (count_of_lineSeries_for_measured_x<1) )
    {
        // Inform the user that the step is finished
        #ifdef CATKIN_MAKE
        ROS_ERROR("[CSONE CONTROLLER GUI] Request analyse of the step response, but the line series have a count of zero.");
        #endif
        // End this function
        return;
    }

    // Get the start and end setpoint
    if ( count_of_lineSeries_for_setpoint_x>0 )
    {
        // Get the start point
        setpoint_start = vector_of_points_for_setpoint_x.at(0).y();
        // Get the end point
        setpoint_end = vector_of_points_for_setpoint_x.at(count_of_lineSeries_for_setpoint_x-1).y();
        // DEBUGGING: print out the start and end values
        #ifdef CATKIN_MAKE
        ROS_INFO_STREAM("[CSONE CONTROLLER GUI] Setpoint at (start,end) = ( " << setpoint_start << " , " << setpoint_end << " )");
        #endif
    }


    // Get the rise time, overshoot, and settling time
    if ( (count_of_lineSeries_for_setpoint_x>0) && (count_of_lineSeries_for_measured_x>0) )
    {
        // Only continue if the step was at least 1 centimeter
        if ( (setpoint_end-setpoint_start)<(-0.01) || (0.01)<(setpoint_end-setpoint_start) )
        {
            // Determine if it is a step up of a step down
            float step_direction = 0.0;
            if (setpoint_start<setpoint_end)
            {
                step_direction = 1.0;
            }
            else
            {
                step_direction = -1.0;
            }

            // Compute the threshold for the rise time
            float rise_time_threshold = setpoint_start + 0.9*(setpoint_end-setpoint_start);

            // Compute the threshold for the settling time
            float settiling_time_upper_threshold = setpoint_end + step_direction * 0.05 * (setpoint_end-setpoint_start);
            float settiling_time_lower_threshold = setpoint_end - step_direction * 0.05 * (setpoint_end-setpoint_start);

            // Prepare a few variables needed
            float this_time = 0.0;
            float this_val  = 0.0;
            bool riseTimeFound = false;
            overshoot_value = step_direction*(-100.0);

            // Iterate through the points
            for (int i = 0; i < vector_of_points_for_measured_x.size(); ++i)
            {
                // Get the values for this point
                this_time = vector_of_points_for_measured_x.at(i).x();
                this_val  = vector_of_points_for_measured_x.at(i).y();

                // Check if the rise time is exceeded
                if (!riseTimeFound)
                {
                    // Check if the rise time threshold is exceeded
                    if ( (step_direction*this_val) > (step_direction*rise_time_threshold) )
                    {
                        // Save this as the rise time
                        rise_time = this_time;
                        // Set the flag that it has been found
                        riseTimeFound = true;
                    }
                }

                // Keep track of the maximum value for the overshoot
                overshoot_value = step_direction * std::max( step_direction*overshoot_value , step_direction*this_val );

                // Check if outside the settling time threshold
                if ( (this_val<settiling_time_lower_threshold) || (settiling_time_upper_threshold<this_val) )
                {
                    // Update the settling time to the time for this iteration
                    settling_time = this_time;
                }
            }

            // Convert the settling time to a percentage
            overshoot_percent = (overshoot_value - setpoint_end) / (setpoint_end-setpoint_start) * 100.0;

            // DEBUGGING: print out the start and end values
            #ifdef CATKIN_MAKE
            ROS_INFO_STREAM("[CSONE CONTROLLER GUI] (rise time,overshoot value, os percent,settling time) = ( " << rise_time << " , " << overshoot_value << " , " << overshoot_percent << " , " << settling_time << " )");
            #endif
        }
    }

    // DISPLAY THE RESULTS IN THE FIELDS OF THE GUI

    // Lock the mutex
    m_chart_mutex.lock();

    // Call the "update" function for each
    updateStepAnalysisLineEdit(setpoint_start     , ui->lineEdit_step_start         , 3);
    updateStepAnalysisLineEdit(setpoint_end       , ui->lineEdit_step_end           , 3);
    updateStepAnalysisLineEdit(overshoot_value    , ui->lineEdit_overshoot_value    , 3);
    updateStepAnalysisLineEdit(overshoot_percent  , ui->lineEdit_overshoot_percent  , 1);
    updateStepAnalysisLineEdit(rise_time          , ui->lineEdit_rise_time          , 2);
    updateStepAnalysisLineEdit(settling_time      , ui->lineEdit_settling_time      , 2);


    // Unlock the mutex
    m_chart_mutex.unlock();


}



void CsoneControllerTab::updateStepAnalysisLineEdit(float value, QLineEdit * lineEdit, int num_dec_places)
{
    // Initialise a string variable for adding the "+"
    QString qstr = "";

    // Check if the value is "valid" or not
    if (value > -999.0)
    {
        if (value < 0.0f) qstr = ""; else qstr = "+";
        lineEdit->setText(qstr + QString::number( value, 'f', num_dec_places));
    }
    else
    {
        lineEdit->setText("---");
    }
}



//    ----------------------------------------------------------------------------------
//     SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    S      E        T    P   P  O   O   I   NN  N    T
//     SSS   EEE      T    PPPP   O   O   I   N N N    T
//        S  E        T    P      O   O   I   N  NN    T
//    SSSS   EEEEE    T    P       OOO   III  N   N    T
//
//     CCCC  H   H    A    N   N   GGGG  EEEEE  DDDD
//    C      H   H   A A   NN  N  G      E      D   D
//    C      HHHHH  A   A  N N N  G      EEE    D   D
//    C      H   H  AAAAA  N  NN  G   G  E      D   D
//     CCCC  H   H  A   A  N   N   GGGG  EEEEE  DDDD
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K
//    C       A A   L      L      B   B   A A   C      K  K
//    C      A   A  L      L      BBBB   A   A  C      KKK
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//    ----------------------------------------------------------------------------------


#ifdef CATKIN_MAKE
void CsoneControllerTab::setpointChangedCallback(const dfall_pkg::SetpointWithHeader& newSetpoint)
{
    // EXTRACT THE SETPOINT
    float x = newSetpoint.x;
    float y = newSetpoint.y;
    float z = newSetpoint.z;
    float yaw = newSetpoint.yaw;

    // Lock the mutex
    m_chart_mutex.lock();

    // Update the class variables for the current setpoint
    m_current_setpoint_x   = x;
    m_current_setpoint_y   = y;
    m_current_setpoint_z   = z;
    m_current_setpoint_yaw = yaw;

    // Unlock the mutex
    m_chart_mutex.unlock();
}
#endif




#ifdef CATKIN_MAKE
void CsoneControllerTab::integratorStateChangedCallback(const dfall_pkg::IntWithHeader& newIntegratorState)
{
    // EXTRACT THE NEW STATE
    int new_state = newIntegratorState.data;

    // Lock the mutex
    //m_chart_mutex.lock();

    switch (new_state)
    {
        case INTEGRATOR_FLAG_ON:
        {
            ui->label_integrator_state->setText("on");
            break;
        }
        case INTEGRATOR_FLAG_OFF:
        {
            ui->label_integrator_state->setText("off");
            break;
        }
        case INTEGRATOR_FLAG_UNKNOWN:
        {
            ui->label_integrator_state->setText("Unknown");
            break;
        }
        default:
        {
            ui->label_integrator_state->setText("Not recognised");
            break;
        }
    }

    // Unlock the mutex
    //m_chart_mutex.unlock();
}
#endif









//    ----------------------------------------------------------------------------------
//    RRRR   EEEEE   QQQ   U   U  EEEEE   SSSS  TTTTT     N   N  EEEEE  W     W
//    R   R  E      Q   Q  U   U  E      S        T       NN  N  E      W     W
//    RRRR   EEE    Q   Q  U   U  EEE     SSS     T       N N N  EEE    W     W
//    R   R  E      Q  Q   U   U  E          S    T       N  NN  E       W W W
//    R   R  EEEEE   QQ Q   UUU   EEEEE  SSSS     T       N   N  EEEEE    W W
//
//     SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    S      E        T    P   P  O   O   I   NN  N    T
//     SSS   EEE      T    PPPP   O   O   I   N N N    T
//        S  E        T    P      O   O   I   N  NN    T
//    SSSS   EEEEE    T    P       OOO   III  N   N    T
//    ----------------------------------------------------------------------------------


void CsoneControllerTab::publishSetpoint(float x, float y, float z, float yaw_degrees)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    dfall_pkg::SetpointWithHeader msg;

    // Fill the header of the message
    fillSetpointMessageHeader( msg );

    // Fill in the (x,y,z,yaw) values
    msg.x   = x;
    msg.y   = y;
    msg.z   = z;
    msg.yaw = yaw_degrees * DEG2RAD;

    // Publish the setpoint
    this->requestSetpointChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[CSONE CONTROLLER GUI] Published request for setpoint change to: [" << x << ", "<< y << ", "<< z << ", "<< yaw_degrees << "]");
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] would publish request for: [" << x << ", "<< y << ", "<< z << ", "<< yaw_degrees << "]";
#endif
}





void CsoneControllerTab::publishControllerParameters(float k, float T, float alpha)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    dfall_pkg::SetpointWithHeader msg;

    // Fill the header of the message
    fillSetpointMessageHeader( msg );

    // Fill in the controller values
    msg.x   = k;
    msg.y   = T;
    msg.z   = alpha;

    // Publish the setpoint
    this->requestControllerParameterChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[CSONE CONTROLLER GUI] Published request for new controller parameters (k,T,alpha) =  (" << k << ", "<< T << ", "<< alpha << ")");
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] would publish request for: [" << k << ", "<< T << ", "<< alpha << "]";
#endif

    // Call the function to convert this to a discrete-time state-space controller
    convertIntoDiscreteTimeParameters(k, T, alpha);
}


void CsoneControllerTab::publishLagControllerParameters(float T, float alpha)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    dfall_pkg::SetpointWithHeader msg;

    // Fill the header of the message
    fillSetpointMessageHeader( msg );

    // Fill in the controller values
    msg.y   = T;
    msg.z   = alpha;

    // Publish the setpoint
    this->requestLagControllerParameterChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[CSONE CONTROLLER GUI] Published request for new lag controller parameters (T,alpha) =  (" << T << ", "<< alpha << ")");
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] would publish request for new lag controller parameters (T,alpha): [" << T << ", "<< alpha << "]";
#endif

    // Call the function to convert this to a discrete-time state-space controller
    convertLagIntoDiscreteTimeParameters(T, alpha);
}


void CsoneControllerTab::publishRequestForIntegratorStateChange(int flag_to_publish)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    dfall_pkg::IntWithHeader msg;

    // Fill the header of the message
    fillIntMessageHeader( msg );

    // Fill in the controller values
    msg.data = flag_to_publish;

    // Publish the setpoint
    this->requestIntegratorStateChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[CSONE CONTROLLER GUI] Published request for integrator state to change with flag " << flag_to_publish << ")");
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] would publish request for integrator state to change with flag " << flag_to_publish << ")";
#endif
}





void CsoneControllerTab::publishRequestForTimeDelayChange(int time_delay_to_publish)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    dfall_pkg::IntWithHeader msg;

    // Fill the header of the message
    fillIntMessageHeader( msg );

    // Fill in the controller values
    msg.data = time_delay_to_publish;

    // Publish the setpoint
    this->requestTimeDelayChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[CSONE CONTROLLER GUI] Published request for time delay of " << time_delay_to_publish << " milliseconds");
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] would publish request for time delay of " << time_delay_to_publish << " milliseconds" << "\n";
#endif
}


void CsoneControllerTab::publishRequestForPitchErrorChange(float pitch_error_to_publish)
{
#ifdef CATKIN_MAKE
    // Initialise the message as a local variable
    dfall_pkg::SetpointWithHeader msg;

    // Fill the header of the message
    fillSetpointMessageHeader( msg );

    // Fill in the value
    msg.x = pitch_error_to_publish;

    // Publish the setpoint
    this->requestPitchErrorChangePublisher.publish(msg);

    // Inform the user about the change
    ROS_INFO_STREAM("[CSONE CONTROLLER GUI] Published request for pitch error of " << pitch_error_to_publish << " degrees");
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] would publish request for pitch error of " << pitch_error_to_publish << " degrees" << "\n";
#endif
}




//    ----------------------------------------------------------------------------------
//
//
//
//
//
//    ----------------------------------------------------------------------------------

// CHANGE CONTROLLER PARAMETERS INTO DISCRETE TIME FUNCTION
void CsoneControllerTab::convertIntoDiscreteTimeParameters(float k, float T, float alpha)
{
    float control_frequency = 200.0;

    if (alpha > 1){alpha = 1;} else if (alpha<0.1){alpha = 0.1;}

    if (T > 100){T = 100;} else if (T<0.1){T = 0.1;}

    // Lock the mutex
    m_simulation_mutex.lock();

    // Compute the A,B,C,D matrices
    m_lead_compensator_A = pow(2.71828,(-1.0/(control_frequency*alpha*T)));
    m_lead_compensator_B = -alpha*T*(m_lead_compensator_A-1.0);
    m_lead_compensator_C = k/(alpha*T)*(1.0-1.0/alpha);
    m_lead_compensator_D = k/alpha;

    // Reset the state of the lead compensator to zero
    m_lead_compensator_state = 0.0;

#ifdef CATKIN_MAKE
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] Parameters changed to k=" << k << ", T=" << T << ", alpha=" << alpha << "\n";
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] Matrices changed to A=" << m_lead_compensator_A << ", B=" << m_lead_compensator_B << ", C=" << m_lead_compensator_C << ", D=" << m_lead_compensator_D;
#endif

    // Unlock the mutex
    m_simulation_mutex.unlock();

}


// CHANGE CONTROLLER PARAMETERS INTO DISCRETE TIME FUNCTION
void CsoneControllerTab::convertLagIntoDiscreteTimeParameters(float T, float alpha)
{
    float control_frequency = 200.0;

    if (alpha > 100){alpha = 100;} else if (alpha<1){alpha = 1;}

    if (T > 100){T = 100;} else if (T<0){T = 0;}

    // Lock the mutex
    m_simulation_mutex.lock();

    // Compute the A,B,C,D matrices
    if (T > 0){
        m_lag_compensator_A = pow(2.71828,(-1.0/(control_frequency*alpha*T)));
        m_lag_compensator_B = -alpha*T*(m_lag_compensator_A-1.0);
        m_lag_compensator_C = (alpha - 1.0)/(alpha*T);
    }
    else{
        m_lag_compensator_A = 0;
        m_lag_compensator_B = 0;
        m_lag_compensator_C = 0;
    }
    m_lag_compensator_D = 1;

    // Reset the state of the lead compensator to zero
    m_lag_compensator_state = 0.0;

#ifdef CATKIN_MAKE
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] Lag parameters changed to T=" << T << ", alpha=" << alpha << "\n";
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] Lag matrices changed to A=" << m_lag_compensator_A << ", B=" << m_lag_compensator_B << ", C=" << m_lag_compensator_C << ", D=" << m_lag_compensator_D;
#endif

    // Unlock the mutex
    m_simulation_mutex.unlock();

}


void CsoneControllerTab::simulate_step_response()
{
    // GET THE CONTROLLER PARAMETERS INTO LOCAL VARIABLES

    // Lock the mutex
    m_simulation_mutex.lock();

    float A = m_lead_compensator_A;
    float B = m_lead_compensator_B;
    float C = m_lead_compensator_C;
    float D = m_lead_compensator_D;

    float controller_state = 0.0;

    float lag_A = m_lag_compensator_A;
    float lag_B = m_lag_compensator_B;
    float lag_C = m_lag_compensator_C;
    float lag_D = m_lag_compensator_D;

    float lag_controller_state = 0.0;

    // Unlock the mutex
    m_simulation_mutex.unlock();

    // GET THE STEP SPECIFICATIONS INTO LOCAL VARIABLES

    // Lock the mutex
    m_chart_mutex.lock();

    float current_setpoint_x = m_current_setpoint_x;

    float step_size = validate_and_get_value_from_lineEdit(ui->lineEdit_step_size,-2.0,2.0,2,0.5);

    m_step_response_data_recording_duration = validate_and_get_value_from_lineEdit(ui->lineEdit_step_duration,1.0,100.0,0,20.0);
    float duration = m_step_response_data_recording_duration;

    // Unlock the mutex
    m_chart_mutex.unlock();



    // Take the new value if available, otherwise use default
    float time_delay_float = validate_and_get_value_from_lineEdit(ui->lineEdit_time_delay,0.0,10000.0,0,0.0);
    float time_delay_int = int(time_delay_float);

    // Compute the number of milliseconds per time step
    float delta_T_in_milliseconds = 1000.0 / 200.0;

    // Convert the time delay to a number of time steps
    int time_delay_in_steps = int( (float(time_delay_int) + 0.1) / delta_T_in_milliseconds );

    // Wrap this value into the allowed limits
    if (time_delay_in_steps<0)
        time_delay_in_steps=0;
    if(time_delay_in_steps>(300-1))
        time_delay_in_steps=300-1;
#ifdef CATKIN_MAKE
#else
    // TO ASSIST WITH DEBUGGING WHEN COMPILED AND RUN IN "QtCreator"
    QTextStream(stdout) << "[CSONE CONTROLLER GUI] time delay in steps = " << time_delay_in_steps << "\n";
#endif


    // PERFORM THE DURATION

    // Initialise things
    float control_frequency = 200.0;
    float control_deltaT = 0.005;

    // Model parameters
    float k_x = 0.2;
    float m = 0.3;
    float g = 9.81;

    // Discrete time x subsystem matrices
    //  for k_x / m = 0.15, g = 9.81, T = 0.005:
    //        A =
    //                    x1         x2         x3
    //         x1          1   0.004998  0.0001226
    //         x2          0     0.9993    0.04903
    //         x3          0          0          1

    //        B =
    //                    u1
    //         x1  2.043e-07
    //         x2  0.0001226
    //         x3      0.005

    // in general:
    float A_x_x = 1.0;
    float A_x_xdot = control_deltaT * (1 - k_x / m * control_deltaT / 2);
    float A_x_pitch = g * control_deltaT * control_deltaT / 2;
    float A_xdot_x = 0.0;
    float A_xdot_xdot = 1 - k_x / m * control_deltaT;
    float A_xdot_pitch = g * control_deltaT;
    float A_pitch_x = 0.0;
    float A_pitch_xdot = 0.0;
    float A_pitch_pitch = 1.0;
    float B_x_pitchRate = g * control_deltaT * control_deltaT * control_deltaT / 6;
    float B_xdot_pitchRate = g * control_deltaT * control_deltaT / 2;
    float B_pitch_pitchRate = control_deltaT;

    float sim_time = 0.0;

    int num_time_steps = int( duration / control_deltaT );

    int num_point_per_append = 10;
    int next_index = 0;

    QVector<float> sim_traj_time (num_point_per_append,0);

    //std::vector<float>  sim_traj_x (num_time_steps,0);
    QVector<float> sim_traj_x (num_point_per_append,0);
    //QList<QPointF> sim_traj_x;

    QVector<float> sim_traj_pitch (num_point_per_append,0);

    // The inertial x measurement during a time window of 300 measurements
    QVector<float>  x_buffer (300,0);

    float x_reference_start = current_setpoint_x;
    // Determine the new x setpoint
    float x_reference_end = 0.0;
    if (x_reference_start < 0)
        x_reference_end =  0.5 * step_size;
    else
        x_reference_end = -0.5 * step_size;

    // Initialise the current state
    float current_x     = x_reference_start;
    float current_xdot  = 0.0;
    float current_pitch = 0.0;

    // Initialise variables for keeping track of the min and max
    float min_x_value_sim     = -0.01;
    float max_x_value_sim     =  0.01;
    float min_pitch_value_sim = -0.01;
    float max_pitch_value_sim =  0.01;

    // Update the min and max
    min_x_value_sim = std::min( min_x_value_sim , current_x );
    max_x_value_sim = std::max( max_x_value_sim , current_x );
    min_pitch_value_sim = std::min( min_pitch_value_sim , current_pitch*float(RAD2DEG) );
    max_pitch_value_sim = std::max( max_pitch_value_sim , current_pitch*float(RAD2DEG) );


    // Put in the initial conditions
    sim_traj_time[next_index]  = sim_time;

    sim_traj_x[next_index]     = current_x;
    //sim_traj_x.append( QPointF(sim_time,current_x) );

    sim_traj_pitch[next_index] = current_pitch;

    next_index++;





    // Lock the mutex
    m_chart_mutex.lock();

    // For the x reference
    m_lineSeries_for_sim_setpoint_x->append(0.0,      x_reference_start );
    m_lineSeries_for_sim_setpoint_x->append(0.0,      x_reference_end   );
    m_lineSeries_for_sim_setpoint_x->append(duration, x_reference_end   );

    // Unlock the mutex
    m_chart_mutex.unlock();


    int write_index = 0;
    int read_index = 0;


    // Iterate over the duration
    for ( int itime=1 ; itime<num_time_steps ; itime++ )
    {
        // Increment the write index
        write_index += 1;
        // And wrap it back into range if necessary
        if (write_index>=x_buffer.size())
        {
            write_index = 0;
        }

        // Compute the next read index based on the delay
        read_index = write_index - time_delay_in_steps;
        // And wrap it back into range if necessary
        if (read_index<0)
        {
            read_index += x_buffer.size();
        }

        // Write the new data to the buffer
        x_buffer[write_index] = current_x;

        // Read the data for this time step from the buffer
        float x_for_this_time_step = x_buffer[read_index];

        // Compute the error for this time step
        float x_error = x_reference_end - x_for_this_time_step;

        // Compute the input to apply
        // > FIRST: compute the lead controller intermediate output
        float lead_u  = C*controller_state + D*x_error;
        // > SECOND: compute the new pitch reference
        float pitch_ref  = lag_C*lag_controller_state + lag_D*lead_u;
        // > THIRD: evaluate the state update equation
        controller_state = A*controller_state + B*x_error;
        lag_controller_state = lag_A*lag_controller_state + lag_B*lead_u;
        // > FOURTH: perform the inner controller
        float pitch_rate = 5.0*(pitch_ref-current_pitch);


        // Perform the evolution of the state
        current_x     = A_x_x * current_x + A_x_xdot * current_xdot + A_x_pitch * current_pitch + B_x_pitchRate * pitch_rate;
        current_xdot  = A_xdot_x * current_x + A_xdot_xdot * current_xdot + A_xdot_pitch * current_pitch + B_xdot_pitchRate * pitch_rate;
        current_pitch = A_pitch_x * current_x + A_pitch_xdot * current_xdot + A_pitch_pitch * current_pitch + B_pitch_pitchRate * pitch_rate;

        // Update the min and max
        min_x_value_sim = std::min( min_x_value_sim , current_x );
        max_x_value_sim = std::max( max_x_value_sim , current_x );
        min_pitch_value_sim = std::min( min_pitch_value_sim , current_pitch*float(RAD2DEG) );
        max_pitch_value_sim = std::max( max_pitch_value_sim , current_pitch*float(RAD2DEG) );

        // Increment the simulation time
        sim_time += control_deltaT;

        // Store the state and time
        sim_traj_time[next_index]  = sim_time;

        sim_traj_x[next_index]     = current_x;
        //sim_traj_x.append( QPointF(sim_time,current_x) );

        sim_traj_pitch[next_index] = current_pitch;

        // Increment the next index pointer
        next_index++;

        // Update the chart if required
        if (next_index >= num_point_per_append)
        {
            // Lock the mutex
            m_chart_mutex.lock();
            // Iterate through the point
            for (int ipoint=0 ; ipoint<num_point_per_append ; ipoint++)
            {
                m_lineSeries_for_sim_measured_x->append( sim_traj_time[ipoint] , sim_traj_x[ipoint] );
                m_lineSeries_for_sim_measured_pitch->append( sim_traj_time[ipoint] , sim_traj_pitch[ipoint]*RAD2DEG );

                //ui->chartView_for_x->repaint();
            }
            next_index = 0;
            // Unlock the mutex
            m_chart_mutex.unlock();
        }
    }


    // PUT THE SIMULATION RESULTS INTO THE PLOTTED LINE SERIES

    // Lock the mutex
    m_chart_mutex.lock();

    // Update the chart with any remaining points
    if (next_index > 0)
    {
        // Iterate through the point
        for (int ipoint=0 ; ipoint<next_index ; ipoint++)
        {
            m_lineSeries_for_sim_measured_x->append( sim_traj_time[ipoint] , sim_traj_x[ipoint] );
            m_lineSeries_for_sim_measured_pitch->append( sim_traj_time[ipoint] , sim_traj_pitch[ipoint]*RAD2DEG );
        }
        next_index = 0;
    }

    // Rezoom the x position chart
    float diff_of_x_values_sim = max_x_value_sim - min_x_value_sim;
    ui->chartView_for_x->chart()->axisY()->setMin( min_x_value_sim - 0.1*diff_of_x_values_sim );
    ui->chartView_for_x->chart()->axisY()->setMax( max_x_value_sim + 0.1*diff_of_x_values_sim );
    // Rezoom the pitch angles chart
    float diff_of_pitch_values_sim = max_pitch_value_sim - min_pitch_value_sim;
    ui->chartView_for_pitch->chart()->axisY()->setMin( min_pitch_value_sim - 0.1*diff_of_pitch_values_sim );
    ui->chartView_for_pitch->chart()->axisY()->setMax( max_pitch_value_sim + 0.1*diff_of_pitch_values_sim );

    // Unlock the mutex
    m_chart_mutex.unlock();


    // Set the flag that a simulation is completed
    m_simulation_mutex.lock();
    m_simulationIsInProgress = false;
    // Set the button to not be avaialble
    ui->simulate_step_response_button->setEnabled(true);
    ui->clear_simulation_button->setEnabled(true);
    m_simulation_mutex.unlock();


}



//    ----------------------------------------------------------------------------------
//      A     GGGG  EEEEE  N   N  TTTTT     III  DDDD    SSSS
//     A A   G      E      NN  N    T        I   D   D  S
//    A   A  G      EEE    N N N    T        I   D   D   SSS
//    AAAAA  G   G  E      N  NN    T        I   D   D      S
//    A   A   GGGG  EEEEE  N   N    T       III  DDDD   SSSS
//    ----------------------------------------------------------------------------------


void CsoneControllerTab::setAgentIDsToCoordinate(QVector<int> agentIDs , bool shouldCoordinateAll)
{

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

        // // > Request the current setpoint
        ros::ServiceClient getCurrentSetpointServiceClient = agent_base_nodeHandle.serviceClient<dfall_pkg::GetSetpointService>("CsoneControllerService/GetCurrentSetpoint", false);
        dfall_pkg::GetSetpointService getSetpointCall;
        getSetpointCall.request.data = 0;
        getCurrentSetpointServiceClient.waitForExistence(ros::Duration(0.0));
        if(getCurrentSetpointServiceClient.call(getSetpointCall))
        {
            setpointChangedCallback(getSetpointCall.response.setpointWithHeader);
        }
        else
        {
            // Inform the user
            ROS_INFO("[CSONE CONTROLLER GUI] Failed to get setpoint from controller using the \"GetCurrentSetpoint\" service");
        }

        // SUBSCRIBERS
        // > For receiving message that the setpoint was changed
        setpointChangedSubscriber = agent_base_nodeHandle.subscribe("CsoneControllerService/SetpointChanged", 1, &CsoneControllerTab::setpointChangedCallback, this);
    }
    else
    {
        // Unsubscribe
        setpointChangedSubscriber.shutdown();

        // Lock the mutex
        m_chart_mutex.lock();

        // Set information back to the default
        m_current_setpoint_x   = 0.0;
        m_current_setpoint_y   = 0.0;
        m_current_setpoint_z   = 0.4;
        m_current_setpoint_yaw = 0.0;

        // Unlock the mutex
        m_chart_mutex.unlock();

    }
#endif
}






//    ----------------------------------------------------------------------------------
//    M   M   SSSS   GGG      H   H  EEEEE    A    DDDD   EEEEE  RRRR
//    MM MM  S      G   G     H   H  E       A A   D   D  E      R   R
//    M M M   SSS   G         HHHHH  EEE    A   A  D   D  EEE    RRRR
//    M   M      S  G   G     H   H  E      AAAAA  D   D  E      R   R
//    M   M  SSSS    GGGG     H   H  EEEEE  A   A  DDDD   EEEEE  R   R
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
// Fill the header for a message
void CsoneControllerTab::fillIntMessageHeader( dfall_pkg::IntWithHeader & msg )
{
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            msg.shouldCheckForAgentID = false;
            break;
        }
        case TYPE_COORDINATOR:
        {
            // Lock the mutex
            m_agentIDs_toCoordinate_mutex.lock();
            // Add the "coordinate all" flag
            msg.shouldCheckForAgentID = !(m_shouldCoordinateAll);
            // Add the agent IDs if necessary
            if (!m_shouldCoordinateAll)
            {
                for ( int irow = 0 ; irow < m_vector_of_agentIDs_toCoordinate.size() ; irow++ )
                {
                    msg.agentIDs.push_back( m_vector_of_agentIDs_toCoordinate[irow] );
                }
            }
            // Unlock the mutex
            m_agentIDs_toCoordinate_mutex.unlock();
            break;
        }

        default:
        {
            msg.shouldCheckForAgentID = true;
            ROS_ERROR("[CSONE CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }
}
#endif




#ifdef CATKIN_MAKE
// Fill the header for a message
void CsoneControllerTab::fillSetpointMessageHeader( dfall_pkg::SetpointWithHeader & msg )
{
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            msg.shouldCheckForAgentID = false;
            break;
        }
        case TYPE_COORDINATOR:
        {
            // Lock the mutex
            m_agentIDs_toCoordinate_mutex.lock();
            // Add the "coordinate all" flag
            msg.shouldCheckForAgentID = !(m_shouldCoordinateAll);
            // Add the agent IDs if necessary
            if (!m_shouldCoordinateAll)
            {
                for ( int irow = 0 ; irow < m_vector_of_agentIDs_toCoordinate.size() ; irow++ )
                {
                    msg.agentIDs.push_back( m_vector_of_agentIDs_toCoordinate[irow] );
                }
            }
            // Unlock the mutex
            m_agentIDs_toCoordinate_mutex.unlock();
            break;
        }

        default:
        {
            msg.shouldCheckForAgentID = true;
            ROS_ERROR("[CSONE CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }
}
#endif



#ifdef CATKIN_MAKE
// Fill the header for a message
void CsoneControllerTab::fillCustomButtonMessageHeader( dfall_pkg::CustomButtonWithHeader & msg )
{
    switch (m_type)
    {
        case TYPE_AGENT:
        {
            msg.shouldCheckForAgentID = false;
            break;
        }
        case TYPE_COORDINATOR:
        {
            // Lock the mutex
            m_agentIDs_toCoordinate_mutex.lock();
            // Add the "coordinate all" flag
            msg.shouldCheckForAgentID = !(m_shouldCoordinateAll);
            // Add the agent IDs if necessary
            if (!m_shouldCoordinateAll)
            {
                for ( int irow = 0 ; irow < m_vector_of_agentIDs_toCoordinate.size() ; irow++ )
                {
                    msg.agentIDs.push_back( m_vector_of_agentIDs_toCoordinate[irow] );
                }
            }
            // Unlock the mutex
            m_agentIDs_toCoordinate_mutex.unlock();
            break;
        }

        default:
        {
            msg.shouldCheckForAgentID = true;
            ROS_ERROR("[CSONE CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }
}
#endif





//    ----------------------------------------------------------------------------------
//    III  DDDD       &&&      TTTTT  Y   Y  PPPP   EEEEE
//     I   D   D     &           T     Y Y   P   P  E
//     I   D   D      &          T      Y    PPPP   EEE
//     I   D   D     & & &       T      Y    P      E
//    III  DDDD       &&&        T      Y    P      EEEEE
//    ----------------------------------------------------------------------------------



#ifdef CATKIN_MAKE
bool CsoneControllerTab::getTypeAndIDParameters()
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
        ROS_ERROR("[CSONE CONTROLLER TAB GUI] Failed to get type");
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
        ROS_ERROR("[CSONE CONTROLLER TAB GUI] The 'type' parameter retrieved was not recognised.");
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
                ROS_ERROR("[CSONE CONTROLLER TAB GUI] Failed to get agentID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[CSONE CONTROLLER TAB GUI] Is of type AGENT with ID = " << m_ID);
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
                ROS_ERROR("[CSONE CONTROLLER TAB GUI] Failed to get coordID");
            }
            else
            {
                // Inform the user about the type and ID
                ROS_INFO_STREAM("[CSONE CONTROLLER TAB GUI] Is of type COORDINATOR with ID = " << m_ID);
            }
            break;
        }

        default:
        {
            // Throw an error if the type is not recognised
            return_was_successful = false;
            ROS_ERROR("[CSONE CONTROLLER TAB GUI] The 'm_type' variable was not recognised.");
            break;
        }
    }

    // Return
    return return_was_successful;
}
#endif
