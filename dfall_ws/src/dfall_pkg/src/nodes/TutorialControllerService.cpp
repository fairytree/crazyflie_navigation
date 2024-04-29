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
//    Tutorial controller to teach Eigen and OSQP on a MPC controller
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/TutorialControllerService.h"

//    ----------------------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    III M   M PPPP  L     EEEEE M   M EEEEE N   N TTTTT   A   TTTTT III  OOO  N   N
//     I  MM MM P   P L     E     MM MM E     NN  N   T    A A    T    I  O   O NN  N
//     I  M M M PPPP  L     EEE   M M M EEE   N N N   T   A   A   T    I  O   O N N N
//     I  M   M P     L     E     M   M E     N  NN   T   AAAAA   T    I  O   O N  NN
//    III M   M P     LLLLL EEEEE M   M EEEEE N   N   T   A   A   T   III  OOO  N   N
//    ----------------------------------------------------------------------------------


// *** SETUP THE MPC OPTIMIZATION PROBLEM WITH OSQP ***
void setup_mpc_optimization()
{
	try
    {
    	// *** COMPUTE THE NUMBER OF DECISION VARIABLES ***
    	int num_state_decision_variables = (yaml_prediction_horizon + 1) * const_num_states;
    	int num_input_decision_variables = yaml_prediction_horizon * const_num_inputs;

    	int num_decision_variables = num_state_decision_variables + num_input_decision_variables;



    	// *** SPECIFY THE STATE COST MATRIX Q AND THE INPUT COST MATRIX R ***
		m_state_cost_matrix = MatrixXf::Zero(const_num_states, const_num_states);
		for (int i = 0; i < const_num_states; i++)
			m_state_cost_matrix(i, i) = yaml_state_cost_diagonals[i];

		MatrixXf input_cost_matrix = MatrixXf::Zero(const_num_inputs, const_num_inputs);
		for (int i = 0; i < const_num_inputs; i++)
			input_cost_matrix(i, i) = yaml_input_cost_diagonals[i];



		// *** SPECIFY THE A, B DISCRETE LTI DYNAMICS MATRICES ***
		MatrixXf A_dynamics_matrix = MatrixXf::Identity(const_num_states, const_num_states);
		A_dynamics_matrix.block(0, 3, 3, 3) = 1.0 / yaml_control_frequency * MatrixXf::Identity(3, 3);
		A_dynamics_matrix(0, 7) = const_gravity / (2.0 * yaml_control_frequency * yaml_control_frequency);
		A_dynamics_matrix(1, 6) = -const_gravity / (2.0 * yaml_control_frequency * yaml_control_frequency);
		A_dynamics_matrix(3, 7) = const_gravity / yaml_control_frequency;
		A_dynamics_matrix(4, 6) = -const_gravity / yaml_control_frequency;

		MatrixXf B_dynamics_matrix = MatrixXf::Zero(const_num_states, const_num_inputs);
		B_dynamics_matrix(0, 2) = const_gravity / (6.0 * yaml_control_frequency * yaml_control_frequency * yaml_control_frequency);
		B_dynamics_matrix(1, 1) = -const_gravity / (6.0 * yaml_control_frequency * yaml_control_frequency * yaml_control_frequency);
		B_dynamics_matrix(2, 0) = 1.0 / (2.0 * yaml_cf_mass_in_grams * yaml_control_frequency * yaml_control_frequency);
		B_dynamics_matrix(3, 2) = const_gravity / (2.0 * yaml_control_frequency * yaml_control_frequency);
		B_dynamics_matrix(4, 1) = -const_gravity / (2.0 * yaml_control_frequency * yaml_control_frequency);
		B_dynamics_matrix(5, 0) = 1.0 / (yaml_cf_mass_in_grams * yaml_control_frequency);
		B_dynamics_matrix.bottomRightCorner(2, 2) = 1.0 / yaml_control_frequency * MatrixXf::Identity(2, 2);



    	// *** BUILD THE OSQP QUADRATIC COST MATRIX P ***
    	// order of decision variables: (state_0, input_0, ... , state_{T-1}, input_{T-1}, state_T)
		MatrixXf osqp_P_matrix = MatrixXf::Zero(num_decision_variables, num_decision_variables);

		// TODO:	Build osqp_P_matrix
		// 			order of decision variables: (state_0, input_0, ... , state_{T-1}, input_{T-1}, state_T)
		//
		//			available variables:	const_num_states				:	number of states (8)
		//									const_num_inputs				:	number of inputs (3)
		//									const_num_states_plus_inputs	:	number of states plus inputs (11)
		//									yaml_prediction_horizon			:	prediction horizon T
		//									m_state_cost_matrix				:	quadratic state cost matrix Q
		//									input_cost_matrix				:	quadratic input cost matrix R



		
		// *** BUILD THE OSQP LINEAR COST VECTOR q ***
		// this is where we specify the state setpoint to the optimization
		m_osqp_q_vector = MatrixXf::Zero(num_decision_variables, 1);

		// TODO:	Build m_osqp_q_vector
		// 			order of decision variables: (state_0, input_0, ... , state_{T-1}, input_{T-1}, state_T)
		//
		//			available variables:	const_num_states				:	number of states (8)
		//									const_num_states_plus_inputs	:	number of states plus inputs (11)
		//									yaml_prediction_horizon			:	prediction horizon T
		//									m_state_cost_matrix				:	quadratic state cost matrix Q
		//									m_state_setpoint_vector			:	current state reference x_{ref}

		// Step 1: build the OSQP q vector for one time step
		m_osqp_q_vector_for_one_time_step = MatrixXf::Zero(const_num_states, 1);	// to be replaced


		// Step 2: fill in all block entries of the OSQP q vector




		// *** BUILD THE OSQP EQUALITY CONSTRAINTS MATRIX FOR THE DYNAMICS ***
		// each state decision variable will have an equality constraint specifying its dynamics
		MatrixXf osqp_A_equality_matrix = MatrixXf::Zero(num_state_decision_variables, num_decision_variables);
		
		// TODO:	osqp_A_equality_matrix
		// 			order of decision variables: (state_0, input_0, ... , state_{T-1}, input_{T-1}, state_T)
		//
		//			available variables:	const_num_states				:	number of states (8)
		//									const_num_inputs				:	number of inputs (3)
		//									const_num_states_plus_inputs	:	number of states plus inputs (11)
		//									yaml_prediction_horizon			:	prediction horizon T
		//									m_state_cost_matrix				:	quadratic state cost matrix Q
		//									m_state_setpoint_vector			:	current state reference x_{ref}

		// Step 1: the first const_num_states equality constraints set the initial state and are of the form 0_{num_states} = I * x_0



		// Step 2: build the [A, B, -I] block for the dynamics of one time step 0_{num_states} = A * x_i + B * u_i - I * x_{i+1}
		MatrixXf ABI_matrix = MatrixXf::Zero(const_num_states, const_num_states_plus_inputs + const_num_states);
		ABI_matrix.leftCols(const_num_states) = A_dynamics_matrix;
		ABI_matrix.middleCols(const_num_states, const_num_inputs) = B_dynamics_matrix;
		ABI_matrix.rightCols(const_num_states) = -MatrixXf::Identity(const_num_states, const_num_states);



		// Step 3: fill in the dynamics blocks



		// Step 4: set the OSQP LHS and RHS inequality vectors to 0 to get equality
		MatrixXf osqp_l_equality_vector = MatrixXf::Zero(num_state_decision_variables, 1);
		MatrixXf osqp_u_equality_vector = MatrixXf::Zero(num_state_decision_variables, 1);


		
		// *** BUILD THE OSQP INEQUALITY CONSTRAINTS MARTIX FOR THE STATE AND INPUT BOUNDS ***
		MatrixXf osqp_A_inequality_matrix = MatrixXf::Identity(num_decision_variables, num_decision_variables);

		// build the OSQP bound vectors l and u for one time step
		MatrixXf osqp_l_inequality_vector_for_one_time_step = MatrixXf::Zero(const_num_states_plus_inputs, 1);
		MatrixXf osqp_u_inequality_vector_for_one_time_step = MatrixXf::Zero(const_num_states_plus_inputs, 1);

		// state constraints for one time step
		for (int i = 0; i < const_num_states; i++)
		{
			osqp_l_inequality_vector_for_one_time_step(i) = yaml_min_state_constraints[i];
			osqp_u_inequality_vector_for_one_time_step(i) = yaml_max_state_constraints[i];
		}
		// input constraints for one time step
		for (int i = 0; i < const_num_inputs; i++)
		{
			osqp_l_inequality_vector_for_one_time_step(const_num_states + i) = yaml_min_input_constraints[i];
			osqp_u_inequality_vector_for_one_time_step(const_num_states + i) = yaml_max_input_constraints[i];
		}
		// subtract feedforward equilibrium thrust from total thrust bounds
		osqp_l_inequality_vector_for_one_time_step(const_num_states) -= m_cf_weight_in_newtons;
		osqp_u_inequality_vector_for_one_time_step(const_num_states) -= m_cf_weight_in_newtons;

		// build the OSQP bound vectors l and u for all time steps
		MatrixXf osqp_l_inequality_vector = MatrixXf::Zero(num_decision_variables, 1);
		MatrixXf osqp_u_inequality_vector = MatrixXf::Zero(num_decision_variables, 1);
		for (int i = 0; i < yaml_prediction_horizon; i++)
		{
			osqp_l_inequality_vector.middleRows(i * const_num_states_plus_inputs, const_num_states_plus_inputs) = osqp_l_inequality_vector_for_one_time_step;
			osqp_u_inequality_vector.middleRows(i * const_num_states_plus_inputs, const_num_states_plus_inputs) = osqp_u_inequality_vector_for_one_time_step;
		}
		
		// add the terminal state constraints
		osqp_l_inequality_vector.bottomRows(const_num_states) = osqp_l_inequality_vector_for_one_time_step.topRows(const_num_states);
		osqp_u_inequality_vector.bottomRows(const_num_states) = osqp_u_inequality_vector_for_one_time_step.topRows(const_num_states);



		// *** CONCATENATE THE OSQP INEQUALITY AND EQUALITY CONSTRAINTS ***
		MatrixXf osqp_A_matrix = MatrixXf::Zero(osqp_A_equality_matrix.rows() + osqp_A_inequality_matrix.rows(), num_decision_variables);
		MatrixXf osqp_l_vector = MatrixXf::Zero(osqp_l_equality_vector.rows() + osqp_l_inequality_vector.rows(), 1);
		MatrixXf osqp_u_vector = MatrixXf::Zero(osqp_u_equality_vector.rows() + osqp_u_inequality_vector.rows(), 1);

		// equality constraints
		osqp_A_matrix.topRows(osqp_A_equality_matrix.rows()) = osqp_A_equality_matrix;
		osqp_l_vector.topRows(osqp_l_equality_vector.rows()) = osqp_l_equality_vector;
		osqp_u_vector.topRows(osqp_u_equality_vector.rows()) = osqp_u_equality_vector;
		
		// inequality constraints
		osqp_A_matrix.bottomRows(osqp_A_inequality_matrix.rows()) = osqp_A_inequality_matrix;
		osqp_l_vector.bottomRows(osqp_l_inequality_vector.rows()) = osqp_l_inequality_vector;
		osqp_u_vector.bottomRows(osqp_u_inequality_vector.rows()) = osqp_u_inequality_vector;


		
		// *** OSQP MODEL SETUP ***
		// follows 'Setup and solve' example (https://osqp.org/docs/examples/setup-and-solve.html)

		// first make sure any previosuly setup models are deallocated from memory
		osqp_extended_cleanup();

		// convert Eigen matrices to CSC format
		csc* osqp_P_csc = eigen2csc(osqp_P_matrix);
		csc* osqp_A_csc = eigen2csc(osqp_A_matrix);

		// convert Eigen vectors to c_float arrays
		// this copy will be used in the OSQPData data structure, which will be destroyed after setup is complete
		c_float* osqp_q_cfloat = (c_float*) c_malloc(m_osqp_q_vector.rows() * sizeof(c_float));
		c_float* osqp_l_cfloat = (c_float*) c_malloc(osqp_l_vector.rows() * sizeof(c_float));
		c_float* osqp_u_cfloat = (c_float*) c_malloc(osqp_u_vector.rows() * sizeof(c_float));

		// this copy will be used during runtime to change the initial states and the tracking setpoints
		m_osqp_q_runtime_cfloat = (c_float*) c_malloc(m_osqp_q_vector.rows() * sizeof(c_float));
		m_osqp_l_runtime_cfloat = (c_float*) c_malloc(osqp_l_vector.rows() * sizeof(c_float));
		m_osqp_u_runtime_cfloat = (c_float*) c_malloc(osqp_u_vector.rows() * sizeof(c_float));

		// the following syntax copies Eigen vectors to c_float arrays
		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_q_cfloat, m_osqp_q_vector.rows(), m_osqp_q_vector.cols()) = m_osqp_q_vector.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(m_osqp_q_runtime_cfloat, m_osqp_q_vector.rows(), m_osqp_q_vector.cols()) = m_osqp_q_vector.cast<c_float>();

		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_l_cfloat, osqp_l_vector.rows(), osqp_l_vector.cols()) = osqp_l_vector.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(m_osqp_l_runtime_cfloat, osqp_l_vector.rows(), osqp_l_vector.cols()) = osqp_l_vector.cast<c_float>();

		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_u_cfloat, osqp_u_vector.rows(), osqp_u_vector.cols()) = osqp_u_vector.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(m_osqp_u_runtime_cfloat, osqp_u_vector.rows(), osqp_u_vector.cols()) = osqp_u_vector.cast<c_float>();

		// populate OSQP model data
	    OSQPData* osqp_data = (OSQPData*) c_malloc(sizeof(OSQPData));
	    osqp_data->n = osqp_A_matrix.cols();
	    osqp_data->m = osqp_A_matrix.rows();
	    osqp_data->P = osqp_P_csc;
	    osqp_data->q = osqp_q_cfloat;
	    osqp_data->A = osqp_A_csc;
	    osqp_data->l = osqp_l_cfloat;
	    osqp_data->u = osqp_u_cfloat;

		// OSQP solver settings: define as default, then change settings as desired 
	    OSQPSettings* osqp_settings = (OSQPSettings*) c_malloc(sizeof(OSQPSettings));
	    osqp_set_default_settings(osqp_settings);
	    // d_osqp_settings->verbose = true;

	    // setup OSQP workspace
	    m_osqp_work = osqp_setup(osqp_data, osqp_settings);

	    // clear data and settings after setting up to allow subseqeuent setups
	    osqp_cleanup_data(osqp_data);
	    c_free(osqp_settings);

	    // set the MPC optimization setup success flag
	    if (!m_osqp_work)
	    {
	    	m_mpc_optimization_setup_success = false;

	    	ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization setup with OSQP failed");
	    	ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization must be (re-)setup");

	    	// communicate optimization setup status to GUI
			send_mpc_optimization_setup_status_to_gui();

	    	return;
	    }

	    // setup successful flag
	    m_mpc_optimization_setup_success = true;

	    ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization setup with OSQP successful");

	    // communicate optimization setup status to GUI
		send_mpc_optimization_setup_status_to_gui();
    }

  	catch(std::exception& e)
    {
    	m_mpc_optimization_setup_success = false;

	    ROS_INFO_STREAM("[TUTORIAL CONTROLLER] MPC optimization setup with OSQP exception, error message: " << e.what());
	    ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization must be (re-)setup");

	    // communicate optimization setup status to GUI
		send_mpc_optimization_setup_status_to_gui();
  	}
  	catch(...)
  	{
  		m_mpc_optimization_setup_success = false;

    	ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization setup with OSQP unknown exception");
    	ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization must be (re-)setup");

    	// communicate optimization setup status to GUI
		send_mpc_optimization_setup_status_to_gui();
  	}
}



// *** CHANGE THE MPC OPTIMIZATION SETPOINT ***
void change_mpc_optimization_setpoint()
{
	try
	{
		// *** UPDATE THE OSQP LINEAR COST VECTOR q ***
		// update the OSQP q vector for one time step
		m_osqp_q_vector_for_one_time_step = -m_state_cost_matrix * m_state_setpoint_vector;

		// fill in the block entries of the OSQP q vector
		for (int i = 0; i <= yaml_prediction_horizon; i++)
			m_osqp_q_vector.middleRows(i * const_num_states_plus_inputs, const_num_states) = m_osqp_q_vector_for_one_time_step;

		// convert Eigen vector to c_float array
		Matrix<c_float, Dynamic, Dynamic>::Map(m_osqp_q_runtime_cfloat, m_osqp_q_vector.rows(), m_osqp_q_vector.cols()) = m_osqp_q_vector.cast<c_float>();

		// update OSQP linear cost
		osqp_update_lin_cost(m_osqp_work, m_osqp_q_runtime_cfloat);



	    // *** INFORM THE USER ***
	    ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization setpoint updated with OSQP successfully");
	}

  	catch(std::exception& e)
    {
    	m_mpc_optimization_setup_success = false;

	    ROS_INFO_STREAM("[TUTORIAL CONTROLLER] MPC optimization setpoint update with OSQP exception, error message: " << e.what());
	    ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization must be (re-)setup");

	    // communicate optimization setup status to GUI
		send_mpc_optimization_setup_status_to_gui();
  	}
  	catch(...)
  	{
    	m_mpc_optimization_setup_success = false;

	    ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization setpoint update with OSQP unknown exception");
	    ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization must be (re-)setup");

	    // communicate optimization setup status to GUI
		send_mpc_optimization_setup_status_to_gui();
  	}
}

// *** UPDATE INITIAL CONDITION AND SOLVE OPTIMIZATION ***
bool update_initial_condition_and_solve_mpc_optimization()
{
	try
	{
		// *** UPDATE THE EQUALITY CONSTRAINT VECTORS FOR THE INITIAL STATE ***
		// the initial state equality constraint is the first constraint defined
		for (int i = 0; i < const_num_states; i++)
		{
			m_osqp_l_runtime_cfloat[i] = m_current_state_vector(i);
			m_osqp_u_runtime_cfloat[i] = m_current_state_vector(i);
		}
		
		// update the OSQP constraint bounds
		osqp_update_bounds(m_osqp_work, m_osqp_l_runtime_cfloat, m_osqp_u_runtime_cfloat);



		// *** SOLVE OPTIMIZATION ***
		osqp_solve(m_osqp_work);
		


		// *** GET SOLUTION IF SUCCESSFULLY SOLVED ***
		// status_val > 0 means that an optimal solution was found
		// refer to OSQP "constants.h" for the detailed status codes
		if (m_osqp_work->info->status_val > 0)
		{	
			// get the MPC input to apply, which is the optimal input for the first time step
			for (int i = 0; i < const_num_inputs; i++)
				m_mpc_input_vector_to_apply(i) = m_osqp_work->solution->x[const_num_states + i];


			// inform the user
			ROS_INFO_STREAM("[TUTORIAL CONTROLLER] MPC optimal solution found with OSQP, status: " << m_osqp_work->info->status);
			ROS_INFO_STREAM("Thrust: "     << m_mpc_input_vector_to_apply(0));
			ROS_INFO_STREAM("Roll Rate: "  << m_mpc_input_vector_to_apply(1));
			ROS_INFO_STREAM("Pitch Rate: " << m_mpc_input_vector_to_apply(2));
			ROS_INFO_STREAM("Objective: " << m_osqp_work->info->obj_val);
			ROS_INFO_STREAM("Runtime: " << m_osqp_work->info->run_time);

			return true;
		}
		

		// we arrive here if optimal solution was not found
		ROS_INFO_STREAM("[TUTORIAL CONTROLLER] MPC failed to find optimal solution with OSQP, status: " << m_osqp_work->info->status);

		return false;
	}

  	catch(std::exception& e)
    {
    	m_mpc_optimization_setup_success = false;

	    ROS_INFO_STREAM("[TUTORIAL CONTROLLER] MPC optimization solution with OSQP exception, error message: " << e.what());
	    ROS_INFO("[TUTORIAL CONTROLLER] MPC optimization must be (re-)setup");

	    // communicate optimization setup status to GUI
		send_mpc_optimization_setup_status_to_gui();
  	}
  	catch(...)
  	{
  		m_mpc_optimization_setup_success = false;

    	ROS_INFO("[MPC CONTROLLER] MPC optimization solution with OSQP unknown exception");
    	ROS_INFO("[MPC CONTROLLER] MPC optimization must be (re-)setup");

    	// communicate optimization setup status to GUI
		send_mpc_optimization_setup_status_to_gui();
  	}
}


// *** HELPER FUNCTIONS ***

// Convert Eigen Dense matrix to CSC format used in OSQP
csc* eigen2csc(const MatrixXf& eigen_dense_mat)
{
	// First convert Eigen Dense matrix to Eigen Sparse
	SparseMatrix<float> eigen_sparse_mat = eigen_dense_mat.sparseView();

	// Second convert Eigen Sparse matrix to TRIPLET format, because it is not as mind bending as CSC format
	csc* trip_mat = csc_spalloc(eigen_sparse_mat.rows(), eigen_sparse_mat.cols(), eigen_sparse_mat.nonZeros(), 1, 1);
	trip_mat->nz = eigen_sparse_mat.nonZeros();
	// The following code was found under 'Iterating over the nonzero coefficients' section here: https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
	int i = 0;
	for (int j = 0; j < eigen_sparse_mat.outerSize(); j++)
		for (SparseMatrix<float>::InnerIterator it(eigen_sparse_mat, j); it; ++it)
			{
				trip_mat->x[i] = it.value();	// Value
				trip_mat->i[i] = it.row();		// Row index
				trip_mat->p[i] = it.col();		// Column index
				
				i++;
			}

	// Third convert TRIPLET matrix to CSC format, using OSQP built in function
	csc* csc_mat = triplet_to_csc(trip_mat, OSQP_NULL);

	// triplet_to_csc makes new copy of matrix, so intermediate TRIPLET matrix must be de-allocated
	csc_spfree(trip_mat);

	return csc_mat;
}


// COMMUNICATE OPTIMIZATION SETUP STATUS TO GUI
void send_mpc_optimization_setup_status_to_gui()
{
	// Use x field of Setpoint msg data structure because too lazy to define new message type
	SetpointWithHeader msg;
	msg.x = 1.0 * m_mpc_optimization_setup_success;
	
	// Publish the message
	m_optimizationSetupStatusChangedPublisher.publish(msg);
}


// OSQP EXTENDED CLEANUP
// frees any data structures to which memory was allocated
void osqp_extended_cleanup()
{
	osqp_cleanup(m_osqp_work);
	c_free(m_osqp_q_runtime_cfloat);
	c_free(m_osqp_l_runtime_cfloat);
	c_free(m_osqp_u_runtime_cfloat);
}


// OSQP DATA CLEANUP
// frees all the memory allocated to the OSQPData data structure
void osqp_cleanup_data(OSQPData* data)
{
	if (!data)
		return;

	csc_spfree(data->P);
	csc_spfree(data->A);
	c_free(data->q);
	c_free(data->l);
	c_free(data->u);

	c_free(data);
}



//    ------------------------------------------------------------------------------
//     OOO   U   U  TTTTT  EEEEE  RRRR 
//    O   O  U   U    T    E      R   R
//    O   O  U   U    T    EEE    RRRR
//    O   O  U   U    T    E      R  R
//     OOO    UUU     T    EEEEE  R   R
//
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L           L       OOO    OOO   PPPP
//    C      O   O  NN  N    T    R   R  O   O  L           L      O   O  O   O  P   P
//    C      O   O  N N N    T    RRRR   O   O  L           L      O   O  O   O  PPPP
//    C      O   O  N  NN    T    R  R   O   O  L           L      O   O  O   O  P
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL       LLLLL   OOO    OOO   P
//    ----------------------------------------------------------------------------------

// This function is the callback that is linked to the "TutorialController" service that
// is advertised in the main function. This must have arguments that match the
// "input-output" behaviour defined in the "Controller.srv" file (located in the "srv"
// folder)
//
// The arument "request" is a structure provided to this service with the following two
// properties:
//
// >> request.ownCrazyflie
// This property is itself a structure of type "FlyingVehicleState",  which is defined in the
// file "FlyingVehicleState.msg", and has the following properties
//     string vehicleName
//     float64 x                         The x position of the Crazyflie [metres]
//     float64 y                         The y position of the Crazyflie [metres]
//     float64 z                         The z position of the Crazyflie [metres]
//     float64 roll                      The roll component of the intrinsic Euler angles [radians]
//     float64 pitch                     The pitch component of the intrinsic Euler angles [radians]
//     float64 yaw                       The yaw component of the intrinsic Euler angles [radians]
//     float64 acquiringTime #delta t    The time elapsed since the previous "FlyingVehicleState" was received [seconds]
//     bool isValid                      A boolean indicted whether the Crazyflie for visible at the time of this measurement
// The values in these properties are directly the measurement taken by the Vicon
// motion capture system of the Crazyflie that is to be controlled by this service
//
// >> request.otherCrazyflies
// This property is an array of "FlyingVehicleState" structures, what allows access to the
// Vicon measurements of other Crazyflies.
//
// The argument "response" is a structure that is expected to be filled in by this
// service by this function, it has only the following property
//
// >> response.ControlCommand
// This property is iteself a structure of type "ControlCommand", which is
// defined in the file "ControlCommand.msg", and has the following properties:
//     uint16 motorCmd1                 The command sent to the Crazyflie for motor 1
//     uint16 motorCmd2                 ... same for motor 2
//     uint16 motorCmd3                 ... same for motor 3
//     uint16 motorCmd4                 ... same for motor 4
//     uint8 xControllerMode            The mode sent to the Crazyflie for what controller to run for the body frame x-axis
//     uint8 yControllerMode            ... same body frame y-axis
//     uint8 zControllerMode            ... same body frame z-axis
//     uint8 yawControllerMode          ... same body frame yaw
//     float32 xControllerSetpoint      The setpoint sent to the Crazyflie for the body frame x-axis controller
//     float32 yControllerSetpoint      ... same body frame y-axis
//     float32 zControllerSetpoint      ... same body frame z-axis
//     float32 yawControllerSetpoint    ... same body frame yaw
// 
// IMPORTANT NOTES FOR "{x,y,z,yaw}ControllerMode"  AND AXIS CONVENTIONS
// > The allowed values for "{x,y,z,yaw}ControllerMode" are in the
//   "Constants.h" header file, they are:
//   - CF_ONBOARD_CONTROLLER_MODE_OFF
//   - CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE
//   - CF_ONBOARD_CONTROLLER_MODE_ANGLE
//   - CF_ONBOARD_CONTROLLER_MODE_VELOCITY
//   - CF_ONBOARD_CONTROLLER_MODE_POSITION
//
// > The most common option to use for the {x,y,yaw} controller is
//   the CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE option.
//
// > The most common option to use for the {z} controller is
//   the CF_ONBOARD_CONTROLLER_MODE_OFF option, and thus the
//   body frame z-axis is controlled by the motorCmd{1,2,3,4}
//   values that you set.
//
// > When the CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE is selected, then:
//   1) the ".xControllerSetpoint", ".yControllerSetpoint", and
//      ".yawControllerSetpoint" properties of "response.ControlCommand"
//      specify the angular rate in [radians/second] that will be requested
//      from the PID controllers running in the Crazyflie 2.0 firmware.
//   2) the axis convention for the roll, pitch, and yaw body rates,
//      i.e., as set in the {y,x,yaw}ControllerSetpoint properties of
//      the "response.ControlCommand" that you return, is a right-hand
//      coordinate axes with x-forward and z-upwards (i.e., the positive
//      z-axis is aligned with the direction of positive thrust). To
//      assist, the ASCII art below depicts this convention.
//   3) the ".motorCmd1" to ".motorCmd4" properties of
//      "response.ControlCommand" are the baseline motor commands
//      requested from the Crazyflie, with the adjustment for body rates
//      being added on top of this in the firmware (i.e., as per the
//      code of the "distribute_power" found in the firmware).
//
// ASCII ART OF THE CRAZYFLIE 2.0 LAYOUT
//
//  > This is a top view,
//  > M1 to M4 stand for Motor 1 to Motor 4,
//  > "CW"  indicates that the motor rotates Clockwise,
//  > "CCW" indicates that the motor rotates Counter-Clockwise,
//  > By right-hand axis convention, the positive z-direction points our of the screen,
//  > This being a "top view" means tha the direction of positive thrust produced
//    by the propellers is also out of the screen.
//
//        ____                         ____
//       /    \                       /    \
//  (CW) | M4 |           x           | M1 | (CCW)
//       \____/\          ^          /\____/
//            \ \         |         / /
//             \ \        |        / /
//              \ \______ | ______/ /
//               \        |        /
//                |       |       |
//        y <-------------o       |
//                |               |
//               / _______________ \
//              / /               \ \
//             / /                 \ \
//        ____/ /                   \ \____
//       /    \/                     \/    \
// (CCW) | M3 |                       | M2 | (CW)
//       \____/                       \____/
//
//
//
bool calculateControlOutput(Controller::Request &request, Controller::Response &response)
{

	// This is the "start" of the outer loop controller, add all your control
	// computation here, or you may find it convienient to create functions
	// to keep you code cleaner


	// Define a local array to fill in with the state error
	float stateErrorInertial[9];

	// Fill in the (x,y,z) position error
	stateErrorInertial[0] = request.ownCrazyflie.x - m_setpoint[0];
	stateErrorInertial[1] = request.ownCrazyflie.y - m_setpoint[1];
	stateErrorInertial[2] = request.ownCrazyflie.z - m_setpoint[2];

	

	// Compute an estimate of the velocity
	// > Via finite differences if receiveing
	//   Motion Capture data
	if (request.ownCrazyflie.type == FLYING_VEHICLE_STATE_TYPE_MOCAP_MEASUREMENT)
	{
		// > But only if this is NOT the first call
		//   to the controller
		if (!request.isFirstControllerCall)
		{
			// > Compute as simply the derivative between
			//   the current and previous position
			stateErrorInertial[3] = (stateErrorInertial[0] - m_previous_stateErrorInertial[0]) * yaml_control_frequency;
			stateErrorInertial[4] = (stateErrorInertial[1] - m_previous_stateErrorInertial[1]) * yaml_control_frequency;
			stateErrorInertial[5] = (stateErrorInertial[2] - m_previous_stateErrorInertial[2]) * yaml_control_frequency;
		}
		else
		{
			// Set the velocities to zero
			stateErrorInertial[3] = 0.0;
			stateErrorInertial[4] = 0.0;
			stateErrorInertial[5] = 0.0;
		}
	}
	// > Else, via finite difference if receiveing
	//   onboard state estimate data
	else if (request.ownCrazyflie.type == FLYING_VEHICLE_STATE_TYPE_CRAZYFLIE_STATE_ESTIMATE)
	{
		stateErrorInertial[3] = request.ownCrazyflie.vx;
		stateErrorInertial[4] = request.ownCrazyflie.vy;
		stateErrorInertial[5] = request.ownCrazyflie.vz;
	}
	else
	{
		ROS_ERROR_STREAM("[TUTORIAL CONTROLLER] Received a request.ownCrazyflie with unrecognised type, request.ownCrazyflie.type = " << request.ownCrazyflie.type );
	}
	

	// Fill in the roll and pitch angle measurements directly
	stateErrorInertial[6] = request.ownCrazyflie.roll;
	stateErrorInertial[7] = request.ownCrazyflie.pitch;
	

	// Fill in the yaw angle error
	// > This error should be "unwrapped" to be in the range
	//   ( -pi , pi )
	// > First, get the yaw error into a local variable
	float yawError = request.ownCrazyflie.yaw - m_setpoint[3];
	// > Second, "unwrap" the yaw error to the interval ( -pi , pi )
	while (yawError > PI) {yawError -= 2 * PI;}
	while (yawError < -PI) {yawError += 2 * PI;}
	// > Third, put the "yawError" into the "stateError" variable
	stateErrorInertial[8] = yawError;



	// FILL IN THE STATE FOR THE MPC OPTIMIZATION
	// note that MPC controller does not control yaw
	m_current_state_vector(0) = request.ownCrazyflie.x;
	m_current_state_vector(1) = request.ownCrazyflie.y;
	m_current_state_vector(2) = request.ownCrazyflie.z;
	m_current_state_vector(3) = stateErrorInertial[3];
	m_current_state_vector(4) = stateErrorInertial[4];
	m_current_state_vector(5) = stateErrorInertial[5];
	m_current_state_vector(6) = request.ownCrazyflie.roll;
	m_current_state_vector(7) = request.ownCrazyflie.pitch;



	// CONVERSION INTO BODY FRAME
	// Conver the state erorr from the Inertial frame into the Body frame
	// > Note: the function "convertIntoBodyFrame" is implemented in this file
	//   and by default does not perform any conversion. The equations to convert
	//   the state error into the body frame should be implemented in that function
	//   for successful completion of the classroom exercise
	float stateErrorBody[9];
	convertIntoBodyFrame(stateErrorInertial, stateErrorBody, request.ownCrazyflie.yaw);


	// SAVE THE STATE ERROR TO BE USED NEXT TIME THIS FUNCTION IS CALLED
	// > as we have already used previous error we can now update it update it
	for (int i = 0; i < 9; i++)
		m_previous_stateErrorInertial[i] = stateErrorInertial[i];


	
	// INITIALIZE CONTROLLER COMMANDS
	float totalThrust_command;
	float rollRate_command;
	float pitchRate_command;
	float yawRate_command;

	// YAW CONTROL
	// controlled by backup LQR controller and not MPC
	yawRate_command = 0.0;
	for (int i = 0; i < 9; i++)
		yawRate_command -= m_gainMatrixYawRate[i] * stateErrorBody[i];

	

	// IF MPC OPTIMIZATION IS SETUP SUCCESSFULLY, ATTEMPT TO USE MPC CONTROL
	bool use_mpc_controller = false;
	if (m_mpc_optimization_setup_success)
		use_mpc_controller = update_initial_condition_and_solve_mpc_optimization();

	// if MPC optimization successfully found a solution, use solution
	if (use_mpc_controller)
	{
		totalThrust_command = m_mpc_input_vector_to_apply(0) + m_cf_weight_in_newtons;
		rollRate_command = m_mpc_input_vector_to_apply(1);
		pitchRate_command = m_mpc_input_vector_to_apply(2);
	}

	// else, issue 0 command
	else
	{
		totalThrust_command = 0.0;
		rollRate_command = 0.0;
		pitchRate_command = 0.0;
		yawRate_command = 0.0;
	}
	


	// *** SET THE REPONSE VARIABLE WITH THE INPUT COMMANDS ***
	
	// THRUST COMMANDS FOR Z
	// > NOTE: the function "newtons2cmd_for_crazyflie" converts the input argument
	//         from Newtons to the 16-bit command expected by the Crazyflie.
	float perMotorThrust_command = totalThrust_command / 4.0;
	response.controlOutput.motorCmd1 = newtons2cmd_for_crazyflie(perMotorThrust_command);
	response.controlOutput.motorCmd2 = newtons2cmd_for_crazyflie(perMotorThrust_command);
	response.controlOutput.motorCmd3 = newtons2cmd_for_crazyflie(perMotorThrust_command);
	response.controlOutput.motorCmd4 = newtons2cmd_for_crazyflie(perMotorThrust_command);
	
	// Set the onboard z-controller to be OFF
	// > This is because we commands the motor thrusts and
	//   hence an "inner" control loop is NOT needed onboard
	//   to control the z-height
	response.controlOutput.zControllerMode = CF_ONBOARD_CONTROLLER_MODE_OFF;


	// PITCH RATE COMMAND FOR X
	// Put the computed pitch rate into the "response" variable
	// > The "controller mode" specifies that it is an
	//   angular rate setpoint
	response.controlOutput.xControllerMode     = CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE;
	response.controlOutput.xControllerSetpoint = pitchRate_command;
	

	// ROLL RATE COMMAND FOR Y
	// Put the computed roll rate into the "response" variable
	// > The "controller mode" specifies that it is an
	//   angular rate setpoint
	response.controlOutput.yControllerMode     = CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE;
	response.controlOutput.yControllerSetpoint = rollRate_command;


	// YAW RATE COMMAND FOR YAW
	// Put the computed yaw rate into the "response" variable
	// > The "controller mode" specifies that it is an
	//   angular rate setpoint
	response.controlOutput.yawControllerMode     = CF_ONBOARD_CONTROLLER_MODE_ANGULAR_RATE;
	response.controlOutput.yawControllerSetpoint = yawRate_command;



	//  ***********************************************************
	//  DDDD   EEEEE  BBBB   U   U   GGGG       M   M   SSSS   GGGG
	//  D   D  E      B   B  U   U  G           MM MM  S      G
	//  D   D  EEE    BBBB   U   U  G           M M M   SSS   G
	//  D   D  E      B   B  U   U  G   G       M   M      S  G   G
	//  DDDD   EEEEE  BBBB    UUU    GGGG       M   M  SSSS    GGGG

	// DEBUGGING CODE:
	// As part of the D-FaLL-System we have defined a message type names"DebugMsg".
	// By fill this message with data and publishing it you can display the data in
	// real time using rpt plots. Instructions for using rqt plots can be found on
	// the wiki of the D-FaLL-System repository

	// Instantiate a local variable of type "DebugMsg", see the file "DebugMsg.msg"
	// (located in the "msg" folder) to see the full structure of this message.
	DebugMsg debugMsg;

	// Fill the debugging message with the data provided by Vicon
	debugMsg.vicon_x = request.ownCrazyflie.x;
	debugMsg.vicon_y = request.ownCrazyflie.y;
	debugMsg.vicon_z = request.ownCrazyflie.z;
	debugMsg.vicon_roll = request.ownCrazyflie.roll;
	debugMsg.vicon_pitch = request.ownCrazyflie.pitch;
	debugMsg.vicon_yaw = request.ownCrazyflie.yaw;

	// Fill in the debugging message with any other data you would like to display
	// in real time. For example, it might be useful to display the thrust
	// adjustment computed by the z-altitude controller.
	// The "DebugMsg" type has 10 properties from "value_1" to "value_10", all of
	// type "float64" that you can fill in with data you would like to plot in
	// real-time.
	debugMsg.value_1 = totalThrust_command - m_cf_weight_in_newtons;
	debugMsg.value_2 = rollRate_command;
	debugMsg.value_3 = pitchRate_command;
	debugMsg.value_4 = yawRate_command;
	// ......................
	// debugMsg.value_10 = your_variable_name;

	// Publish the "debugMsg"
	m_debugPublisher.publish(debugMsg);

	// An alternate debugging technique is to print out data directly to the
	// command line from which this node was launched.

	// An example of "printing out" the data from the "request" argument to the
	// command line. This might be useful for debugging.
	// ROS_INFO_STREAM("x-coordinates: " << request.ownCrazyflie.x);
	// ROS_INFO_STREAM("y-coordinates: " << request.ownCrazyflie.y);
	// ROS_INFO_STREAM("z-coordinates: " << request.ownCrazyflie.z);
	// ROS_INFO_STREAM("roll:  " << request.ownCrazyflie.roll);
	// ROS_INFO_STREAM("pitch: " << request.ownCrazyflie.pitch);
	// ROS_INFO_STREAM("yaw:   " << request.ownCrazyflie.yaw);
	// ROS_INFO_STREAM("Delta t: " << request.ownCrazyflie.acquiringTime);

	// An example of "printing out" the control actions computed.
	// ROS_INFO_STREAM("thrustAdjustment = " << thrustAdjustment);
	// ROS_INFO_STREAM("controlOutput.xControllerSetpoint   = " << response.controlOutput.xControllerSetpoint);
	// ROS_INFO_STREAM("controlOutput.yControllerSetpoint   = " << response.controlOutput.yControllerSetpoint);
	// ROS_INFO_STREAM("controlOutput.yawControllerSetpoint = " << response.controlOutput.yawControllerSetpoint);

	// An example of "printing out" the per motor 16-bit command computed.
	// ROS_INFO_STREAM("controlOutput.cmd1 = " << response.controlOutput.motorCmd1);
	// ROS_INFO_STREAM("controlOutput.cmd3 = " << response.controlOutput.motorCmd2);
	// ROS_INFO_STREAM("controlOutput.cmd2 = " << response.controlOutput.motorCmd3);
	// ROS_INFO_STREAM("controlOutput.cmd4 = " << response.controlOutput.motorCmd4);

	// Return "true" to indicate that the control computation was performed successfully
	return true;
	}




//    ------------------------------------------------------------------------------
//    RRRR    OOO   TTTTT    A    TTTTT  EEEEE       III  N   N  TTTTT   OOO
//    R   R  O   O    T     A A     T    E            I   NN  N    T    O   O
//    RRRR   O   O    T    A   A    T    EEE          I   N N N    T    O   O
//    R  R   O   O    T    AAAAA    T    E            I   N  NN    T    O   O
//    R   R   OOO     T    A   A    T    EEEEE       III  N   N    T     OOO
//
//    BBBB    OOO   DDDD   Y   Y       FFFFF  RRRR     A    M   M  EEEEE
//    B   B  O   O  D   D   Y Y        F      R   R   A A   MM MM  E
//    BBBB   O   O  D   D    Y         FFF    RRRR   A   A  M M M  EEE
//    B   B  O   O  D   D    Y         F      R  R   AAAAA  M   M  E
//    BBBB    OOO   DDDD     Y         F      R   R  A   A  M   M  EEEEE
//    ----------------------------------------------------------------------------------

// The arguments for this function are as follows:
// stateInertial
// This is an array of length 9 with the estimates the error of of the following values
// relative to the sepcifed setpoint:
//     stateInertial[0]    x position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[1]    y position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[2]    z position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[3]    x-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[4]    y-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[5]    z-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[6]    The roll  component of the intrinsic Euler angles [radians]
//     stateInertial[7]    The pitch component of the intrinsic Euler angles [radians]
//     stateInertial[8]    The yaw   component of the intrinsic Euler angles [radians]
// 
// stateBody
// This is an empty array of length 9, this function should fill in all elements of this
// array with the same ordering as for the "stateInertial" argument, expect that the (x,y)
// position and (x,y) velocities are rotated into the body frame.
//
// yaw_measured
// This is the yaw component of the intrinsic Euler angles in [radians] as measured by
// the Vicon motion capture system
//
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured)
{

	float sinYaw = sin(yaw_measured);
	float cosYaw = cos(yaw_measured);

	// Fill in the (x,y,z) position estimates to be returned
	stateBody[0] = stateInertial[0] * cosYaw  +  stateInertial[1] * sinYaw;
	stateBody[1] = stateInertial[1] * cosYaw  -  stateInertial[0] * sinYaw;
	stateBody[2] = stateInertial[2];

	// Fill in the (x,y,z) velocity estimates to be returned
	stateBody[3] = stateInertial[3] * cosYaw  +  stateInertial[4] * sinYaw;
	stateBody[4] = stateInertial[4] * cosYaw  -  stateInertial[3] * sinYaw;
	stateBody[5] = stateInertial[5];

	// Fill in the (roll,pitch,yaw) estimates to be returned
	stateBody[6] = stateInertial[6];
	stateBody[7] = stateInertial[7];
	stateBody[8] = stateInertial[8];
}





//    ----------------------------------------------------------------------------------
//    N   N  EEEEE  W     W        SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    NN  N  E      W     W       S      E        T    P   P  O   O   I   NN  N    T
//    N N N  EEE    W     W        SSS   EEE      T    PPPP   O   O   I   N N N    T
//    N  NN  E       W W W            S  E        T    P      O   O   I   N  NN    T
//    N   N  EEEEE    W W         SSSS   EEEEE    T    P       OOO   III  N   N    T
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K
//    C       A A   L      L      B   B   A A   C      K  K
//    C      A   A  L      L      BBBB   A   A  C      KKK
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//    ----------------------------------------------------------------------------------


// REQUEST SETPOINT CHANGE CALLBACK
// This function does NOT need to be edited 
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Check if the request if for the default setpoint
		if (newSetpoint.buttonID == REQUEST_DEFAULT_SETPOINT_BUTTON_ID)
		{
			setNewSetpoint(
					yaml_default_setpoint[0],
					yaml_default_setpoint[1],
					yaml_default_setpoint[2],
					0.0
				);
		}
		else
		{
			// Call the function for actually setting the setpoint
			setNewSetpoint(
					newSetpoint.x,
					newSetpoint.y,
					newSetpoint.z,
					newSetpoint.yaw
				);
		}
	}
}


// CHANGE SETPOINT FUNCTION
// This function does NOT need to be edited 
void setNewSetpoint(float x, float y, float z, float yaw)
{
	// Put the new setpoint into the class variable
	m_setpoint[0] = x;
	m_setpoint[1] = y;
	m_setpoint[2] = z;
	m_setpoint[3] = yaw;

	// Put the new setpoint in the state setpoint
	m_state_setpoint_vector(0) = x;
	m_state_setpoint_vector(1) = y;
	m_state_setpoint_vector(2) = z;

	// Update MPC optimization setpoint
	if (m_mpc_optimization_setup_success)
		change_mpc_optimization_setpoint();


	// Publish the change so that the network is updated
	// (mainly the "flying agent GUI" is interested in
	// displaying this change to the user)

	// Instantiate a local variable of type "SetpointWithHeader"
	SetpointWithHeader msg;
	// Fill in the setpoint
	msg.x   = x;
	msg.y   = y;
	msg.z   = z;
	msg.yaw = yaw;
	// Publish the message
	m_setpointChangedPublisher.publish(msg);
}


// GET CURRENT SETPOINT SERVICE CALLBACK
// This function does NOT need to be edited 
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response)
{
	// Directly put the current setpoint into the response
	response.setpointWithHeader.x   = m_setpoint[0];
	response.setpointWithHeader.y   = m_setpoint[1];
	response.setpointWithHeader.z   = m_setpoint[2];
	response.setpointWithHeader.yaw = m_setpoint[3];
	// Return
	return true;
}





//    ----------------------------------------------------------------------------------
//     CCCC  U   U   SSSS  TTTTT   OOO   M   M
//    C      U   U  S        T    O   O  MM MM
//    C      U   U   SSS     T    O   O  M M M
//    C      U   U      S    T    O   O  M   M
//     CCCC   UUU   SSSS     T     OOO   M   M
//
//     CCCC   OOO   M   M  M   M    A    N   N  DDDD
//    C      O   O  MM MM  MM MM   A A   NN  N  D   D
//    C      O   O  M M M  M M M  A   A  N N N  D   D
//    C      O   O  M   M  M   M  AAAAA  N  NN  D   D
//     CCCC   OOO   M   M  M   M  A   A  N   N  DDDD
//    ----------------------------------------------------------------------------------

// CUSTOM COMMAND RECEIVED CALLBACK
// This function CAN be edited to respond when the buttons
// in the GUI are pressed
void customCommandReceivedCallback(const CustomButtonWithHeader& commandReceived)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , commandReceived.shouldCheckForAgentID , commandReceived.agentIDs );

	if (isRevelant)
	{
		// Extract the data from the message
		int custom_button_index = commandReceived.button_index;
		float float_data        = commandReceived.float_data;
		//std::string string_data = commandReceived.string_data;

		// Switch between the button pressed
		switch(custom_button_index)
		{

			// > FOR CUSTOM BUTTON 1 - SETUP OPTIMIZATION
			case 1:
			{
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[TUTORIAL CONTROLLER] Button 1 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 1
				setup_mpc_optimization();

				break;
			}

			default:
			{
				// Let the user know that the command was not recognised
				ROS_INFO_STREAM("[TUTORIAL CONTROLLER] A button clicked command was received in the controller but not recognised, message.button_index = " << custom_button_index << ", and message.float_data = " << float_data );
				break;
			}
		}
	}
}





//    ----------------------------------------------------------------------------------
//    L       OOO     A    DDDD
//    L      O   O   A A   D   D
//    L      O   O  A   A  D   D
//    L      O   O  AAAAA  D   D
//    LLLLL   OOO   A   A  DDDD
//
//    PPPP     A    RRRR     A    M   M  EEEEE  TTTTT  EEEEE  RRRR    SSSS
//    P   P   A A   R   R   A A   MM MM  E        T    E      R   R  S
//    PPPP   A   A  RRRR   A   A  M M M  EEE      T    EEE    RRRR    SSS
//    P      AAAAA  R  R   AAAAA  M   M  E        T    E      R  R       S
//    P      A   A  R   R  A   A  M   M  EEEEE    T    EEEEE  R   R  SSSS
//    ----------------------------------------------------------------------------------


// TIMER CALLBACK FOR SENDING THE LOAD YAML REQUEST
// This function does NOT need to be edited
void timerCallback_initial_load_yaml(const ros::TimerEvent&)
{
	// Create a node handle to the selected parameter service
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	// Create the service client as a local variable
	ros::ServiceClient requestLoadYamlFilenameServiceClient = nodeHandle_to_own_agent_parameter_service.serviceClient<LoadYamlFromFilename>("requestLoadYamlFilename", false);
	// Create the service call as a local variable
	LoadYamlFromFilename loadYamlFromFilenameCall;
	// Specify the Yaml filename as a string
	loadYamlFromFilenameCall.request.stringWithHeader.data = "TutorialController";
	// Set for whom this applies to
	loadYamlFromFilenameCall.request.stringWithHeader.shouldCheckForAgentID = false;
	// Wait until the serivce exists
	requestLoadYamlFilenameServiceClient.waitForExistence(ros::Duration(-1));
	// Make the service call
	if(requestLoadYamlFilenameServiceClient.call(loadYamlFromFilenameCall))
	{
		// Nothing to do in this case.
		// The "isReadyTutorialControllerYamlCallback" function
		// will be called once the YAML file is loaded
	}
	else
	{
		// Inform the user
		ROS_ERROR("[TUTORIAL CONTROLLER] The request load yaml file service call failed.");
	}
}


// LOADING OF YAML PARAMETERS
// This function does NOT need to be edited 
void isReadyTutorialControllerYamlCallback(const IntWithHeader & msg)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Extract the data
		int parameter_service_to_load_from = msg.data;
		// Initialise a local variable for the namespace
		std::string namespace_to_use;
		// Load from the respective parameter service
		switch(parameter_service_to_load_from)
		{
			// > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
			case LOAD_YAML_FROM_AGENT:
			{
				ROS_INFO("[TUTORIAL CONTROLLER] Now fetching the TutorialController YAML parameter values from this agent.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				ROS_INFO("[TUTORIAL CONTROLLER] Now fetching the TutorialController YAML parameter values from this agent's coordinator.");
				namespace_to_use = m_namespace_to_coordinator_parameter_service;
				break;
			}

			default:
			{
				ROS_ERROR("[TUTORIAL CONTROLLER] Paramter service to load from was NOT recognised.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
		}
		// Create a node handle to the selected parameter service
		ros::NodeHandle nodeHandle_to_use(namespace_to_use);
		// Call the function that fetches the parameters
		fetchTutorialControllerYamlParameters(nodeHandle_to_use);
	}
}


// This function CAN BE edited for successful completion of the
// exercise, and the use of parameters fetched from the YAML file
// is highly recommended to make tuning of your controller easier
// and quicker.
void fetchTutorialControllerYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the file:
	// TutorialController.yaml

	// Add the "TutorialController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "TutorialController");



	// GET THE PARAMETERS:

	// The mass of the crazyflie
	yaml_cf_mass_in_grams = getParameterFloat(nodeHandle_for_paramaters , "mass");

	// The frequency at which the "computeControlOutput" is being called, as determined
	// by the frequency at which the Vicon system provides position and attitude data
	yaml_control_frequency = getParameterFloat(nodeHandle_for_paramaters, "control_frequency");

	// The default setpoint, the ordering is (x, y, z, yaw),
	// with unit [meters, meters, meters, radians]
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "default_setpoint", yaml_default_setpoint, 3);

	// The MPC prediction horizon, in discrete time steps
	yaml_prediction_horizon = getParameterInt(nodeHandle_for_paramaters , "prediction_horizon");

	// The MPC state cost matrix diagonal entries, the ordering is (x, y, z, xdot, ydot, zdot, roll, pitch, yaw)
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "state_cost_diagonals", yaml_state_cost_diagonals, const_num_states);

	// The MPC input cost matrix diagonal entries, the ordering is (totalThrust, rollRate, pitchRate, yawRate)
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "input_cost_diagonals", yaml_input_cost_diagonals, const_num_inputs);

	// The MPC state constraints, the ordering is (x, y, z, xdot, ydot, zdot, roll, pitch, yaw)
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "min_state_constraints", yaml_min_state_constraints, const_num_states);
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "max_state_constraints", yaml_max_state_constraints, const_num_states);

	// The MPC input constraints, the ordering is (totalThrust, rollRate, pitchRate, yawRate)
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "min_input_constraints", yaml_min_input_constraints, const_num_inputs);
	getParameterFloatVectorKnownLength(nodeHandle_for_paramaters, "max_input_constraints", yaml_max_input_constraints, const_num_inputs);

	// > DEBUGGING: Print out one of the parameters that was loaded to
	//   debug if the fetching of parameters worked correctly
	ROS_INFO_STREAM("[TUTORIAL CONTROLLER] DEBUGGING: the fetched TutorialController/mass = " << yaml_cf_mass_in_grams);



	// PROCESS THE PARAMTERS

	// > Compute the feed-forward force that we need to counteract
	//   gravity (i.e., mg) in units of [Newtons]
	m_cf_weight_in_newtons = yaml_cf_mass_in_grams * const_gravity / 1000.0;

	// DEBUGGING: Print out one of the computed quantities
	ROS_INFO_STREAM("[TUTORIAL CONTROLLER] DEBUGGING: thus the weight of this agent in [Newtons] = " << m_cf_weight_in_newtons);
}





//    ----------------------------------------------------------------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------


// This function does NOT need to be edited 
int main(int argc, char* argv[]) {
	// Initialize variables that cannot be initialized in header file
	m_state_setpoint_vector(2) = 0.4;


	// Starting the ROS-node
	ros::init(argc, argv, "TutorialControllerService");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "TutorialControllerService" node
	std::string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[TUTORIAL CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);



	// AGENT ID AND COORDINATOR ID

	// NOTES:
	// > If you look at the "Agent.launch" file in the "launch" folder,
	//   you will see the following line of code:
	//   <param name="agentID" value="$(optenv ROS_NAMESPACE)" />
	//   This line of code adds a parameter named "agentID" to the
	//   "FlyingAgentClient" node.
	// > Thus, to get access to this "agentID" paremeter, we first
	//   need to get a handle to the "FlyingAgentClient" node within which this
	//   controller service is nested.


	// Get the ID of the agent and its coordinator
	bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/FlyingAgentClient" , &m_agentID , &m_coordID);

	// Stall the node IDs are not valid
	if ( !isValid_IDs )
	{
		ROS_ERROR("[TUTORIAL CONTROLLER] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[TUTORIAL CONTROLLER] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
	}



	// PARAMETER SERVICE NAMESPACE AND NODEHANDLES:

	// NOTES:
	// > The parameters that are specified thorugh the *.yaml files
	//   are managed by a separate node called the "Parameter Service"
	// > A separate node is used for reasons of speed and generality
	// > To allow for a distirbuted architecture, there are two
	//   "ParamterService" nodes that are relevant:
	//   1) the one that is nested under the "m_agentID" namespace
	//   2) the one that is nested under the "m_coordID" namespace
	// > The following lines of code create the namespace (as strings)
	//   to there two relevant "ParameterService" nodes.
	// > The node handles are also created because they are needed
	//   for the ROS Subscriptions that follow.

	// Set the class variable "m_namespace_to_own_agent_parameter_service",
	// i.e., the namespace of parameter service for this agent
	m_namespace_to_own_agent_parameter_service = m_namespace + "/ParameterService";

	// Set the class variable "m_namespace_to_coordinator_parameter_service",
	// i.e., the namespace of parameter service for this agent's coordinator
	constructNamespaceForCoordinatorParameterService( m_coordID, m_namespace_to_coordinator_parameter_service );

	// Inform the user of what namespaces are being used
	ROS_INFO_STREAM("[TUTORIAL CONTROLLER] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	ROS_INFO_STREAM("[TUTORIAL CONTROLLER] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	ros::Subscriber safeContoller_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "TutorialController", 1, isReadyTutorialControllerYamlCallback);
	ros::Subscriber safeContoller_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("TutorialController", 1, isReadyTutorialControllerYamlCallback);



	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

	// The yaml files for the controllers are not added to
	// "Parameter Service" as part of launching.
	// The process for loading the yaml parameters is to send a
	// service call containing the filename of the *.yaml file,
	// and then a message will be received on the above subscribers
	// when the paramters are ready.
	// > NOTE IMPORTANTLY that by using a serice client
	//   we stall the availability of this node until the
	//   paramter service is ready
	// > NOTE FURTHER that calling on the service directly from here
	//   often means the yaml file is not actually loaded. So we
	//   instead use a timer to delay the loading

	// Create a single-shot timer
	ros::Timer timer_initial_load_yaml = nodeHandle.createTimer(ros::Duration(1.0), timerCallback_initial_load_yaml, true);
	timer_initial_load_yaml.start();





    // PUBLISHERS AND SUBSCRIBERS

    // Instantiate the class variable "m_debugPublisher" to be a
    // "ros::Publisher". This variable advertises under the name
    // "DebugTopic" and is a message with the structure defined
    //  in the file "DebugMsg.msg" (located in the "msg" folder).
    m_debugPublisher = nodeHandle.advertise<DebugMsg>("DebugTopic", 1);

	// Instantiate the local variable "requestSetpointChangeSubscriber"
	// to be a "ros::Subscriber" type variable that subscribes to the
	// "RequestSetpointChange" topic and calls the class function
	// "requestSetpointChangeCallback" each time a messaged is received
	// on this topic and the message is passed as an input argument to
	// the callback function. This subscriber will mainly receive
	// messages from the "flying agent GUI" when the setpoint is changed
	// by the user.
	ros::Subscriber requestSetpointChangeSubscriber = nodeHandle.subscribe("RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Same again but instead for changes requested by the coordinator.
	// For this we need to first create a node handle to the coordinator:
	std::string namespace_to_coordinator;
	constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);
	// And now we can instantiate the subscriber:
	ros::Subscriber requestSetpointChangeSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("TutorialControllerService/RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Instantiate the class variable "m_setpointChangedPublisher" to
	// be a "ros::Publisher". This variable advertises under the name
	// "SetpointChanged" and is a message with the structure defined
	// in the file "SetpointWithHeader.msg" (located in the "msg" folder).
	// This publisher is used by the "flying agent GUI" to update the
	// field that displays the current setpoint for this controller.
	m_setpointChangedPublisher = nodeHandle.advertise<SetpointWithHeader>("SetpointChanged", 1);

	// Instantiate the class variable "m_optimizationSetupStatusChangedPublisher" to
	// be a "ros::Publisher". This variable advertises under the name
	// "OptimizationSetupStatusChanged" and is a message with the structure defined
	// in the file "SetpointWithHeader.msg" (located in the "msg" folder).
	// This publisher is used by the "flying agent GUI" to update the
	// optimization setup status indicators.
	m_optimizationSetupStatusChangedPublisher = nodeHandle.advertise<SetpointWithHeader>("OptimizationSetupStatusChanged", 1);

	// Instantiate the local variable "getCurrentSetpointService" to be
	// a "ros::ServiceServer" type variable that advertises the service
	// called "GetCurrentSetpoint". This service has the input-output
	// behaviour defined in the "GetSetpointService.srv" file (located
	// in the "srv" folder). This service, when called, is provided with
	// an integer (that is essentially ignored), and is expected to respond
	// with the current setpoint of the controller. When a request is made
	// of this service the "getCurrentSetpointCallback" function is called.
	ros::ServiceServer getCurrentSetpointService = nodeHandle.advertiseService("GetCurrentSetpoint", getCurrentSetpointCallback);



    // Instantiate the local variable "service" to be a "ros::ServiceServer" type
    // variable that advertises the service called "TutorialController". This service has
    // the input-output behaviour defined in the "Controller.srv" file (located in the
    // "srv" folder). This service, when called, is provided with the most recent
    // measurement of the Crazyflie and is expected to respond with the control action
    // that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
    // this is where the "outer loop" controller function starts. When a request is made
    // of this service the "calculateControlOutput" function is called.
    ros::ServiceServer service = nodeHandle.advertiseService("TutorialController", calculateControlOutput);

    // Instantiate the local variable "customCommandSubscriber" to be a "ros::Subscriber"
    // type variable that subscribes to the "GUIButton" topic and calls the class
    // function "customCommandReceivedCallback" each time a messaged is received on this topic
    // and the message received is passed as an input argument to the callback function.
    ros::Subscriber customCommandReceivedSubscriber = nodeHandle.subscribe("CustomButtonPressed", 1, customCommandReceivedCallback);

    // Same again but instead for changes requested by the coordinator.
	// For this we need to first create a node handle to the coordinator:
	//std::string namespace_to_coordinator;
	//constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	//ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);
	// And now we can instantiate the subscriber:
	ros::Subscriber customCommandReceivedSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("TutorialControllerService/CustomButtonPressed", 1, customCommandReceivedCallback);

	



    // Print out some information to the user.
    ROS_INFO("[TUTORIAL CONTROLLER] Service ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}
