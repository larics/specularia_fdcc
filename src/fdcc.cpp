/******************************************************************************
File name: virtual_model.cpp
Description: Class that implements virtual model dynamics using RBDL.
Author: Bruno Maric
******************************************************************************/

#include "fdcc/fdcc.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

#define DEG2RAD 0.01745329252

FDCC::FDCC()
{

	ros::NodeHandle n;
	ros::NodeHandle private_node_handle_("~");
	// constructor
	//this->dummy = 1;

	this->InitJointStatesLoaded = false;

	// init Model
	this->model = new Model();

	// load URDF
	this->loadURDF("/home/bmaric/kuka_ws/src/FDCC/urdf/kr10r1100sixx.urdf");

	// init PD controllers
	this->PD_ctrl[0] = new PidControllerBase(0, 0, 0);
	this->PD_ctrl[1] = new PidControllerBase(0, 0, 0);
	this->PD_ctrl[2] = new PidControllerBase(0, 0, 0);
	this->PD_ctrl[3] = new PidControllerBase(0, 0, 0);
	this->PD_ctrl[4] = new PidControllerBase(0, 0, 0);
	this->PD_ctrl[5] = new PidControllerBase(0, 0, 0);

	// load PD params
	this->loadPDParams();

	// load Impedance control params
	this->loadImpedanceParams();

	// load Force/Torque sensor configuration
	this->loadFTSensorParams();

	// load Robot Constraints
	this->loadRobotConstraints();

	// init publishers
	//this->kuka_kr10_joints_pub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/position_trajectory_controller/follow_joint_trajectory/goal", 1);
	this->kuka_kr10_joints_pub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/joint_trajectory_action/goal", 1);
	this->kuka_kr10_joint_trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/position_trajectory_controller/command", 1);
	this->fdcc_state_pub = n.advertise<fdcc::fdcc_state>("/fdcc/state", 1);
	this->fdcc_force_pub = n.advertise<fdcc::fdcc_forces>("/fdcc/forces", 1);


	// init subscribers
	this->XDesiredSub = n.subscribe("/pose_desired", 1, &FDCC::XDesiredCallback, this);
	this->ForceSensorSub = n.subscribe("/optoforce_node/OptoForceWrench_filtered", 1, &FDCC::ForceSensorCallback, this);
	this->JointStateSub = n.subscribe("/joint_states", 1, &FDCC::JointStateCallback, this);
	this->DesiredForceSub = n.subscribe("/fdcc/desired_force", 1, &FDCC::DesiredForceCallback, this);


}

FDCC::~FDCC()
{
	// destructor

}

void FDCC::loadURDF(const char * filename)
{
	while (!Addons::URDFReadFromFile(filename, this->model, false))
	{
		std::cout << "Loading URDF..." << endl;
 	}
	
	// no gravity
	this->model->gravity = Vector3d (0., 0., 0.);

	// Init system states
	this->Q 		= VectorNd::Zero (this->model->dof_count);
	this->QDot 		= VectorNd::Zero (this->model->dof_count);
	this->QDDot 	= VectorNd::Zero (this->model->dof_count);
	this->Q0 		= VectorNd::Zero (this->model->dof_count);
	this->QDot0 	= VectorNd::Zero (this->model->dof_count);
	this->QDDot0 	= VectorNd::Zero (this->model->dof_count);

	this->X 		= SpatialVector::Zero(6);
	this->XDot 		= SpatialVector::Zero(6);
	this->XDDot 	= SpatialVector::Zero(6);

	this->X_desired = SpatialVector::Zero(6);
	this->E_desired = Matrix3d::Zero();

    this->Imp_c		= MatrixNd::Zero(this->model->dof_count, this->model->dof_count);
    this->Imp_k		= MatrixNd::Zero(this->model->dof_count, this->model->dof_count);
    this->Imp_I		= MatrixNd::Zero(this->model->dof_count, this->model->dof_count);

	this->Tau 		= VectorNd::Zero (this->model->dof_count);

	this->F_sensor	= SpatialVector::Zero(6);
	this->F_desired_tool	= SpatialVector::Zero(6);
	this->F_desired_base	= SpatialVector::Zero(6);

	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	
	this->Fsp 			= SpatialVector::Zero(6);
	this->Fd 			= SpatialVector::Zero(6);
	this->Fcollinear 	= SpatialVector::Zero(6);

	this->Msp 			= SpatialVector::Zero(6);
	this->Md 			= SpatialVector::Zero(6);
	this->Mcollinear 	= SpatialVector::Zero(6);


	this->Jacobian_matrix = MatrixNd::Zero(6, 6);

}

void FDCC::loadImpedanceParams(void)
{
	ros::NodeHandle private_node_handle_("~");
	std::string configFile;
	std::string path = ros::package::getPath("fdcc");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/ImpedanceControlConfig.yaml"));

    // Open file
    YAML::Node config = YAML::LoadFile(configFile);

    double value[6];

    std::vector<double> temp;

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
	{
		// take first key and save it to variable key
		std::string key = it->first.as<std::string>();
		//printf("%s\n", key);

		if (key.compare("ImpedanceControlConfiguration") == 0)
		{
			// imp_c
			temp = config[key]["imp_c"]["row0"].as<std::vector<double> >();
			
			this->Imp_c(0, 0) = temp.at(0);
			this->Imp_c(0, 1) = temp.at(1);
			this->Imp_c(0, 2) = temp.at(2);
			this->Imp_c(0, 3) = temp.at(3);
			this->Imp_c(0, 4) = temp.at(4);
			this->Imp_c(0, 5) = temp.at(5);

			temp = config[key]["imp_c"]["row1"].as<std::vector<double> >();
			
			this->Imp_c(1, 0) = temp.at(0);
			this->Imp_c(1, 1) = temp.at(1);
			this->Imp_c(1, 2) = temp.at(2);
			this->Imp_c(1, 3) = temp.at(3);
			this->Imp_c(1, 4) = temp.at(4);
			this->Imp_c(1, 5) = temp.at(5);

			temp = config[key]["imp_c"]["row2"].as<std::vector<double> >();
			
			this->Imp_c(2, 0) = temp.at(0);
			this->Imp_c(2, 1) = temp.at(1);
			this->Imp_c(2, 2) = temp.at(2);
			this->Imp_c(2, 3) = temp.at(3);
			this->Imp_c(2, 4) = temp.at(4);
			this->Imp_c(2, 5) = temp.at(5);

			temp = config[key]["imp_c"]["row3"].as<std::vector<double> >();
			
			this->Imp_c(3, 0) = temp.at(0);
			this->Imp_c(3, 1) = temp.at(1);
			this->Imp_c(3, 2) = temp.at(2);
			this->Imp_c(3, 3) = temp.at(3);
			this->Imp_c(3, 4) = temp.at(4);
			this->Imp_c(3, 5) = temp.at(5);

			temp = config[key]["imp_c"]["row4"].as<std::vector<double> >();
			
			this->Imp_c(4, 0) = temp.at(0);
			this->Imp_c(4, 1) = temp.at(1);
			this->Imp_c(4, 2) = temp.at(2);
			this->Imp_c(4, 3) = temp.at(3);
			this->Imp_c(4, 4) = temp.at(4);
			this->Imp_c(4, 5) = temp.at(5);

			temp = config[key]["imp_c"]["row5"].as<std::vector<double> >();
			
			this->Imp_c(5, 0) = temp.at(0);
			this->Imp_c(5, 1) = temp.at(1);
			this->Imp_c(5, 2) = temp.at(2);
			this->Imp_c(5, 3) = temp.at(3);
			this->Imp_c(5, 4) = temp.at(4);
			this->Imp_c(5, 5) = temp.at(5);

			// imp_k
			temp = config[key]["imp_k"]["row0"].as<std::vector<double> >();
			
			this->Imp_k(0, 0) = temp.at(0);
			this->Imp_k(0, 1) = temp.at(1);
			this->Imp_k(0, 2) = temp.at(2);
			this->Imp_k(0, 3) = temp.at(3);
			this->Imp_k(0, 4) = temp.at(4);
			this->Imp_k(0, 5) = temp.at(5);

			temp = config[key]["imp_k"]["row1"].as<std::vector<double> >();
			
			this->Imp_k(1, 0) = temp.at(0);
			this->Imp_k(1, 1) = temp.at(1);
			this->Imp_k(1, 2) = temp.at(2);
			this->Imp_k(1, 3) = temp.at(3);
			this->Imp_k(1, 4) = temp.at(4);
			this->Imp_k(1, 5) = temp.at(5);

			temp = config[key]["imp_k"]["row2"].as<std::vector<double> >();
			
			this->Imp_k(2, 0) = temp.at(0);
			this->Imp_k(2, 1) = temp.at(1);
			this->Imp_k(2, 2) = temp.at(2);
			this->Imp_k(2, 3) = temp.at(3);
			this->Imp_k(2, 4) = temp.at(4);
			this->Imp_k(2, 5) = temp.at(5);

			temp = config[key]["imp_k"]["row3"].as<std::vector<double> >();
			
			this->Imp_k(3, 0) = temp.at(0);
			this->Imp_k(3, 1) = temp.at(1);
			this->Imp_k(3, 2) = temp.at(2);
			this->Imp_k(3, 3) = temp.at(3);
			this->Imp_k(3, 4) = temp.at(4);
			this->Imp_k(3, 5) = temp.at(5);

			temp = config[key]["imp_k"]["row4"].as<std::vector<double> >();
			
			this->Imp_k(4, 0) = temp.at(0);
			this->Imp_k(4, 1) = temp.at(1);
			this->Imp_k(4, 2) = temp.at(2);
			this->Imp_k(4, 3) = temp.at(3);
			this->Imp_k(4, 4) = temp.at(4);
			this->Imp_k(4, 5) = temp.at(5);

			temp = config[key]["imp_k"]["row5"].as<std::vector<double> >();
			
			this->Imp_k(5, 0) = temp.at(0);
			this->Imp_k(5, 1) = temp.at(1);
			this->Imp_k(5, 2) = temp.at(2);
			this->Imp_k(5, 3) = temp.at(3);
			this->Imp_k(5, 4) = temp.at(4);
			this->Imp_k(5, 5) = temp.at(5);

			// Imp_I
			temp = config[key]["imp_I"]["row0"].as<std::vector<double> >();
			
			this->Imp_I(0, 0) = temp.at(0);
			this->Imp_I(0, 1) = temp.at(1);
			this->Imp_I(0, 2) = temp.at(2);
			this->Imp_I(0, 3) = temp.at(3);
			this->Imp_I(0, 4) = temp.at(4);
			this->Imp_I(0, 5) = temp.at(5);

			temp = config[key]["imp_I"]["row1"].as<std::vector<double> >();
			
			this->Imp_I(1, 0) = temp.at(0);
			this->Imp_I(1, 1) = temp.at(1);
			this->Imp_I(1, 2) = temp.at(2);
			this->Imp_I(1, 3) = temp.at(3);
			this->Imp_I(1, 4) = temp.at(4);
			this->Imp_I(1, 5) = temp.at(5);

			temp = config[key]["imp_I"]["row2"].as<std::vector<double> >();
			
			this->Imp_I(2, 0) = temp.at(0);
			this->Imp_I(2, 1) = temp.at(1);
			this->Imp_I(2, 2) = temp.at(2);
			this->Imp_I(2, 3) = temp.at(3);
			this->Imp_I(2, 4) = temp.at(4);
			this->Imp_I(2, 5) = temp.at(5);

			temp = config[key]["imp_I"]["row3"].as<std::vector<double> >();
			
			this->Imp_I(3, 0) = temp.at(0);
			this->Imp_I(3, 1) = temp.at(1);
			this->Imp_I(3, 2) = temp.at(2);
			this->Imp_I(3, 3) = temp.at(3);
			this->Imp_I(3, 4) = temp.at(4);
			this->Imp_I(3, 5) = temp.at(5);

			temp = config[key]["imp_I"]["row4"].as<std::vector<double> >();
			
			this->Imp_I(4, 0) = temp.at(0);
			this->Imp_I(4, 1) = temp.at(1);
			this->Imp_I(4, 2) = temp.at(2);
			this->Imp_I(4, 3) = temp.at(3);
			this->Imp_I(4, 4) = temp.at(4);
			this->Imp_I(4, 5) = temp.at(5);

			temp = config[key]["imp_I"]["row5"].as<std::vector<double> >();
			
			this->Imp_I(5, 0) = temp.at(0);
			this->Imp_I(5, 1) = temp.at(1);
			this->Imp_I(5, 2) = temp.at(2);
			this->Imp_I(5, 3) = temp.at(3);
			this->Imp_I(5, 4) = temp.at(4);
			this->Imp_I(5, 5) = temp.at(5);

			
			this->Imp_limit_t1_min = config[key]["limits"]["limit_t1_min"].as<double>();
			this->Imp_limit_t1_max = config[key]["limits"]["limit_t1_max"].as<double>();
			this->Imp_limit_t2_min = config[key]["limits"]["limit_t2_min"].as<double>();
			this->Imp_limit_t2_max = config[key]["limits"]["limit_t2_max"].as<double>();
			this->Imp_limit_t3_min = config[key]["limits"]["limit_t3_min"].as<double>();
			this->Imp_limit_t3_max = config[key]["limits"]["limit_t3_max"].as<double>();

			this->Imp_limit_f1_min = config[key]["limits"]["limit_f1_min"].as<double>();
			this->Imp_limit_f1_max = config[key]["limits"]["limit_f1_max"].as<double>();
			this->Imp_limit_f2_min = config[key]["limits"]["limit_f2_min"].as<double>();
			this->Imp_limit_f2_max = config[key]["limits"]["limit_f2_max"].as<double>();
			this->Imp_limit_f3_min = config[key]["limits"]["limit_f3_min"].as<double>();
			this->Imp_limit_f3_max = config[key]["limits"]["limit_f3_max"].as<double>();
			
		}		 
	}
}

void FDCC::loadPDParams(void)
{
	ros::NodeHandle private_node_handle_("~");
	std::string configFile;
	std::string path = ros::package::getPath("fdcc");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/ForcePDConfig.yaml"));

    // Open file
    YAML::Node config = YAML::LoadFile(configFile);

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
	{
		// take first key and save it to variable key
		std::string key = it->first.as<std::string>();
		//printf("%s\n", key);

		if (key.compare("ForcePDConfig") == 0)
		{
			// delta t
			this->delta_t		= config[key]["dt"].as<double>();

			// PD Kp
			this->ForcePD_Kp[0] = config[key]["Kp"]["spatial0"].as<double>();
			this->ForcePD_Kp[1] = config[key]["Kp"]["spatial1"].as<double>();
			this->ForcePD_Kp[2] = config[key]["Kp"]["spatial2"].as<double>();
			this->ForcePD_Kp[3] = config[key]["Kp"]["spatial3"].as<double>();
			this->ForcePD_Kp[4] = config[key]["Kp"]["spatial4"].as<double>();
			this->ForcePD_Kp[5] = config[key]["Kp"]["spatial5"].as<double>();

			// PD Kd
			this->ForcePD_Kd[0] = config[key]["Kd"]["spatial0"].as<double>();
			this->ForcePD_Kd[1] = config[key]["Kd"]["spatial1"].as<double>();
			this->ForcePD_Kd[2] = config[key]["Kd"]["spatial2"].as<double>();
			this->ForcePD_Kd[3] = config[key]["Kd"]["spatial3"].as<double>();
			this->ForcePD_Kd[4] = config[key]["Kd"]["spatial4"].as<double>();
			this->ForcePD_Kd[5] = config[key]["Kd"]["spatial5"].as<double>();

			this->PD_limit_t1_min = config[key]["limits"]["limit_t1_min"].as<double>();
			this->PD_limit_t1_max = config[key]["limits"]["limit_t1_max"].as<double>();
			this->PD_limit_t2_min = config[key]["limits"]["limit_t2_min"].as<double>();
			this->PD_limit_t2_max = config[key]["limits"]["limit_t2_max"].as<double>();
			this->PD_limit_t3_min = config[key]["limits"]["limit_t3_min"].as<double>();
			this->PD_limit_t3_max = config[key]["limits"]["limit_t3_max"].as<double>();

			this->PD_limit_f1_min = config[key]["limits"]["limit_f1_min"].as<double>();
			this->PD_limit_f1_max = config[key]["limits"]["limit_f1_max"].as<double>();
			this->PD_limit_f2_min = config[key]["limits"]["limit_f2_min"].as<double>();
			this->PD_limit_f2_max = config[key]["limits"]["limit_f2_max"].as<double>();
			this->PD_limit_f3_min = config[key]["limits"]["limit_f3_min"].as<double>();
			this->PD_limit_f3_max = config[key]["limits"]["limit_f3_max"].as<double>();
		}		 
	}

	// Set PD params
	double t_min = this->delta_t * 0.1;
	double t_max = this->delta_t * 1.5;

	// PD[0]
	this->PD_ctrl[0]->setTdMin(t_min);
	this->PD_ctrl[0]->setTdMax(t_max);
	this->PD_ctrl[0]->setKp(this->ForcePD_Kp[0]);
	this->PD_ctrl[0]->setKd(this->ForcePD_Kd[0]);
	// PD[1]
	this->PD_ctrl[1]->setTdMin(t_min);
	this->PD_ctrl[1]->setTdMax(t_max);
	this->PD_ctrl[1]->setKp(this->ForcePD_Kp[1]);
	this->PD_ctrl[1]->setKd(this->ForcePD_Kd[1]);
	// PD[2]
	this->PD_ctrl[2]->setTdMin(t_min);
	this->PD_ctrl[2]->setTdMax(t_max);
	this->PD_ctrl[2]->setKp(this->ForcePD_Kp[2]);
	this->PD_ctrl[2]->setKd(this->ForcePD_Kd[2]);
	// PD[3]
	this->PD_ctrl[3]->setTdMin(t_min);
	this->PD_ctrl[3]->setTdMax(t_max);
	this->PD_ctrl[3]->setKp(this->ForcePD_Kp[3]);
	this->PD_ctrl[3]->setKd(this->ForcePD_Kd[3]);
	// PD[4]
	this->PD_ctrl[4]->setTdMin(t_min);
	this->PD_ctrl[4]->setTdMax(t_max);
	this->PD_ctrl[4]->setKp(this->ForcePD_Kp[4]);
	this->PD_ctrl[4]->setKd(this->ForcePD_Kd[4]);
	// PD[5]
	this->PD_ctrl[5]->setTdMin(t_min);
	this->PD_ctrl[5]->setTdMax(t_max);
	this->PD_ctrl[5]->setKp(this->ForcePD_Kp[5]);
	this->PD_ctrl[5]->setKd(this->ForcePD_Kd[5]);

	// Set limits
	this->PD_ctrl[0]->setUMin(this->PD_limit_t1_min);
	this->PD_ctrl[0]->setUMax(this->PD_limit_t1_max);	
	this->PD_ctrl[1]->setUMin(this->PD_limit_t2_min);
	this->PD_ctrl[1]->setUMax(this->PD_limit_t2_max);
	this->PD_ctrl[2]->setUMin(this->PD_limit_t3_min);
	this->PD_ctrl[2]->setUMax(this->PD_limit_t3_max);

	this->PD_ctrl[3]->setUMin(this->PD_limit_f1_min);
	this->PD_ctrl[3]->setUMax(this->PD_limit_f1_max);
	this->PD_ctrl[4]->setUMin(this->PD_limit_f2_min);
	this->PD_ctrl[4]->setUMax(this->PD_limit_f2_max);
	this->PD_ctrl[5]->setUMin(this->PD_limit_f3_min);
	this->PD_ctrl[5]->setUMax(this->PD_limit_f3_max);
}

void FDCC::loadFTSensorParams(void)
{

	ros::NodeHandle private_node_handle_("~");
	std::string configFile;
	std::string path = ros::package::getPath("fdcc");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/FTSensorConfig.yaml"));

    // Open file
    YAML::Node config = YAML::LoadFile(configFile);

    double value[6];

    std::vector<double> temp;

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
	{
		// take first key and save it to variable key
		std::string key = it->first.as<std::string>();
		//printf("%s\n", key);

		if (key.compare("FTSensorConfiguration") == 0)
		{
			// Forces
			temp = config[key]["Zero_readings"]["Forces"].as<std::vector<double> >();
			
			this->FTSensor_offset_force[0] = temp.at(0);
			this->FTSensor_offset_force[1] = temp.at(1);
			this->FTSensor_offset_force[2] = temp.at(2);

			// Torques
			temp = config[key]["Zero_readings"]["Torques"].as<std::vector<double> >();

			this->FTSensor_offset_torque[0] = temp.at(0);
			this->FTSensor_offset_torque[1] = temp.at(1);
			this->FTSensor_offset_torque[2] = temp.at(2);

			// init sensor reading (before first sensor msg arrives)
			this->F_sensorReading.wrench.torque.z = this->FTSensor_offset_torque[2];
			this->F_sensorReading.wrench.torque.y = this->FTSensor_offset_torque[1];
			this->F_sensorReading.wrench.torque.x = this->FTSensor_offset_torque[0];
			this->F_sensorReading.wrench.force.z  = this->FTSensor_offset_force[2];
			this->F_sensorReading.wrench.force.y  = this->FTSensor_offset_force[1];
			this->F_sensorReading.wrench.force.x  = this->FTSensor_offset_force[0];

		}		 
	}
}

void FDCC::loadRobotConstraints	(void)
{
ros::NodeHandle private_node_handle_("~");
	std::string configFile;
	std::string path = ros::package::getPath("fdcc");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/KR10Constraints.yaml"));

    // Open file
    YAML::Node config = YAML::LoadFile(configFile);

    double value[6];

    std::vector<double> temp;

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
	{
		// take first key and save it to variable key
		std::string key = it->first.as<std::string>();
		//printf("%s\n", key);

		if (key.compare("KR10_constraints") == 0)
		{
			// Joint position limits	
			// joint_a1
			this->joint_limit_position_min[0] = config[key]["joint_position"]["joint_a1_min"].as<double>();
			this->joint_limit_position_max[0] = config[key]["joint_position"]["joint_a1_max"].as<double>();
			// joint_a2
			this->joint_limit_position_min[1] = config[key]["joint_position"]["joint_a2_min"].as<double>();
			this->joint_limit_position_max[1] = config[key]["joint_position"]["joint_a2_max"].as<double>();
			// joint_a3
			this->joint_limit_position_min[2] = config[key]["joint_position"]["joint_a3_min"].as<double>();
			this->joint_limit_position_max[2] = config[key]["joint_position"]["joint_a3_max"].as<double>();
			// joint_a4
			this->joint_limit_position_min[3] = config[key]["joint_position"]["joint_a4_min"].as<double>();
			this->joint_limit_position_max[3] = config[key]["joint_position"]["joint_a4_max"].as<double>();
			// joint_a5
			this->joint_limit_position_min[4] = config[key]["joint_position"]["joint_a5_min"].as<double>();
			this->joint_limit_position_max[4] = config[key]["joint_position"]["joint_a5_max"].as<double>();
			// joint_a6
			this->joint_limit_position_min[5] = config[key]["joint_position"]["joint_a6_min"].as<double>();
			this->joint_limit_position_max[5] = config[key]["joint_position"]["joint_a6_max"].as<double>();
			
			// Joint velocity limits
			// joint_a1
			this->joint_limit_velocity[0] = config[key]["joint_velocity"]["joint_a1"].as<double>();
			// joint_a2
			this->joint_limit_velocity[1] = config[key]["joint_velocity"]["joint_a2"].as<double>();
			// joint_a3
			this->joint_limit_velocity[2] = config[key]["joint_velocity"]["joint_a3"].as<double>();
			// joint_a4
			this->joint_limit_velocity[3] = config[key]["joint_velocity"]["joint_a4"].as<double>();
			// joint_a5
			this->joint_limit_velocity[4] = config[key]["joint_velocity"]["joint_a5"].as<double>();
			// joint_a6
			this->joint_limit_velocity[5] = config[key]["joint_velocity"]["joint_a6"].as<double>();

			// Joint acceleration limits
			// joint_a1
			this->joint_limit_acceleration[0] = config[key]["joint_acceleration"]["joint_a1"].as<double>();
			// joint_a2
			this->joint_limit_acceleration[1] = config[key]["joint_acceleration"]["joint_a2"].as<double>();
			// joint_a3
			this->joint_limit_acceleration[2] = config[key]["joint_acceleration"]["joint_a3"].as<double>();
			// joint_a4
			this->joint_limit_acceleration[3] = config[key]["joint_acceleration"]["joint_a4"].as<double>();
			// joint_a5
			this->joint_limit_acceleration[4] = config[key]["joint_acceleration"]["joint_a5"].as<double>();
			// joint_a6
			this->joint_limit_acceleration[5] = config[key]["joint_acceleration"]["joint_a6"].as<double>();
		}		 
	}

}

void FDCC::CalcForwardDynamics(SpatialVector Fc)
{
	// 1. Init external forces

	this->Fext[6] = Fc;

	// 2. Calculate Forward Dynamics
	ForwardDynamics (*this->model, this->Q, this->QDot, this->Tau, this->QDDot, &Fext);

	// 3. Integrate QDDot -> QDot -> Q
	// integrate
	for (int i = 0; i < 6; i++)
	{		
		// check joint acceleration limit;
		if (this->QDDot(i) > this->joint_limit_acceleration[i])
		{
			this->QDDot(i) = this->joint_limit_acceleration[i];
		}
		else if (this->QDDot(i) < -1*this->joint_limit_acceleration[i])
		{
			this->QDDot(i) = -1*this->joint_limit_acceleration[i];
		}

		// integrate acceleration
		this->QDot(i) = this->QDot(i) + this->QDDot(i)*this->delta_t;

		// check joint velocity limit;
		if (this->QDot(i) > this->joint_limit_velocity[i]*DEG2RAD)
		{
			this->QDot(i) 	= this->joint_limit_velocity[i]*DEG2RAD;
			this->QDDot(i) 	= 0;
		}
		else if (this->QDot(i) < -1*this->joint_limit_velocity[i]*DEG2RAD)
		{
			this->QDot(i) = -1*this->joint_limit_velocity[i]*DEG2RAD;
			this->QDDot(i) 	= 0;
		}

		// integrate velocity
		this->Q(i) = this->Q(i) + this->QDot(i)*this->delta_t;

		// check joint velocity limit;
		if (this->Q(i) > this->joint_limit_position_max[i]*DEG2RAD)
		{
			this->Q(i) = this->joint_limit_position_max[i]*DEG2RAD;
			this->QDDot(i) 	= 0;
			this->QDot(i)	= 0;
		}
		else if (this->Q(i) < this->joint_limit_position_min[i]*DEG2RAD)
		{
			this->Q(i) = this->joint_limit_position_min[i]*DEG2RAD;
			this->QDDot(i) 	= 0;
			this->QDot(i)	= 0;
		}

	}

}

void FDCC::CalcForwardKinematics(VectorNd Q, VectorNd QDot, VectorNd QDDot)
{
	VectorNd base_position = Vector3d::Zero ();
	Vector3d temp = Vector3d::Zero();


	UpdateKinematics(*this->model, this->Q, this->QDot, this->QDDot);

	//std::cout << "X_base: " << this->model->X_base[6] << endl;

	// Update X
	//temp = CalcBodyToBaseCoordinates(*this->model, &this->Q, 6, base_position );
	this->X = SpatialVector::Zero(6);
	Vector3d tempX = Vector3d::Zero();

	// position
	tempX = CalcBodyToBaseCoordinates(*this->model, this->Q, 6, Vector3d(0., 0., 0.0));
	this->X(3) = tempX(0);
	this->X(4) = tempX(1);
	this->X(5) = tempX(2);

	// orientation
	this->E = Matrix3d::Zero();

	this->E = CalcBodyWorldOrientation(*this->model, this->Q, 6);

	this->X(0) = this->E(0, 0);
	this->X(1) = this->E(0, 1);
	this->X(2) = this->E(0, 2);


	this->XDot = CalcPointVelocity6D(*this->model, this->Q, this->QDot, 6, base_position, false);
	this->XDDot = CalcPointAcceleration6D(*this->model, this->Q, this->QDot, this->QDDot, 6, base_position, false);

	//std::cout << "X: " << this->X.transpose() << endl;
	//std::cout << "XDot: " << this->XDot.transpose() << endl;
	//std::cout << "XDDot: " << this->XDDot.transpose() << endl;
}

SpatialVector FDCC::ConvertImpedanceForceToSpatial(SpatialVector Fbase, SpatialVector PointPosition)
{
	// Spatial transform addon translation
	/*
	Fbase(0) +=  -( Fbase(4)*PointPosition(5) - Fbase(5)*PointPosition(4));
	Fbase(1) +=  -(-Fbase(3)*PointPosition(5) + Fbase(5)*PointPosition(3));
	Fbase(2) +=  -( Fbase(3)*PointPosition(4) - Fbase(4)*PointPosition(3));
	*/
	return Fbase; 
}

SpatialVector FDCC::ConvertSensorForceToSpatial(SpatialVector Fsensor, SpatialVector PointPosition)
{
	SpatialVector Fret = SpatialVector::Zero();

	// Convert torques and forces given in tool coordinate system to base coordinate system
	MatrixNd T = MatrixNd::Zero(6, 6);
	Matrix3d Rx = MatrixNd::Zero(3, 3);
	Matrix3d temp = MatrixNd::Zero(3, 3);

	// fill E matrix
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			T(i, j) = this->E(i, j);
			T(i+3, j+3) = this->E(i, j);
		}

	// Calculate -ERx
	Rx(0, 0) = 	0;
	Rx(0, 1) = -this->X(5);
	Rx(0, 2) = 	this->X(4);
	Rx(1, 0) = 	this->X(5);
	Rx(1, 1) = 	0;
	Rx(1, 2) = -this->X(3);
	Rx(2, 0) = -this->X(4);
	Rx(2, 1) = 	this->X(3);
	Rx(2, 2) = 	0;

	temp = -this->E*Rx;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			T(i+3, j) = Rx(i, j);		
		}

	// Spatial transform 
	Fret = T.inverse()*Fsensor;

	//std::cout << "Fbase: " << Fbase.transpose() << endl;

	Fret(0) +=  -( Fret(4)*PointPosition(5) - Fret(5)*PointPosition(4));
	Fret(1) +=  -(-Fret(3)*PointPosition(5) + Fret(5)*PointPosition(3));
	Fret(2) +=  -( Fret(3)*PointPosition(4) - Fret(4)*PointPosition(3));

	return Fret; 
}

SpatialVector	FDCC::ConvertSpatialBaseToTool		(SpatialVector Fbase, SpatialVector PointPosition)
{
	SpatialVector Fret = SpatialVector::Zero();

	// Convert torques and forces given in tool coordinate system to base coordinate system
	MatrixNd T = MatrixNd::Zero(6, 6);
	Matrix3d Rx = MatrixNd::Zero(3, 3);
	Matrix3d temp = MatrixNd::Zero(3, 3);

	// fill E matrix
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			T(i, j) = this->E(i, j);
			T(i+3, j+3) = this->E(i, j);
		}

	// Calculate -ERx
	Rx(0, 0) = 	0;
	Rx(0, 1) = -this->X(5);
	Rx(0, 2) = 	this->X(4);
	Rx(1, 0) = 	this->X(5);
	Rx(1, 1) = 	0;
	Rx(1, 2) = -this->X(3);
	Rx(2, 0) = -this->X(4);
	Rx(2, 1) = 	this->X(3);
	Rx(2, 2) = 	0;

	temp = -this->E*Rx;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			T(i+3, j) = Rx(i, j);		
		}

	// Spatial transform 
	Fret = T*Fbase;

	//std::cout << "Fbase: " << Fbase.transpose() << endl;
/*
	Fret(0) +=  -( Fret(4)*PointPosition(5) - Fret(5)*PointPosition(4));
	Fret(1) +=  -(-Fret(3)*PointPosition(5) + Fret(5)*PointPosition(3));
	Fret(2) +=  -( Fret(3)*PointPosition(4) - Fret(4)*PointPosition(3));
*/
	return Fret;
}

SpatialVector	FDCC::ConvertSpatialToolToBase		(SpatialVector Ftool, SpatialVector PointPosition)
{
	SpatialVector Fret = SpatialVector::Zero();

	// Convert torques and forces given in tool coordinate system to base coordinate system
	MatrixNd T = MatrixNd::Zero(6, 6);
	Matrix3d Rx = MatrixNd::Zero(3, 3);
	Matrix3d temp = MatrixNd::Zero(3, 3);

	// fill E matrix
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			T(i, j) = this->E(i, j);
			T(i+3, j+3) = this->E(i, j);
		}

	// Calculate -ERx
	Rx(0, 0) = 	0;
	Rx(0, 1) = -this->X(5);
	Rx(0, 2) = 	this->X(4);
	Rx(1, 0) = 	this->X(5);
	Rx(1, 1) = 	0;
	Rx(1, 2) = -this->X(3);
	Rx(2, 0) = -this->X(4);
	Rx(2, 1) = 	this->X(3);
	Rx(2, 2) = 	0;

	temp = -this->E*Rx;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			T(i+3, j) = Rx(i, j);		
		}

	// Spatial transform 
	Fret = T.inverse()*Ftool;

	//std::cout << "Fbase: " << Fbase.transpose() << endl;

	Fret(0) +=  -( Fret(4)*PointPosition(5) - Fret(5)*PointPosition(4));
	Fret(1) +=  -(-Fret(3)*PointPosition(5) + Fret(5)*PointPosition(3));
	Fret(2) +=  -( Fret(3)*PointPosition(4) - Fret(4)*PointPosition(3));

	return Fret;
}


void FDCC::XDesiredCallback(const geometry_msgs::Pose &msg)
{


	// TODO: Convert quaternion to approach vector 
	//this->X_desired(0) = msg.orientation.x ; //2*msg.orientation.x*msg.orientation.z + 2*msg.orientation.y*msg.orientation.w;
	//this->X_desired(1) = msg.orientation.y ; //2*msg.orientation.y*msg.orientation.z - 2*msg.orientation.x*msg.orientation.w;
	//this->X_desired(2) = msg.orientation.z ; //1 - 2*msg.orientation.x*msg.orientation.x - 2*msg.orientation.y*msg.orientation.y;

	// Convert quaternion to transformation matrix
	this->E_desired(0, 0) = 1 - 2*msg.orientation.y*msg.orientation.y - 2*msg.orientation.z*msg.orientation.z;
	this->E_desired(1, 0) = 2*msg.orientation.x*msg.orientation.y + 2*msg.orientation.z*msg.orientation.w;
	this->E_desired(2, 0) = 2*msg.orientation.x*msg.orientation.z - 2*msg.orientation.y*msg.orientation.w;

	this->E_desired(0, 1) = 2*msg.orientation.x*msg.orientation.y - 2*msg.orientation.z*msg.orientation.w;
	this->E_desired(1, 1) = 1 - 2*msg.orientation.x*msg.orientation.x - 2*msg.orientation.z*msg.orientation.z;
	this->E_desired(2, 1) = 2*msg.orientation.y*msg.orientation.z + 2*msg.orientation.x*msg.orientation.w;


	this->E_desired(0, 2) = 2*msg.orientation.x*msg.orientation.z + 2*msg.orientation.y*msg.orientation.w;
	this->E_desired(1, 2) = 2*msg.orientation.y*msg.orientation.z - 2*msg.orientation.x*msg.orientation.w;
	this->E_desired(2, 2) = 1 - 2*msg.orientation.x*msg.orientation.x - 2*msg.orientation.y*msg.orientation.y;

	// TODO preurediti ovo pametnije!!!
	// approach vector - x vector
	this->X_desired(0) = this->E_desired(0, 0); //2*msg.orientation.x*msg.orientation.z + 2*msg.orientation.y*msg.orientation.w;
	this->X_desired(1) = this->E_desired(1, 0); //2*msg.orientation.y*msg.orientation.z - 2*msg.orientation.x*msg.orientation.w;
	this->X_desired(2) = this->E_desired(2, 0); //1 - 2*msg.orientation.x*msg.orientation.x - 2*msg.orientation.y*msg.orientation.y;

	this->X_desired(3) = msg.position.x;
	this->X_desired(4) = msg.position.y;
	this->X_desired(5) = msg.position.z;
}

void FDCC::ForceSensorCallback (const geometry_msgs::WrenchStamped &msg)
{
	//std::cout << "msg: " << msg << endl;
	this->F_sensorReading = msg;
}

void FDCC::JointStateCallback (const sensor_msgs::JointState &msg)
{
	if (this->InitJointStatesLoaded == false)
	{
		// Set init Q
		this->Q(0) = msg.position[0];
		this->Q(1) = msg.position[1];
		this->Q(2) = msg.position[2];
		this->Q(3) = msg.position[3];
		this->Q(4) = msg.position[4];
		this->Q(5) = msg.position[5];

		// Set init X
		this->CalcForwardKinematics(this->Q, this->QDot, this->QDDot);

		// Set init X_desired = X
		this->X_desired = this->X;
		this->E_desired = this->E;

		ROS_INFO("Init Joint States Loaded!");
		// set flag to true
		this->InitJointStatesLoaded = true;

	}
	
}

void FDCC::DesiredForceCallback 	(const fdcc::FDCCForceCommandMsg &msg)
{

	this->F_desired_tool(0) = msg.torque_x;
	this->F_desired_tool(1) = msg.torque_y;
	this->F_desired_tool(2) = msg.torque_z;

	this->F_desired_tool(3) = msg.force_x;
	this->F_desired_tool(4) = msg.force_y;
	this->F_desired_tool(5) = msg.force_z;



	return;
}

SpatialVector FDCC::ImpedanceControl (SpatialVector X_desired, SpatialVector F_desired)
{
	SpatialVector F = SpatialVector::Zero(this->model->dof_count);
	SpatialVector DeltaX = SpatialVector::Zero(this->model->dof_count);

	// START TOOL FRAME TRANSFORM



	// END TOOL FRAME TRANSFORM

	// Rotate to fit approach x-vector
	DeltaX = X_desired - this->X;
	// Vector product - rotation vector
	DeltaX(0) = ( this->X(1)*X_desired(2) - this->X(2)*X_desired(1));
	DeltaX(1) = (-this->X(0)*X_desired(2) + this->X(2)*X_desired(0));
	DeltaX(2) = ( this->X(0)*X_desired(1) - this->X(1)*X_desired(0));
	
	// Scalar product - scale rotation vector
	DeltaX(0) = DeltaX(0)*(X_desired(0)*this->X(0) + X_desired(1)*this->X(1) + X_desired(2)*this->X(2));
	DeltaX(1) = DeltaX(1)*(X_desired(0)*this->X(0) + X_desired(1)*this->X(1) + X_desired(2)*this->X(2));
	DeltaX(2) = DeltaX(2)*(X_desired(0)*this->X(0) + X_desired(1)*this->X(1) + X_desired(2)*this->X(2));

	// Rotate to fit z vector
	Matrix3d Erot = this->E_desired.transpose()*this->E.inverse();
	//std::cout << "Delta X " << DeltaX(0) << ", "<< DeltaX(1) << ", "<< DeltaX(2) <<endl;
	//std::cout << "Approach vector " << X_desired(0) << ", "<< X_desired(1) << ", "<< X_desired(2) <<endl;
	//std::cout << "Rot angle " << atan2(Erot(2, 1), Erot(1, 1))/3.14 << endl;
	DeltaX(0) += X_desired(0)*(-atan2(Erot(2, 1), Erot(1, 1))/3.14);
	DeltaX(1) += X_desired(1)*(-atan2(Erot(2, 1), Erot(1, 1))/3.14);
	DeltaX(2) += X_desired(2)*(-atan2(Erot(2, 1), Erot(1, 1))/3.14);

	F = this->Imp_c*DeltaX;

	this->Msp(0) 	= F(0);
	this->Msp(1) 	= F(1);
	this->Msp(2) 	= F(2);
	this->Fsp(0) 	= F(3);
	this->Fsp(1) 	= F(4);
	this->Fsp(2) 	= F(5);


	F = F - this->Imp_k*this->XDot - this->Imp_I*this->XDDot;

	//std::cout << "F: " << F.transpose() << endl;
	F_desired = this->ConvertSpatialToolToBase(F_desired, this->X);
	

	
	// substract collinear part
	this->Md(0)		= F_desired(0);
	this->Md(1)		= F_desired(1);
	this->Md(2)		= F_desired(2);	
	this->Fd(0)		= F_desired(3);
	this->Fd(1)		= F_desired(4);
	this->Fd(2)		= F_desired(5);	

	
	
	if (this->Fd.norm() != 0)
		this->Fcollinear = ((this->Fsp(0)*this->Fd(0) + this->Fsp(1)*this->Fd(1) + this->Fsp(2)*this->Fd(2))/this->Fd.norm())*(this->Fd/this->Fd.norm());
	else
	{
		this->Fcollinear(0) = 0;
		this->Fcollinear(1) = 0;
		this->Fcollinear(2) = 0;
	}

	if (this->Md.norm() != 0)
		this->Mcollinear = ((this->Msp(0)*this->Md(0) + this->Msp(1)*this->Md(1) + this->Msp(2)*this->Md(2))/this->Md.norm())*(this->Md/this->Md.norm());
	else
	{
		this->Mcollinear(0) = 0;
		this->Mcollinear(1) = 0;
		this->Mcollinear(2) = 0;
	}

	if (this->Fd.norm() != 0)
		std::cout << "Fcollinear: " << ((this->Fsp(0)*this->Fd(0) + this->Fsp(1)*this->Fd(1) + this->Fsp(2)*this->Fd(2))/this->Fd.norm())*(this->Fd/this->Fd.norm()).transpose()  << endl;
	if (this->Md.norm() != 0)
		std::cout << "Mcollinear: " << ((this->Msp(0)*this->Md(0) + this->Msp(1)*this->Md(1) + this->Msp(2)*this->Md(2))/this->Md.norm())*(this->Md/this->Md.norm()).transpose() << endl;

	F(0) = F(0) - this->Mcollinear(0);
	F(1) = F(1) - this->Mcollinear(1);
	F(2) = F(2) - this->Mcollinear(2);
	F(3) = F(3) - this->Fcollinear(0);
	F(4) = F(4) - this->Fcollinear(1);
	F(5) = F(5) - this->Fcollinear(2);
	
	
	// Check limits
	if (F(0) > this->Imp_limit_t1_max)	
		F(0) = this->Imp_limit_t1_max;
	else if (F(0) < this->Imp_limit_t1_min)
		F(0) = this->Imp_limit_t1_min;

	if (F(1) > this->Imp_limit_t2_max)	
		F(1) = this->Imp_limit_t2_max;
	else if (F(1) < this->Imp_limit_t2_min)
		F(1) = this->Imp_limit_t2_min;
	
	if (F(2) > this->Imp_limit_t3_max)	
		F(2) = this->Imp_limit_t3_max;
	else if (F(2) < this->Imp_limit_t3_min)
		F(2) = this->Imp_limit_t3_min;

	if (F(3) > this->Imp_limit_f1_max)	
		F(3) = this->Imp_limit_f1_max;
	else if (F(3) < this->Imp_limit_f1_min)
		F(3) = this->Imp_limit_f1_min;

	if (F(4) > this->Imp_limit_f2_max)	
		F(4) = this->Imp_limit_f2_max;
	else if (F(4) < this->Imp_limit_f2_min)
		F(4) = this->Imp_limit_f2_min;

	if (F(5) > this->Imp_limit_f3_max)	
		F(5) = this->Imp_limit_f3_max;
	else if (F(5) < this->Imp_limit_f3_min)
		F(5) = this->Imp_limit_f3_min;


	return F;
}


void FDCC::ControlLoop(void)
{

	std::cout << "rate: " <<1.0/this->delta_t << endl;

	ros::Rate r(1.0/(this->delta_t));
	

	// init temp variables
	SpatialVector F_imp_base, F_imp_tool, F_net, F_ctrl_tool, F_ctrl_base, F_desired_tool_temp;
	geometry_msgs::WrenchStamped F_sensorReading_temp;

	F_imp_base = SpatialVector::Zero(6);
	F_imp_tool = SpatialVector::Zero(6);
	F_net = SpatialVector::Zero(6);
	F_ctrl_base = SpatialVector::Zero(6);
	F_ctrl_tool = SpatialVector::Zero(6);
	F_desired_tool_temp = SpatialVector::Zero(6);

	control_msgs::FollowJointTrajectoryActionGoal joint_msg;

	// init move
	//->createRobotTrajectoryMsg();

	ros::Time::init();

	while (ros::ok())
	{
		//after calling this function ROS will processes our callbacks
        ros::spinOnce();

        if (this->InitJointStatesLoaded)
        {
	        	// START control loop
        	// prepare variables
        	F_sensorReading_temp = this->F_sensorReading;
  
			// 1. Forward kinematics
			this->CalcForwardKinematics(this->Q, this->QDot, this->QDDot);

			// 2. Impedance Control - tool frame
			F_imp_base = this->ImpedanceControl(this->X_desired, this->F_desired_tool);
			// convert to tool frame
			F_imp_tool = this->ConvertSpatialBaseToTool(F_imp_base, this->X);
			//F_imp_tool = this->ImpedanceControl(this->X_desired, F_desired_tool_temp);
			// 3. Sensor Reading
			// Prepare Sensor Data - tool frame
			this->F_sensor(0) =  (F_sensorReading_temp.wrench.torque.z - this->FTSensor_offset_torque[2])/1.0;
			this->F_sensor(1) =  (F_sensorReading_temp.wrench.torque.y - this->FTSensor_offset_torque[1])/1.0;
			this->F_sensor(2) = -(F_sensorReading_temp.wrench.torque.x - this->FTSensor_offset_torque[0])/1.0;		
			this->F_sensor(3) =  (F_sensorReading_temp.wrench.force.z - this->FTSensor_offset_force[2]);
			this->F_sensor(4) =  (F_sensorReading_temp.wrench.force.y - this->FTSensor_offset_force[1]);
			this->F_sensor(5) = -(F_sensorReading_temp.wrench.force.x - this->FTSensor_offset_force[0]);

			
			//this->F_sensor = this->ConvertSensorForceToSpatial(this->F_sensor, this->X);
			//this->F_sensor = this->ConvertSpatialBaseToTool(this->F_sensor, this->X);

			
			// 4. Calculate Fnet - tool frame
			F_net =  F_imp_tool + this->F_desired_tool + this->F_sensor;
			

			// Calculate F_ctrl
			F_ctrl_tool(0) = (double) (1.0*this->PD_ctrl[0]->compute(F_net(0)));
			F_ctrl_tool(1) = (double) (1.0*this->PD_ctrl[1]->compute(F_net(1)));
			F_ctrl_tool(2) = (double) (1.0*this->PD_ctrl[2]->compute(F_net(2)));
			F_ctrl_tool(3) = (double) (1.0*this->PD_ctrl[3]->compute(F_net(3)));
			F_ctrl_tool(4) = (double) (1.0*this->PD_ctrl[4]->compute(F_net(4)));
			F_ctrl_tool(5) = (double) (1.0*this->PD_ctrl[5]->compute(F_net(5)));

			// 5. Forward Dynamics including integration - base frame
			
			F_ctrl_base = this->ConvertSpatialToolToBase(F_ctrl_tool, this->X);
			this->CalcForwardDynamics(F_ctrl_base);

			//std::cout << "Q: " << this->Q.transpose() << endl;

			// 6. Move robot
			this->createRobotTrajectoryMsg();
			//this->CreateJointStatesMsg();
			this->CreateFDCCStateMsg();

			fdcc::fdcc_forces force_msg;

			force_msg.header.stamp = ros::Time::now();
			force_msg.impedance_force.push_back(F_imp_tool(0));
			force_msg.impedance_force.push_back(F_imp_tool(1));
			force_msg.impedance_force.push_back(F_imp_tool(2));
			force_msg.impedance_force.push_back(F_imp_tool(3));
			force_msg.impedance_force.push_back(F_imp_tool(4));
			force_msg.impedance_force.push_back(F_imp_tool(5));

			force_msg.sensor_force.push_back(this->F_sensor(0));
			force_msg.sensor_force.push_back(this->F_sensor(1));
			force_msg.sensor_force.push_back(this->F_sensor(2));
			force_msg.sensor_force.push_back(this->F_sensor(3));
			force_msg.sensor_force.push_back(this->F_sensor(4));
			force_msg.sensor_force.push_back(this->F_sensor(5));

			force_msg.desired_force.push_back(this->F_desired_tool(0));
			force_msg.desired_force.push_back(this->F_desired_tool(1));
			force_msg.desired_force.push_back(this->F_desired_tool(2));
			force_msg.desired_force.push_back(this->F_desired_tool(3));
			force_msg.desired_force.push_back(this->F_desired_tool(4));
			force_msg.desired_force.push_back(this->F_desired_tool(5));

			force_msg.net_force.push_back(F_net(0));
			force_msg.net_force.push_back(F_net(1));
			force_msg.net_force.push_back(F_net(2));
			force_msg.net_force.push_back(F_net(3));
			force_msg.net_force.push_back(F_net(4));
			force_msg.net_force.push_back(F_net(5));

			force_msg.control_force.push_back(F_ctrl_tool(0));
			force_msg.control_force.push_back(F_ctrl_tool(1));
			force_msg.control_force.push_back(F_ctrl_tool(2));
			force_msg.control_force.push_back(F_ctrl_tool(3));
			force_msg.control_force.push_back(F_ctrl_tool(4));
			force_msg.control_force.push_back(F_ctrl_tool(5));

			this->fdcc_force_pub.publish(force_msg);
			// END of control loop

        }
        else
        {
        	// Joint states still not loaded
        	ROS_INFO("Robot Joint States still not loaded!");
        }
        
		r.sleep();
		ros::spinOnce(); 
	}

	//std::cout << "ROS ok exceeded." << endl;
}

void FDCC::SetDeltaT (double delta_t)
{
	this->delta_t = delta_t;
}

double FDCC::GetDeltaT (void)
{
	return this->delta_t;
}


void FDCC::createRobotTrajectoryMsg( void )
{

	control_msgs::FollowJointTrajectoryActionGoal return_value_gazebo;
	

	return_value_gazebo.goal.trajectory.joint_names.push_back("joint_a1");
	return_value_gazebo.goal.trajectory.joint_names.push_back("joint_a2");
	return_value_gazebo.goal.trajectory.joint_names.push_back("joint_a3");
	return_value_gazebo.goal.trajectory.joint_names.push_back("joint_a4");
	return_value_gazebo.goal.trajectory.joint_names.push_back("joint_a5");
	return_value_gazebo.goal.trajectory.joint_names.push_back("joint_a6");


	trajectory_msgs::JointTrajectoryPoint point_current_gazebo;

	point_current_gazebo.positions.push_back(this->Q(0));
	point_current_gazebo.positions.push_back(this->Q(1));
	point_current_gazebo.positions.push_back(this->Q(2));
	point_current_gazebo.positions.push_back(this->Q(3));
	point_current_gazebo.positions.push_back(this->Q(4));
	point_current_gazebo.positions.push_back(this->Q(5));

	point_current_gazebo.velocities.push_back(0);
	point_current_gazebo.velocities.push_back(0);
	point_current_gazebo.velocities.push_back(0);
	point_current_gazebo.velocities.push_back(0);
	point_current_gazebo.velocities.push_back(0);
	point_current_gazebo.velocities.push_back(0);

	point_current_gazebo.accelerations.push_back(0);
	point_current_gazebo.accelerations.push_back(0);
	point_current_gazebo.accelerations.push_back(0);
	point_current_gazebo.accelerations.push_back(0);
	point_current_gazebo.accelerations.push_back(0);
	point_current_gazebo.accelerations.push_back(0);

	point_current_gazebo.time_from_start = ros::Duration(0.00000001); // zasto nije nula?

	return_value_gazebo.goal.trajectory.points.push_back(point_current_gazebo);

	return_value_gazebo.goal.goal_time_tolerance = ros::Duration(0.02);

	this->kuka_kr10_joints_pub.publish(return_value_gazebo);
	//return return_value;	


	trajectory_msgs::JointTrajectory return_value;
	

	return_value.joint_names.push_back("joint_a1");
	return_value.joint_names.push_back("joint_a2");
	return_value.joint_names.push_back("joint_a3");
	return_value.joint_names.push_back("joint_a4");
	return_value.joint_names.push_back("joint_a5");
	return_value.joint_names.push_back("joint_a6");

	trajectory_msgs::JointTrajectoryPoint point_current;

	point_current.positions.push_back(this->Q(0));
	point_current.positions.push_back(this->Q(1));
	point_current.positions.push_back(this->Q(2));
	point_current.positions.push_back(this->Q(3));
	point_current.positions.push_back(this->Q(4));
	point_current.positions.push_back(this->Q(5));

	point_current.time_from_start = ros::Duration(0.02); 

	return_value.points.push_back(point_current);

	this->kuka_kr10_joint_trajectory_pub.publish(return_value);

};

void FDCC::CreateFDCCStateMsg(void)
{

	fdcc::fdcc_state msg2pub;

	std_msgs::Header h2;
	ros::Time Tprep = ros::Time::now();
	h2.stamp.sec =  Tprep.sec  ; 
	h2.stamp.nsec =  Tprep.nsec;
	
	msg2pub.header.stamp = h2.stamp;

	msg2pub.joint_position.push_back(this->Q(0));
	msg2pub.joint_position.push_back(this->Q(1));
	msg2pub.joint_position.push_back(this->Q(2));
	msg2pub.joint_position.push_back(this->Q(3));
	msg2pub.joint_position.push_back(this->Q(4));
	msg2pub.joint_position.push_back(this->Q(5));

	msg2pub.joint_velocity.push_back(this->QDot(0));
	msg2pub.joint_velocity.push_back(this->QDot(1));
	msg2pub.joint_velocity.push_back(this->QDot(2));
	msg2pub.joint_velocity.push_back(this->QDot(3));
	msg2pub.joint_velocity.push_back(this->QDot(4));
	msg2pub.joint_velocity.push_back(this->QDot(5));

	msg2pub.joint_acceleration.push_back(this->QDDot(0));
	msg2pub.joint_acceleration.push_back(this->QDDot(1));
	msg2pub.joint_acceleration.push_back(this->QDDot(2));
	msg2pub.joint_acceleration.push_back(this->QDDot(3));
	msg2pub.joint_acceleration.push_back(this->QDDot(4));
	msg2pub.joint_acceleration.push_back(this->QDDot(5));

	// make quaternion and publish it in cartesian_position
	double qw, qx, qy, qz;

	qw = sqrt(1 + this->E.transpose()(0, 0) + this->E.transpose()(1, 1) + this->E.transpose()(2, 2))/2;
	qx = (this->E.transpose()(2, 1) - this->E.transpose()(1, 2)) / (4*qw);
	qy = (this->E.transpose()(0, 2) - this->E.transpose()(2, 0)) / (4*qw);
	qz = (this->E.transpose()(1, 0) - this->E.transpose()(0, 1)) / (4*qw);

	msg2pub.cartesian_position.push_back(this->X_desired(3));
	msg2pub.cartesian_position.push_back(this->X_desired(4));
	msg2pub.cartesian_position.push_back(this->X_desired(5));
	msg2pub.cartesian_position.push_back(qx);
	msg2pub.cartesian_position.push_back(qy);
	msg2pub.cartesian_position.push_back(qz);
	msg2pub.cartesian_position.push_back(qw);
	


	this->fdcc_state_pub.publish(msg2pub);

}

