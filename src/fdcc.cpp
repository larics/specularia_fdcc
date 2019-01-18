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

	// init publishers
	this->kuka_kr10_joints_pub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/position_trajectory_controller/follow_joint_trajectory/goal", 1);
	//this->kuka_kr10_joints_pub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/joint_trajectory_action/goal", 1);
	
	this->kuka_kr10_joint_state_emulator_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
	this->fdcc_state_pub = n.advertise<fdcc::fdcc_state>("/fdcc_state", 1);


	// init subscribers
	this->XDesiredSub = n.subscribe("/pose_desired", 1, &FDCC::XDesiredCallback, this);
	this->ForceSensorSub = n.subscribe("/optoforce_node/OptoForceWrench", 1, &FDCC::ForceSensorCallback, this);

	// bind constraint

	//this->CS.AddContactConstraint(6, Vector3d (0., 0., 0.05), Vector3d (1., 0., 0.));
	//this->CS.AddContactConstraint(6, Vector3d (0., 0., 0.05), Vector3d (0., 1., 0.));
	//this->CS.AddContactConstraint(6, Vector3d (0., 0., 0.05), Vector3d (0., 0., 1.));

	//this->CS.Bind(*this->model);

	// run()

	// joint limits !! modify this to load from YAML
	this->joint_limit_min[0] = DEG2RAD * (-170.0);
	this->joint_limit_max[0] = DEG2RAD * ( 170);
	this->joint_limit_min[1] = DEG2RAD * (-190.0);
	this->joint_limit_max[1] = DEG2RAD * ( 45);
	this->joint_limit_min[2] = DEG2RAD * (-120.0);
	this->joint_limit_max[2] = DEG2RAD * ( 156);
	this->joint_limit_min[3] = DEG2RAD * (-185.0);
	this->joint_limit_max[3] = DEG2RAD * ( 185);
	this->joint_limit_min[4] = DEG2RAD * (-120.0);
	this->joint_limit_max[4] = DEG2RAD * ( 120);
	this->joint_limit_min[5] = DEG2RAD * (-350.0);
	this->joint_limit_max[5] = DEG2RAD * ( 350);

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
	this->Q(1) = -1.57;
	this->Q(2) = 1.57;
	//this->Q(4) = 1.57;
	//this->Q(4) = 0.5;
	
	//this->Q(1) = -2;
	//this->Q(2) = 2;

	this->X 		= SpatialVector (1.0, 0., 0.0, 0.62, 0., 0.995);
	this->XDot 		= SpatialVector (0., 0., 0., 0., 0., 0.);
	this->XDDot 	= SpatialVector (0., 0., 0., 0., 0., 0.);
	this->X0 		= SpatialVector (1.0, 0., 0.0, 0.62, 0., 0.995);
	this->XDot0 	= SpatialVector (0., 0., 0., 0., 0., 0.);
	this->XDDot0 	= SpatialVector (0., 0., 0., 0., 0., 0.);

	this->X_desired	= SpatialVector (1.0, 0., 0.0, 0.62, 0., 0.995);
	this->E_desired = Matrix3d::Zero();
	this->E_desired(0, 0) = 1;
	this->E_desired(1, 1) = 1;
	this->E_desired(2, 2) = 1;
	


    this->Imp_c		= MatrixNd::Zero(this->model->dof_count, this->model->dof_count);
    this->Imp_k		= MatrixNd::Zero(this->model->dof_count, this->model->dof_count);
    this->Imp_I		= MatrixNd::Zero(this->model->dof_count, this->model->dof_count);

	this->Tau 		= VectorNd::Zero (this->model->dof_count);

	this->F_sensor	= SpatialVector	( 0.097, -0.718, 0.471, -46.2, 1.6, 3.1);
	this->F_desired	= SpatialVector	(0., 0., 0., 0., 0., 0.);

	this->F_sensorReading.wrench.torque.z = 0.097;
	this->F_sensorReading.wrench.torque.y = 0.471;
	this->F_sensorReading.wrench.torque.x = -0.718;
	this->F_sensorReading.wrench.force.z = -46.2;
	this->F_sensorReading.wrench.force.y = 3.1;
	this->F_sensorReading.wrench.force.x = 1.6;

	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	this->Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	


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
}

void FDCC::CalcForwardDynamics(SpatialVector Fc)
{
	// Init external forces
	
	// #1
	//CalcBodySpatialJacobian(*this->model, this->Q, 6, this->Jacobian_matrix);

	//MatrixNd G = MatrixNd::Zero(6, 6);
	//CalcPointJacobian(*this->model, this->Q, 6, Vector3d(0., 0., 0.05), G);

	//std::cout << "G: " << G << endl;
	//std::cout << "G inverse: " << G.inverse() << endl;
	//this->Tau = G*Fc;
	//std::cout << this->Jacobian_matrix.completeOrthogonalDecomposition().pseudoInverse()*Fc << endl;
	
	//std::cout << "Tau: " << this->Tau.transpose() << endl;

	//std::cout << "F_ctrl: " << Fc.transpose() << endl;
	this->Fext[6] = Fc;


	//std::cout << "Tau: " << this->Tau.transpose() << endl;

	//Fext.push_back(Fc);

	//this->model->gravity = Vector3d (0., 0., 0.);

	// Calculate FrowardDynamics
	ForwardDynamics (*this->model, this->Q, this->QDot, this->Tau, this->QDDot, &Fext);
	//ForwardDynamics (*this->model, this->Q, this->QDot, this->Tau, this->QDDot);
	

	// #2
	//this->Fext[5] = Fc;
	//ForwardDynamicsConstraintsDirect(*this->model, this->Q, this->QDot, this->Tau, this->CS, this->QDDot, &Fext);
	//ForwardDynamicsConstraintsNullSpace(*this->model, this->Q, this->QDot, this->Tau, this->CS, this->QDDot, &Fext);

	double temp_QDot, temp_Q;
	// integrate
	for (int i = 0; i < 6; i++)
	{
		
		temp_QDot 	= this->QDot(i) + this->QDDot(i)*this->delta_t;
		temp_Q 		= this->Q(i) 	+ this->QDot(i)*this->delta_t;
		
		// check joint limit
		if (temp_Q > this->joint_limit_min[i] && temp_Q < this->joint_limit_max[i])
		{
			this->QDot(i) = temp_QDot;
			this->Q(i) = temp_Q;
		}
		else
		{
			this->QDDot(i) = 0;
			this->QDot(i) = 0;
		}
		
		//this->QDot(i) 	= this->QDot(i) + this->QDDot(i)*this->delta_t;
		//this->Q(i) 		= this->Q(i) 	+ this->QDot(i)*this->delta_t;
	}

	this->CheckJointLimits();
	
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
	//std::cout << "E6: " << this->E << endl;

	//this->X(0) = tempE(0, 0);
	//this->X(1) = tempE(1, 0);
	//this->X(2) = tempE(2, 0);

	this->X(0) = this->E(0, 0);
	this->X(1) = this->E(0, 1);
	this->X(2) = this->E(0, 2);


	this->XDot = CalcPointVelocity6D(*this->model, this->Q, this->QDot, 6, base_position, false);
	this->XDDot = CalcPointAcceleration6D(*this->model, this->Q, this->QDot, this->QDDot, 6, base_position, false);

	std::cout << "X: " << this->X.transpose() << endl;
	//std::cout << "XDot: " << this->XDot.transpose() << endl;
	//std::cout << "XDDot: " << this->XDDot.transpose() << endl;
}

SpatialVector FDCC::ConvertImpedanceForceToSpatial(SpatialVector Fbase, VectorNd PointPosition)
{

	// Convert torques and forces given in base coordinate system to SpatialForce in given PointPostion

	// Spatial transform addon translation
	Fbase(0) +=  -( Fbase(4)*PointPosition(5) - Fbase(5)*PointPosition(4));
	Fbase(1) +=  -(-Fbase(3)*PointPosition(5) + Fbase(5)*PointPosition(3));
	Fbase(2) +=  -( Fbase(3)*PointPosition(4) - Fbase(4)*PointPosition(3));

	return Fbase; 
}

SpatialVector FDCC::ConvertSensorForceToSpatial(SpatialVector Fsensor, VectorNd PointPosition)
{
	SpatialVector Fbase = SpatialVector::Zero();

	// Convert torques and forces given in tool coordinate system to base coordinate system
	MatrixNd T = MatrixNd::Zero(6, 6);
	Matrix3d Rx = Matrix3d::Zero();
	Matrix3d temp = Matrix3d::Zero();

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
	//std::cout << T << endl;

	Fbase = T.inverse()*Fsensor;

	//std::cout << "Fbase: " << Fbase.transpose() << endl;

	Fbase(0) +=  -( Fbase(4)*PointPosition(5) - Fbase(5)*PointPosition(4));
	Fbase(1) +=  -(-Fbase(3)*PointPosition(5) + Fbase(5)*PointPosition(3));
	Fbase(2) +=  -( Fbase(3)*PointPosition(4) - Fbase(4)*PointPosition(3));

	return Fbase; 
}

void FDCC::SetInitCartesianState (SpatialVector X0, SpatialVector XDot0, SpatialVector XDDot0)
{
	this->X0 		= X0;
	this->XDot0 	= XDot0;
	this->XDDot0	= XDDot0;
	return;
}

void FDCC::SetInitJointPoseState (VectorNd Q0, VectorNd QDot0, VectorNd QDDot0)
{
	this->Q0 		= Q0;
	this->QDot0 	= QDot0;
	this->QDDot0 	= QDDot0;
	return;
}

void FDCC::SetDesiredToolPosition(SpatialVector X_desired)
{
	this->X_desired = X_desired;	
}

void FDCC::SetDesiredToolForce(SpatialVector F_desired)
{
	this->F_desired = F_desired;	
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

SpatialVector FDCC::ImpedanceControl (SpatialVector X_desired)
{
	SpatialVector F = SpatialVector::Zero(this->model->dof_count);
	SpatialVector DeltaX = SpatialVector::Zero(this->model->dof_count);


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

	F = this->Imp_c*DeltaX - this->Imp_k*this->XDot - this->Imp_I*this->XDDot;

	

	//std::cout << " XDesired: " << X_desired.transpose() << endl;

	// Spatial transform addon translation
	//F(0) +=  -( F(4)*this->X(5) - F(5)*this->X(4));
	//F(1) +=  -(-F(3)*this->X(5) + F(5)*this->X(3));
	//F(2) +=  -( F(3)*this->X(4) - F(4)*this->X(3));

	// Convert Force in base coordinate system to Spatial Force


	return this->ConvertImpedanceForceToSpatial(F, this->X);
}

void FDCC::ControlLoop(void)
{

	std::cout << "rate: " <<1.0/this->delta_t << endl;

	ros::Rate r(1.0/this->delta_t);
	

	// init temp variables
	SpatialVector F_imp, F_net, F_ctrl;

	control_msgs::FollowJointTrajectoryActionGoal joint_msg;

	// init move
	//->createRobotTrajectoryMsg();

	ros::Time::init();

	while (ros::ok())
	{
		//after calling this function ROS will processes our callbacks
        ros::spinOnce();

		// Forward kinematics
		this->CalcForwardKinematics(this->Q, this->QDot, this->QDDot);

		// Impedance Control
		F_imp = this->ImpedanceControl(this->X_desired);


		// Prepare Sensor Data
		this->F_sensor(0) = (this->F_sensorReading.wrench.torque.z-0.097)*2.0;
		this->F_sensor(1) = (this->F_sensorReading.wrench.torque.y-0.471)*2.0;
		this->F_sensor(2) = -(this->F_sensorReading.wrench.torque.x+0.718)*2.0;		
		this->F_sensor(3) = (this->F_sensorReading.wrench.force.z+46.2)/10.0;
		this->F_sensor(4) = (this->F_sensorReading.wrench.force.y-3.1)/10.0;
		this->F_sensor(5) = -(this->F_sensorReading.wrench.force.x-1.6)/10.0;
		//std::cout << "F_sensor: " << this->F_sensor.transpose() << endl;
		this->F_sensor = this->ConvertSensorForceToSpatial(this->F_sensor, this->X);

		//std::cout << "F_sensor spatial: " << this->F_sensor.transpose() << endl;

		// Calculate Fnet
		F_net = F_imp + this->F_desired + this->F_sensor;
		//F_net = SpatialVector::Zero(6);
		//std::cout << "F_net: " << F_net.transpose() << endl;

		// Calculate F_ctrl
		F_ctrl(0) = (double) (1.0*this->PD_ctrl[0]->compute(F_net(0)));
		F_ctrl(1) = (double) (1.0*this->PD_ctrl[1]->compute(F_net(1)));
		F_ctrl(2) = (double) (1.0*this->PD_ctrl[2]->compute(F_net(2)));
		F_ctrl(3) = (double) (1.0*this->PD_ctrl[3]->compute(F_net(3)));
		F_ctrl(4) = (double) (1.0*this->PD_ctrl[4]->compute(F_net(4)));
		F_ctrl(5) = (double) (1.0*this->PD_ctrl[5]->compute(F_net(5)));

		//std::cout << "F_ctrl: " << F_ctrl.transpose() << endl;

		//F_ctrl = SpatialVector(0.0, -0.02*this->X(5), 0.0, -0.02, 0.0, 0.0);
		//F_ctrl = SpatialVector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		

		// Forward Dynamics including integration
		
		this->CalcForwardDynamics(F_ctrl);

		std::cout << "Q: " << this->Q.transpose() << endl;
		//std::cout << "QDot: " << this->QDot.transpose() << endl;		
		//std::cout << "QDDot: " << this->QDDot.transpose() << endl;

		// Move robot
		//this->F_sensor = SpatialVector(0., 0., 0., 0., 0., 0.);

		this->createRobotTrajectoryMsg();
		//this->CreateJointStatesMsg();
		this->CreateFDCCStateMsg();

		//std::cout << "End of control loop."<< endl;
		r.sleep();
		ros::spinOnce(); 
	}

	//std::cout << "ROS ok exceeded." << endl;
}

bool FDCC::CheckJointLimits()
{
	int qdot_limit = 100;
	int qddot_limit = 100;

	if (this->Q(0) < this->joint_limit_min[0] || this->Q(0) > this->joint_limit_max[0])
		return false;	
	else if (this->Q(1) < this->joint_limit_min[1] || this->Q(1) > this->joint_limit_max[1])
		return false;
	else if (this->Q(2) < this->joint_limit_min[2] || this->Q(2) > this->joint_limit_max[2])
		return false;
	else if (this->Q(3) < this->joint_limit_min[3] || this->Q(3) > this->joint_limit_max[3])
		return false;
	else if (this->Q(4) < this->joint_limit_min[4] || this->Q(4) > this->joint_limit_max[4])
		return false;
	else if (this->Q(5) < this->joint_limit_min[5] || this->Q(5) > this->joint_limit_max[5])
		return false;
	
	// check QDot limits
	if (this->QDot(0) < -qdot_limit || this->QDot(0) > qdot_limit)
		return false;
	else if (this->QDot(1) < -qdot_limit || this->QDot(1) > qdot_limit)
		return false;
	else if (this->QDot(2) < -qdot_limit || this->QDot(2) > qdot_limit)
		return false;
	else if (this->QDot(3) < -qdot_limit || this->QDot(3) > qdot_limit)
		return false;
	else if (this->QDot(4) < -qdot_limit || this->QDot(4) > qdot_limit)
		return false;
	else if (this->QDot(5) < -qdot_limit || this->QDot(5) > qdot_limit)
		return false;

	// check QDot limits
	if (this->QDDot(0) < -qddot_limit || this->QDDot(0) > qddot_limit)
		return false;
	else if (this->QDDot(1) < -qddot_limit || this->QDDot(1) > qddot_limit)
		return false;
	else if (this->QDDot(2) < -qddot_limit || this->QDDot(2) > qddot_limit)
		return false;
	else if (this->QDDot(3) < -qddot_limit || this->QDDot(3) > qddot_limit)
		return false;
	else if (this->QDDot(4) < -qddot_limit || this->QDDot(4) > qddot_limit)
		return false;
	else if (this->QDDot(5) < -qddot_limit || this->QDDot(5) > qddot_limit)
		return false;


	return true;

}

void FDCC::SetDeltaT (double delta_t)
{
	this->delta_t = delta_t;
}

double FDCC::GetDeltaT (void)
{
	return this->delta_t;
}

void FDCC::CreateJointStatesMsg(void)
{
	sensor_msgs::JointState return_value;

	std_msgs::Header h;
	h.stamp = ros::Time::now();

	return_value.header.stamp = h.stamp;

	return_value.name.push_back("joint_a1");
	return_value.name.push_back("joint_a2");
	return_value.name.push_back("joint_a3");
	return_value.name.push_back("joint_a4");
	return_value.name.push_back("joint_a5");
	return_value.name.push_back("joint_a6");

	return_value.position.push_back(this->Q(0));
	return_value.position.push_back(this->Q(1));
	return_value.position.push_back(this->Q(2));
	return_value.position.push_back(this->Q(3));
	return_value.position.push_back(this->Q(4));
	return_value.position.push_back(this->Q(5));

	return_value.velocity.push_back(this->QDot(0));
	return_value.velocity.push_back(this->QDot(1));
	return_value.velocity.push_back(this->QDot(2));
	return_value.velocity.push_back(this->QDot(3));
	return_value.velocity.push_back(this->QDot(4));
	return_value.velocity.push_back(this->QDot(5));
		
	this->kuka_kr10_joint_state_emulator_pub.publish(return_value);
}

void FDCC::createRobotTrajectoryMsg( void )
{

	control_msgs::FollowJointTrajectoryActionGoal return_value;
	

	return_value.goal.trajectory.joint_names.push_back("joint_a1");
	return_value.goal.trajectory.joint_names.push_back("joint_a2");
	return_value.goal.trajectory.joint_names.push_back("joint_a3");
	return_value.goal.trajectory.joint_names.push_back("joint_a4");
	return_value.goal.trajectory.joint_names.push_back("joint_a5");
	return_value.goal.trajectory.joint_names.push_back("joint_a6");


	trajectory_msgs::JointTrajectoryPoint point_current;

	point_current.positions.push_back(this->Q(0));
	point_current.positions.push_back(this->Q(1));
	point_current.positions.push_back(this->Q(2));
	point_current.positions.push_back(this->Q(3));
	point_current.positions.push_back(this->Q(4));
	point_current.positions.push_back(this->Q(5));
/*
	point_current.velocities.push_back(this->QDot(0));
	point_current.velocities.push_back(this->QDot(1));
	point_current.velocities.push_back(this->QDot(2));
	point_current.velocities.push_back(this->QDot(3));
	point_current.velocities.push_back(this->QDot(4));
	point_current.velocities.push_back(this->QDot(5));

	point_current.accelerations.push_back(this->QDDot(0));
	point_current.accelerations.push_back(this->QDDot(1));
	point_current.accelerations.push_back(this->QDDot(2));
	point_current.accelerations.push_back(this->QDDot(3));
	point_current.accelerations.push_back(this->QDDot(4));
	point_current.accelerations.push_back(this->QDDot(5));
*/
	point_current.velocities.push_back(0);
	point_current.velocities.push_back(0);
	point_current.velocities.push_back(0);
	point_current.velocities.push_back(0);
	point_current.velocities.push_back(0);
	point_current.velocities.push_back(0);

	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);

	point_current.time_from_start = ros::Duration(0.015);

	return_value.goal.trajectory.points.push_back(point_current);

	return_value.goal.goal_time_tolerance = ros::Duration(0.01);

	this->kuka_kr10_joints_pub.publish(return_value);
	//return return_value;	
	
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

	msg2pub.cartesian_position.push_back(this->X(0));
	msg2pub.cartesian_position.push_back(this->X(1));
	msg2pub.cartesian_position.push_back(this->X(2));
	msg2pub.cartesian_position.push_back(this->X(3));
	msg2pub.cartesian_position.push_back(this->X(4));
	msg2pub.cartesian_position.push_back(this->X(5));

	this->fdcc_state_pub.publish(msg2pub);

}

void FDCC::testing(void)
{
	/*
	VectorNd Q = VectorNd::Zero (this->model->dof_count);
	VectorNd QDot = VectorNd::Zero (this->model->dof_count);
	VectorNd Tau = VectorNd::Zero (this->model->dof_count);
	VectorNd QDDot = VectorNd::Zero (this->model->dof_count);

	std::vector<Math::SpatialVector> Fext;

	Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	Fext.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));

	//std::cout << "Fext: " << Fext << endl;
	
	std::cout << "Q: " << Q.transpose() << endl;
	std::cout << "QDot: " << QDot.transpose() << endl;
	std::cout << "Tau: " << Tau.transpose() << endl;
	std::cout << "QDDot: " << QDDot.transpose() << endl;
	
	//ForwardDynamics (*this->model, Q, QDot, Tau, QDDot, &Fext);
	//std::cout << QDDot.transpose() << std::endl;
	*/

	

}
