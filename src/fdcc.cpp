/******************************************************************************
File name: virtual_model.cpp
Description: Class that implements virtual model dynamics using RBDL.
Author: Bruno Maric
******************************************************************************/

#include "fdcc/fdcc.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

FDCC::FDCC()
{

	ros::NodeHandle n;
	ros::NodeHandle private_node_handle_("~");
	// constructor
	//this->dummy = 1;

	// init Model
	this->model = new Model();

	// load URDF
	this->loadURDF("/home/bmaric/catkin_ws/src/fdcc/urdf/kr10r1100sixx.urdf");

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

	// init publisher
	this->kuka_kr10_joints_pub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/joint_trajectory_action/goal", 10);

	// bind constraint

	//this->CS.AddContactConstraint(6, Vector3d (0., 0., 0.05), Vector3d (1., 0., 0.));
	//this->CS.AddContactConstraint(6, Vector3d (0., 0., 0.05), Vector3d (0., 1., 0.));
	//this->CS.AddContactConstraint(6, Vector3d (0., 0., 0.05), Vector3d (0., 0., 1.));

	//this->CS.Bind(*this->model);

	// run()

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
	//this->Q(4) = 0.5;
	this->X 		= SpatialVector (0., 0., 1., 0.620446, 0., 0.995);
	this->XDot 		= SpatialVector (0., 0., 0., 0., 0., 0.);
	this->XDDot 	= SpatialVector (0., 0., 0., 0., 0., 0.);
	this->X0 		= SpatialVector (0., 0., 1., 0.620446, 0., 0.995);
	this->XDot0 	= SpatialVector (0., 0., 0., 0., 0., 0.);
	this->XDDot0 	= SpatialVector (0., 0., 0., 0., 0., 0.);

	this->X_desired	= SpatialVector (0., 0., 1., 0.57, 0., 0.995);

    this->Imp_c		= MatrixNd::Zero(this->model->dof_count, this->model->dof_count);
    this->Imp_k		= MatrixNd::Zero(this->model->dof_count, this->model->dof_count);
    this->Imp_I		= MatrixNd::Zero(this->model->dof_count, this->model->dof_count);

	this->Tau 		= VectorNd::Zero (this->model->dof_count);

	this->F_sensor	= SpatialVector	(0., 0., 0., 0., 0., 0.);
	this->F_desired	= SpatialVector	(0., 0., 0., 0., 0., 0.);

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

	std::cout << "F_ctrl: " << Fc.transpose() << endl;
	this->Fext[6] = Fc;


	std::cout << "Tau: " << this->Tau.transpose() << endl;

	//Fext.push_back(Fc);

	//this->model->gravity = Vector3d (0., 0., 0.);

	// Calculate FrowardDynamics
	ForwardDynamics (*this->model, this->Q, this->QDot, this->Tau, this->QDDot, &Fext);
	//ForwardDynamics (*this->model, this->Q, this->QDot, this->Tau, this->QDDot);
	

	// #2
	//this->Fext[5] = Fc;
	//ForwardDynamicsConstraintsDirect(*this->model, this->Q, this->QDot, this->Tau, this->CS, this->QDDot, &Fext);
	//ForwardDynamicsConstraintsNullSpace(*this->model, this->Q, this->QDot, this->Tau, this->CS, this->QDDot, &Fext);

	// integrate
	for (int i = 0; i < 6; i++)
	{
		this->QDot(i) 	= this->QDot(i) + this->QDDot(i)*this->delta_t;
		this->Q(i) 		= this->Q(i) 	+ this->QDot(i)*this->delta_t;
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
	tempX = CalcBodyToBaseCoordinates(*this->model, this->Q, 6, Vector3d(0., 0., 0.05));
	this->X(3) = tempX(0);
	this->X(4) = tempX(1);
	this->X(5) = tempX(2);

	// orientation
	Matrix3d tempE = Matrix3d::Zero();
	tempE = CalcBodyWorldOrientation(*this->model, this->Q,6);

	this->X(0) = tempE(0, 0);
	this->X(1) = tempE(0, 1);
	this->X(2) = tempE(0, 2);
	
	this->XDot = CalcPointVelocity6D(*this->model, this->Q, this->QDot, 6, base_position, false);
	this->XDDot = CalcPointAcceleration6D(*this->model, this->Q, this->QDot, this->QDDot, 6, base_position, false);

	std::cout << "X: " << this->X.transpose() << endl;
	std::cout << "XDot: " << this->XDot.transpose() << endl;
	std::cout << "XDDot: " << this->XDDot.transpose() << endl;
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

SpatialVector FDCC::ImpedanceControl (SpatialVector X_desired)
{
	SpatialVector F = SpatialVector::Zero(this->model->dof_count);

	F = this->Imp_c*(X_desired - this->X); // - this->Imp_k*this->XDot - this->Imp_I*this->XDDot;

	return F;
}

void FDCC::ControlLoop(void)
{

	std::cout << "rate: " <<1.0/this->delta_t << endl;

	ros::Rate r(1.0/this->delta_t);
	

	// init temp variables
	SpatialVector F_imp, F_net, F_ctrl;

	control_msgs::FollowJointTrajectoryActionGoal joint_msg;

	// init move
	this->createRobotTrajectoryMsg();

	while (ros::ok())
	{
		//after calling this function ROS will processes our callbacks
        ros::spinOnce();

		//std::cout << "Control loop..." << endl;

		// Forward kinematics
		this->CalcForwardKinematics(this->Q, this->QDot, this->QDDot);

		// Updejtati kinematiku!!!!!!!
		// Impedance Control
		F_imp = this->ImpedanceControl(this->X_desired);

		//F_imp = SpatialVector(0.0, 0., 0., 0.5, 0., 0.);

		std::cout << "F_imp: " << F_imp.transpose() << endl;
		// Read Sensor Data

		// Calculate Fnet
		F_net = F_imp + this->F_desired + this->F_sensor;
		
		std::cout << "F_net: " << F_net.transpose() << endl;

		// Calculate F_ctrl
		F_ctrl(0) = (double) (-1.0*this->PD_ctrl[0]->compute(F_net(0)));
		F_ctrl(1) = (double) (-1.0*this->PD_ctrl[1]->compute(F_net(1)));
		F_ctrl(2) = (double) (-1.0*this->PD_ctrl[2]->compute(F_net(2)));
		F_ctrl(3) = (double) (-1.0*this->PD_ctrl[3]->compute(F_net(3)));
		F_ctrl(4) = (double) (-1.0*this->PD_ctrl[4]->compute(F_net(4)));
		F_ctrl(5) = (double) (-1.0*this->PD_ctrl[5]->compute(F_net(5)));


		F_ctrl = SpatialVector(0.0, 0.0, 0.0, 0.0, 0.01, 0.0);
		

		// Forward Dynamics including integration
		
		this->CalcForwardDynamics(F_ctrl);

		std::cout << "Q: " << this->Q.transpose() << endl;
		std::cout << "QDot: " << this->QDot.transpose() << endl;		
		std::cout << "QDDot: " << this->QDDot.transpose() << endl;

		// Move robot
		this->F_sensor = SpatialVector(0., 0., 0., 0., 0., 0.);

		this->createRobotTrajectoryMsg();

		std::cout << endl;
		r.sleep();
	}
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

	control_msgs::FollowJointTrajectoryActionGoal return_value;
	
	
	std_msgs::Header h;
	h.stamp = ros::Time::now();
	return_value.goal.trajectory.joint_names.push_back("joint_a1");
	return_value.goal.trajectory.joint_names.push_back("joint_a2");
	return_value.goal.trajectory.joint_names.push_back("joint_a3");
	return_value.goal.trajectory.joint_names.push_back("joint_a4");
	return_value.goal.trajectory.joint_names.push_back("joint_a5");
	return_value.goal.trajectory.joint_names.push_back("joint_a6");

	  //traj_vector_prep.goal_id.stamp = h.stamp;
	std::ostringstream convert;
	convert << h.stamp.nsec;
	return_value.goal_id.id = "Trajectory " + convert.str();
	std_msgs::Header h2, h_temp;
	ros::Time Tprep = ros::Time::now();
	h2.stamp.sec =  Tprep.sec  ; 
	h2.stamp.nsec =  Tprep.nsec + 100;
	return_value.goal.trajectory.header.stamp = h2.stamp; //kada da krene
	return_value.header.stamp = h2.stamp;

	//std::cout << sampledTrajectory.pose_joint_1.size() << endl;

	double temp_time;
	double time_step = 0.01; //0.02; ///50.0;

	double new_time = 2;

	new_time = h2.stamp.sec + h2.stamp.nsec*pow(10, -9);
	new_time = time_step;


	trajectory_msgs::JointTrajectoryPoint point_current;

	point_current.positions.push_back(this->Q(0));
	point_current.positions.push_back(this->Q(1));
	point_current.positions.push_back(this->Q(2));
	point_current.positions.push_back(this->Q(3));
	point_current.positions.push_back(this->Q(4));
	point_current.positions.push_back(this->Q(5));

	
	point_current.velocities.push_back(this->QDot(0));
	point_current.velocities.push_back(this->QDot(0));
	point_current.velocities.push_back(this->QDot(0));
	point_current.velocities.push_back(this->QDot(0));
	point_current.velocities.push_back(this->QDot(0));
	point_current.velocities.push_back(this->QDot(0));

	point_current.accelerations.push_back(this->QDDot(0));
	point_current.accelerations.push_back(this->QDDot(0));
	point_current.accelerations.push_back(this->QDDot(0));
	point_current.accelerations.push_back(this->QDDot(0));
	point_current.accelerations.push_back(this->QDDot(0));
	point_current.accelerations.push_back(this->QDDot(0));

	//new_time = new_time + time_step;
	//point_current.time_from_start.sec = (int) (floor(h2.stamp.sec + 0.01));
	//point_current.time_from_start.nsec = (int) (h2.stamp.nsec + (0.01)*pow(10, 9));
	//point_current.time_from_start.nsec = (int) (((double) 0.01)*pow(10, 9)); //(new_time - floor(new_time))*pow(10, 9); //(new_time % ceil(new_time))*pow(10, 9);
	//point_current.time_from_start.nsec = (int) (0.01*pow(10, 9)); //(new_time - floor(new_time))*pow(10, 9); //(new_time % ceil(new_time))*pow(10, 9);

	//point_current.time_from_start.sec = 0.01;

	point_current.time_from_start = ros::Duration(0.01);

	//std::cout << point_current.time_from_start.sec<<", "<< (new_time - floor(new_time))*pow(10, 9) <<", " <<h2.stamp<<", "<<h2.stamp.sec<<", "<<h2.stamp.nsec<<", " << endl;

	return_value.goal.trajectory.points.push_back(point_current);

	return_value.goal.goal_time_tolerance = ros::Duration(0.01);

	this->kuka_kr10_joints_pub.publish(return_value);
	//return return_value;	
	
};

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