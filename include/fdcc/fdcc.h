/******************************************************************************
File name: virtual_model.h
Description: Class that implements virtual model dynamics using RBDL.
Author: Bruno Maric
******************************************************************************/

#include "ros/ros.h"
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

#include <iostream>

#include "rbdl/rbdl.h"
#include "rbdl/addons/urdfreader/urdfreader.h"
#include <vector>

#include "fdcc/pid_controller.h"

#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

class FDCC{
	public:
		FDCC();
		~FDCC();

		void 	loadURDF(const char * filename);
		void 	loadImpedanceParams(void);
		void 	loadPDParams(void);
		void 	CalcForwardDynamics(SpatialVector Fc);
		void 	CalcForwardKinematics(VectorNd Q, VectorNd QDot, VectorNd QDDot);

		void 	SetInitCartesianState (SpatialVector X0, SpatialVector XDot0, SpatialVector XDDot0);
		void 	SetInitJointPoseState (VectorNd Q0, VectorNd QDot0, VectorNd QDDot0);	
		void 	SetDesiredToolPosition(SpatialVector X_desired);
		void 	SetDesiredToolForce(SpatialVector F_desired);

		SpatialVector ImpedanceControl (SpatialVector X_desired);
		void 	ControlLoop(void);



		void 	SetDeltaT (double delta_t);
		double 	GetDeltaT (void);

		void createRobotTrajectoryMsg(void);


		void 	testing(void);


	private:
		int 	dummy;
		Model* 	model;
		PidControllerBase* 	PD_ctrl[6];

		// virtual model init states
		// Joint init states
		VectorNd	Q0;
		VectorNd	QDot0;
		VectorNd	QDDot0;

		// Tool init states
		SpatialVector 	X0;
		SpatialVector	XDot0;
		SpatialVector	XDDot0;

		// virtual model states
		// Joint states
		VectorNd	Q;
		VectorNd	QDot;
		VectorNd	QDDot;

		// Tool states
		SpatialVector 	X;
		SpatialVector	XDot;
		SpatialVector	XDDot;

		// Tool desired position and force
		SpatialVector X_desired;
		SpatialVector F_desired;

		// Joint forces
		VectorNd Tau;

		// Sensor readings
		SpatialVector F_sensor;

		std::vector<Math::SpatialVector> Fext;


		// Control params
		// Impedance control constants
		MatrixNd 	Imp_c;
		MatrixNd	Imp_k;
		MatrixNd	Imp_I;

		// Force PD Params
		double ForcePD_Kp[6];
		double ForcePD_Kd[6];

		// Integration time constant
		double	delta_t;

		// Publishers
		ros::Publisher kuka_kr10_joints_pub;	

		//Jocobian
		MatrixNd Jacobian_matrix;	
		ConstraintSet CS;	
};