/******************************************************************************
File name: virtual_model.h
Description: Class that implements virtual model dynamics using RBDL.
Author: Bruno Maric
******************************************************************************/

#include "ros/ros.h"
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

#include <tf/tf.h>


#include <iostream>

#include "rbdl/rbdl.h"
#include "rbdl/addons/urdfreader/urdfreader.h"
#include <vector>

#include "fdcc/pid_controller.h"

#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/JointState.h"

#include <fdcc/fdcc_state.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

class FDCC{
	public:
		FDCC();
		~FDCC();

		void 			loadURDF				(const char * filename);
		void 			loadImpedanceParams		(void);
		void 			loadPDParams 			(void);
		void 			CalcForwardDynamics		(SpatialVector Fc);
		void 			CalcForwardKinematics	(VectorNd Q, VectorNd QDot, VectorNd QDDot);

		SpatialVector	ConvertImpedanceForceToSpatial	(SpatialVector Fbase, VectorNd PointPosition);
		SpatialVector	ConvertSensorForceToSpatial		(SpatialVector Fsensor, VectorNd PointPosition);

		void 			SetInitCartesianState 	(SpatialVector X0, SpatialVector XDot0, SpatialVector XDDot0);
		void 			SetInitJointPoseState 	(VectorNd Q0, VectorNd QDot0, VectorNd QDDot0);	
		void 			SetDesiredToolPosition	(SpatialVector X_desired);
		void 			SetDesiredToolForce		(SpatialVector F_desired);

		void 			XDesiredCallback		(const geometry_msgs::Pose &msg);
		void 			ForceSensorCallback		(const geometry_msgs::WrenchStamped &msg);

		SpatialVector 	ImpedanceControl 		(SpatialVector X_desired);
		void 			ControlLoop				(void);

		bool 			CheckJointLimits		();



		void 		SetDeltaT (double delta_t);
		double 		GetDeltaT (void);

		void 		CreateJointStatesMsg(void);
		void 		createRobotTrajectoryMsg(void);
		void 		CreateFDCCStateMsg(void);


		void 		testing(void);


	private:
		int 	dummy;
		Model* 	model;
		PidControllerBase* 	PD_ctrl[6];

		// Publishers
		ros::Publisher kuka_kr10_joints_pub;
		ros::Publisher kuka_kr10_joint_state_emulator_pub;
		ros::Publisher fdcc_state_pub;

		// Subscribers
		ros::Subscriber XDesiredSub;
		ros::Subscriber ForceSensorSub;

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
		Matrix3d		E;
		SpatialVector 	X;
		SpatialVector	XDot;
		SpatialVector	XDDot;

		// Tool desired position and force
		Matrix3d		E_desired;
		SpatialVector 	X_desired;
		SpatialVector 	F_desired;

		// Joint forces
		VectorNd Tau;

		// Sensor readings
		SpatialVector F_sensor;
		geometry_msgs::WrenchStamped F_sensorReading;

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

		//Jocobian
		MatrixNd Jacobian_matrix;	
		ConstraintSet CS;	

		// joint limits
		double joint_limit_min[6];
		double joint_limit_max[6];

		double joint_velocity_limit_min[6];
		double joint_velocity_limit_max[6];

};