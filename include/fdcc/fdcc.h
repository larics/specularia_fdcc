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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>

#include <fdcc/fdcc_state.h>
#include <fdcc/FDCCForceCommandMsg.h>

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
		void 			loadFTSensorParams		(void);
		void 			CalcForwardDynamics		(SpatialVector Fc);
		void 			CalcForwardKinematics	(VectorNd Q, VectorNd QDot, VectorNd QDDot);

		SpatialVector	ConvertImpedanceForceToSpatial	(SpatialVector Fbase, SpatialVector PointPosition);
		SpatialVector	ConvertSensorForceToSpatial		(SpatialVector Fsensor, SpatialVector PointPosition);

		void 			XDesiredCallback		(const geometry_msgs::Pose &msg);
		void 			ForceSensorCallback		(const geometry_msgs::WrenchStamped &msg);
		void 			JointStateCallback 		(const sensor_msgs::JointState &msg);
		void 			DesiredForceCallback 	(const fdcc::FDCCForceCommandMsg &msg);

		SpatialVector 	ImpedanceControl 		(SpatialVector X_desired);
		void 			ControlLoop				(void);

		bool 			CheckJointLimits		();



		void 		SetDeltaT (double delta_t);
		double 		GetDeltaT (void);

		
		void 		createRobotTrajectoryMsg(void);
		void 		CreateFDCCStateMsg(void);


	private:
		bool 	InitJointStatesLoaded;
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
		ros::Subscriber JointStateSub;
		ros::Subscriber DesiredForceSub;

		// virtual model init states
		// Joint init states
		VectorNd	Q0;
		VectorNd	QDot0;
		VectorNd	QDDot0;

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
		SpatialVector 	F_desired_tool;
		SpatialVector	F_desired_base;

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

		// F/T Sensor init params
		double FTSensor_offset_force[3];
		double FTSensor_offset_torque[3];

		//Jocobian
		MatrixNd Jacobian_matrix;	
		ConstraintSet CS;	

		// joint limits
		double joint_limit_min[6];
		double joint_limit_max[6];

		double joint_velocity_limit_min[6];
		double joint_velocity_limit_max[6];

		tf::Matrix3x3	E_tf;

};