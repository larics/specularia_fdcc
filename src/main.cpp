
#include <iostream>

#include "ros/ros.h"
#include <ros/package.h>

//#include <rbdl/rbdl.h>
#include "fdcc/fdcc.h"

//using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {

	ros::init(argc, argv, "FDCC_Node");

	ros::NodeHandle n;
	ros::NodeHandle private_node_handle_("~");

	FDCC v_model;

	v_model.testing();
	v_model.ControlLoop();

	return 0;
}
