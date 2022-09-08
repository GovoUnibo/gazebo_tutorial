#ifndef WORLD_CABLE_PLUGIN
#define WORLD_CABLE_PLUGIN

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "ros/ros.h"
#include "ros/package.h"
#include <geometry_msgs/Pose.h>
#include <string>

#include <sdf_sphere.hpp>

using namespace gazebo;
using namespace sdf_sphere;


enum _INFO_MSG_TYPE {CABLE_PARAM, PLUGIN_IS_RUNNING};

enum _FATAL_ERROR_MSG_TYPE {PARAMETER_MISSING, ROS_ERROR};




class CableSpawner : public WorldPlugin, private SphereSdf
{
    private:
        physics::WorldPtr world;
        float width = 0.0, height = 0.0;
        int vertical_res = 0, horizontal_res = 0;
        float offset_x = 0.0, offset_y = 0.0, offset_z = 0.0, sphere_radius = 0.0;
        float mass = 0.0, stiffness = 0.0, damping = 0.0;
        bool gravity = true;
        bool small = false;
        bool testing = false;

        void checkROSInitializzation();


        void riseErrorMsg(_FATAL_ERROR_MSG_TYPE enum_msg);
        void riseInfoMsg(_INFO_MSG_TYPE enum_msg);


        bool readCableParameters(sdf::ElementPtr _sdf);

      public: 
        CableSpawner();
        ~CableSpawner();


        void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf_world);

};

GZ_REGISTER_WORLD_PLUGIN(CableSpawner)



#endif