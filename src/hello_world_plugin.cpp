#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

using namespace gazebo;

class WorldPluginTutorial : public WorldPlugin
{
  public:
    WorldPluginTutorial() : WorldPlugin()
    {
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) //this function is mandatory because this is the one that gazebo will execute when we load the plugin
    {
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      ROS_INFO("Hello World!");
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&WorldPluginTutorial::OnUpdate, this));
    }

    void OnUpdate()
    {

      if (ros::ok())
        ROS_INFO("Plugin Is Running!!!!!!!");
      else
        ROS_INFO("Nha");
    }

  private: 
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
};


GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
