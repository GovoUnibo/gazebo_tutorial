
#include <WorldCablePlugin.hpp>
using namespace std;
using namespace gazebo;
using namespace sdf_sphere;
using namespace sdf_builder;


CableSpawner::CableSpawner() : WorldPlugin(), SphereSdf() {}
CableSpawner::~CableSpawner(){}







 
void CableSpawner::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf_world)
{

    checkROSInitializzation(); // Make sure the ROS node for Gazebo has already been initialized

    if (readCableParameters(_sdf_world))
      riseInfoMsg(CABLE_PARAM);
          
    sdf::SDF sphereSDF;
    using namespace std;


    setModelName("sphere");
    setModelPose(0, 0, 0, 0, 0, 0);

    addLink("link_1", 2, 1, std::vector<float>{1,0,0.5,0,0,0});
    setSelfCollide(true);
    
    addLink("link_2", 1, 0.5, std::vector<float>{0.2,0,0.5,0,0,0});
    addJoint(TypeOfJoint::revolute, std::vector<int>{1,0,0});
    setSelfCollide(true);
    
    addLink("link_3", 0.1, 0.1, std::vector<float>{0.9,0,0.5,0,0,0});
    setSelfCollide(true);
    

    cout << getSDF() << endl;
     sphereSDF.SetFromString(getSDF());


     //Demonstrate using a custom model name.
     sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
     model->GetAttribute("name")->SetFromString("unique_sphere");
     _parent->InsertModelSDF(sphereSDF);

        
}



bool CableSpawner::readCableParameters(sdf::ElementPtr _sdf)
{
  // TROVARE UN ALGORITMO CHE ALZI ERRORE NEL CASO IN CUI UN PARAMETRO NON SIA STATO SETTATO DENTRO AL FILE .WORLD
  if(_sdf->HasElement("width"))
    width = _sdf->Get<float>("width");

  if(_sdf->HasElement("height"))
    height = _sdf->Get<float>("height");

  if(_sdf->HasElement("vertical_resolution"))
    vertical_res = _sdf->Get<int>("vertical_resolution");

  if(_sdf->HasElement("horizontal_resolution"))
    horizontal_res = _sdf->Get<int>("horizontal_resolution");

  if(_sdf->HasElement("offset_x"))
    offset_x = _sdf->Get<float>("offset_x");

  if(_sdf->HasElement("offset_y"))
    offset_y = _sdf->Get<float>("offset_y");
  
  if(_sdf->HasElement("offset_z"))
    offset_z = _sdf->Get<float>("offset_z");

  if(_sdf->HasElement("sphere_radius"))
    sphere_radius = _sdf->Get<float>("sphere_radius");

  if(_sdf->HasElement("mass"))
    mass = _sdf->Get<float>("mass");         

  if(_sdf->HasElement("stiffness"))
    stiffness = _sdf->Get<float>("stiffness");

  if(_sdf->HasElement("damping"))
    damping = _sdf->Get<float>("damping");

  if(_sdf->HasElement("gravity"))
    gravity = _sdf->Get<bool>("gravity");

  return true;

}

void CableSpawner::checkROSInitializzation()
{
  if (!ros::isInitialized())
  {
    riseErrorMsg(ROS_ERROR);
    return;
  }
  riseInfoMsg(PLUGIN_IS_RUNNING);
}















































void CableSpawner::riseErrorMsg(_FATAL_ERROR_MSG_TYPE enum_msg)
{
  switch (enum_msg)
  {
  case ROS_ERROR:
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    
    break;
  
  case PARAMETER_MISSING:
    ROS_FATAL_STREAM("A CABLE PARAMETER IS MISSING");
    break;
  
  default:
    break;
  }
              
}

void CableSpawner::riseInfoMsg(_INFO_MSG_TYPE enum_msg)
{
  switch (enum_msg)
  {
  case CABLE_PARAM:
    ROS_INFO_STREAM("width = "                  << this->width);
    ROS_INFO_STREAM("height = "                 << this->height);
    ROS_INFO_STREAM("vertical_resolution = "    << this->vertical_res);
    ROS_INFO_STREAM("horizontal_resolution = "  << this->horizontal_res);
    ROS_INFO_STREAM("offset_x = "               << this->offset_x);
    ROS_INFO_STREAM("offset_y = "               << this->offset_y);
    ROS_INFO_STREAM("offset_z = "               << this->offset_z);
    ROS_INFO_STREAM("sphere_radius = "          << this->sphere_radius);
    ROS_INFO_STREAM("mass = "                   << this->mass);
    ROS_INFO_STREAM("stiffness = "              << this->stiffness);
    ROS_INFO_STREAM("damping = "                << this->damping);
    ROS_INFO_STREAM("gravity = "                << this->gravity);
    break;
  
  case PLUGIN_IS_RUNNING:
    ROS_INFO("Cable Plugin Started");
    break;
  
  default:
    break;
  }
              
}

