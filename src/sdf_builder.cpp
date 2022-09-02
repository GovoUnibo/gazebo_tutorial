#include <sdf_builder.hpp>

using namespace sdf_builder;
using namespace std;

SdfBuilder::SdfBuilder(/* args */)
{
}
    
SdfBuilder::~SdfBuilder()
{
}


void SdfBuilder::setModelName(string model_name)
{
    open_model      = "<model name ='"+model_name+"'>\n";
}

void SdfBuilder::setLinkName(string link_name)
{
    open_link = "<link name ='" + link_name +"'>\n";
    open_collision  = "<collision name='collision'>\n";
    open_visual     = "<visual name='visual'>\n";
}

void SdfBuilder::setModelType(ModelGeometry model)
{
    this->model_type = model;
    switch (model)
    {
        case sphere:
            open_geometry_model  = "<sphere>"; 
            close_geometry_model = "</sphere>";
            open_dimension  = "<radius>"; 
            close_dimension = "</radius>";
        break;

        case box:
            open_geometry_model  = "<box>"; 
            close_geometry_model = "</box>";
            open_dimension  = "<size>"; 
            close_dimension = "<size/>\n";
        break;
    
        case cylinder:
            open_geometry_model  = "<cylinder>"; 
            close_geometry_model = "</cylinder>";
        break;
    }

}



void SdfBuilder::setModelPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0)
{
    this->model_pose = to_string(x) +" "+ to_string(y) +" "+ to_string(z) +" "+ to_string(roll) +" "+ to_string(pitch) +" "+ to_string(yaw);
}

void SdfBuilder::setLinkPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0)
{
    this->link_pose = to_string(x) +" "+ to_string(y) +" "+ to_string(z) +" "+ to_string(roll) +" "+ to_string(pitch) +" "+ to_string(yaw);
}

void SdfBuilder::setInertiaMomentParam(float i_xx,float i_xy,float i_xz,float i_yy,float i_yz,float i_zz)
{
    this->i_xx = std::to_string(i_xx);
    this->i_xy = std::to_string(i_xy);
    this->i_xz = std::to_string(i_xz);
    this->i_yy = std::to_string(i_yy);
    this->i_yz = std::to_string(i_yz);
    this->i_zz = std::to_string(i_zz);
}
void SdfBuilder::setMass(float mass)
{ 
    this->mass = to_string(mass);
}

void SdfBuilder::setLinkDimension(std::string dimension)
{
    this->object_dimension = dimension;
}

string SdfBuilder::getXmlVersion() { return "<?xml version='1.0'?>\n";}

string SdfBuilder::getOpenSdf() { return "<sdf version ='1.4'>\n";}
string SdfBuilder::getCloseSdf(){return "</sdf>";}

string SdfBuilder::getOpenModel() {return open_model;}
string SdfBuilder::getCloseModel() {return "</model>";}

string SdfBuilder::getOpenLink(){return open_link;}
string SdfBuilder::getCloseLink(){return "</link>";}

string SdfBuilder::getMass(){return "<mass>"+mass+"</mass>";}

string SdfBuilder::getInertial()
{
    return "<inertial>" +
            getMass()+
            "<inertia>\n" +
            "<ixx>"+this->i_xx+"</ixx>" +
            "<ixy>"+this->i_xy+"</ixy>" +
            "<ixz>"+this->i_xz+"</ixz>" +
            "<iyy>"+this->i_yy+"</iyy>" +
            "<iyz>"+this->i_yz+"</iyz>" +
            "<izz>"+this->i_zz+"</izz>\n" +

            "</inertia>" +
            "</inertial>\n";
}

string SdfBuilder::getGeometry()
{
    return  open_geometry + 
            open_geometry_model +
            open_dimension + object_dimension + close_dimension + 
            close_geometry_model +
            close_geometry;
}
string SdfBuilder::getCollision()
{
    return  open_collision + 
            getGeometry() +
            close_collision;

}
string SdfBuilder::getVisual()
{
    return  open_visual +
            getGeometry() +
            close_visual;        
}

string SdfBuilder::getModelPose()   {return open_pose + model_pose + close_pose;}
string SdfBuilder::getLinkPose()    {return open_pose + link_pose + close_pose;};

















SphereSdf::SphereSdf() : SdfBuilder()
{ 
    setModelType(sphere);   
}
SphereSdf::~SphereSdf(){}

void SphereSdf::setModelName(std::string model_name)    { SdfBuilder::setModelName(model_name); }
void SphereSdf::setLinkName(std::string link_name)      { SdfBuilder::setLinkName(link_name); }

void SphereSdf::setModelPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0){SdfBuilder::setModelPose(x,y,z,roll,pitch,yaw);}
void SphereSdf::setLinkPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0){SdfBuilder::setLinkPose(x,y,z,roll,pitch,yaw);}
void SphereSdf::setRadius(float radius)
{ 
    this->radius = radius;
    SdfBuilder::setLinkDimension(to_string(radius));
}
void SphereSdf::setMass(float mass)
{
    this->mass = abs(mass);
    SdfBuilder::setMass(mass);
}

void SphereSdf::computeInertiaMatrix()
{
    float i_xx, i_xy, i_xz, i_yy, i_yz, i_zz;

    i_xx = (2.0/5.0)*(this->mass*pow(this->radius,2));
    i_xy =  0.0;
    i_xz =  0.0;
    i_yy = (2.0/5.0)*(this->mass*pow(this->radius,2)); 
    i_yz =  0.0;
    i_zz = (2.0/5.0)*(this->mass*pow(this->radius,2)); 
    SdfBuilder::setInertiaMomentParam(i_xx, i_xy, i_xz, i_yy, i_yz, i_zz);
}

string SphereSdf::getSDF()
{
    computeInertiaMatrix();

    return  //SdfBuilder::getXmlVersion() + 
            SdfBuilder::getOpenSdf()    +
            SdfBuilder::getOpenModel()  +
            SdfBuilder::getModelPose()  +
            SdfBuilder::getOpenLink()   +
            SdfBuilder::getInertial()   +
            SdfBuilder::getLinkPose()   +
            SdfBuilder::getCollision()  +
            SdfBuilder::getVisual()     +
            SdfBuilder::getCloseLink()  +
            SdfBuilder::getCloseModel() +
            SdfBuilder::getCloseSdf();

}


BoxSdf::BoxSdf() : SdfBuilder()
{
     setModelType(box);
}
BoxSdf::~BoxSdf(){}

void BoxSdf::setModelName(std::string model_name)    { SdfBuilder::setModelName(model_name); }
void BoxSdf::setLinkName(std::string link_name)      { SdfBuilder::setLinkName(link_name); }








CylinderSdf::CylinderSdf() : SdfBuilder()
{
    setModelType(box);
}
CylinderSdf::~CylinderSdf(){}

void CylinderSdf::setModelName(std::string model_name)    { SdfBuilder::setModelName(model_name); }
void CylinderSdf::setLinkName(std::string link_name)      { SdfBuilder::setLinkName(link_name); }