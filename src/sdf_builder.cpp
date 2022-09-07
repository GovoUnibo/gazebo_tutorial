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
    this->link_name_list.push_back(link_name);
    open_link.push_back("<link name ='" + link_name +"'>\n");
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

void SdfBuilder::setJoint(std::string joint_name, std::string parent, std::string child, TypeOfJoint type, vector<int> axis = vector<int>{1,0,0})
{
    switch (type)
    {
        case fixed:
            joint.push_back("<joint name='" + joint_name +"' type='fixed'>\n" +
                            "<parent>" + parent + "</parent>\n" +
                            "<child>" + child + "</child>\n" +
                            "</joint>\n"
                            );
        break;

        case revolute:
                        joint.push_back("<joint name='" + joint_name +"' type='revolute'>\n" +
                            "<parent>" + parent + "</parent>" +
                            "<child>" + child + "</child>" +
                            "<axis>" + to_string(axis[0]) +to_string(axis[1]) + to_string(axis[2])  +"</axis>"+
                            "</joint>"
                            );
        break;
    
        case prismatic:
                        joint.push_back("<joint name='" + joint_name +"' type='prismatic'>\n" +
                            "<parent>" + parent + "</parent>" +
                            "<child>" + child + "</child>" +
                            "<axis>" + to_string(axis[0]) +to_string(axis[1]) + to_string(axis[2])  +"</axis>"+
                            "</joint>"
                            );
        break;

        case revolute2:
           ;
        break;
    }

}

void SdfBuilder::setModelPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0)
{
    this->model_pose = to_string(x) +" "+ to_string(y) +" "+ to_string(z) +" "+ to_string(roll) +" "+ to_string(pitch) +" "+ to_string(yaw);
}

void SdfBuilder::setLinkPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0)
{
    this->link_pose.push_back(to_string(x) +" "+ to_string(y) +" "+ to_string(z) +" "+ to_string(roll) +" "+ to_string(pitch) +" "+ to_string(yaw));
}

void SdfBuilder::setInertiaMomentParam(float i_xx,float i_xy,float i_xz,float i_yy,float i_yz,float i_zz)
{
    this->i_xx.push_back(to_string(i_xx));
    this->i_xy.push_back(to_string(i_xy));
    this->i_xz.push_back(to_string(i_xz));
    this->i_yy.push_back(to_string(i_yy));
    this->i_yz.push_back(to_string(i_yz));
    this->i_zz.push_back(to_string(i_zz));
}
void SdfBuilder::setMass(float mass)
{ 
    this->mass.push_back(to_string(mass));
}

void SdfBuilder::setLinkDimension(std::string dimension)
{
    this->object_dimension.push_back(dimension);
}

string SdfBuilder::getXmlVersion() { return "<?xml version='1.0'?>\n";}

string SdfBuilder::getOpenSdf() { return "<sdf version ='1.7'>\n";}
string SdfBuilder::getCloseSdf(){return "</sdf>";}

string SdfBuilder::getOpenModel() {return open_model;}
string SdfBuilder::getCloseModel() {return "</model>";}

string SdfBuilder::getOpenLink(int index_link=0){return open_link[index_link];}
string SdfBuilder::getCloseLink(){return "</link>\n";}

string SdfBuilder::getMass(int index_link=0){return "<mass>"+ mass[index_link] +"</mass>";}

string SdfBuilder::getInertial(int index_link=0)
{
    return "<inertial>" +
            getMass(index_link)+
            "<inertia>\n" +
            "<ixx>"+this->i_xx[index_link]+"</ixx>" +
            "<ixy>"+this->i_xy[index_link]+"</ixy>" +
            "<ixz>"+this->i_xz[index_link]+"</ixz>" +
            "<iyy>"+this->i_yy[index_link]+"</iyy>" +
            "<iyz>"+this->i_yz[index_link]+"</iyz>" +
            "<izz>"+this->i_zz[index_link]+"</izz>\n" +

            "</inertia>" +
            "</inertial>\n";
}

string SdfBuilder::getGeometry(int index_geometry=0)
{
    return  open_geometry + 
            open_geometry_model +
            open_dimension + object_dimension[index_geometry] + close_dimension + 
            close_geometry_model +
            close_geometry;
}
string SdfBuilder::getCollision(int index_link=0)
{
    return  open_collision + 
            getGeometry(index_link) +
            close_collision;

}
string SdfBuilder::getVisual(int index_link=0)
{
    return  open_visual +
            getGeometry(index_link) +
            close_visual;        
}

string SdfBuilder::getModelPose()   {return open_pose + model_pose + close_pose;}
string SdfBuilder::getLinkPose(int index_link=0)    {return open_pose + link_pose[index_link] + close_pose;};

const bool SdfBuilder::modelHasJoint() {return !(joint.empty());}

const int SdfBuilder::getNumOfJoint() {return (int)joint.size();}

string SdfBuilder::getJoint(int index_link)
{
    return joint[index_link];
}

string SdfBuilder::getLinkName(int index_link)
{
    return link_name_list[index_link];
}











SphereSdf::SphereSdf() : SdfBuilder()
{ 
    setModelType(sphere);
}
SphereSdf::~SphereSdf(){}

void SphereSdf::setModelName(std::string model_name)    { SdfBuilder::setModelName(model_name); }
void SphereSdf::setLinkName(std::string link_name)      { SdfBuilder::setLinkName(link_name); num_of_link += 1;}

void SphereSdf::setModelPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0)
{SdfBuilder::setModelPose(x,y,z,roll,pitch,yaw);}

void SphereSdf::setLinkPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0)
{SdfBuilder::setLinkPose(x,y,z,roll,pitch,yaw);}

void SphereSdf::setRadius(float radius)
{ 
    this->radius.push_back(radius);
    SdfBuilder::setLinkDimension(to_string(radius));
}
void SphereSdf::setMass(float mass)
{
    this->mass.push_back((mass));
    SdfBuilder::setMass(mass);
}


//void SphereSdf::addLink(string link_name, float mass, float radius, vector<float> pose)
void SphereSdf::addLink(string link_name="", float mass=1, float radius=1, vector<float> pose= vector<float>{0,0,0,0,0,0})
{
    if( (int)num_of_link>0)
    {   string parent_link = SdfBuilder::getLinkName(num_of_link-1);
        SphereSdf::addJoint(parent_link, link_name, fixed);
    }
    SphereSdf::setLinkName(link_name);
    //cout << num_of_link << endl;
    SphereSdf::setLinkPose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    SphereSdf::setMass(mass);
    SphereSdf::setRadius(radius);

}

void SphereSdf::addJoint(std::string parent, std::string child, TypeOfJoint type)
{   
    string joint_name = parent + "_to_" + child;
    SdfBuilder::setJoint(joint_name, parent, child, type);
}

void SphereSdf::addJoint(std::string joint_name, std::string parent, std::string child, TypeOfJoint type)
{   
    SdfBuilder::setJoint(joint_name, parent, child, type);
}

void SphereSdf::computeInertiaMatrix(float mass, float radius)
{
    float i_xx, i_xy, i_xz, i_yy, i_yz, i_zz;

    i_xx = (2.0/5.0)*(mass*pow(radius,2));
    i_xy =  0.0;
    i_xz =  0.0;
    i_yy = (2.0/5.0)*(mass*pow(radius,2)); 
    i_yz =  0.0;
    i_zz = (2.0/5.0)*(mass*pow(radius,2)); 
    SdfBuilder::setInertiaMomentParam(i_xx, i_xy, i_xz, i_yy, i_yz, i_zz);
}



string SphereSdf::getSDF()
{
    

    string open_sdf = SdfBuilder::getXmlVersion() + SdfBuilder::getOpenSdf();
    string model = SdfBuilder::getOpenModel() + SdfBuilder::getModelPose();

    string link_list = "";
    for(int i=0; i<num_of_link; i++)
    {   
        computeInertiaMatrix(mass[i], radius[i]);
        link_list  +=   SdfBuilder::getOpenLink(i)      +        
                        SdfBuilder::getInertial(i)      +
                        SdfBuilder::getLinkPose(i)      +
                        SdfBuilder::getCollision(i)     +
                        SdfBuilder::getVisual(i)        +
                        SdfBuilder::getCloseLink()
                        ;

    }
    string joint_list = "";

    for(int j=0; j<SdfBuilder::getNumOfJoint(); j++)
        joint_list += SdfBuilder::getJoint(j);

    
    return  open_sdf +
            model +
            link_list +
            joint_list +
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