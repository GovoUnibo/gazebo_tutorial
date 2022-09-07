#ifndef SDF_BUILDER
#define SDF_BUILDER


#include <string>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <vector>



namespace sdf_builder{

enum ModelGeometry{sphere, box, cylinder};
enum TypeOfJoint{fixed, revolute, prismatic, gearbox, revolute2, ball, universal, piston};

    class SdfBuilder
    {
        

        std::string open_model = "<model name='default_model_name'>\n";

        std::string open_geometry_model, close_geometry_model;

        std::string open_geometry = "<geometry>\n", close_geometry = "</geometry>\n";
        std::string open_collision= "<collision name='collision'>\n", close_collision = "</collision>\n", collision_size; 
        std::string open_visual= "<visual name='visual'>\n", close_visual = "</visual>\n",  visual_size;

        std::string open_mass = "<mass>", close_mass = "</mass>\n";

        std::string open_pose = "<pose>", close_pose = "</pose>\n", model_pose;

        std::string open_dimension, close_dimension;

        std::vector<std::string> mass, object_dimension;
        std::vector<std::string> open_link, link_pose, link_name_list, joint;
        std::vector<std::string> i_xx, i_xy, i_xz, i_yy, i_yz, i_zz;

        ModelGeometry model_type;

        std::string getGeometry(int index_geometry); //usato per prendere sdf della collision e visual uguali


    public:

        SdfBuilder(/* args */);
        ~SdfBuilder();
        void setModelType(ModelGeometry);
        
        void setModelName(std::string model_name);
        void setLinkName(std::string link_name);
        void setModelPose(float x, float y, float z, float roll, float pitch, float yaw);
        void setLinkPose(float x, float y, float z, float roll, float pitch, float yaw);
        void setJoint(std::string joint_name, std::string parent, std::string child, TypeOfJoint type, std::vector<int> axis);
        void setMass(float mass);
        void setInertiaMomentParam(float i_xx,float i_xy,float i_xz,float i_yy,float i_yz,float i_zz);
        void setLinkDimension(std::string object_dimension);

        std::string getXmlVersion();

        std::string getOpenSdf();
        std::string getCloseSdf();

        std::string getOpenModel();
        std::string getCloseModel();
        std::string getModelPose();

        std::string getOpenLink(int index_link);
        std::string getCloseLink();

        std::string getMass(int index_link);
        std::string getInertial(int index_link);
        std::string getCollision(int index_link);
        std::string getVisual(int index_link);
        std::string getLinkPose(int index_link);
        const bool modelHasJoint();
        const int getNumOfJoint(); 
        std::string getJoint(int index_link);
        std::string getLinkName(int index_link);


        

    };
    









    
    class SphereSdf : private SdfBuilder
    {
        public:
             SphereSdf();
            ~SphereSdf();

            void setModelPose(float x, float y, float z, float roll, float pitch, float yaw);
            void setLinkPose(float x, float y, float z, float roll, float pitch, float yaw);

            void setModelName(std::string model_name);
            void setLinkName(std::string link_name);
            void setRadius(float radius);
            void setMass(float mass);

            void addLink(std::string link_name, float mass, float radius, std::vector<float> pose);

            void addJoint(std::string parent, std::string child, TypeOfJoint type);
            std::string getSDF();

        private:
            std::vector<float> radius;
            std::vector<float> mass;
            void computeInertiaMatrix(float mass, float );

            void addJoint(std::string joint_name, std::string parent, std::string child, TypeOfJoint type);
            std::string getInertial();
            std::string _sdf;

            int num_of_link=0;
    };
    



    class BoxSdf : private SdfBuilder
    {
        public:
            BoxSdf();
            ~BoxSdf();

            void setModelName(std::string model_name);
            void setLinkName(std::string link_name);

            void setModelPose(float x, float y, float z, float roll, float pitch, float yaw);
            void setLinkPose(float x, float y, float z, float roll, float pitch, float yaw);
        private:
            
    };

    class CylinderSdf : private SdfBuilder
    {
        public: 
            CylinderSdf();
            ~CylinderSdf();

            void setModelName(std::string model_name);
            void setLinkName(std::string link_name);
            
            void setLinkPose(float x, float y, float z, float roll, float pitch, float yaw);
        
        private:
            
    };
}


#endif