#ifndef SDF_BUILDER
#define SDF_BUILDER


#include <string>
#include <stdlib.h>
#include <cmath>
#include <iostream>



namespace sdf_builder{

enum ModelGeometry{sphere, box, cylinder};

    class SdfBuilder
    {
        std::string i_xx, i_xy, i_xz, i_yy, i_yz, i_zz;

        std::string open_model = "<model name='default_model_name'>\n";

        std::string open_geometry_model, close_geometry_model;

        std::string open_geometry = "<geometry>\n", close_geometry = "</geometry>\n";
        std::string open_collision= "<collision name='collision'>\n", close_collision = "</collision>\n", collision_size; 
        std::string open_visual= "<visual name='visual'>\n", close_visual = "</visual>\n",  visual_size;

        std::string open_mass = "<mass>", close_mass = "</mass>\n", mass;

        std::string open_pose = "<pose>", close_pose = "</pose>\n", model_pose;

        std::string open_link= "<link name='defualt_link_name'>\n", link_pose;
        std::string object_dimension, open_dimension, close_dimension;
        
        ModelGeometry model_type;

        std::string getGeometry(); //usato per prendere sdf della collision e visual uguali


    public:

        SdfBuilder(/* args */);
        ~SdfBuilder();
        void setModelType(ModelGeometry);
        
        void setModelName(std::string model_name);
        void setLinkName(std::string link_name);
        void setModelPose(float x, float y, float z, float roll, float pitch, float yaw);
        void setLinkPose(float x, float y, float z, float roll, float pitch, float yaw);
        void setMass(float mass);
        void setInertiaMomentParam(float i_xx,float i_xy,float i_xz,float i_yy,float i_yz,float i_zz);
        void setLinkDimension(std::string object_dimension);

        std::string getXmlVersion();

        std::string getOpenSdf();
        std::string getCloseSdf();

        std::string getOpenModel();
        std::string getCloseModel();

        std::string getOpenLink();
        std::string getCloseLink();

        std::string getMass();
        std::string getInertial();
        std::string getCollision();
        std::string getVisual();
        std::string getModelPose();
        std::string getLinkPose();

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

            std::string getSDF();

        private:
            float radius = 1.0;
            float mass = 1.0;
            void computeInertiaMatrix();
            std::string getInertial();
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