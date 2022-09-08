#ifndef SDF_SPHERE
#define SDF_SPHERE


#include<sdf_builder.hpp>

namespace sdf_sphere{

    class SphereSdf : private sdf_builder::SdfBuilder
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
            void setSelfCollide(bool self_collide);

            void setFriction(float mu1, float mu2, std::vector<float> fdir1, float slip1, float slip2);
            void setMu1(float mu1);
            void setMu2(float mu2);
            void setFdir1(std::vector<float> fdir1);
            void setSlip1(float slip1);
            void setSlip2(float slip2);

            void addLink(std::string link_name, float mass, float radius, std::vector<float> pose);

            void addJoint(std::string parent, std::string child, sdf_builder::TypeOfJoint type, std::vector<int> axis);
            void addJoint(sdf_builder::TypeOfJoint type, std::vector<int> axis); //no need to specify parent, it takes in list the last element
            std::string getSDF();

        private:
            std::vector<float> radius;
            std::vector<float> mass;
            void computeInertiaMatrix(float mass, float );

            std::string getInertial();
            std::string _sdf;

            int num_of_link=-1;
    };

}

#endif