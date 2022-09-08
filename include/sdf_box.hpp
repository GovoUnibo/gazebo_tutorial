
#ifndef SDF_BOX
#define SDF_BOX

#include<sdf_builder.hpp>


namespace sdf_box{
    
    class BoxSdf : private sdf_builder::SdfBuilder
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

}
#endif