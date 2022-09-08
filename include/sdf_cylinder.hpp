#ifndef SDF_CYLINDER
#define SDF_CYLINDER    

#include <sdf_builder.hpp>

namespace sdf_cylinder{
    
    class CylinderSdf : private sdf_builder::SdfBuilder
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