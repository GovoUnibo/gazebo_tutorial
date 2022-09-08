
#include<sdf_cylinder.hpp>

using namespace sdf_builder;
using namespace sdf_cylinder;
using namespace std;

CylinderSdf::CylinderSdf() : SdfBuilder()
{
    setModelType(box);
}
CylinderSdf::~CylinderSdf(){}

void CylinderSdf::setModelName(std::string model_name)    { SdfBuilder::setModelName(model_name); }
void CylinderSdf::setLinkName(std::string link_name)      { SdfBuilder::setLinkName(link_name); }