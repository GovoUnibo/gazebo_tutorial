
#include<sdf_box.hpp>
using namespace sdf_builder;
using namespace sdf_box;



BoxSdf::BoxSdf() : SdfBuilder()
{
     setModelType(box);
}
BoxSdf::~BoxSdf(){}

void BoxSdf::setModelName(std::string model_name)    { SdfBuilder::setModelName(model_name); }
void BoxSdf::setLinkName(std::string link_name)      { SdfBuilder::setLinkName(link_name); }




