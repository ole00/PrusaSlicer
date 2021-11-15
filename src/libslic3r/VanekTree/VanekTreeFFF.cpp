#include "VanekTree.hpp"
#include "VanekTreeFFF.hpp"

#include "SLA/SupportPointGenerator.hpp"

namespace Slic3r {

using vanektree::Junction;

class VanekFFFBuilder : public vanektree::Builder
{
public:

    bool add_bridge(const Junction &from, const Junction &to) override
    {
        return false;
    }

    bool add_merger(const Junction &node,
                    const Junction &closest,
                    const Junction &merge_node) override
    {
        return false;
    }

    bool add_ground_bridge(const Junction &from, const Junction &to) override
    {
        return false;
    }


    bool add_mesh_bridge(const Junction &from, const Junction &to) override
    {
        return false;
    }

    void report_unroutable_support(size_t root_id) override {}

    void report_unroutable_junction(const Junction &j) override {}
};


// Transformation without rotation around Z and without a shift by X and Y.
static Transform3d print_trafo(const ModelObject &model_object)
{
    ModelInstance &model_instance = *model_object.instances.front();
    Vec3d          offset         = model_instance.get_offset();
    Vec3d          rotation       = model_instance.get_rotation();
    offset(0) = 0.;
    offset(1) = 0.;
    rotation(2) = 0.;

    auto trafo = Transform3d::Identity();
    trafo.translate(offset);
    trafo.rotate(Eigen::AngleAxisd(rotation.z(), Vec3d::UnitZ()));
    trafo.rotate(Eigen::AngleAxisd(rotation.y(), Vec3d::UnitY()));
    trafo.rotate(Eigen::AngleAxisd(rotation.x(), Vec3d::UnitX()));
    trafo.scale(model_instance.get_scaling_factor());
    trafo.scale(model_instance.get_mirror());

    if (model_instance.is_left_handed())
        trafo = Eigen::Scaling(Vec3d(-1., 1., 1.)) * trafo;

    return trafo;
}

void build_vanek_tree_fff(PrintObject &po)
{
    auto tr = print_trafo(*po.model_object());

    indexed_triangle_set its = po.model_object()->raw_indexed_triangle_set();
    its_transform(its, tr);
}

} // namespace Slic3r
