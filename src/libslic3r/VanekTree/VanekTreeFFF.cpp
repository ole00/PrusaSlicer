#include <vector>

#include "VanekTree.hpp"
#include "VanekTreeFFF.hpp"

#include "SLA/SupportPointGenerator.hpp"

namespace Slic3r {

using vanektree::Junction;

class VanekFFFBuilder : public vanektree::Builder
{
    enum class EType { Bridge, Merger, GroundBridge, MeshBridge };
    using Junctions = std::vector<Junction>;

    struct Element
    {
        EType ntype;
        std::vector<Junction> junctions;
        Element(EType nt, Junctions data)
            : ntype{nt}, junctions{std::move(data)}
        {}
    };

    std::vector<Element>  m_nodes;
    std::vector<size_t>   m_unroutable;
    std::vector<Junction> m_unroutable_junc;

public:

    bool add_bridge(const Junction &from, const Junction &to) override
    {
        m_nodes.emplace_back(EType::Bridge, Junctions{from, to});

        return true;
    }

    bool add_merger(const Junction &node,
                    const Junction &closest,
                    const Junction &merge_node) override
    {
        m_nodes.emplace_back(EType::Merger, Junctions{node, closest, merge_node});

        return true;
    }

    bool add_ground_bridge(const Junction &from, const Junction &to) override
    {
        m_nodes.emplace_back(EType::GroundBridge, Junctions{from, to});

        return true;
    }


    bool add_mesh_bridge(const Junction &from, const Junction &to) override
    {
        m_nodes.emplace_back(EType::MeshBridge, Junctions{from, to});

        return true;
    }

    void report_unroutable_support(size_t root_id) override
    {
        m_unroutable.emplace_back(root_id);
    }

    void report_unroutable_junction(const Junction &j) override
    {
        m_unroutable_junc.emplace_back(j);
    }
};


//// Transformation without rotation around Z and without a shift by X and Y.
//static Transform3d print_trafo(const ModelObject &model_object)
//{
//    ModelInstance &model_instance = *model_object.instances.front();
//    Vec3d          offset         = model_instance.get_offset();
//    Vec3d          rotation       = model_instance.get_rotation();
//    offset(0) = 0.;
//    offset(1) = 0.;
//    rotation(2) = 0.;

//    auto trafo = Transform3d::Identity();
//    trafo.translate(offset);
//    trafo.rotate(Eigen::AngleAxisd(rotation.z(), Vec3d::UnitZ()));
//    trafo.rotate(Eigen::AngleAxisd(rotation.y(), Vec3d::UnitY()));
//    trafo.rotate(Eigen::AngleAxisd(rotation.x(), Vec3d::UnitX()));
//    trafo.scale(model_instance.get_scaling_factor());
//    trafo.scale(model_instance.get_mirror());

//    if (model_instance.is_left_handed())
//        trafo = Eigen::Scaling(Vec3d(-1., 1., 1.)) * trafo;

//    return trafo;
//}

static std::vector<ExPolygons> get_slices(const PrintObject &po)
{
    std::vector<ExPolygons> ret;

    return ret;
}

void build_vanek_tree_fff(PrintObject &po)
{
    auto tr = po.trafo_centered();

    indexed_triangle_set its = po.model_object()->raw_indexed_triangle_set();
    its_transform(its, tr);

    VanekFFFBuilder builder;
    auto props = vanektree::Properties{}.bed_shape({vanektree::make_bed_poly(its)});

    vanektree::build_tree(its, {}, builder, props);
}

} // namespace Slic3r
