#include <vector>

#include "VanekTree.hpp"
#include "VanekTreeFFF.hpp"

#include "SLA/SupportPointGenerator.hpp"
#include "SLA/SupportTreeMesher.hpp"
#include "SLA/SupportTreeBuilder.hpp"
#include "SLA/SupportTreeBuildsteps.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include "libslic3r/Layer.hpp"

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

    std::vector<indexed_triangle_set> generate_meshes(PrintObject &p) const;
};

std::vector<indexed_triangle_set> VanekFFFBuilder::generate_meshes(PrintObject &p) const
{
    constexpr size_t steps = 45;

    std::vector<indexed_triangle_set> ret;

    for (const Element &node : m_nodes) {
        Vec3d                p1 = node.junctions[0].pos.cast<double>();
        Vec3d                p2 = node.junctions[1].pos.cast<double>();
        double               R1 = node.junctions[0].R;
        double               R2 = node.junctions[1].R;

        switch(node.ntype) {
        case EType::Bridge: {
            ret.emplace_back(sla::get_mesh(sla::DiffBridge{p1, p2, R1, R2}, steps));
            break;
        }
        case EType::GroundBridge: {
            ret.emplace_back(sla::get_mesh(sla::DiffBridge{p1, p2, R1, R2}, steps));
            ret.emplace_back(sla::get_mesh(sla::Pedestal{p2, 10., 2*R2, R2}, steps));
            break;
        }
        case EType::Merger: {
            Vec3d   p3 = node.junctions[2].pos.cast<double>();
            double  R3 = node.junctions[2].R;
            ret.emplace_back(sla::get_mesh(sla::DiffBridge{p1, p3, R1, R3}, steps));
            ret.emplace_back(sla::get_mesh(sla::DiffBridge{p2, p3, R2, R3}, steps));
            ret.emplace_back(sla::get_mesh(sla::Junction{p3, R3}, steps));
            break;
        }
        case EType::MeshBridge: {
            ret.emplace_back(sla::get_mesh(sla::DiffBridge{p1, p2, R1, R2}, steps));
            break;
        }
        }
    }

    return ret;
}


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
    auto ret = reserve_vector<ExPolygons>(po.layer_count());

    for (const Layer *l : po.layers())
        ret.emplace_back(l->merged(0.f));

    return ret;
}

static std::vector<float> get_slice_grid(const PrintObject &po)
{
    auto ret = reserve_vector<float>(po.layer_count());

    for (const Layer *l : po.layers()) {
        ret.emplace_back(l->slice_z);
    }

    return ret;
}

std::vector<Junction> create_root_points(const PrintObject          &po,
                                         const indexed_triangle_set &its)
{
    sla::IndexedMesh imesh{its};

    auto slices = get_slices(po);
    auto slice_grid = get_slice_grid(po);
    sla::SupportPointGenerator supgen{imesh, slices, slice_grid, {}, []{}, [](int) {}};
    supgen.execute(slices, slice_grid);

    sla::SupportTreeConfig stcfg;
    sla::SupportableMesh sm{imesh, supgen.output(), stcfg};

    auto builder = make_unique<sla::SupportTreeBuilder>();
    sla::SupportTreeBuildsteps bsteps{*builder, sm};
    bsteps.filter();

    auto root_pts = reserve_vector<vanektree::Junction>(supgen.output().size());
    for (const auto &h : builder->heads())
        root_pts.emplace_back(h.junction_point().cast<float>(), h.r_back_mm);

    return root_pts;
}


//auto sp = po.slicing_parameters();

//auto layer = po.add_support_layer(1, 0, sp.first_object_layer_height, po.layers().front()->print_z);
//Polyline line;
//line.points = {{18000000, 18000000}, {19000000, 19000000}};

//Flow fl = support_material_1st_layer_flow(&po, sp.first_object_layer_height);

//ExtrusionPath ep(line, ExtrusionPath{erSupportMaterial, fl.mm3_per_mm(), fl.width(), fl.height()});
//layer->support_fills.append(ep);

void build_vanek_tree_fff(PrintObject &po)
{
    auto tr = po.trafo_centered();

    indexed_triangle_set its = po.model_object()->raw_indexed_triangle_set();
    its_transform(its, tr);

    auto root_pts = create_root_points(po, its);
    auto slice_grid = get_slice_grid(po);

    VanekFFFBuilder builder;
    auto props = vanektree::Properties{}.bed_shape({vanektree::make_bed_poly(its)})
                                        .widening_factor(1.2);

    vanektree::build_tree(its, root_pts, builder, props);

    std::vector<indexed_triangle_set> meshes = builder.generate_meshes(po);

    indexed_triangle_set unimesh;
    for (auto &m : meshes)
        its_merge(unimesh, m);

    std::vector<ExPolygons> unislices = slice_mesh_ex(unimesh, slice_grid, 0.1);
    auto sp = po.slicing_parameters();

    for (size_t lid = 1; lid < unislices.size(); ++lid) {
        auto &sl = unislices[lid];
        auto layer = po.add_support_layer(1, 0, sp.layer_height, po.layers()[lid]->print_z);
        Flow fl = support_material_flow(&po, sp.first_object_layer_height);
        for (const ExPolygon &p : sl) {
            ExtrusionPath ep(Polyline{p.contour.points}, ExtrusionPath{erSupportMaterial, fl.mm3_per_mm(), fl.width(), fl.height()});
            layer->support_fills.append(ep);
        }
    }
}

} // namespace Slic3r
