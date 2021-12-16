#include <vector>

#include "VanekTree.hpp"
#include "VanekTreeFFF.hpp"

#include "SLA/SupportPointGenerator.hpp"
#include "SLA/SupportTreeMesher.hpp"
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
        switch(node.ntype) {
        case EType::Bridge: {
            Vec3d                p1 = node.junctions[0].pos.cast<double>();
            Vec3d                p2 = node.junctions[1].pos.cast<double>();
            double               R1 = node.junctions[0].R;
            double               R2 = node.junctions[1].R;

            ret.emplace_back(sla::get_mesh(sla::DiffBridge{p1, p2, R1, R2}, steps));
        }
        case EType::GroundBridge: {

        }
        case EType::Merger: {

        }
        case EType::MeshBridge: {

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

//std::vector<Junction> create_root_points(const PrintObject          &po,
//                                         const indexed_triangle_set &its)
//{
//    sla::IndexedMesh imesh{its};

//    auto slices = get_slices(po);
//    auto slice_grid = get_slice_grid(po);
//    sla::SupportPointGenerator supgen{imesh, slices, slice_grid, {}, []{}, [](int) {}};
//    supgen.execute(slices, slice_grid);

//    auto root_pts = reserve_vector<vanektree::Junction>(supgen.output().size());
//    for (auto &sp : supgen.output())
//        root_pts.emplace_back(sp.pos);
//}

void build_vanek_tree_fff(PrintObject &po)
{
    auto layer = po.add_support_layer(1, 0, 2, 100.);
    Polyline line;
    line.points = {{0, 0}, {150000000, 150000000}};

    Flow fl = po.all_regions().front().get().flow(po, frPerimeter, 0.2);
    layer->support_fills.append(ExtrusionPath(line, ExtrusionPath{erPerimeter, fl.mm3_per_mm(), fl.width(), fl.height()}));

//    auto tr = po.trafo_centered();

//    indexed_triangle_set its = po.model_object()->raw_indexed_triangle_set();
//    its_transform(its, tr);

//    sla::IndexedMesh imesh{its};

//    auto slices = get_slices(po);
//    auto slice_grid = get_slice_grid(po);
//    sla::SupportPointGenerator supgen{imesh, slices, slice_grid, {}, []{}, [](int) {}};
//    supgen.execute(slices, slice_grid);

//    auto root_pts = reserve_vector<vanektree::Junction>(supgen.output().size());
//    for (auto &sp : supgen.output())
//        root_pts.emplace_back(sp.pos);

//    VanekFFFBuilder builder;
//    auto props = vanektree::Properties{}.bed_shape({vanektree::make_bed_poly(its)});

//    vanektree::build_tree(its, root_pts, builder, props);

//    std::vector<indexed_triangle_set> meshes = builder.generate_meshes(po);
//    std::vector<ExPolygons> unislices(slice_grid.size());

//    for (auto &m : meshes) {
//        std::vector<ExPolygons>  slices = slice_mesh_ex(m, slice_grid, 0);
//        for (size_t i = 0; i < slices.size(); ++i)
//            unislices[i].insert(unislices[i].end(), slices[i].begin(), slices[i].end());
//    }

//    for (size_t lid = 0; lid < unislices.size(); ++lid) {
//        auto &sl = unislices[lid];
//        sl = union_ex(sl);
//        auto layer = po.add_support_layer(lid, 0, po.config().layer_height, slice_grid[lid]);
//        layer->lslices = unislices[lid];
//    }
}

} // namespace Slic3r
