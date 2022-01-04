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

    std::vector<indexed_triangle_set> generate_meshes() const;
};

std::vector<indexed_triangle_set> VanekFFFBuilder::generate_meshes() const
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

static sla::SupportTreeConfig get_default_sla_spconfig() {
    sla::SupportTreeConfig cfg;

    cfg.head_front_radius_mm = 0.2;
    cfg.head_penetration_mm = 0.4;
    cfg.head_width_mm = 3.0;
    cfg.head_back_radius_mm = 0.5;
    cfg.head_fallback_radius_mm = 0.5 * 0.6;
    cfg.bridge_slope = PI / 4;

    return cfg;
}

std::vector<sla::Head> create_root_points(const PrintObject          &po,
                                         const indexed_triangle_set &its)
{
    sla::IndexedMesh imesh{its};

    auto slices = get_slices(po);
    auto slice_grid = get_slice_grid(po);
    sla::SupportTreeConfig stcfg = get_default_sla_spconfig();
    stcfg.head_width_mm = 2.;
    sla::SupportPointGenerator::Config spgen_cfg;
    spgen_cfg.head_diameter = 2 * stcfg.head_front_radius_mm;
    sla::SupportPointGenerator supgen{imesh, slices, slice_grid, spgen_cfg, []{}, [](int) {}};
    supgen.execute(slices, slice_grid);

    sla::SupportableMesh sm{imesh, supgen.output(), stcfg};

    auto builder = make_unique<sla::SupportTreeBuilder>();
    sla::SupportTreeBuildsteps bsteps{*builder, sm};
    bsteps.filter();

    return builder->heads();
}

static ExtrusionPaths expolys_to_extrusions(const ExPolygons &expolys, const Flow &fl)
{
    ExtrusionPaths extrusions;
    size_t count = std::accumulate(expolys.begin(), expolys.end(), size_t(0),
                                   [](size_t s, auto &p) {
                                       return s + 1 + p.holes.size();
                                   });

    extrusions.reserve(count);

    for (const auto &expoly : expolys) {
        Polyline path{expoly.contour.points};
        if (!path.empty())
            path.points.emplace_back(expoly.contour.points.front());

        extrusions.emplace_back(path,
                                ExtrusionPath{erSupportMaterial, fl.mm3_per_mm(),
                                              fl.width(), fl.height()});
    }

    return extrusions;
}

void build_vanek_tree_fff(PrintObject &po)
{
    auto tr = po.trafo_centered();

    indexed_triangle_set its = po.model_object()->raw_indexed_triangle_set();
    its_transform(its, tr);

    auto root_heads = create_root_points(po, its);
    auto slice_grid = get_slice_grid(po);

    VanekFFFBuilder builder;
    auto props = vanektree::Properties{}.bed_shape({vanektree::make_bed_poly(its)})
                                        .widening_factor(1.);

    auto root_pts = reserve_vector<vanektree::Junction>(root_heads.size());
    for (const auto &h : root_heads)
        root_pts.emplace_back(h.junction_point().cast<float>(), h.r_back_mm);

    vanektree::build_tree(its, root_pts, builder, props);

    std::vector<indexed_triangle_set> meshes = builder.generate_meshes();

    indexed_triangle_set unimesh;
    for (auto &m : meshes)
        its_merge(unimesh, m);

    for (auto &head : root_heads) {
        its_merge(unimesh, sla::get_mesh(head, 45));
    }

    std::vector<ExPolygons> unislices = slice_mesh_ex(unimesh, slice_grid, 0.1);
    auto sp = po.slicing_parameters();

    if (!unislices.empty()) {
        auto &sl = unislices.front();
        Flow fl = support_material_1st_layer_flow(&po, sp.first_object_layer_height);
        auto  layer = po.add_support_layer(0, 0, sp.first_object_layer_height,
                                           po.layers().front()->print_z);
        layer->support_fills.append(expolys_to_extrusions(sl, fl));
    }

    for (size_t lid = 1; lid < unislices.size(); ++lid) {
        auto &sl = unislices[lid];
        auto layer = po.add_support_layer(1, 0, sp.layer_height, po.layers()[lid]->print_z);
        Flow fl = support_material_flow(&po, sp.layer_height);
        layer->support_fills.append(expolys_to_extrusions(sl, fl));
    }
}

} // namespace Slic3r
