/**
 * In this file we will implement the automatic SLA support tree generation.
 *
 */

#include <numeric>
#include <libslic3r/SLA/SupportTree.hpp>
#include <libslic3r/SLA/SpatIndex.hpp>
#include <libslic3r/SLA/SupportTreeBuilder.hpp>
#include <libslic3r/SLA/SupportTreeBuildsteps.hpp>
#include <libslic3r/SLA/SupportTreeVanek.hpp>

#include <libslic3r/MTUtils.hpp>
#include <libslic3r/ClipperUtils.hpp>
#include <libslic3r/Model.hpp>
#include <libslic3r/TriangleMeshSlicer.hpp>

#include <libnest2d/optimizers/nlopt/genetic.hpp>
#include <libnest2d/optimizers/nlopt/subplex.hpp>
#include <boost/log/trivial.hpp>
#include <libslic3r/I18N.hpp>

//! macro used to mark string used at localization,
//! return same string
#define L(s) Slic3r::I18N::translate(s)

namespace Slic3r {
namespace sla {

void SupportTree::retrieve_full_mesh(indexed_triangle_set &outmesh) const {
    its_merge(outmesh, retrieve_mesh(MeshType::Support));
    its_merge(outmesh, retrieve_mesh(MeshType::Pad));
}

std::vector<ExPolygons> SupportTree::slice(const std::vector<float> &grid,
                                           float                     cr) const
{
    const indexed_triangle_set &sup_mesh = retrieve_mesh(MeshType::Support);
    const indexed_triangle_set &pad_mesh = retrieve_mesh(MeshType::Pad);

    using Slices = std::vector<ExPolygons>;
    auto slices = reserve_vector<Slices>(2);

    if (!sup_mesh.empty()) {
        slices.emplace_back();
        slices.back() = slice_mesh_ex(sup_mesh, grid, cr, ctl().cancelfn);
    }

    if (!pad_mesh.empty()) {
        slices.emplace_back();

        auto bb = bounding_box(pad_mesh);
        auto maxzit = std::upper_bound(grid.begin(), grid.end(), bb.max.z());
        
        auto cap = grid.end() - maxzit;
        auto padgrid = reserve_vector<float>(size_t(cap > 0 ? cap : 0));
        std::copy(grid.begin(), maxzit, std::back_inserter(padgrid));

        slices.back() = slice_mesh_ex(pad_mesh, padgrid, cr, ctl().cancelfn);
    }

    size_t len = grid.size();
    for (const Slices &slv : slices) { len = std::min(len, slv.size()); }

    // Either the support or the pad or both has to be non empty
    if (slices.empty()) return {};

    Slices &mrg = slices.front();

    for (auto it = std::next(slices.begin()); it != slices.end(); ++it) {
        for (size_t i = 0; i < len; ++i) {
            Slices &slv = *it;
            std::copy(slv[i].begin(), slv[i].end(), std::back_inserter(mrg[i]));
            slv[i] = {}; // clear and delete
        }
    }

    return mrg;
}

static ExPolygon make_bed_poly(const indexed_triangle_set &its)
{
    auto bb = bounding_box(its);

    BoundingBox bbcrd{scaled(to_2d(bb.min)), scaled(to_2d(bb.max))};
    bbcrd.offset(scaled(10.));
    Point min = bbcrd.min, max = bbcrd.max;
    ExPolygon ret = {{min.x(), min.y()}, {max.x(), min.y()}, {max.x(), max.y()}, {min.x(), max.y()}};

    return ret;
}

class VanekTreeBuilder: public vanektree::Builder {
    SupportTreeBuilder &m_builder;
    const SupportTreeConfig  &m_cfg;

public:
    VanekTreeBuilder(SupportTreeBuilder &builder, const SupportTreeConfig &cfg)
        : m_builder{builder}, m_cfg{cfg}
    {}

    bool add_bridge(const vanektree::Junction &from,
                    const vanektree::Junction &to) override
    {
        m_builder.add_diffbridge(from.pos.cast<double>(),
                                 to.pos.cast<double>(), from.R, to.R);

        return true;
    }

    bool add_junction(const vanektree::Junction &jp) override
    {
        m_builder.add_junction(jp.pos.cast<double>(), jp.R);

        return true;
    }

    bool add_ground_bridge(const vanektree::Junction &from,
                           const vanektree::Junction &to) override
    {
        Vec3d startp = from.pos.cast<double>();
        Vec3d endp   = startp;
        endp.z()     = m_builder.ground_level;

        long pid = m_builder.add_pillar(endp, startp.z() - endp.z(), to.R);
        m_builder.add_pillar_base(pid, m_cfg.base_height_mm,
                                  m_cfg.base_radius_mm);

        return true;
    }

    bool add_mesh_bridge(const vanektree::Junction &from,
                         const vanektree::Junction &to) override
    {
        Vec3f  dir        = from.pos - to.pos;
        double availablew = dir.norm();
        Vec3f  dirn       = dir / availablew;
        double minw       = 2 * m_cfg.head_front_radius_mm + 2 * to.R -
                      m_cfg.head_penetration_mm;

        if (availablew > minw) {
            double w     = std::min(availablew, minw + m_cfg.head_width_mm);
            Vec3f to_pos = to.pos + (w - to.R) * dirn;

            Anchor anchor{to.R,
                          m_cfg.head_front_radius_mm,
                          w - minw,
                          m_cfg.head_penetration_mm,
                          dirn.cast<double>(),
                          to.pos.cast<double>()};

            m_builder.add_anchor(anchor);
            add_bridge(from, {to_pos, to.R});
        } else {
            Vec3f to_pos = to.pos + 2 * to.R * dirn;
            add_bridge(from, {to_pos, to.R});
            add_junction(to);
        }

        return true;
    }

    bool report_unroutable(size_t root_id) override
    {
        BOOST_LOG_TRIVIAL(error) << "Cannot route support id " << root_id;
        return true;
    }
};

SupportTree::UPtr SupportTree::create(const SupportableMesh &sm,
                                      const JobController &  ctl)
{
    auto builder = make_unique<SupportTreeBuilder>();
    builder->m_ctl = ctl;
    
    if (sm.cfg.enabled) {

        builder->ground_level = sm.emesh.ground_level() - sm.cfg.object_elevation_mm;
        SupportTreeBuildsteps buildsteps{*builder, sm};

        // Do just the filtering step, it will also create the pinheads.
        buildsteps.filter();

        auto roots = reserve_vector<vanektree::Junction>(builder->heads().size());
        for (const auto &h : builder->heads())
            roots.emplace_back(h.junction_point().cast<float>(), h.r_back_mm);

        auto &its = *sm.emesh.get_triangle_mesh();
        vanektree::build_tree(its, roots, VanekTreeBuilder {*builder, sm.cfg},
                              vanektree::Properties{}.bed_shape({make_bed_poly(its)})
                              .ground_level(builder->ground_level)
                              .max_slope(sm.cfg.bridge_slope)
                              .widening_factor(sm.cfg.pillar_widening_factor));

        builder->merge_and_cleanup();   // clean metadata, leave only the meshes.
    } else {
        // If a pad gets added later, it will be in the right Z level
        builder->ground_level = sm.emesh.ground_level();
    }
    
    return std::move(builder);
}

}} // namespace Slic3r::sla
