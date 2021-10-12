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
    const SupportableMesh  &m_sm;

public:
    VanekTreeBuilder(SupportTreeBuilder &builder, const SupportableMesh &sm)
        : m_builder{builder}, m_sm{sm}
    {}

    bool add_bridge(const vanektree::Junction &from,
                    const vanektree::Junction &to) override
    {
        Vec3d fromd = from.pos.cast<double>(), tod = to.pos.cast<double>();
        auto hit = m_sm.emesh.query_ray_hit(fromd, (tod - fromd).normalized());

        bool ret = hit.distance() > (tod - fromd).norm();

        if (ret)
            m_builder.add_diffbridge(fromd, tod, from.R, to.R);

        return ret;
    }

    bool add_merger(const vanektree::Junction &node,
                    const vanektree::Junction &closest,
                    const vanektree::Junction &merge_node) override
    {
        Vec3d from1d = node.pos.cast<double>(),
              from2d = closest.pos.cast<double>(),
              tod    = merge_node.pos.cast<double>();

        auto hit1 = m_sm.emesh.query_ray_hit(from1d, (tod - from1d).normalized());
        auto hit2 = m_sm.emesh.query_ray_hit(from2d, (tod - from2d).normalized());

        bool ret = hit1.distance() > (tod - from1d).norm() &&
                   hit2.distance() > (tod - from2d).norm();

        if (ret) {
            m_builder.add_diffbridge(from1d, tod, node.R, merge_node.R);
            m_builder.add_diffbridge(from2d, tod, closest.R, merge_node.R);
            m_builder.add_junction(tod, merge_node.R);
        }

        return ret;
    }

    bool add_ground_bridge(const vanektree::Junction &from,
                           const vanektree::Junction &to) override
    {
        Vec3d startp = from.pos.cast<double>();
        Vec3d endp   = startp;
        endp.z()     = m_builder.ground_level;

        auto hit = m_sm.emesh.query_ray_hit(startp, DOWN);

        if (!hit.is_hit()) {
            long pid = m_builder.add_pillar(endp, startp.z() - endp.z(), to.R);
            m_builder.add_pillar_base(pid, m_sm.cfg.base_height_mm,
                                      m_sm.cfg.base_radius_mm);
        }

        return !hit.is_hit();
    }

    bool add_mesh_bridge(const vanektree::Junction &from,
                         const vanektree::Junction &to) override
    {
        Vec3f  dir        = from.pos - to.pos;
        double availablew = dir.norm();
        Vec3f  dirn       = dir / availablew;
        Vec3d  dirnd      = dirn.cast<double>();
        double minw       = 2 * m_sm.cfg.head_front_radius_mm + 2 * to.R -
                            m_sm.cfg.head_penetration_mm;

        Vec3d fromd = from.pos.cast<double>(), tod = to.pos.cast<double>();
        auto hit = m_sm.emesh.query_ray_hit(fromd, -dirnd);
        bool ret = std::abs(hit.distance() - availablew) < 10 * EPSILON;

        if (ret) {
            if (availablew > minw) {
                double w = std::min(availablew, minw + m_sm.cfg.head_width_mm);

                Anchor anchor{to.R,     m_sm.cfg.head_front_radius_mm,
                              w - minw, m_sm.cfg.head_penetration_mm,
                              dirnd,    to.pos.cast<double>()};

                Vec3d to_pos = tod + (w - to.R) * dirnd;
                m_builder.add_diffbridge(fromd, to_pos, from.R, to.R);
                m_builder.add_anchor(anchor);
            } else {
                Vec3d to_pos = tod + 2. * to.R * dirnd;
                m_builder.add_junction(tod, to.R);
                m_builder.add_diffbridge(fromd, to_pos, from.R, to.R);
            }
        }

        return ret;
    }

    void report_unroutable_support(size_t root_id) override
    {
        BOOST_LOG_TRIVIAL(error) << "Cannot route support id " << root_id;
    }

    void report_unroutable_junction(const vanektree::Junction &j) override
    {
        BOOST_LOG_TRIVIAL(error) << "Cannot route junction at " << j.pos.x()
                                 << " " << j.pos.y() << " " << j.pos.z();
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
        vanektree::build_tree(its, roots, VanekTreeBuilder {*builder, sm},
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
