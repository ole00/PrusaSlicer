#include "SupportTreeVanek.hpp"

#include <numeric>
#include <optional>
#include <deque>

#include <igl/random_points_on_mesh.h>

#include "SLA/SupportTreeBuildsteps.hpp"
#include "libslic3r/Tesselate.hpp"
#include "libslic3r/KDTreeIndirect.hpp"

namespace Slic3r { namespace sla { namespace vanektree {

static std::optional<Vec3f> find_merge_pt(const Vec3f &A,
                                          const Vec3f &B,
                                          float        max_slope)
{
    Vec3f Da = (B - A).normalized(), Db = -Da;
    auto [polar_da, azim_da] = dir_to_spheric(Da);
    auto [polar_db, azim_db] = dir_to_spheric(Db);
    polar_da = std::max(polar_da, float(PI) - max_slope);
    polar_db = std::max(polar_db, float(PI) - max_slope);

    Da = spheric_to_dir<float>(polar_da, azim_da);
    Db = spheric_to_dir<float>(polar_db, azim_db);

    double t1 = (A.z() * Db.x() + Db.z() * B.x() - B.z() * Db.x() - Db.z() * A.x()) /
                (Da.x() * Db.z() - Da.z() * Db.x());

    double t2 = (A.x() + Da.x() * t1 - B.x()) / Db.x();

    return t1 > 0. && t2 > 0. ? A + t1 * Da : std::optional<Vec3f>{};
}

static void to_eigen_mesh(const indexed_triangle_set &its,
                          Eigen::MatrixXd &           V,
                          Eigen::MatrixXi &           F)
{
    V.resize(its.vertices.size(), 3);
    F.resize(its.indices.size(), 3);
    for (unsigned int i = 0; i < its.indices.size(); ++i)
        F.row(i) = its.indices[i];

    for (unsigned int i = 0; i < its.vertices.size(); ++i)
        V.row(i) = its.vertices[i].cast<double>();
}

static std::vector<Junction> sample_mesh(const indexed_triangle_set &its,
                                         double radius = .1)
{
    std::vector<Junction> ret;

    double surface_area = 0.;
    for (const Vec3i &face : its.indices) {
        std::array<Vec3f, 3> tri = {its.vertices[face(0)],
                                    its.vertices[face(1)],
                                    its.vertices[face(2)]};

        auto U = tri[1] - tri[0], V = tri[2] - tri[0];
        surface_area += 0.5 * U.cross(V).norm();
    }

    int N = surface_area / (PI * radius * radius);

    Eigen::MatrixXd B;
    Eigen::MatrixXi FI;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    to_eigen_mesh(its, V, F);
    igl::random_points_on_mesh(N, V, F, B, FI);

    ret.reserve(size_t(N));
    for (int i = 0; i < FI.size(); i++) {
        Vec3i face = its.indices[FI(i)];
        Vec3f c    = B.row(i)(0) * its.vertices[face(0)] +
                  B.row(i)(1) * its.vertices[face(1)] +
                  B.row(i)(2) * its.vertices[face(2)];
        ret.emplace_back(c);
    }

    return ret;
}

static std::vector<Junction> sample_bed(const ExPolygons &bed,
                                        float             z,
                                        double            radius = 1.)
{
    std::vector<Vec3f> ret;

    auto triangles = triangulate_expolygons_3d(bed, z);
    indexed_triangle_set its;
    its.vertices.reserve(triangles.size());

    for (size_t i = 0; i < triangles.size(); i += 3) {
        its.vertices.emplace_back(triangles[i].cast<float>());
        its.vertices.emplace_back(triangles[i + 1].cast<float>());
        its.vertices.emplace_back(triangles[i + 2].cast<float>());

        its.indices.emplace_back(i, i + 1, i + 2);
    }

    return sample_mesh(its, radius);
}

inline double merge_distance(const Vec3f &a, const Vec3f &b, double bridge_slope)
{
    auto mergept = find_merge_pt(a, b, bridge_slope);
    float ret = std::numeric_limits<float>::infinity();

    if (mergept)
        ret = (a - *mergept).norm();

    return ret;
};

enum PtType { SUPP, MESH, BED, JUNCTION, NONE };

class PointCloud {
    const std::vector<Junction> &m_roots;
    std::vector<Junction> m_junctions, m_meshpoints, m_bedpoints;

    const double bridge_slope;
    const double cos2bridge_slope;

    const size_t I1, I2, I3;

    std::vector<bool> m_searchable_indices;
    size_t m_reachable_cnt;

    struct CoordFn {
        const PointCloud *self; CoordFn(PointCloud *s) : self{s} {}
        float             operator()(size_t nodeid, size_t dim) const
        {
            return self->get_coord(nodeid)(int(dim));
        }
    };

    KDTreeIndirect<3, float, CoordFn> m_ktree;

    bool is_outside_support_cone(const Vec3f &supp, const Vec3f &pt)
    {
        Vec3d D = (pt - supp).cast<double>();
        double dot_sq = -D.z() * std::abs(-D.z());

        return dot_sq < D.squaredNorm() * cos2bridge_slope;
    }

public:
    PointCloud(const indexed_triangle_set & M,
               const std::vector<Junction> &support_roots,
               const Properties &           props)
        : m_roots{support_roots}
//        , m_meshpoints{sample_mesh(M, props.sampling_radius())}
        , m_bedpoints{sample_bed(props.bed_shape(),
                                 props.ground_level(),
                                 props.sampling_radius())}
        , bridge_slope{props.max_slope()}
        , cos2bridge_slope{std::cos(bridge_slope) *
                           std::abs(std::cos(bridge_slope))}
        , I1{m_bedpoints.size()}
        , I2{I1 + m_meshpoints.size()}
        , I3{I2 + m_roots.size()}
        , m_searchable_indices(I3, true)
        , m_reachable_cnt{I3}
        , m_ktree{CoordFn{this}, I2} // Only for bed and mesh points
    {
    }

    PtType get_type(size_t node_id) const
    {
        PtType ret = NONE;

        if (node_id < I1) ret = BED;
        else if (node_id < I2) ret = MESH;
        else if (node_id < I3) ret = SUPP;
        else if (node_id < I3 + m_junctions.size()) ret = JUNCTION;

        return ret;
    }

    Junction get(size_t node_id) const
    {
        switch(get_type(node_id)) {
        case BED: return m_bedpoints[node_id];
        case MESH: return m_meshpoints[node_id - I1];
        case SUPP:  return m_roots [node_id - I2];
        case JUNCTION: return m_junctions[node_id - I3];
        case NONE: ;
        }

        return Vec3f{};
    }

    size_t get_support_id(size_t node_id) const { return node_id - I2; }

    Vec3f get_coord(size_t node_id) const { return get(node_id).pos; }

    double get_distance(const Vec3f &p, size_t node)
    {
        auto t = get_type(node);

        switch (t) {
        case MESH:
        case BED: {
            // Points of mesh or bed which are outside of the support cone of
            // 'pos' must be discarded.
            if (is_outside_support_cone(p, get_coord(node)))
                return std::numeric_limits<double>::infinity();
            else
                return (get_coord(node) - p).norm();
        }
        case SUPP:
        case JUNCTION:
            return merge_distance(p, get_coord(node), bridge_slope);
        case NONE:
            ;
        }
        return std::numeric_limits<double>::infinity();
    }

    size_t insert_junction(const Junction &p)
    {
        size_t new_id = I3 + m_junctions.size();
        m_junctions.emplace_back(p);
        m_searchable_indices.emplace_back(true);
        ++m_reachable_cnt;

        return new_id;
    }

    void remove_node(size_t node_id)
    {
        m_searchable_indices[node_id] = false;
        --m_reachable_cnt;
    }

    size_t reachable_count() const { return m_reachable_cnt; }

    template<class Fn> void foreach_reachable(const Vec3f &pos, Fn &&visitor)
    {
        auto closest_anchors =
            find_closest_points<3>(m_ktree, pos, [this, &pos](size_t id) {
                return m_searchable_indices[id] &&
                       !is_outside_support_cone(pos, get_coord(id));
            });

        for (size_t anchor : closest_anchors)
            visitor(anchor, get_distance(pos, anchor));

        for (size_t i = I2; i < m_searchable_indices.size(); ++i)
            if (m_searchable_indices[i])
                visitor(i, get_distance(pos, i));
    }

    std::deque<size_t> start_queue() const
    {
        std::deque<size_t> ptsqueue(m_roots.size());
        std::iota(ptsqueue.begin(), ptsqueue.end(), I2);

        auto zcmp = [this](size_t a, size_t b) {
            return get_coord(a).z() < get_coord(b).z();
        };

        std::sort(ptsqueue.begin(), ptsqueue.end(), zcmp);

        return ptsqueue;
    }
};

bool build_tree(const indexed_triangle_set & its,
                const std::vector<Junction> &support_roots,
                Builder &                    builder,
                const Properties &           properties)
{
    PointCloud nodes(its, support_roots, properties);

    std::deque<size_t> ptsqueue = nodes.start_queue();
    auto zcmp = [&nodes](size_t a, size_t b) {
        return nodes.get_coord(a).z() < nodes.get_coord(b).z();
    };

    auto report_unroutable = [&nodes, &builder] (size_t node_id) {
        switch(nodes.get_type(node_id)) {
        case SUPP:
            builder.report_unroutable_support(nodes.get_support_id(node_id));
            break;
        default: builder.report_unroutable_junction(nodes.get(node_id));
        }
    };

    const double WF = properties.widening_factor();

    while (!ptsqueue.empty()) {
        size_t node_id = ptsqueue.back();
        ptsqueue.pop_back();

        Junction node = nodes.get(node_id);
        nodes.remove_node(node_id);

        struct NodeDistance { size_t node_id; double distance; };
        auto distances = reserve_vector<NodeDistance>(nodes.reachable_count());

        nodes.foreach_reachable(node.pos, [&distances](size_t id, double distance) {
            if (!std::isinf(distance))
                distances.emplace_back(NodeDistance{id, distance});
        });

        std::sort(distances.begin(), distances.end(), [](auto &a, auto &b){
            return a.distance < b.distance;
        });

        if (distances.empty()) { report_unroutable(node_id); continue; }

        auto closest_it = distances.begin();
        bool routed = false;
        while (closest_it != distances.end() && !routed) {
            size_t closest_node_id = closest_it->node_id;
            Junction closest_node = nodes.get(closest_node_id);

            auto type = nodes.get_type(closest_node_id);

            switch (type) {
            case BED: {
                closest_node.R = node.R * WF;
                routed = builder.add_ground_bridge(node, closest_node);
                break;
            }
            case MESH: {
                closest_node.R = node.R ;
                routed = builder.add_mesh_bridge(node, closest_node);
                break;
            }
            case SUPP:
            case JUNCTION: {
                if (auto mergept = find_merge_pt(node.pos, closest_node.pos, properties.max_slope())) {
                    if ((*mergept - closest_node.pos).norm() > EPSILON) {
                        double R = std::max(node.R * WF, closest_node.R * WF);
                        Junction mergenode{*mergept, R};

                        if ((routed = builder.add_merger(node, closest_node, mergenode))) {
                            size_t new_idx = nodes.insert_junction(mergenode);
                            auto   it      = std::lower_bound(ptsqueue.begin(), ptsqueue.end(), new_idx, zcmp);
                            ptsqueue.insert(it, new_idx);

                            // Remove the connected support point from the queue
                            it = std::lower_bound(ptsqueue.begin(), ptsqueue.end(), closest_node_id, zcmp);
                            if (it != ptsqueue.end())
                                ptsqueue.erase(it);

                            nodes.remove_node(closest_node_id);
                        }
                    } else
                        routed = builder.add_bridge(node, closest_node);
                }

                break;
            }
            case NONE:;
            }

            ++closest_it;
        }

        if (!routed)
            report_unroutable(node_id);
    }

    return true;
}

}}} // namespace Slic3r::sla::vanektree
