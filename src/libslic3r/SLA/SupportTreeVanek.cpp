#include "SupportTreeVanek.hpp"

#include <numeric>
#include <optional>
#include <deque>

#include <igl/random_points_on_mesh.h>
#include "boost/log/trivial.hpp"

#include "SLA/SpatIndex.hpp"
#include "SLA/SupportTreeBuildsteps.hpp"
#include "libslic3r/Tesselate.hpp"

namespace Slic3r { namespace sla {

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

static void to_eigen_mesh(const indexed_triangle_set &its, Eigen::MatrixXd &V, Eigen::MatrixXi &F)
{
    V.resize(its.vertices.size(), 3);
    F.resize(its.indices.size(), 3);
    for (unsigned int i = 0; i < its.indices.size(); ++i)
        F.row(i) = its.indices[i];

    for (unsigned int i = 0; i < its.vertices.size(); ++i)
        V.row(i) = its.vertices[i].cast<double>();
}

static std::vector<Vec3f> sample_mesh(const indexed_triangle_set &its,
                                      double                      radius = 1.)
{
    std::vector<Vec3f> ret;

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

static ExPolygon make_bed_poly(const indexed_triangle_set &its)
{
    auto bb = bounding_box(its);

    BoundingBox bbcrd{scaled(to_2d(bb.min)), scaled(to_2d(bb.max))};
    bbcrd.offset(scaled(10.));
    Point min = bbcrd.min, max = bbcrd.max;
    ExPolygon ret = {{min.x(), min.y()}, {max.x(), min.y()}, {max.x(), max.y()}, {min.x(), max.y()}};

    return ret;
}

static std::vector<Vec3f> sample_bed(const ExPolygon &bed, float z)
{
    std::vector<Vec3f> ret;

    auto triangles = triangulate_expolygon_3d(bed, z);
    indexed_triangle_set its;
    its.vertices.reserve(triangles.size());

    for (size_t i = 0; i < triangles.size(); i += 3) {
        its.vertices.emplace_back(triangles[i].cast<float>());
        its.vertices.emplace_back(triangles[i + 1].cast<float>());
        its.vertices.emplace_back(triangles[i + 2].cast<float>());

        its.indices.emplace_back(i, i + 1, i + 2);
    }

    return sample_mesh(its);
}

inline double merge_distance(const Vec3f &a, const Vec3f &b, double bridge_slope) {
    auto mergept = find_merge_pt(a, b, bridge_slope);
    float ret = std::numeric_limits<float>::infinity();

    if (mergept)
        ret = (a - *mergept).norm();

    return ret;
};

std::vector<Vec3f> copy_support_points(const SupportPoints &pts)
{
    auto ret = reserve_vector<Vec3f>(pts.size());
    for (const auto &p : pts)
        ret.emplace_back(p.pos);

    return ret;
}

std::vector<Vec3f> get_head_junctions(const SupportTreeBuilder &builder)
{
    auto ret = reserve_vector<Vec3f>(builder.heads().size());
    for (const auto &h : builder.heads())
        ret.emplace_back(h.junction_point().cast<float>());

    return ret;
}

bool build_vanek_tree(SupportTreeBuilder &builder, const SupportableMesh &sm)
{
    builder.ground_level = sm.emesh.ground_level() - sm.cfg.object_elevation_mm;

    SupportTreeBuildsteps buildsteps{builder, sm};

    // Do just the filtering step, it will also create the pinheads.
    buildsteps.filter();

    enum PtType { SUPP, MESH, BED, JUNCTION, NONE };

    struct PointCloud {
        std::vector<Vec3f> supppts, junctions, meshpts, bedpts;
        sla::PointfIndex spatindex;
        const double bridge_slope;

        const size_t I1, I2, I3;

        PointCloud(const SupportableMesh &M, const SupportTreeBuilder &builder)
            : supppts{get_head_junctions(builder)}
            , meshpts{sample_mesh(*M.emesh.get_triangle_mesh())}
            , bedpts {sample_bed(make_bed_poly(*M.emesh.get_triangle_mesh()),  M.emesh.ground_level() - M.cfg.object_elevation_mm)}
            , bridge_slope{M.cfg.bridge_slope}
            , I1{supppts.size()}, I2{I1 + meshpts.size()}, I3{I2 + bedpts.size()}
        {
            for (size_t i = 0; i < supppts.size(); ++i)
                spatindex.insert(std::make_pair(supppts[i], i));
            for (size_t i = 0; i < meshpts.size(); ++i)
                spatindex.insert(std::make_pair(meshpts[i], I1 + i));
            for (size_t i = 0; i < bedpts.size(); ++i)
                spatindex.insert(std::make_pair(bedpts[i], I2 + i));
        }

        PtType get_type(size_t i) const
        {
            PtType ret = NONE;

            if (i < I1) ret = SUPP;
            else if (i < I2) ret = MESH;
            else if (i < I3) ret = BED;
            else if (i < I3 + junctions.size()) ret = JUNCTION;

            return ret;
        }

        Vec3f get_coord(size_t i) const
        {
            switch(get_type(i)) {
            case SUPP: return supppts[i];
            case MESH: return meshpts[i - I1];
            case BED:  return bedpts [i - I2];
            case JUNCTION: return junctions[i - I3];
            case NONE: ;
            }

            return Vec3f{};
        }

        double get_distance(const Vec3f &p, const PointfIndexEl &el)
        {
            auto t = get_type(el.second);

            switch (t) {
            case MESH:
            case BED: {
                // Points of mesh or bed which are outside of the support cone of
                // 'pos' must be discarded.
                Vec3f D = el.first - p;
                double Ddist = D.norm();
                double polar = std::acos(D.z() / Ddist);
                if (polar < PI - bridge_slope)
                    return std::numeric_limits<double>::infinity();
                else
                    return Ddist;
            }
            case SUPP:
            case JUNCTION:
                return merge_distance(p, el.first, bridge_slope);
            case NONE:
                ;
            }
            return std::numeric_limits<double>::infinity();
        }

        size_t insert_junction(const Vec3f &p)
        {
            size_t new_idx = I3 + junctions.size();
            junctions.emplace_back(p);
            spatindex.insert(std::make_pair(p, new_idx));
            return new_idx;
        }
    } nodes(sm, builder);

    std::deque<size_t> ptsqueue(sm.pts.size());
    std::iota(ptsqueue.begin(), ptsqueue.end(), 0);

    auto zcmp = [&nodes](size_t a, size_t b) {
        return nodes.get_coord(a).z() < nodes.get_coord(b).z();
    };

    std::sort(ptsqueue.begin(), ptsqueue.end(), zcmp);

    double R = sm.cfg.head_back_radius_mm;

    while (!ptsqueue.empty()) {
        size_t idx = ptsqueue.back();
        ptsqueue.pop_back();

        Vec3f posf = nodes.get_coord(idx);
        nodes.spatindex.remove(std::make_pair(posf, idx));

        auto res   = nodes.spatindex.query(posf, nodes.spatindex.size());

        std::vector<std::pair<size_t, double>> distances(res.size(), std::make_pair(0, std::nan("")));
        for (size_t i = 0; i < res.size(); ++i)
            distances[i] = std::make_pair(i, nodes.get_distance(posf, res[i]));

        std::sort(distances.begin(), distances.end(), [](auto &a, auto &b){
            return a.second < b.second;
        });

        // The first in res is probably the same as the query point.
//        std::sort(res.begin(), res.end(),
//                  [&posf, &nodes](const PointfIndexEl &e1, const PointfIndexEl &e2) {
//                      return nodes.get_distance(posf, e1) < nodes.get_distance(posf, e2);
//                  });

        if (std::isinf(distances[0].second)) {
            BOOST_LOG_TRIVIAL(error) << "Could not route node " << idx;
            continue;
        }

        PointfIndexEl closest = res[distances[0].first];

        auto type = nodes.get_type(closest.second);

        switch (type) {
        case BED: {
            Vec3d posd = posf.cast<double>();
            Vec3d endp = posd;
            endp.z() = builder.ground_level;
            builder.add_pillar(sla::Pillar(endp, posd.z() - endp.z(), R));
            break;
        }
        case MESH: {
            builder.add_bridge(posf.cast<double>(), closest.first.cast<double>(), R);
            break;
        }
        case SUPP:
        case JUNCTION: {
            auto mergept = find_merge_pt(posf, closest.first.cast<float>(), sm.cfg.bridge_slope);
            if (!mergept) continue;

            Vec3d mergeptd = mergept->cast<double>();

            if ((*mergept - closest.first.cast<float>()).norm() > EPSILON) {
                size_t new_idx = nodes.insert_junction(*mergept);
                auto it = std::lower_bound(ptsqueue.begin(), ptsqueue.end(), new_idx, zcmp);
                ptsqueue.insert(it, new_idx);

                // Remove the connected support point from the queue
                it = std::lower_bound(ptsqueue.begin(), ptsqueue.end(), closest.second, zcmp);
                if (it != ptsqueue.end()) {
                    ptsqueue.erase(it);
                }

                nodes.spatindex.remove(closest);
                builder.add_bridge(closest.first.cast<double>(), mergeptd, R);
            }
            builder.add_bridge(posf.cast<double>(), mergeptd, R);
            builder.add_junction(sla::Junction(mergeptd, R));
            break;
        }
        case NONE: ;
        }
    }

    return false;
}

}} // namespace Slic3r::sla
