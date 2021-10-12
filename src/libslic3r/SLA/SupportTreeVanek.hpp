#ifndef SUPPORTTREEVANEK_HPP
#define SUPPORTTREEVANEK_HPP

#include "SupportTreeBuilder.hpp"

namespace Slic3r { namespace sla { namespace vanektree {

class Properties {
    double m_widening_factor = 1.;
    double m_max_slope = PI / 4.;
    double m_ground_level = 0.;
    double m_sampling_radius = .5;

    ExPolygons m_bed_shape;
public:

    Properties& widening_factor(double val) noexcept { m_widening_factor = val; return *this; }
    Properties& max_slope(double val) noexcept { m_max_slope = val; return *this; }
    Properties& ground_level(double val) noexcept { m_ground_level = val; return *this; }
    Properties& sampling_radius(double val) noexcept { m_sampling_radius = val; return *this; }
    Properties& bed_shape(ExPolygons bed) noexcept { m_bed_shape = std::move(bed); return *this; }


    double widening_factor() const noexcept { return m_widening_factor; }
    double max_slope() const noexcept { return m_max_slope; }
    double ground_level() const noexcept { return m_ground_level; }
    double sampling_radius() const noexcept { return m_sampling_radius; }
    const ExPolygons &bed_shape() const noexcept { return m_bed_shape; }
};

struct Junction
{
    Vec3f  pos;
    double R;
    Junction(const Vec3f &p, double r = 0.) : pos{p}, R{r} {}
};


class Builder {
public:

    virtual ~Builder() = default;
    virtual bool add_bridge(const Junction &from, const Junction &to) = 0;
    virtual bool add_merger(const Junction &node, const Junction &closest, const Junction &merge_node) = 0;
    virtual bool add_ground_bridge(const Junction &from, const Junction &to) = 0;
    virtual bool add_mesh_bridge(const Junction &from, const Junction &to) = 0;

    virtual void report_unroutable_support (size_t root_id) = 0;
    virtual void report_unroutable_junction(const Junction &j) = 0;
};

//bool build_tree(SupportTreeBuilder & builder, const SupportableMesh &sm);

bool build_tree(const indexed_triangle_set & its,
                const std::vector<Junction> &support_roots,
                Builder &                    builder,
                const Properties &           properties = {});

inline bool build_tree(const indexed_triangle_set & its,
                const std::vector<Junction> &support_roots,
                Builder &&                    builder,
                const Properties &           properties = {})
{
    return build_tree(its, support_roots, builder, properties);
}

}}} // namespace Slic3r::sla::vanektree

#endif // SUPPORTTREEVANEK_HPP
