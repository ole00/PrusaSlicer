#ifndef SUPPORTTREEVANEK_HPP
#define SUPPORTTREEVANEK_HPP

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace Slic3r { namespace vanektree {

// Vanek tree input parameters. This is an in-line fillable structure with
// setters returning self references.
class Properties
{
    double m_widening_factor = 1.1;
    double m_max_slope       = PI / 4.;
    double m_ground_level    = 0.;
    double m_sampling_radius = .5;

    ExPolygons m_bed_shape;

public:

    // How much a child node's radius will be scaled.
    Properties &widening_factor(double val) noexcept
    {
        m_widening_factor = val;
        return *this;
    }
    // Maximum slope for bridges of the tree
    Properties &max_slope(double val) noexcept
    {
        m_max_slope = val;
        return *this;
    }
    // Z level of the ground
    Properties &ground_level(double val) noexcept
    {
        m_ground_level = val;
        return *this;
    }
    // How far should sample points be in the mesh and the ground
    Properties &sampling_radius(double val) noexcept
    {
        m_sampling_radius = val;
        return *this;
    }
    // Shape of the print bed (ground)
    Properties &bed_shape(ExPolygons bed) noexcept
    {
        m_bed_shape = std::move(bed);
        return *this;
    }

    double widening_factor() const noexcept { return m_widening_factor; }
    double max_slope() const noexcept { return m_max_slope; }
    double ground_level() const noexcept { return m_ground_level; }
    double sampling_radius() const noexcept { return m_sampling_radius; }
    const ExPolygons &bed_shape() const noexcept { return m_bed_shape; }
};

// A junction of the vanek tree with position and radius.
struct Junction
{
    Vec3f  pos;
    double R;
    Junction(const Vec3f &p, double r = 0.) : pos{p}, R{r} {}
};

// An output interface for the Vanek tree generator function. Consider each
// method as a callback and implement the actions that need to be done.
class Builder
{
public:
    virtual ~Builder() = default;

    // A simple bridge from junction to junction.
    virtual bool add_bridge(const Junction &from, const Junction &to) = 0;

    // An Y shaped structure with two starting points and a merge point below
    // them. The angles will respect the max_slope setting.
    virtual bool add_merger(const Junction &node,
                            const Junction &closest,
                            const Junction &merge_node) = 0;

    // Add an anchor bridge to the ground (print bed)
    virtual bool add_ground_bridge(const Junction &from,
                                   const Junction &to) = 0;

    // Add an anchor bridge to the model body
    virtual bool add_mesh_bridge(const Junction &from, const Junction &to) = 0;

    // Report nodes that can not be routed to an endpoint (model or ground)
    virtual void report_unroutable_support(size_t root_id)     = 0;
    virtual void report_unroutable_junction(const Junction &j) = 0;
};

// Build the actual tree.
// its:           The input mesh
// support_roots: The input support points
// builder:       The output interface, describes how to build the tree
// properties:    Parameters of the tree
//
// Notes:
// The original algorithm implicitly ensures that the generated tree avoids
// the model body. This implementation uses point sampling of the mesh thus an
// explicit check is needed if the part of the tree being inserted properly
// avoids the model. This can be done in the builder implementation. Each
// method can return a boolean indicating whether the given branch can or
// cannot be inserted. If a particular path is unavailable, the algorithm
// will try a few other paths as well. If all of them fail, one of the
// report_unroutable_* methods will be called as a last resort.
bool build_tree(const indexed_triangle_set & its,
                const std::vector<Junction> &support_roots,
                Builder &                    builder,
                const Properties &           properties = {});

inline bool build_tree(const indexed_triangle_set & its,
                       const std::vector<Junction> &support_roots,
                       Builder &&                   builder,
                       const Properties &           properties = {})
{
    return build_tree(its, support_roots, builder, properties);
}

// Helper function to derive a bed polygon only from the model bounding box.
inline ExPolygon make_bed_poly(const indexed_triangle_set &its)
{
    auto bb = bounding_box(its);

    BoundingBox bbcrd{scaled(to_2d(bb.min)), scaled(to_2d(bb.max))};
    bbcrd.offset(scaled(10.));
    Point     min = bbcrd.min, max = bbcrd.max;
    ExPolygon ret = {{min.x(), min.y()},
                     {max.x(), min.y()},
                     {max.x(), max.y()},
                     {min.x(), max.y()}};

    return ret;
}

}} // namespace Slic3r::vanektree

#endif // SUPPORTTREEVANEK_HPP
