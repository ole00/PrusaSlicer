#ifndef SLA_SPATINDEX_HPP
#define SLA_SPATINDEX_HPP

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <libslic3r/BoundingBox.hpp>

namespace Slic3r {
namespace sla {

template<class Scalar>
using PointIndexEl_ = std::pair<Vec<3, Scalar>, unsigned>;

template <class Scalar>
class PointIndex_ {
    class Impl;

    // We use Pimpl because it takes a long time to compile boost headers which
    // is the engine of this class. We include it only in the cpp file.
    std::unique_ptr<Impl> m_impl;
public:

    using Element = PointIndexEl_<Scalar>;
    using Pt = Vec<3, Scalar>;

    PointIndex_();
    ~PointIndex_();

    PointIndex_(const PointIndex_&);
    PointIndex_(PointIndex_&&);
    PointIndex_& operator=(const PointIndex_ &cpy);
    PointIndex_& operator=(PointIndex_ &&);

//    PointIndex_(const PointIndex_ &cpy) : m_impl(new Impl(*cpy.m_impl)) {}
//    PointIndex_(PointIndex_ &&cpy) : m_impl(std::move(cpy.m_impl)) {}
//    PointIndex_& operator=(const PointIndex_ &cpy)
//    {
//        m_impl.reset(new Impl(*cpy.m_impl));
//        return *this;
//    }
//    PointIndex_& operator=(PointIndex_ &&cpy)
//    {
//        m_impl.swap(cpy.m_impl);
//        return *this;
//    }

    void insert(const Element &el);
//    void insert(const Element &el) { m_impl->m_store.insert(el); }

    bool remove(const Element &el);
//    bool remove(const Element &el) { return m_impl->m_store.remove(el) == 1; }

    void insert(const Pt& v, unsigned idx)
    {
        insert(std::make_pair(v, unsigned(idx)));
    }

    std::vector<Element> query(std::function<bool(const Element&)>) const;
    std::vector<Element> nearest(const Pt  &, unsigned k = 1) const;
    std::vector<Element> query  (const Pt &v, unsigned k = 1) const // wrapper
    {
        return nearest(v, k);
    }

    // For testing
    size_t size() const;
//    size_t size() const { return m_impl->m_store.size(); }
    bool empty() const { return size() == 0; }

//    template<class Fn>
//    void foreach(Fn fn) { for(auto& el : m_impl->m_store) fn(el); }
    void foreach(std::function<void(const Element& el)> fn);

//    template<class Fn>
//    void foreach(Fn fn) const { for(const auto &el : m_impl->m_store) fn(el); }
    void foreach(std::function<void(const Element& el)> fn) const;
};

extern template class PointIndex_<float>;
extern template class PointIndex_<double>;

using PointIndex = PointIndex_<double>;
using PointIndexEl = PointIndex::Element;
using PointfIndex = PointIndex_<float>;
using PointfIndexEl = PointfIndex::Element;

using BoxIndexEl = std::pair<Slic3r::BoundingBox, unsigned>;

class BoxIndex {
    class Impl;
    
    // We use Pimpl because it takes a long time to compile boost headers which
    // is the engine of this class. We include it only in the cpp file.
    std::unique_ptr<Impl> m_impl;
public:
    
    BoxIndex();
    ~BoxIndex();
    
    BoxIndex(const BoxIndex&);
    BoxIndex(BoxIndex&&);
    BoxIndex& operator=(const BoxIndex&);
    BoxIndex& operator=(BoxIndex&&);
    
    void insert(const BoxIndexEl&);
    void insert(const BoundingBox& bb, unsigned idx)
    {
        insert(std::make_pair(bb, unsigned(idx)));
    }
    
    bool remove(const BoxIndexEl&);

    enum QueryType { qtIntersects, qtWithin };

    std::vector<BoxIndexEl> query(const BoundingBox&, QueryType qt);
    
    // For testing
    size_t size() const;
    bool empty() const { return size() == 0; }
    
    void foreach(std::function<void(const BoxIndexEl& el)> fn);
};

}
}

#endif // SPATINDEX_HPP
