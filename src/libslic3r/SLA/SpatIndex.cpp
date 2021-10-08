#include "SpatIndex.hpp"

// for concave hull merging decisions
#include <libslic3r/SLA/BoostAdapter.hpp>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4244)
#pragma warning(disable: 4267)
#endif

#include "boost/geometry/index/rtree.hpp"

#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace Slic3r { namespace sla {

/* **************************************************************************
 * PointIndex implementation
 * ************************************************************************** */

template<class T> class PointIndex_<T>::Impl {
public:
    using BoostIndex = boost::geometry::index::
        rtree<Element, boost::geometry::index::rstar<16, 4> /* ? */>;

    BoostIndex m_store;
};

template<class T> PointIndex_<T>::PointIndex_(): m_impl(new Impl()) {}
template<class T> PointIndex_<T>::~PointIndex_() {}

template<class T> PointIndex_<T>::PointIndex_(const PointIndex_ &cpy): m_impl(new Impl(*cpy.m_impl)) {}
template<class T> PointIndex_<T>::PointIndex_(PointIndex_&& cpy): m_impl(std::move(cpy.m_impl)) {}

template<class T> PointIndex_<T>& PointIndex_<T>::operator=(const PointIndex_ &cpy)
{
    m_impl.reset(new Impl(*cpy.m_impl));
    return *this;
}

template<class T>
PointIndex_<T>& PointIndex_<T>::operator=(PointIndex_ &&cpy)
{
    m_impl.swap(cpy.m_impl);
    return *this;
}

template<class T>
void PointIndex_<T>::insert(const PointIndexEl_<T> &el)
{
    m_impl->m_store.insert(el);
}

template<class T>
bool PointIndex_<T>::remove(const PointIndexEl_<T>& el)
{
    return m_impl->m_store.remove(el) == 1;
}

template<class T>
std::vector<PointIndexEl_<T>>
PointIndex_<T>::query(std::function<bool(const PointIndexEl_<T> &)> fn) const
{
    namespace bgi = boost::geometry::index;

    std::vector<PointIndexEl_<T>> ret;
    m_impl->m_store.query(bgi::satisfies(fn), std::back_inserter(ret));

    return ret;
}

template<class T>
std::vector<PointIndexEl_<T>> PointIndex_<T>::nearest(const Vec<3, T> &el, unsigned k) const
{
    namespace bgi = boost::geometry::index;
    auto ret = reserve_vector<PointIndexEl_<T>>(k);
    m_impl->m_store.query(bgi::nearest(el, k), std::back_inserter(ret));
    return ret;
}

template<class T> size_t PointIndex_<T>::size() const
{
    return m_impl->m_store.size();
}

template<class T>
void PointIndex_<T>::foreach (std::function<void(const Element &el)> fn)
{
    for(auto& el : m_impl->m_store) fn(el);
}

template<class T>
void PointIndex_<T>::foreach (std::function<void(const Element &el)> fn) const
{
    for(auto& el : m_impl->m_store) fn(el);
}

template class PointIndex_<float>;
template class PointIndex_<double>;

/* **************************************************************************
 * BoxIndex implementation
 * ************************************************************************** */

class BoxIndex::Impl {
public:
    using BoostIndex = boost::geometry::index::
        rtree<BoxIndexEl, boost::geometry::index::rstar<16, 4> /* ? */>;

    BoostIndex m_store;
};

BoxIndex::BoxIndex(): m_impl(new Impl()) {}
BoxIndex::~BoxIndex() {}

BoxIndex::BoxIndex(const BoxIndex &cpy): m_impl(new Impl(*cpy.m_impl)) {}
BoxIndex::BoxIndex(BoxIndex&& cpy): m_impl(std::move(cpy.m_impl)) {}

BoxIndex& BoxIndex::operator=(const BoxIndex &cpy)
{
    m_impl.reset(new Impl(*cpy.m_impl));
    return *this;
}

BoxIndex& BoxIndex::operator=(BoxIndex &&cpy)
{
    m_impl.swap(cpy.m_impl);
    return *this;
}

void BoxIndex::insert(const BoxIndexEl &el)
{
    m_impl->m_store.insert(el);
}

bool BoxIndex::remove(const BoxIndexEl& el)
{
    return m_impl->m_store.remove(el) == 1;
}

std::vector<BoxIndexEl> BoxIndex::query(const BoundingBox &qrbb,
                                        BoxIndex::QueryType qt)
{
    namespace bgi = boost::geometry::index;

    std::vector<BoxIndexEl> ret; ret.reserve(m_impl->m_store.size());

    switch (qt) {
    case qtIntersects:
        m_impl->m_store.query(bgi::intersects(qrbb), std::back_inserter(ret));
        break;
    case qtWithin:
        m_impl->m_store.query(bgi::within(qrbb), std::back_inserter(ret));
    }

    return ret;
}

size_t BoxIndex::size() const
{
    return m_impl->m_store.size();
}

void BoxIndex::foreach(std::function<void (const BoxIndexEl &)> fn)
{
    for(auto& el : m_impl->m_store) fn(el);
}

}} // namespace Slic3r::sla
