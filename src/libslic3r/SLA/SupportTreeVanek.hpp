#ifndef SUPPORTTREEVANEK_HPP
#define SUPPORTTREEVANEK_HPP

#include "SupportTreeBuilder.hpp"

namespace Slic3r { namespace sla {

bool build_vanek_tree(SupportTreeBuilder & builder, const SupportableMesh &sm);

}}

#endif // SUPPORTTREEVANEK_HPP
