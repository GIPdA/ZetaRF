#pragma once

#include "type_traits.hpp"

namespace std {

namespace detail {

template <class _Tp>
struct __declval_aux
{
    static const bool __instance = false;
    static typename add_rvalue_reference<_Tp>::type __dummy();
};

} // namespace detail

template <class _Tp>
inline typename add_rvalue_reference<_Tp>::type declval()
{
  // protect declval() from call:
  static_assert(detail::__declval_aux<_Tp>::__instance, "assert");
  return detail::__declval_aux<_Tp>::__dummy();
}

} // namespace std