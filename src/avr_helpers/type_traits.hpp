#pragma once

namespace std {

// *** integral_constant ***
template<class T, T v>
struct integral_constant {
    static constexpr T value = v;
    using value_type = T;
    using type = integral_constant; // using injected-class-name
    constexpr operator value_type() const noexcept { return value; }
    constexpr value_type operator()() const noexcept { return value; } //since c++14
};

typedef integral_constant<bool, true>   true_type;
typedef integral_constant<bool, false>  false_type;


// *** is_same ***
template<class T, class U>
struct is_same : std::false_type {};
 
template<class T>
struct is_same<T, T> : std::true_type {};


// *** is_union ***
namespace detail {

template <class T>
struct __uoc_aux // union or class
{
  private:
    template <class _Up>
    static true_type __test( int _Up::* );

    template <class>
    static false_type __test(...);

  public:
    static const bool __value = is_same<decltype(__test<T>(0)),true_type>::value;
};

template <typename _Tp>
struct __is_union_or_class :
    public integral_constant<bool, __uoc_aux<_Tp>::__value>
{ };
} // namespace detail

template <class _Tp>
struct is_union :
    public integral_constant<bool,
                             detail::__is_union_or_class<_Tp>::value
                             >
{ };


// *** is_class ***
namespace detail {
template <class T>
std::integral_constant<bool, !std::is_union<T>::value> test(int T::*);
 
template <class>
std::false_type test(...);
} // namespace detail
 
template <class T>
struct is_class : decltype(detail::test<T>(nullptr))
{};


// *** is_base_of ***
template <class _Base, class _Derived>
struct is_base_of :
    public
            integral_constant<bool, __is_base_of(_Base,_Derived)>
{ };


// *** enable_if ***
template<bool B, class T = void>
struct enable_if {};
 
template<class T>
struct enable_if<true, T> { typedef T type; };

template< bool B, class T = void >
using enable_if_t = typename enable_if<B,T>::type;



// [20.5.6.2] reference modifications:

template <class _Tp>
struct remove_reference
{
    typedef _Tp type;
};

template <class _Tp>
struct remove_reference<_Tp&>
{
    typedef _Tp type;
};


template <class _Tp>
struct add_rvalue_reference
{
    typedef typename remove_reference<_Tp>::type&& type;
};

template <class _Tp>
typename add_rvalue_reference<_Tp>::type declval();

} // namespace std
