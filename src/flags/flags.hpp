#ifndef ENUM_CLASS_FLAGS_HPP
#define ENUM_CLASS_FLAGS_HPP

#include "allow_flags.hpp"

namespace flags {

constexpr struct empty_t {
  constexpr empty_t() noexcept = default;
} empty;


template <class E>
class flags {
public:
  static_assert(is_flags<E>::value,
                "flags::flags is disallowed for this type; "
                "use ALLOW_FLAGS_FOR_ENUM macro.");

  using enum_type = E;
  using underlying_type = uint32_t;
  using impl_type = uint32_t;

  constexpr static size_t bit_size() { return sizeof(impl_type) * 8; }

public:
  flags() noexcept = default;
  flags(flags const& fl) noexcept = default;
  flags &operator=(flags const& fl) noexcept = default;
  flags(flags &&fl) noexcept = default;
  flags &operator=(flags &&fl) noexcept= default;

  explicit constexpr flags(empty_t) noexcept : val_(0) {}

  constexpr flags(enum_type e) noexcept : 
    val_(static_cast<impl_type>(e)) {}

  constexpr flags(impl_type e) noexcept : 
    val_(e) {}

  flags &operator=(enum_type e) noexcept {
    val_ = static_cast<impl_type>(e);
    return *this;
  }

  constexpr operator bool() const noexcept { return val_ != 0; }

  constexpr bool operator!() const noexcept { return !val_; }

  friend constexpr bool operator==(flags fl1, flags fl2) {
    return fl1.val_ == fl2.val_;
  }

  friend constexpr bool operator!=(flags fl1, flags fl2) {
    return fl1.val_ != fl2.val_;
  }


  constexpr flags operator~() const noexcept { return flags(~val_); }

  flags &operator|=(flags const& fl) noexcept {
    val_ |= fl.val_;
    return *this;
  }

  flags &operator&=(flags const& fl) noexcept {
    val_ &= fl.val_;
    return *this;
  }

  flags &operator^=(flags const& fl) noexcept {
    val_ ^= fl.val_;
    return *this;
  }


  flags &operator|=(enum_type e) noexcept {
    val_ |= static_cast<impl_type>(e);
    return *this;
  }

  flags &operator&=(enum_type e) noexcept {
    val_ &= static_cast<impl_type>(e);
    return *this;
  }

  flags &operator^=(enum_type e) noexcept {
    val_ ^= static_cast<impl_type>(e);
    return *this;
  }

  friend constexpr flags operator|(flags const& f1, flags const& f2) noexcept {
    return flags(static_cast<impl_type>(f1.val_ | f2.val_));
  }

  friend constexpr flags operator&(flags const& f1, flags const& f2) noexcept {
    return flags(static_cast<impl_type>(f1.val_ & f2.val_));
  }

  friend  constexpr flags operator^(flags const& f1, flags const& f2) noexcept {
    return flags(static_cast<impl_type>(f1.val_ ^ f2.val_));
  }


  constexpr underlying_type underlying_value() const noexcept {
    return static_cast<underlying_type>(val_);
  }

  void set_underlying_value(underlying_type newval) noexcept {
    val_ = static_cast<impl_type>(newval);
  }

  constexpr bool empty() const noexcept { return !val_; }

  constexpr size_t max_size() const noexcept { return bit_size(); }

  void clear() noexcept { val_ = 0; }
  // Returns true if e was set
  bool clear(enum_type e) noexcept {
    impl_type const v = val_;
    val_ &= ~static_cast<impl_type>(e);
    return v != val_;
  }

  void clear(flags const& fl) noexcept {
    val_ &= ~fl.val_;
  }

  constexpr bool has(enum_type e) const noexcept {
    return val_ & static_cast<impl_type>(e);
  }

private:
  impl_type val_ {0};
};

} // namespace flags


template <class E>
constexpr auto operator|(E e1, E e2) noexcept
-> typename std::enable_if<flags::is_flags<E>::value,
                           flags::flags<E>>::type {
  return flags::flags<E>(e1) | e2;
}

template <class E>
constexpr auto operator&(E e1, E e2) noexcept
-> typename std::enable_if<flags::is_flags<E>::value,
                           flags::flags<E>>::type {
  return flags::flags<E>(e1) & e2;
}

template <class E>
constexpr auto operator^(E e1, E e2) noexcept
-> typename std::enable_if<flags::is_flags<E>::value,
                           flags::flags<E>>::type {
  return flags::flags<E>(e1) ^ e2;
}

template <class E>
constexpr auto operator~(E e) noexcept
-> typename std::enable_if<flags::is_flags<E>::value,
                           flags::flags<E>>::type {
  return ~flags::flags<E>(e);
}

#endif // ENUM_CLASS_FLAGS_HPP
