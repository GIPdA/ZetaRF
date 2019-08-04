#include <cstdint>
#include <type_traits>
#include <limits>
#include <bitset>

template<typename Enum, bool IsEnum = std::is_enum<Enum>::value>
class bitflag;

template<typename Enum>
class bitflag<Enum, true>
{
public:
  constexpr const static int number_of_bits = std::numeric_limits<typename std::underlying_type<Enum>::type>::digits;

  constexpr bitflag() = default;
  constexpr bitflag(Enum value) : bits(1 << static_cast<std::size_t>(value)) {}
  constexpr bitflag(const bitflag& other) : bits(other.bits) {}

  constexpr bitflag operator|(Enum value) const { bitflag result = *this; result.bits |= 1 << static_cast<std::size_t>(value); return result; }
  constexpr bitflag operator&(Enum value) const { bitflag result = *this; result.bits &= 1 << static_cast<std::size_t>(value); return result; }
  constexpr bitflag operator^(Enum value) const { bitflag result = *this; result.bits ^= 1 << static_cast<std::size_t>(value); return result; }
  constexpr bitflag operator~() const { bitflag result = *this; result.bits.flip(); return result; }

  constexpr bitflag operator&(const bitflag& value) const { bitflag result = *this; result.bits &= value.bits; return result; }

  constexpr bitflag& operator|=(Enum value) { bits |= 1 << static_cast<std::size_t>(value); return *this; }
  constexpr bitflag& operator&=(Enum value) { bits &= 1 << static_cast<std::size_t>(value); return *this; }
  constexpr bitflag& operator^=(Enum value) { bits ^= 1 << static_cast<std::size_t>(value); return *this; }

  constexpr bitflag& operator&=(const bitflag& value) { bits &= value.bits; return *this; }

  constexpr bool any() const { return bits.any(); }
  constexpr bool all() const { return bits.all(); }
  constexpr bool none() const { return bits.none(); }
  constexpr operator bool() { return any(); }

  constexpr bool test(Enum value) const { return bits.test(1 << static_cast<std::size_t>(value)); }
  constexpr void set(Enum value) { bits.set(1 << static_cast<std::size_t>(value)); }
  constexpr void unset(Enum value) { bits.reset(1 << static_cast<std::size_t>(value)); }

private:
  std::bitset<number_of_bits> bits;
};

template<typename Enum>
constexpr auto operator|(Enum left, Enum right) noexcept
    -> std::enable_if_t<std::is_enum<Enum>::value, bitflag<Enum>>
{
  return bitflag<Enum>(left) | right;
}
template<typename Enum>
constexpr auto operator&(Enum left, Enum right) noexcept
    -> std::enable_if_t<std::is_enum<Enum>::value, bitflag<Enum>>
{
  return bitflag<Enum>(left) & right;
}
template<typename Enum>
constexpr auto operator^(Enum left, Enum right) noexcept
    -> std::enable_if_t<std::is_enum<Enum>::value, bitflag<Enum>>
{
  return bitflag<Enum>(left) ^ right;
}
template <typename Enum>
constexpr auto operator~(Enum e) noexcept
    -> std::enable_if_t<std::is_enum<Enum>::value, bitflag<Enum>>
{
    return ~bitflag<Enum>(e);
}
