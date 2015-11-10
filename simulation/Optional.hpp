#ifndef OPTIONAL_HPP
#define OPTIONAL_HPP

namespace TLC {

template<typename T>
class Optional {
public:
  typedef T* PointerType;
  typedef T const* PointerConstType;
  typedef T& RefType;
  typedef T const& RefConstType;

  Optional();
  Optional(T t);
  Optional(Optional const& rhs);

  template<typename T2>
  Optional(Optional<T2> const& rhs);

  Optional(Optional&& rhs);
  ~Optional();

  Optional& operator=(Optional const& rhs);

  template<typename T2>
  Optional& operator=(Optional<T2> const& rhs);

  Optional& operator=(Optional&& rhs);

  bool isValid() const;
  bool isNothing() const;
  explicit operator bool() const;

  PointerConstType ptr() const;
  PointerType ptr();

  PointerConstType operator->() const;
  PointerType operator->();

  RefConstType operator*() const;
  RefType operator*();

  bool operator==(Optional const& rhs) const;
  bool operator!=(Optional const& rhs) const;
  bool operator<(Optional const& rhs) const;

  RefConstType get() const;
  RefType get();

  // Get either the contents of this Optional or the given default.
  T value(T def = T()) const;

  // Get either this value, or if this value is none the given value.
  Optional orOptional(Optional other) const;

  // Takes the value out of this Optional, leaving it Nothing.
  T take();

  // If this Optional is set, assigns it to t and leaves this Optional as Nothing.
  bool put(T& t);

  void set(T t);
  void reset();

  template<typename... Args>
  void emplace(Args&&... t);

  // Apply a function to the contained value if it is not Nothing.
  template<typename Function>
  void exec(Function&& function);

  // Functor map operator.  If this optional is not Nothing, then applies the
  // given function to it and returns the result, otherwise returns Nothing (of
  // the type the function would normally return).
  template<typename Function>
  auto apply(Function&& function) const -> Optional<typename std::decay<decltype(function(std::declval<T>()))>::type>;

  // Monadic bind operator.  Given function should return another Optional.
  template<typename Function>
  auto sequence(Function function) const -> decltype(function(std::declval<T>()));

private:
  T const* data() const;
  T* data();

  union {
    T m_data;
  };

  bool m_initialized;
};

template<typename T>
std::ostream& operator<<(std::ostream& os, Optional<T> const& v);

template<typename T>
Optional<T>::Optional()
  : m_initialized(false) {}

template<typename T>
Optional<T>::Optional(T t)
  : Optional() {
  set(move(t));
}

template<typename T>
Optional<T>::Optional(Optional const& rhs)
  : Optional() {
  operator=(rhs);
}

template<typename T>
template<typename T2>
Optional<T>::Optional(Optional<T2> const& rhs)
  : Optional() {
  operator=(rhs);
}

template<typename T>
Optional<T>::Optional(Optional&& rhs)
  : Optional() {
  operator=(move(rhs));
}

template<typename T>
Optional<T>::~Optional() {
  reset();
}

template<typename T>
Optional<T>& Optional<T>::operator=(Optional const& rhs) {
  if (&rhs == this)
    return *this;

  if (rhs)
    set(*rhs);
  else
    reset();

  return *this;
}

template<typename T>
template<typename T2>
Optional<T>& Optional<T>::operator=(Optional<T2> const& rhs) {
  if (rhs)
    set(*rhs);
  else
    reset();

  return *this;
}

template<typename T>
Optional<T>& Optional<T>::operator=(Optional&& rhs) {
  if (&rhs == this)
    return *this;

  if (rhs)
    set(rhs.take());
  else
    reset();

  return *this;
}

template<typename T>
bool Optional<T>::isValid() const {
  return m_initialized;
}

template<typename T>
bool Optional<T>::isNothing() const {
  return !m_initialized;
}

template<typename T>
Optional<T>::operator bool() const {
  return m_initialized;
}

template<typename T>
auto Optional<T>::ptr() const -> PointerConstType {
  if (m_initialized)
    return data();
  return nullptr;
}

template<typename T>
auto Optional<T>::ptr() -> PointerType {
  if (m_initialized)
    return data();
  return nullptr;
}

template<typename T>
auto Optional<T>::operator->() const -> PointerConstType {
  if (!m_initialized)
    throw std::exception();

  return data();
}

template<typename T>
auto Optional<T>::operator->() -> PointerType {
  if (!m_initialized)
    throw std::exception();

  return data();
}

template<typename T>
auto Optional<T>::operator*() const -> RefConstType {
  return get();
}

template<typename T>
auto Optional<T>::operator*() -> RefType {
  return get();
}

template<typename T>
bool Optional<T>::operator==(Optional const& rhs) const {
  if (!m_initialized && !rhs.m_initialized)
    return true;
  if (m_initialized && rhs.m_initialized)
    return get() == rhs.get();
  return false;
}

template<typename T>
bool Optional<T>::operator!=(Optional const& rhs) const {
  return !operator==(rhs);
}

template<typename T>
bool Optional<T>::operator<(Optional const& rhs) const {
  if (m_initialized && rhs.m_initialized)
    return get() < rhs.get();
  if (!m_initialized && rhs.m_initialized)
    return true;
  return false;
}

template<typename T>
auto Optional<T>::get() const -> RefConstType {
  if (!m_initialized)
    throw std::exception();

  return *data();
}

template<typename T>
auto Optional<T>::get() -> RefType {
  if (!m_initialized)
    throw std::exception();

  return *data();
}

template<typename T>
T Optional<T>::value(T def) const {
  if (m_initialized)
    return *data();
  else
    return def;
}

template<typename T>
Optional<T> Optional<T>::orOptional(Optional other) const {
  if (m_initialized)
    return *this;
  else
    return other;
}

template<typename T>
T Optional<T>::take() {
  if (!m_initialized)
    throw std::exception();

  T val(move(*data()));

  reset();

  return val;
}

template<typename T>
bool Optional<T>::put(T& t) {
  if (m_initialized) {
    t = move(*data());

    reset();

    return true;
  } else {
    return false;
  }
}

template<typename T>
void Optional<T>::set(T t) {
  reset();

  new (data()) T(move(t));
  m_initialized = true;
}

template<typename T>
template<typename... Args>
void Optional<T>::emplace(Args&&... t) {
  reset();

  new (data()) T(forward<Args>(t)...);
  m_initialized = true;
}

template<typename T>
void Optional<T>::reset() {
  if (m_initialized) {
    m_initialized = false;
    data()->~T();
  }
}

template<typename T>
template<typename Function>
auto Optional<T>::apply(Function&& function) const -> Optional<typename std::decay<decltype(function(std::declval<T>()))>::type> {
  if (!isValid())
    return {};
  return function(get());
}

template<typename T>
template<typename Function>
void Optional<T>::exec(Function&& function) {
  if (isValid())
    function(get());
}

template<typename T>
template<typename Function>
auto Optional<T>::sequence(Function function) const -> decltype(function(std::declval<T>())) {
  if (!isValid())
    return {};
  return function(get());
}

template<typename T>
T const* Optional<T>::data() const {
  return &m_data;
}

template<typename T>
T* Optional<T>::data() {
  return &m_data;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, Optional<T> const& v) {
  if (v)
    return os << "Just (" << *v << ")";
  else
    return os << "Nothing";
}

template<typename T>
size_t hash<Optional<T>>::operator()(Optional<T> const& m) const {
  if (!m)
    return 0;
  else
    return hasher(*m);
}

}

#endif
