#pragma one

// Pair class template
template <typename T1, typename T2>
class Pair {
 public:

  Pair() : first_(), second_() {}
  Pair(const T1& first, const T2& second) : first_(first), second_(second) {}

  Pair(const Pair& other) : first_(other.first_), second_(other.second_) {}
  Pair& operator=(const Pair& other) {
    if (this != &other) {
      first_ = other.first_;
      second_ = other.second_;
    }
    return *this;
  }

  Pair(T1&& first, T2&& second) : first_(std::move(first)), second_(std::move(second)) {}
  Pair(Pair&& other) noexcept : first_(std::move(other.first_)), second_(std::move(other.second_)) {}
  Pair& operator=(Pair&& other) noexcept {
    if (this != &other) {
      first_ = std::move(other.first_);
      second_ = std::move(other.second_);
    }
    return *this;
  }

  // Destructor
  ~Pair() = default;

  // Accessors for first and second members
  T1& first() { return first_; }
  const T1& first() const { return first_; }

  T2& second() { return second_; }
  const T2& second() const { return second_; }

 private:
  T1 first_;
  T2 second_;
};

