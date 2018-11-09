#ifndef ARLIB_ERRORS_H
#define ARLIB_ERRORS_H

#include <chrono>
#include <stdexcept>

namespace arlib {
struct terminator_stop_error : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

template <typename Derived> struct terminator {
public:
  bool should_stop() const {
    return static_cast<Derived &>(*this).should_stop();
  }
};

struct always_continue : public terminator<always_continue> {
public:
  bool should_stop() const { return false; }
};

class timer : public terminator<timer> {
public:
  explicit timer(std::chrono::milliseconds timeout)
      : timeout_{std::chrono::duration_cast<std::chrono::microseconds>(
            timeout)},
        t1_{std::chrono::steady_clock::now()} {}
  explicit timer(std::chrono::microseconds timeout)
      : timeout_{timeout}, t1_{std::chrono::steady_clock::now()} {}

  bool should_stop() const {
    auto t2 = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1_);
    return (elapsed > timeout_);
  }

private:
  std::chrono::microseconds timeout_;
  std::chrono::time_point<std::chrono::steady_clock> t1_;
};
} // namespace arlib

#endif