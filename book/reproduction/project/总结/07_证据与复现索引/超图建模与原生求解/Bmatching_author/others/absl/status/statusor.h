#include <string_view>
namespace absl {
class Status {
public:
  enum class StatusCode : int { kOk = 1, kNotFound = 4 };
  StatusCode code;
  explicit Status() { code = StatusCode::kOk; }
  Status(StatusCode _code, std::string_view message) : code(_code) {}
  bool ok() { return code == StatusCode::kOk; }
};
template <class T> class StatusOr {
  T value_;
  Status status_;

public:
  // `StatusOr<T>` is copy constructible if `T` is copy constructible.
  StatusOr(const StatusOr &) = default;
  // `StatusOr<T>` is copy assignable if `T` is copy constructible and copy
  // assignable.
  StatusOr &operator=(const StatusOr &) = default;

  // `StatusOr<T>` is move constructible if `T` is move constructible.
  StatusOr(StatusOr &&) = default;
  // `StatusOr<T>` is moveAssignable if `T` is move constructible and move
  // assignable.
  StatusOr &operator=(StatusOr &&) = default;
  StatusOr(T &v) : value_(std::move(v)), status_(Status()){};
  StatusOr(Status s) : status_(s) {
    std::cerr << (int)s.code << " occured" << std::endl;
  }
  T &value() { return value_; }
  ~StatusOr() {
    std::cout << "AAA" << std::endl << (int)status_.code << std::endl;
    if (ok()) {
      status_.~Status();
      value_.~T();
    } else {
      status_.~Status();
    }
  }
  bool ok() { return status_.ok(); }
};
Status NotFoundError(std::string_view message) {
  return Status(Status::StatusCode::kNotFound, message);
}
} // namespace absl
