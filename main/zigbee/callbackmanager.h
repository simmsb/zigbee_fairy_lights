#pragma once

#include <functional>
#include <vector>

template <typename... X> class CallbackManager;

/** Helper class to allow having multiple subscribers to a callback.
 *
 * @tparam Ts The arguments for the callbacks, wrapped in void().
 */
template <typename... Ts> class CallbackManager<void(Ts...)> {
public:
  /// Add a callback to the list.
  void add(std::function<void(Ts...)> &&callback) {
    this->callbacks_.push_back(std::move(callback));
  }

  /// Call all callbacks in this manager.
  void call(Ts... args) {
    for (auto &cb : this->callbacks_)
      cb(args...);
  }
  size_t size() const { return this->callbacks_.size(); }

  /// Call all callbacks in this manager.
  void operator()(Ts... args) { call(args...); }

protected:
  std::vector<std::function<void(Ts...)>> callbacks_;
};
