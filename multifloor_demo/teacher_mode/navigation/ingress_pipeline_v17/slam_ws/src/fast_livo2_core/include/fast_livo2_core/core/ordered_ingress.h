#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

// One producer, one estimator owner. Overflow is a visible failure, never a
// replacement of IMU history or an unbounded memory allocation. No ROS state.
namespace fastlivo_ingress {
template<class T> class OrderedQueue {
 public:
  struct Stats {
    uint64_t accepted=0, delivered=0, canceled=0, rejected=0;
    size_t pending=0, bytes=0, peak_pending=0, peak_bytes=0;
    bool closed=false;
    std::string failure;
  };
  OrderedQueue(size_t count_limit, size_t byte_limit)
      : count_limit_(count_limit), byte_limit_(byte_limit) {
    if(!count_limit || !byte_limit) throw std::invalid_argument("zero ingress bound");
  }
  void push(T item, size_t bytes) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stats_.closed || !stats_.failure.empty()) { ++stats_.rejected; return; }
    if(queue_.size()>=count_limit_ || bytes>byte_limit_-stats_.bytes) {
      ++stats_.rejected;
      stats_.failure="ingress queue capacity exceeded";
      return;
    }
    queue_.push_back({std::move(item),bytes});
    ++stats_.accepted; stats_.bytes+=bytes; stats_.pending=queue_.size();
    if(stats_.pending>stats_.peak_pending) stats_.peak_pending=stats_.pending;
    if(stats_.bytes>stats_.peak_bytes) stats_.peak_bytes=stats_.bytes;
  }
  bool pop(T& item, Stats* delivery_snapshot=nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!stats_.failure.empty()) throw std::runtime_error(stats_.failure);
    if(queue_.empty()) return false;
    item=std::move(queue_.front().first); stats_.bytes-=queue_.front().second;
    queue_.pop_front(); ++stats_.delivered; stats_.pending=queue_.size();
    if(delivery_snapshot) *delivery_snapshot=stats_;
    return true;
  }
  void fail(const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stats_.failure.empty()) stats_.failure=message;
  }
  void close() {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.closed=true; stats_.canceled+=queue_.size(); queue_.clear();
    stats_.pending=0; stats_.bytes=0;
  }
  Stats stats() const { std::lock_guard<std::mutex> lock(mutex_); return stats_; }
 private:
  const size_t count_limit_, byte_limit_;
  mutable std::mutex mutex_;
  std::deque<std::pair<T,size_t>> queue_;
  Stats stats_;
};
}
