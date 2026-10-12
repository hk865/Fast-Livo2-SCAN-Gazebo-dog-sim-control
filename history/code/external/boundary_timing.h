#pragma once
// Optional bounded observation only: never supplies an estimator input.
#include "diagnostics.h"
#include <array>
#include <time.h>

namespace fastlivo_timing {
constexpr size_t boundary_count=17;
struct Sample { uint64_t wall=0,thread_cpu=0,process_cpu=0; bool valid=true; };
inline uint64_t read_clock(clockid_t id,bool& valid) noexcept {
  timespec value{};
  if(clock_gettime(id,&value)!=0) { valid=false; return 0; }
  return uint64_t(value.tv_sec)*1000000000ULL+uint64_t(value.tv_nsec);
}
inline Sample sample() noexcept {
  Sample s;
  s.wall=read_clock(CLOCK_MONOTONIC,s.valid);
  s.thread_cpu=read_clock(CLOCK_THREAD_CPUTIME_ID,s.valid);
  s.process_cpu=read_clock(CLOCK_PROCESS_CPUTIME_ID,s.valid);
  return s;
}
inline bool enabled() {
  static const bool selected=[] { const char* value=std::getenv("FASTLIVO_BOUNDARY_TIMING");
    return value && std::string(value)=="1"; }();
  return selected && fastlivo_diag::Logger::instance().enabled();
}
struct Counters { uint64_t calls=0,wall=0,thread_cpu=0,process_cpu=0,invalid=0; };
struct Window { uint64_t begin=0,rows=0; std::array<Counters,boundary_count> totals{}; };
inline Window& window() { static thread_local Window value; return value; }
inline void completed(size_t boundary,const Sample& before,const Sample& after) {
  auto& w=window(); if(!w.begin)w.begin=before.wall;
  auto& c=w.totals[boundary]; ++c.calls;
  if(before.valid && after.valid && after.wall>=before.wall && after.thread_cpu>=before.thread_cpu && after.process_cpu>=before.process_cpu) {
    c.wall+=after.wall-before.wall;
    c.thread_cpu+=after.thread_cpu-before.thread_cpu;
    c.process_cpu+=after.process_cpu-before.process_cpu;
  } else ++c.invalid;
  if(after.wall>=w.begin && after.wall-w.begin>=1000000000ULL) {
    std::vector<double> values; values.reserve(5+boundary_count*5);
    values.insert(values.end(),{1,double(++w.rows),double(w.begin),double(after.wall),double(boundary_count)});
    for(const auto& total:w.totals)
      values.insert(values.end(),{double(total.calls),double(total.wall),double(total.thread_cpu),double(total.process_cpu),double(total.invalid)});
    const auto context=fastlivo_diag::Logger::instance().context();
    fastlivo_diag::Logger::instance().raw(300,context.stamp_ns,std::move(values));
    w.begin=after.wall; w.totals={};
  }
}
class Scope {
 public:
  explicit Scope(size_t boundary):boundary_(boundary),active_(enabled()) {
    if(active_) before_=sample();
  }
  ~Scope() { if(active_)completed(boundary_,before_,sample()); }
  Scope(const Scope&)=delete; Scope& operator=(const Scope&)=delete;
 private:
  size_t boundary_; bool active_; Sample before_;
};
}
