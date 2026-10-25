#pragma once
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace scan_planner {
// Capture Linux identity once at initialization, never in the odometry loop.
struct FreshChildOdometryIdentity {
  int64_t pid, ppid, start_ticks;
  static FreshChildOdometryIdentity capture() {
    std::ifstream file("/proc/self/stat"); std::string raw;
    if (!std::getline(file, raw)) throw std::runtime_error("Cannot bind child ACK process identity");
    const auto close = raw.rfind(')');
    if (close == std::string::npos) throw std::runtime_error("Malformed child ACK process identity");
    std::istringstream fields(raw.substr(close+2)); std::string value;
    int64_t parent=0, ticks=0;
    for (int i=0; i<=19; ++i) {
      if (!(fields >> value)) throw std::runtime_error("Incomplete child ACK process identity");
      if (i==1) parent=std::stoll(value);
      if (i==19) ticks=std::stoll(value);
    }
    if (parent!=static_cast<int64_t>(::getppid()) || ticks<=0)
      throw std::runtime_error("Child ACK process identity differs");
    return {static_cast<int64_t>(::getpid()),parent,ticks};
  }
};

template <typename Gid> std::string fresh_child_gid_hex(const Gid &gid) {
  std::ostringstream out; out << std::hex << std::setfill('0');
  for (const auto byte : gid.data) out << std::setw(2) << static_cast<unsigned int>(byte);
  return out.str();
}

inline bool valid_child_ack_nonce(const std::string &nonce) {
  if (nonce.size()!=32) return false;
  for (const char c : nonce) if (!((c>='0' && c<='9') || (c>='a' && c<='f'))) return false;
  return true;
}
} // namespace scan_planner
