#pragma once
#include <string>
namespace teacher_sim {
inline bool DiagnosticCollisionIsHard(unsigned group, const std::string &a, const std::string &b) {
  if (group > 4 || a.empty() || b.empty()) return true;
  if (group == 0) return true;
  const auto model = [](const std::string &name) {
    const auto split = name.find("::");
    return split == std::string::npos ? std::string{} : name.substr(0, split);
  };
  const auto robot = [](const std::string &name) { return name == "go2" || name == "teacher_go2"; };
  const std::string ma = model(a), mb = model(b);
  if (robot(ma) == robot(mb)) return true;
  const std::string other = robot(ma) ? mb : ma;
  return other != "floor_1" && other != "floor_2" && other != "floor_3" && other != "ramp_12" && other != "ramp_23";
}
}
