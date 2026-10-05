// Test-only publication-boundary observer. No message/control modification.
// Capture only verified C++ sensor_msgs/JointState on exact /joint_states.
#include <rcl/publisher.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace {
constexpr uint64_t capacity = 65536;
constexpr uint64_t publisher_capacity = 16;
struct Publisher {
  std::atomic<const rcl_publisher_t *> handle{nullptr};
  std::atomic<bool> active{false};
  bool cpp_joint_state = false;
};
struct Record {
  uint64_t enter_ns = 0, return_ns = 0;
  int64_t header_ns = 0, header_after_ns = 0;
  uintptr_t handle = 0;
  int64_t tid = 0;
  int ret = 0;
  std::atomic<bool> complete{false};
};
Publisher publishers[publisher_capacity];
Record records[capacity];
std::atomic<uint64_t> publisher_count{0}, calls{0}, unmatched_topic_calls{0};
char output[4096] = {};
uint64_t initial_wall_ns = 0, initial_realtime_ns = 0;

uint64_t clock_ns(clockid_t id) {
  timespec t{};
  clock_gettime(id, &t);
  return uint64_t(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}
template <class Function> Function next(const char *name) {
  void *address = dlsym(RTLD_NEXT, name);
  // An unresolved original ABI is a fatal diagnostic setup failure; never
  // silently drop a publication or fabricate an original return code.
  if (!address) _exit(127);
  return reinterpret_cast<Function>(address);
}
bool selected(const rcl_publisher_t *handle) {
  for (uint64_t i = 0; i < publisher_capacity; ++i) {
    if (publishers[i].active.load(std::memory_order_acquire) &&
        publishers[i].handle.load(std::memory_order_acquire) == handle)
      return publishers[i].cpp_joint_state;
  }
  return false;
}
int64_t message_stamp(const void *message) {
  const auto &m = *static_cast<const sensor_msgs::msg::JointState *>(message);
  return int64_t(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;
}
__attribute__((constructor)) void configure() {
  const char *value = std::getenv("DEMO_PUBLISH_AUDIT_OUTPUT");
  // No environment means no shared/default output and no capture.
  if (!value || value[0] != '/' || std::strlen(value) >= sizeof(output)-40) return;
  std::strcpy(output, value);
  initial_wall_ns = clock_ns(CLOCK_MONOTONIC);
  initial_realtime_ns = clock_ns(CLOCK_REALTIME);
}
__attribute__((destructor)) void save() {
  if (!output[0] || calls.load() == 0) return;
  char filename[4096];
  std::snprintf(filename, sizeof(filename), "%s.%ld.jsonl", output, long(getpid()));
  int fd = open(filename, O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0600);
  if (fd < 0) return;  // Missing file is a failed diagnostic, never overwrite.
  FILE *file = fdopen(fd, "w");
  if (!file) { close(fd); return; }
  const uint64_t count = calls.load();
  uint64_t incomplete = 0;
  for (uint64_t i = 0; i < count && i < capacity; ++i)
    if (!records[i].complete.load(std::memory_order_acquire)) ++incomplete;
  std::fprintf(file,
    "{\"kind\":\"capture_summary\",\"version\":1,\"pid\":%ld,"
    "\"capacity\":%llu,\"calls\":%llu,\"overflow\":%llu,"
    "\"incomplete\":%llu,\"publishers\":%llu,"
    "\"unmatched_topic_calls\":%llu,\"initial_wall_ns\":%llu,"
    "\"initial_realtime_ns\":%llu,\"final_wall_ns\":%llu,"
    "\"final_realtime_ns\":%llu}\n",
    long(getpid()), (unsigned long long)capacity, (unsigned long long)count,
    (unsigned long long)(count > capacity ? count-capacity : 0),
    (unsigned long long)incomplete, (unsigned long long)publisher_count.load(),
    (unsigned long long)unmatched_topic_calls.load(),
    (unsigned long long)initial_wall_ns, (unsigned long long)initial_realtime_ns,
    (unsigned long long)clock_ns(CLOCK_MONOTONIC),
    (unsigned long long)clock_ns(CLOCK_REALTIME));
  for (uint64_t i = 0; i < publisher_capacity; ++i) {
    auto handle = publishers[i].handle.load();
    if (handle) std::fprintf(file,
      "{\"kind\":\"publisher\",\"topic\":\"/joint_states\","
      "\"handle\":%llu,\"cpp_sensor_msgs_JointState_verified\":%s}\n",
      (unsigned long long)reinterpret_cast<uintptr_t>(handle),
      publishers[i].cpp_joint_state ? "true" : "false");
  }
  for (uint64_t i = 0; i < count && i < capacity; ++i) {
    const auto &r = records[i];
    if (!r.complete.load(std::memory_order_acquire)) continue;
    std::fprintf(file,
      "{\"kind\":\"publish\",\"sequence\":%llu,\"handle\":%llu,"
      "\"header_ns\":%lld,\"header_after_ns\":%lld,\"enter_wall_ns\":%llu,"
      "\"return_wall_ns\":%llu,\"ret\":%d,\"tid\":%lld}\n",
      (unsigned long long)i, (unsigned long long)r.handle,
      (long long)r.header_ns, (long long)r.header_after_ns,
      (unsigned long long)r.enter_ns, (unsigned long long)r.return_ns,
      r.ret, (long long)r.tid);
  }
  std::fclose(file);
}
}  // namespace

extern "C" rcl_ret_t rcl_publisher_init(
  rcl_publisher_t *publisher, const rcl_node_t *node,
  const rosidl_message_type_support_t *type_support, const char *topic,
  const rcl_publisher_options_t *options) {
  const int entry_errno = errno;
  using Function = decltype(&rcl_publisher_init);
  static Function real = next<Function>("rcl_publisher_init");
  errno = entry_errno;
  const rcl_ret_t ret = real(publisher, node, type_support, topic, options);
  int saved_errno = errno;
  if (ret == RCL_RET_OK && output[0]) {
    // A later publisher can reuse an address. Never keep an old topic/ABI tag.
    for (uint64_t i = 0; i < publisher_capacity; ++i)
      if (publishers[i].handle.load() == publisher)
        publishers[i].active.store(false, std::memory_order_release);
    const char *resolved = rcl_publisher_get_topic_name(publisher);
    if (resolved && std::strcmp(resolved, "/joint_states") == 0) {
      const uint64_t slot = publisher_count.fetch_add(1);
      if (slot < publisher_capacity) {
        // The pointer identity of the generated C++ type-support handle
        // verifies C++ JointState ABI before reading Header fields.
        publishers[slot].cpp_joint_state = type_support ==
          rosidl_typesupport_cpp::get_message_type_support_handle<sensor_msgs::msg::JointState>();
        publishers[slot].handle.store(publisher, std::memory_order_release);
        publishers[slot].active.store(true, std::memory_order_release);
      }
    }
  }
  errno = saved_errno;
  return ret;
}

extern "C" rcl_ret_t rcl_publisher_fini(rcl_publisher_t *publisher, rcl_node_t *node) {
  const int entry_errno = errno;
  using Function = decltype(&rcl_publisher_fini);
  static Function real = next<Function>("rcl_publisher_fini");
  errno = entry_errno;
  const rcl_ret_t ret = real(publisher, node);
  int saved_errno = errno;
  if (ret == RCL_RET_OK)
    for (uint64_t i = 0; i < publisher_capacity; ++i)
      if (publishers[i].handle.load() == publisher)
        publishers[i].active.store(false, std::memory_order_release);
  errno = saved_errno;
  return ret;
}

extern "C" rcl_ret_t rcl_publish(
  const rcl_publisher_t *publisher, const void *message,
  rmw_publisher_allocation_t *allocation) {
  const int entry_errno = errno;
  using Function = decltype(&rcl_publish);
  static Function real = next<Function>("rcl_publish");
  if (!output[0] || !message || !selected(publisher)) {
    if (output[0]) unmatched_topic_calls.fetch_add(1, std::memory_order_relaxed);
    errno = entry_errno;
    return real(publisher, message, allocation);
  }
  const uint64_t slot = calls.fetch_add(1, std::memory_order_relaxed);
  if (slot >= capacity) { errno = entry_errno; return real(publisher, message, allocation); }
  auto &r = records[slot];
  r.handle = reinterpret_cast<uintptr_t>(publisher);
  r.header_ns = message_stamp(message);
  r.tid = syscall(SYS_gettid);
  r.enter_ns = clock_ns(CLOCK_MONOTONIC);
  // Preserve all three arguments, the actual result, and original errno.
  errno = entry_errno;
  const rcl_ret_t ret = real(publisher, message, allocation);
  int saved_errno = errno;
  r.return_ns = clock_ns(CLOCK_MONOTONIC);
  r.ret = ret;
  r.header_after_ns = message_stamp(message);
  r.complete.store(true, std::memory_order_release);
  errno = saved_errno;
  return ret;
}
