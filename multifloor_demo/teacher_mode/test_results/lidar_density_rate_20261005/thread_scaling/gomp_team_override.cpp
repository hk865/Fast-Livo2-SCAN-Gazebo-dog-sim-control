// TEST ONLY: never part of a ROS/Gazebo production command or scope.
// Override only the existing independent-index BuildResidualListOMP region.
#include <dlfcn.h>
#include <omp.h>
#include <sched.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>
using Entry = void (*)(void*);
using Parallel = void (*)(Entry, void*, unsigned, unsigned);
struct Payload { Entry function; void* data; int actual=0; int cpus[128]{}; };
struct Record { unsigned requested, override_team; int actual; std::vector<int> cpus; };
// Keep storage alive until the ELF destructor writes it; no static-destructor
// order dependence. The process reclaims these test-only objects on exit.
static std::vector<Record>* records=new std::vector<Record>;
static std::mutex* record_gate=new std::mutex;
static void observed(void* raw) {
  auto& p = *static_cast<Payload*>(raw);
  const int team=omp_get_num_threads(), index=omp_get_thread_num();
  if(team>128 || index<0 || index>=128) std::abort();
  if(index==0) p.actual=team;
  p.cpus[index]=sched_getcpu();
  p.function(p.data);
}
extern "C" void GOMP_parallel(Entry function,void* data,unsigned requested,unsigned flags) {
  static Parallel real=reinterpret_cast<Parallel>(dlsym(RTLD_NEXT,"GOMP_parallel"));
  if(!real) std::abort();
  Dl_info caller{};
  const bool target=dladdr(__builtin_return_address(0),&caller) && caller.dli_sname
    && std::strstr(caller.dli_sname,"BuildResidualListOMP");
  if(!target) { real(function,data,requested,flags); return; }
  const char* raw=std::getenv("GO2_TEST_LIO_TEAM");
  const unsigned team=raw?std::strtoul(raw,nullptr,10):0;
  if(team!=1 && team!=2 && team!=4 && team!=8) std::abort();
  Payload payload{function,data};
  real(observed,&payload,team,flags);
  std::lock_guard<std::mutex> guard(*record_gate);
  records->push_back({requested,team,payload.actual,std::vector<int>(payload.cpus,payload.cpus+payload.actual)});
}
__attribute__((destructor)) static void save_teams() {
  const char* path=std::getenv("GO2_TEST_TEAM_RECORDS");
  if(!path) return;
  FILE* file=std::fopen(path,"w");
  if(!file) return;
  size_t i=0;
  for(const auto& row:*records) {
    std::fprintf(file,"{\"sequence\":%zu,\"compiled_requested\":%u,\"test_override\":%u,\"actual_team\":%d,\"cpus\":[",++i,row.requested,row.override_team,row.actual);
    for(size_t j=0;j<row.cpus.size();++j) std::fprintf(file,"%s%d",j?",":"",row.cpus[j]);
    std::fprintf(file,"]}\n");
  }
  std::fclose(file);
}
