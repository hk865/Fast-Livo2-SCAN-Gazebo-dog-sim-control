// Test-only VIO team observer, never linked/preloaded by the production runner.
#include <dlfcn.h>
#include <omp.h>
#include <sched.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <mutex>
using Entry=void(*)(void*);using Parallel=void(*)(Entry,void*,unsigned,unsigned);
struct Payload{Entry fn;void*data;int team=0;int cpus[8]{};};
struct Record{unsigned requested;int actual;bool inverse;std::vector<int>cpus;};
static auto*records=new std::vector<Record>;static auto*gate=new std::mutex;
static void observed(void*raw){auto&p=*static_cast<Payload*>(raw);int team=omp_get_num_threads(),i=omp_get_thread_num();if(team>8||i>=8)std::abort();if(i==0)p.team=team;p.cpus[i]=sched_getcpu();p.fn(p.data);}
extern "C" void GOMP_parallel(Entry fn,void*data,unsigned requested,unsigned flags){
 static Parallel real=reinterpret_cast<Parallel>(dlsym(RTLD_NEXT,"GOMP_parallel"));if(!real)std::abort();Dl_info info{};
 bool target=dladdr(__builtin_return_address(0),&info)&&info.dli_sname&&std::strstr(info.dli_sname,"updateState");
 if(!target){real(fn,data,requested,flags);return;}Payload p{fn,data};real(observed,&p,requested,flags);
 std::lock_guard<std::mutex>lock(*gate);records->push_back({requested,p.team,bool(std::strstr(info.dli_sname,"Inverse")),std::vector<int>(p.cpus,p.cpus+p.team)});
}
__attribute__((destructor))static void save(){const char*path=std::getenv("V16_TEST_TEAM_RECORDS");if(!path)return;FILE*f=std::fopen(path,"w");if(!f)return;int i=0;for(const auto&r:*records){std::fprintf(f,"{\"sequence\":%d,\"requested\":%u,\"actual_team\":%d,\"inverse\":%s,\"cpus\":[",++i,r.requested,r.actual,r.inverse?"true":"false");for(size_t j=0;j<r.cpus.size();++j)std::fprintf(f,"%s%d",j?",":"",r.cpus[j]);std::fprintf(f,"]}\n");}std::fclose(f);}
