// TEST-ONLY observation. No team override and never used for production timing.
#include <dlfcn.h>
#include <omp.h>
#include <sched.h>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <vector>
#include <string>
#include <cfenv>
#include <xmmintrin.h>
using Entry=void(*)(void*);using Parallel=void(*)(Entry,void*,unsigned,unsigned);
struct Payload{Entry fn;void* data;int actual=0;int cpus[128]{},round[128]{};unsigned mxcsr[128]{};};
struct Record{std::string caller;unsigned requested;int actual;std::vector<int> cpus,round;std::vector<unsigned> mxcsr;};
static auto* rows=new std::vector<Record>;static auto* gate=new std::mutex;
static void observed(void* raw){auto& p=*static_cast<Payload*>(raw);const int index=omp_get_thread_num(),team=omp_get_num_threads();if(team>128)std::abort();if(index==0)p.actual=team;p.cpus[index]=sched_getcpu();p.round[index]=std::fegetround();p.mxcsr[index]=_mm_getcsr()&0xffc0u;p.fn(p.data);}
extern "C" void GOMP_parallel(Entry fn,void* data,unsigned requested,unsigned flags){static Parallel real=reinterpret_cast<Parallel>(dlsym(RTLD_NEXT,"GOMP_parallel"));if(!real)std::abort();Dl_info info{};dladdr(__builtin_return_address(0),&info);Payload p{fn,data};real(observed,&p,requested,flags);std::lock_guard<std::mutex> lock(*gate);rows->push_back({info.dli_sname?info.dli_sname:"unknown",requested,p.actual,std::vector<int>(p.cpus,p.cpus+p.actual),std::vector<int>(p.round,p.round+p.actual),std::vector<unsigned>(p.mxcsr,p.mxcsr+p.actual)});}
__attribute__((destructor))static void save(){const char* path=std::getenv("V15_GOMP_RECORDS");if(!path)return;FILE* file=std::fopen(path,"w");if(!file)return;for(size_t i=0;i<rows->size();++i){auto& r=(*rows)[i];std::fprintf(file,"{\"sequence\":%zu,\"caller\":\"%s\",\"requested\":%u,\"actual_team\":%d,\"cpus\":[",i+1,r.caller.c_str(),r.requested,r.actual);for(size_t j=0;j<r.cpus.size();++j)std::fprintf(file,"%s%d",j?",":"",r.cpus[j]);std::fprintf(file,"],\"fp_rounding_modes\":[");for(size_t j=0;j<r.round.size();++j)std::fprintf(file,"%s%d",j?",":"",r.round[j]);std::fprintf(file,"],\"mxcsr_control_masks\":[");for(size_t j=0;j<r.mxcsr.size();++j)std::fprintf(file,"%s%u",j?",":"",r.mxcsr[j]);std::fprintf(file,"]}\n");}std::fclose(file);}
