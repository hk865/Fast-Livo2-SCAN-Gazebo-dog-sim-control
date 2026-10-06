// Test-only observation through LD_PRELOAD; production library is unchanged.
#include <dlfcn.h>
#include <omp.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
using Entry=void(*)(void*);
using Parallel=void(*)(Entry,void*,unsigned,unsigned);
struct Payload{Entry function;void* data;unsigned requested;};
static pthread_mutex_t gate=PTHREAD_MUTEX_INITIALIZER;
static void observed(void* raw){
 auto& p=*static_cast<Payload*>(raw);
 pthread_mutex_lock(&gate);
 const char* path=std::getenv("GO2_GOMP_OBSERVATION");
 if(path){FILE* f=std::fopen(path,"a");if(f){std::fprintf(f,"tid=%ld team=%d index=%d requested=%u\n",syscall(SYS_gettid),omp_get_num_threads(),omp_get_thread_num(),p.requested);std::fclose(f);}}
 pthread_mutex_unlock(&gate);
 p.function(p.data);
}
extern "C" void GOMP_parallel(Entry function,void* data,unsigned requested,unsigned flags){
 static Parallel real=reinterpret_cast<Parallel>(dlsym(RTLD_NEXT,"GOMP_parallel"));
 if(!real)std::abort();
 Payload payload{function,data,requested};
 real(observed,&payload,requested,flags);
}
