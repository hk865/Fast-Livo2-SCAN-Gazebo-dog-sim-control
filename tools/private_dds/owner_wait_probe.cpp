#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "probe_identity.hpp"
// Isolated owner observer. No global mutex-lock hook or algorithm mutation.
// Binary layouts are native little endian; field declarations below define schemas.
#include <fastdds/rtps/common/Locator.h>
#include <fastdds/rtps/reader/StatefulReader.h>
#include <rcl/publisher.h>
#include <rtps/messages/RTPSMessageGroup_t.hpp>
#include <memory>
#include <atomic>
#include <pthread.h>
#include <dlfcn.h>
#include <link.h>
#include <elf.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <cstdint>
#include <climits>
#include <chrono>
namespace eprosima {namespace fastdds {namespace rtps {class SharedMemTransport;}}}
namespace rt=eprosima::fastrtps::rtps;
namespace eprosima {namespace fastrtps {namespace rtps {class SendBuffersManager;}}}
namespace ow {
constexpr size_t PCAP=131072, UCAP=4096, HCAP=4096, RCAP=262144;
struct Poll {uint64_t mono,gen,B,runtime,delay,timeslices;uint32_t errors;char state;char syscall[192],wchan[64];};
struct Unlock {uint64_t mono,gen,tid,ptr,caller;int result,pad;};
struct Health {uint64_t begin,end,cpu_begin,cpu_end,tid,caller,send_id,transport,iterator,end_iterator,send_caller;int64_t request_sec,request_nsec;int result,error,locator_covered,locator_kind;uint32_t locator_port;unsigned char locator_address[16];};
struct Pub {uint64_t begin,end,publisher,message;int result,pad;};
Poll polls[PCAP]{};Unlock unlocks[UCAP]{};Health health[HCAP]{};Pub pubs[RCAP]{};
std::atomic<bool> ur[UCAP]{},hr[HCAP]{},pr[RCAP]{};
std::atomic<uint64_t> pn{0},un{0},hn{0},rn{0},udrop{0},hdrop{0},rdrop{0},pdrop{0},inflight{0},send_sequence{0},guarded_base{0};
std::atomic<bool> enabled{false},closing{false},guard_ok{false};
std::atomic<uint64_t> arm_seq{0},arm_ptr{0},arm_gen{0};
uint64_t owner=0;pthread_t observer{};bool started=false;int join_rc=0;uint64_t maps_bytes=0;int maps_truncated=0,maps_error=0;char prefix[2048]{};
using Sh=bool(*)(eprosima::fastdds::rtps::SharedMemTransport*,const rt::octet*,uint32_t,rt::LocatorsIterator*,rt::LocatorsIterator*,const std::chrono::steady_clock::time_point&);
using Sleep=int(*)(const timespec*,timespec*);using UnlockFn=int(*)(pthread_mutex_t*);
using Publish=decltype(&rcl_publish);
using TP=std::chrono::steady_clock::time_point;
using Buf=std::unique_ptr<rt::RTPSMessageGroup_t>;

using Sync=bool(*)(rt::StatefulReader*,rt::CDRMessage_t*,const eprosima::fastdds::rtps::Locators&,const eprosima::fastdds::rtps::Locators&,TP&);
using GetBuf=Buf(*)(rt::SendBuffersManager*,const rt::RTPSParticipantImpl*,const TP&);
std::atomic<Sync> real_sync{nullptr};std::atomic<GetBuf> real_getbuf{nullptr};
std::atomic<Sh> real_sh{nullptr};std::atomic<Sleep> real_sleep{nullptr};std::atomic<UnlockFn> real_unlock{nullptr};std::atomic<Publish> real_pub{nullptr};
uint64_t now(clockid_t id=CLOCK_MONOTONIC) noexcept {timespec t{};int saved=errno;clock_gettime(id,&t);errno=saved;return uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec;}
uint64_t tid() noexcept {int saved=errno;auto value=uint64_t(syscall(SYS_gettid));errno=saved;return value;}
uint64_t cached_tid() noexcept {static thread_local const uint64_t value=tid();return value;}
[[noreturn]] void fail(const char* reason) noexcept {const char* h="owner_wait_probe fail-closed: ";auto a=write(2,h,strlen(h));auto b=write(2,reason,strlen(reason));(void)a;(void)b;_exit(125);}
template<class F> F require(std::atomic<F>& slot,const char* name) noexcept {int saved=errno;F f=slot.load(std::memory_order_acquire);if(!f){f=reinterpret_cast<F>(dlsym(RTLD_NEXT,name));if(!f)fail(name);slot.store(f,std::memory_order_release);}errno=saved;return f;}
struct Context {uint64_t id=0,transport=0,caller=0;rt::LocatorsIterator* iterator=nullptr;rt::LocatorsIterator* end=nullptr;};
thread_local Context context{};
struct Active {bool yes=false;Active() noexcept {if(!enabled.load(std::memory_order_acquire)||closing.load(std::memory_order_acquire))return;inflight.fetch_add(1);if(closing.load()){inflight.fetch_sub(1);return;}yes=true;}~Active(){if(yes)inflight.fetch_sub(1);}};

// ACK2 is the actual three-argument member ABI (plus explicit this).
using Ack=void(*)(rt::StatefulReader*,const rt::WriterProxy*,rt::RTPSMessageSenderInterface*,bool);
std::atomic<Ack> real_ack{nullptr};
constexpr size_t ASCAP=131072, APCAP=131072, AUCAP=4096;
struct AckSpan {uint64_t id,tid,reader,participant,manager,A,M,I,S,begin,end,cpu_begin,cpu_end,caller;int completed,exception;};
struct AckPoll {uint64_t id,tid,reader,participant,A,M,I,S;Poll sample;uint64_t id_after;};
AckSpan ackspans[ASCAP]{};AckPoll ackpolls[APCAP]{};Unlock ackunlocks[AUCAP]{};
std::atomic<bool> asr[ASCAP]{},aur[AUCAP]{};
std::atomic<uint64_t> asn{0},apn{0},aun{0},asdrop{0},apdrop{0},audrop{0};
std::atomic<uint64_t> ack_calls{0},ack_parallel_miss{0},ack_nested_miss{0},ack_snapshot_miss{0},ack_poll_inconsistent{0},ack_stale_arm{0};
std::atomic<uint64_t> ack_slot_owner{0},ack_next_id{0},ack_id{0},ack_tid{0},ack_reader{0},ack_participant{0},ack_A{0},ack_M{0},ack_I{0},ack_S{0};
std::atomic<uint64_t> ack_arm_seq{0},ack_arm_ptr{0},ack_arm_id{0};
thread_local unsigned ack_depth=0;
uint64_t readptr(uint64_t base,unsigned offset) noexcept {uint64_t out{};memcpy(&out,reinterpret_cast<const void*>(base+offset),sizeof(out));return out;}
void set_ack_arm(uint64_t ptr,uint64_t id) noexcept {ack_arm_seq.fetch_add(1,std::memory_order_acq_rel);ack_arm_id.store(id,std::memory_order_relaxed);ack_arm_ptr.store(ptr,std::memory_order_relaxed);ack_arm_seq.fetch_add(1,std::memory_order_release);}
bool get_ack_arm(uint64_t ptr,uint64_t& id) noexcept {for(unsigned i=0;i<3;++i){auto a=ack_arm_seq.load(std::memory_order_acquire);if(a&1)continue;auto p=ack_arm_ptr.load(std::memory_order_relaxed);auto g=ack_arm_id.load(std::memory_order_relaxed);if(a==ack_arm_seq.load(std::memory_order_acquire)){id=g;if(!g||ack_id.load(std::memory_order_acquire)!=g){ack_stale_arm.fetch_add(1);return false;}return ptr&&ptr==p;}}return false;}
bool ack_snapshot(AckPoll& row) noexcept {
 auto id=ack_id.load(std::memory_order_acquire);if(!id)return false;
 row.id=id;row.tid=ack_tid.load(std::memory_order_relaxed);row.reader=ack_reader.load(std::memory_order_relaxed);row.participant=ack_participant.load(std::memory_order_relaxed);row.A=ack_A.load(std::memory_order_relaxed);row.M=ack_M.load(std::memory_order_relaxed);row.I=ack_I.load(std::memory_order_relaxed);row.S=ack_S.load(std::memory_order_relaxed);
 if(id!=ack_id.load(std::memory_order_acquire)){ack_snapshot_miss.fetch_add(1);return false;}return true;
}
struct AckScope {
 Active active;AckSpan row{};bool entered=false,selected=false,done=false;
 AckScope(rt::StatefulReader* reader,uint64_t caller) noexcept {
  int saved=errno;if(!active.yes){errno=saved;return;}entered=true;ack_calls.fetch_add(1);
  if(ack_depth++){ack_nested_miss.fetch_add(1);errno=saved;return;}
  uint64_t zero=0,t=tid();if(!ack_slot_owner.compare_exchange_strong(zero,t,std::memory_order_acq_rel)){ack_parallel_miss.fetch_add(1);errno=saved;return;}
  selected=true;row.id=ack_next_id.fetch_add(1)+1;row.tid=t;row.reader=reinterpret_cast<uint64_t>(reader);row.participant=readptr(row.reader,8);row.manager=readptr(row.participant,0x7d8);row.A=row.reader+0x188;row.M=row.manager;row.I=readptr(row.manager,0x98);row.S=row.participant+0x53c8;row.caller=caller;row.begin=now();row.cpu_begin=now(CLOCK_THREAD_CPUTIME_ID);
  ack_tid.store(t,std::memory_order_relaxed);ack_reader.store(row.reader,std::memory_order_relaxed);ack_participant.store(row.participant,std::memory_order_relaxed);ack_A.store(row.A,std::memory_order_relaxed);ack_M.store(row.M,std::memory_order_relaxed);ack_I.store(row.I,std::memory_order_relaxed);ack_S.store(row.S,std::memory_order_relaxed);ack_id.store(row.id,std::memory_order_release);errno=saved;
 }
 ~AckScope() noexcept {
  int saved=errno;if(selected){row.cpu_end=now(CLOCK_THREAD_CPUTIME_ID);row.end=now();row.completed=done;row.exception=!done;auto i=asn.fetch_add(1);if(i<ASCAP){ackspans[i]=row;asr[i].store(true,std::memory_order_release);}else asdrop.fetch_add(1);ack_id.store(0,std::memory_order_release);ack_slot_owner.store(0,std::memory_order_release);}if(entered)--ack_depth;errno=saved;
 }
};

// Independent chain schema. No old record layout is changed.
enum ChainPhase : uint64_t { CHAIN_ACK=1, CHAIN_SYNC=2, CHAIN_GETBUF=3, CHAIN_SHM=4 };
constexpr size_t CCAP=32768;
struct ChainSpan {uint64_t call_id,parent_id,ack_id,tid,phase,reader,participant,manager,transport,iterator,end_iterator,caller,begin,end,cpu_begin,cpu_end;int result,completed,exception,guid_covered;unsigned char reader_guid[16];};
struct HealthLink {uint64_t health_index,call_id,parent_id,ack_id,tid,phase;};
struct ChainTLS {uint64_t call_id=0,parent_id=0,ack_id=0,phase=0,reader=0,participant=0,manager=0,transport=0;};
thread_local ChainTLS chain{};
ChainSpan chainspans[CCAP]{};HealthLink healthlinks[HCAP]{};
std::atomic<bool> cr[CCAP]{},hlr[HCAP]{};
std::atomic<uint64_t> chain_next_id{0},chain_eligible{0},chain_retained{0},chain_drop{0},health_link_count{0};
std::atomic<uint64_t> phase_total[5]{},phase_selected[5]{},phase_eligible[5]{},phase_retained[5]{},phase_drop[5]{};
struct ChainScope {
 Active active;ChainTLS previous{};ChainSpan row{};uint64_t phase_ordinal=0;bool selected=false,done=false;int result=INT_MIN;
 ChainScope(uint64_t phase,bool wanted,uint64_t reader=0,uint64_t participant=0,uint64_t manager=0,uint64_t transport=0,uint64_t iterator=0,uint64_t end_iterator=0,uint64_t ack=0,uint64_t caller=0) noexcept {
  int saved=errno;if(!active.yes){errno=saved;return;}phase_total[phase].fetch_add(1,std::memory_order_relaxed);if(!wanted){errno=saved;return;}selected=true;phase_ordinal=phase_selected[phase].fetch_add(1,std::memory_order_relaxed)+1;previous=chain;
  row.call_id=chain_next_id.fetch_add(1,std::memory_order_relaxed)+1;row.parent_id=previous.call_id;row.ack_id=phase==CHAIN_ACK?ack:previous.ack_id;row.tid=cached_tid();row.phase=phase;row.reader=reader?reader:previous.reader;bool same_reader=!reader||reader==previous.reader;row.participant=participant?participant:(same_reader?previous.participant:0);row.manager=manager?manager:(same_reader?previous.manager:0);row.transport=transport?transport:previous.transport;row.iterator=iterator;row.end_iterator=end_iterator;row.caller=caller;
  if(row.reader){const auto& guid=reinterpret_cast<const rt::StatefulReader*>(row.reader)->getGuid();memcpy(row.reader_guid,guid.guidPrefix.value,12);memcpy(row.reader_guid+12,guid.entityId.value,4);row.guid_covered=1;}
  row.begin=now();row.cpu_begin=now(CLOCK_THREAD_CPUTIME_ID);chain={row.call_id,row.parent_id,row.ack_id,phase,row.reader,row.participant,row.manager,row.transport};errno=saved;
 }
 ~ChainScope() noexcept {
  int saved=errno;if(selected){row.cpu_end=now(CLOCK_THREAD_CPUTIME_ID);row.end=now();row.result=result;row.completed=done;row.exception=!done;chain=previous;if(phase_ordinal<=256||row.end-row.begin>=1000000ULL||!done){phase_eligible[row.phase].fetch_add(1,std::memory_order_relaxed);auto i=chain_eligible.fetch_add(1,std::memory_order_relaxed);if(i<CCAP){chainspans[i]=row;cr[i].store(true,std::memory_order_release);chain_retained.fetch_add(1);phase_retained[row.phase].fetch_add(1);}else{chain_drop.fetch_add(1);phase_drop[row.phase].fetch_add(1);}}}errno=saved;
 }
};
struct NoteCheck{uintptr_t base;bool found=false;};
int check_notes(dl_phdr_info* info,size_t,void* raw) noexcept {auto& c=*static_cast<NoteCheck*>(raw);if(uintptr_t(info->dlpi_addr)!=c.base)return 0;
 static const unsigned char wanted[20]={PRIVATE_DDS_BUILD_ID_BYTES};
 for(unsigned i=0;i<info->dlpi_phnum;++i){auto& ph=info->dlpi_phdr[i];if(ph.p_type!=PT_NOTE||ph.p_memsz>65536)continue;const auto* p=reinterpret_cast<const unsigned char*>(info->dlpi_addr+ph.p_vaddr);size_t n=ph.p_memsz;
  while(n>=sizeof(ElfW(Nhdr))){ElfW(Nhdr) h{};std::memcpy(&h,p,sizeof(h));size_t ns=(size_t(h.n_namesz)+3)&~size_t(3),ds=(size_t(h.n_descsz)+3)&~size_t(3);if(ns>n-sizeof(h)||ds>n-sizeof(h)-ns)break;
   if(h.n_type==NT_GNU_BUILD_ID&&h.n_namesz==4&&h.n_descsz==20&&!std::memcmp(p+sizeof(h),"GNU",4)&&!std::memcmp(p+sizeof(h)+ns,wanted,20))c.found=true;
   size_t used=sizeof(h)+ns+ds;p+=used;n-=used;
  }
 }return 1;}
void validate(void* const* functions,const uintptr_t* offsets,size_t count) noexcept {char expected[PATH_MAX]{};if(!realpath(PRIVATE_DDS_LIBRARY_PATH,expected))fail("expected realpath");uintptr_t base=0;
 for(size_t i=0;i<count;++i){Dl_info d{};char actual[PATH_MAX]{};if(!functions[i]||!dladdr(functions[i],&d)||!d.dli_fname||!realpath(d.dli_fname,actual)||std::strcmp(actual,expected))fail("actual library path");auto b=reinterpret_cast<uintptr_t>(d.dli_fbase);if(i&&b!=base)fail("method DSO base");base=b;if(reinterpret_cast<uintptr_t>(functions[i])!=base+offsets[i])fail("method ELF offset");}
 NoteCheck c{base,false};dl_iterate_phdr(check_notes,&c);if(!c.found)fail("actual ELF BuildID");guarded_base.store(base,std::memory_order_release);guard_ok.store(true,std::memory_order_release);
}
void set_arm(uint64_t ptr,uint64_t generation) noexcept {arm_seq.fetch_add(1,std::memory_order_acq_rel);arm_gen.store(generation,std::memory_order_relaxed);arm_ptr.store(ptr,std::memory_order_relaxed);arm_seq.fetch_add(1,std::memory_order_release);}
bool get_arm(uint64_t ptr,uint64_t& generation) noexcept {for(unsigned i=0;i<3;++i){auto s=arm_seq.load(std::memory_order_acquire);if(s&1)continue;auto p=arm_ptr.load(std::memory_order_relaxed);auto g=arm_gen.load(std::memory_order_relaxed);if(s==arm_seq.load(std::memory_order_acquire)){generation=g;return ptr&&ptr==p;}}return false;}
int proc_tid(uint64_t target,const char* name,char* buf,size_t cap) noexcept {char path[128];snprintf(path,sizeof(path),"/proc/self/task/%lu/%s",target,name);int f=open(path,O_RDONLY|O_CLOEXEC);if(f<0)return -1;auto n=read(f,buf,cap-1);close(f);if(n<0)return -1;buf[n]=0;return n;}

int proc(const char* name,char* buf,size_t cap) noexcept {return proc_tid(owner,name,buf,cap);}
void observe_ack() noexcept {
 AckPoll row{};if(!ack_snapshot(row)){set_ack_arm(0,0);return;}Poll& p=row.sample;p.mono=now();p.gen=row.id;char stat[4096]{},sched[128]{};
 if(proc_tid(row.tid,"syscall",p.syscall,sizeof(p.syscall))<0)p.errors|=1;
 if(proc_tid(row.tid,"wchan",p.wchan,sizeof(p.wchan))<0)p.errors|=2;
 if(proc_tid(row.tid,"schedstat",sched,sizeof(sched))<0)p.errors|=4;else {unsigned long long a{},b{},c{};if(sscanf(sched,"%llu %llu %llu",&a,&b,&c)==3){p.runtime=a;p.delay=b;p.timeslices=c;}else p.errors|=8;}
 if(proc_tid(row.tid,"stat",stat,sizeof(stat))<0)p.errors|=16;else {auto q=strrchr(stat,')');if(q&&q[1]==' ')p.state=q[2];else p.errors|=32;}
 row.id_after=ack_id.load(std::memory_order_acquire);if(row.id_after!=row.id){p.errors|=64;ack_poll_inconsistent.fetch_add(1);}
 unsigned long nr{},addr{},op{},expect{};if(!(p.errors&(1|64))&&sscanf(p.syscall,"%lu %lx %lx %lx",&nr,&addr,&op,&expect)==4&&nr==SYS_futex&&(op&127)==0&&expect==2){p.B=addr;set_ack_arm(p.B,row.id);}else set_ack_arm(0,0);
 auto n=apn.fetch_add(1);if(n<APCAP)ackpolls[n]=row;else apdrop.fetch_add(1);
}
void* observe(void*) noexcept {uint64_t gen=0,last_B=0;while(!closing.load(std::memory_order_acquire)){
 timespec t{0,20000000};syscall(SYS_nanosleep,&t,nullptr);if(closing.load())break;
 Poll p{};p.mono=now();char stat[4096]{},sched[128]{};
 if(proc("syscall",p.syscall,sizeof(p.syscall))<0)p.errors|=1;
 if(proc("wchan",p.wchan,sizeof(p.wchan))<0)p.errors|=2;
 if(proc("schedstat",sched,sizeof(sched))<0)p.errors|=4;else {unsigned long long a{},b{},c{};if(sscanf(sched,"%llu %llu %llu",&a,&b,&c)==3){p.runtime=a;p.delay=b;p.timeslices=c;}else p.errors|=8;}
 if(proc("stat",stat,sizeof(stat))<0)p.errors|=16;else {auto q=strrchr(stat,')');if(q&&q[1]==' ')p.state=q[2];else p.errors|=32;}
 unsigned long nr{},addr{},op{},expect{};if(!(p.errors&1)&&sscanf(p.syscall,"%lu %lx %lx %lx",&nr,&addr,&op,&expect)==4&&nr==SYS_futex&&(op&127)==0&&expect==2){p.B=addr;if(p.B!=last_B)++gen;p.gen=gen;set_arm(p.B,gen);}else set_arm(0,gen);
 last_B=p.B;auto n=pn.fetch_add(1);if(n<PCAP)polls[n]=p;else pdrop.fetch_add(1);observe_ack();
 }set_arm(0,0);set_ack_arm(0,0);return nullptr;}
void dump(const char* suffix,const void* data,size_t bytes) noexcept {char path[2200];snprintf(path,sizeof(path),"%s.%s",prefix,suffix);int f=open(path,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);if(f<0)return;auto p=static_cast<const char*>(data);while(bytes){auto n=write(f,p,bytes);if(n<=0)break;p+=n;bytes-=size_t(n);}close(f);}
void save_maps() noexcept {char path[2200];snprintf(path,sizeof(path),"%s.maps",prefix);int input=open("/proc/self/maps",O_RDONLY|O_CLOEXEC);if(input<0){maps_error=errno;return;}int output=open(path,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);if(output<0){maps_error=errno;close(input);return;}char b[8192];constexpr size_t limit=524288;while(maps_bytes<limit){auto n=read(input,b,sizeof(b));if(n<0){maps_error=errno;break;}if(!n)break;size_t count=size_t(n);if(count>limit-maps_bytes)count=limit-maps_bytes;size_t offset=0;while(offset<count){auto k=write(output,b+offset,count-offset);if(k<=0){maps_error=errno;break;}offset+=size_t(k);}maps_bytes+=offset;if(maps_error)break;}if(maps_bytes==limit){char c;maps_truncated=read(input,&c,1)>0;}close(input);close(output);}
template<class T,size_t N> void rows(const char* suffix,T(&data)[N],std::atomic<bool>(&ready)[N],uint64_t requested) noexcept {char path[2200];snprintf(path,sizeof(path),"%s.%s",prefix,suffix);int f=open(path,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);if(f<0)return;size_t n=requested<N?requested:N;for(size_t i=0;i<n;++i){if(!ready[i].load(std::memory_order_acquire))continue;auto p=reinterpret_cast<const char*>(&data[i]);size_t left=sizeof(T);while(left){auto k=write(f,p,left);if(k<=0)break;p+=k;left-=size_t(k);}}close(f);}
}
#define SYNC "_ZN8eprosima8fastrtps4rtps14StatefulReader13send_sync_ntsEPNS1_12CDRMessage_tERKNS_7fastdds4rtps8LocatorsES9_RNSt6chrono10time_pointINSA_3_V212steady_clockENSA_8durationIlSt5ratioILl1ELl1000000000EEEEEE"
#define GETBUF "_ZN8eprosima8fastrtps4rtps18SendBuffersManager10get_bufferEPKNS1_19RTPSParticipantImplERKNSt6chrono10time_pointINS6_3_V212steady_clockENS6_8durationIlSt5ratioILl1ELl1000000000EEEEEE"
extern "C" bool sync_chain(rt::StatefulReader*,rt::CDRMessage_t*,const eprosima::fastdds::rtps::Locators&,const eprosima::fastdds::rtps::Locators&,ow::TP&) asm(SYNC);
extern "C" ow::Buf getbuf_chain(rt::SendBuffersManager*,const rt::RTPSParticipantImpl*,const ow::TP&) asm(GETBUF);
extern "C" bool sync_chain(rt::StatefulReader* self,rt::CDRMessage_t* message,const eprosima::fastdds::rtps::Locators& begin,const eprosima::fastdds::rtps::Locators& end,ow::TP& deadline){auto fn=ow::require(ow::real_sync,SYNC);bool wanted=ow::chain.call_id||ow::cached_tid()==ow::owner;ow::ChainScope scope(ow::CHAIN_SYNC,wanted,reinterpret_cast<uint64_t>(self),0,0,0,0,0,0,reinterpret_cast<uint64_t>(__builtin_return_address(0)));bool result=fn(self,message,begin,end,deadline);scope.result=result;scope.done=true;return result;}
extern "C" ow::Buf getbuf_chain(rt::SendBuffersManager* self,const rt::RTPSParticipantImpl* participant,const ow::TP& deadline){auto fn=ow::require(ow::real_getbuf,GETBUF);bool wanted=ow::chain.call_id||ow::cached_tid()==ow::owner;ow::ChainScope scope(ow::CHAIN_GETBUF,wanted,0,reinterpret_cast<uint64_t>(participant),reinterpret_cast<uint64_t>(self),0,0,0,0,reinterpret_cast<uint64_t>(__builtin_return_address(0)));auto result=fn(self,participant,deadline);scope.result=bool(result);scope.done=true;return result;}
#define ACK2 "_ZN8eprosima8fastrtps4rtps14StatefulReader12send_acknackEPKNS1_11WriterProxyEPNS1_26RTPSMessageSenderInterfaceEb"
extern "C" void ack2(rt::StatefulReader*,const rt::WriterProxy*,rt::RTPSMessageSenderInterface*,bool) asm(ACK2);
extern "C" void ack2(rt::StatefulReader* self,const rt::WriterProxy* proxy,rt::RTPSMessageSenderInterface* sender,bool final_flag){auto fn=ow::require(ow::real_ack,ACK2);ow::AckScope scope(self,reinterpret_cast<uint64_t>(__builtin_return_address(0)));ow::ChainScope nested(ow::CHAIN_ACK,true,reinterpret_cast<uint64_t>(self),scope.selected?scope.row.participant:0,scope.selected?scope.row.manager:0,0,0,0,scope.selected?scope.row.id:0,reinterpret_cast<uint64_t>(__builtin_return_address(0)));fn(self,proxy,sender,final_flag);nested.done=true;scope.done=true;}
#define SHMSEND "_ZN8eprosima7fastdds4rtps18SharedMemTransport4sendEPKhjPNS1_16LocatorsIteratorES6_RKNSt6chrono10time_pointINS7_3_V212steady_clockENS7_8durationIlSt5ratioILl1ELl1000000000EEEEEE"
extern "C" bool shm(eprosima::fastdds::rtps::SharedMemTransport*,const rt::octet*,uint32_t,rt::LocatorsIterator*,rt::LocatorsIterator*,const std::chrono::steady_clock::time_point&) asm(SHMSEND);
extern "C" bool shm(eprosima::fastdds::rtps::SharedMemTransport* self,const rt::octet* bytes,uint32_t n,rt::LocatorsIterator* it,rt::LocatorsIterator* end,const std::chrono::steady_clock::time_point& deadline){
 auto fn=ow::require(ow::real_sh,SHMSEND);ow::Active active;if(!active.yes)return fn(self,bytes,n,it,end,deadline);ow::ChainScope nested(ow::CHAIN_SHM,true,0,0,0,reinterpret_cast<uint64_t>(self),reinterpret_cast<uint64_t>(it),reinterpret_cast<uint64_t>(end),0,reinterpret_cast<uint64_t>(__builtin_return_address(0)));int saved=errno;auto old=ow::context;ow::context={ow::send_sequence.fetch_add(1)+1,reinterpret_cast<uint64_t>(self),reinterpret_cast<uint64_t>(__builtin_return_address(0)),it,end};errno=saved;
 try {bool result=fn(self,bytes,n,it,end,deadline);int e=errno;nested.result=result;nested.done=true;ow::context=old;errno=e;return result;}catch(...){int e=errno;ow::context=old;errno=e;throw;}}
extern "C" int pthread_mutex_unlock(pthread_mutex_t* mutex) noexcept {
 auto fn=ow::require(ow::real_unlock,"pthread_mutex_unlock");if(!ow::arm_ptr.load(std::memory_order_relaxed)&&!ow::ack_arm_ptr.load(std::memory_order_relaxed))return fn(mutex);
 ow::Active a;uint64_t gen{},ack_id{};auto ptr=reinterpret_cast<uint64_t>(mutex);bool match=a.yes&&ow::get_arm(ptr,gen);bool ack_match=a.yes&&ow::get_ack_arm(ptr,ack_id);auto caller=reinterpret_cast<uint64_t>(__builtin_return_address(0));int rc=fn(mutex);int e=errno;
 if(rc==0&&(match||ack_match)){auto mono=ow::now(),t=ow::tid();if(match){auto i=ow::un.fetch_add(1);if(i<ow::UCAP){ow::unlocks[i]={mono,gen,t,ptr,caller,rc,0};ow::ur[i].store(true,std::memory_order_release);}else ow::udrop.fetch_add(1);}if(ack_match){auto i=ow::aun.fetch_add(1);if(i<ow::AUCAP){ow::ackunlocks[i]={mono,ack_id,t,ptr,caller,rc,0};ow::aur[i].store(true,std::memory_order_release);}else ow::audrop.fetch_add(1);}}
 errno=e;return rc;
}
extern "C" int nanosleep(const timespec* request,timespec* remaining) {auto fn=ow::require(ow::real_sleep,"nanosleep");auto caller=reinterpret_cast<uintptr_t>(__builtin_return_address(0));if(!ow::enabled.load(std::memory_order_relaxed)||caller!=ow::guarded_base.load()+PRIVATE_DDS_HEALTH_RETURN_PC)return fn(request,remaining);ow::Active a;if(!a.yes)return fn(request,remaining);int entry=errno;auto i=ow::hn.fetch_add(1);if(i>=ow::HCAP){ow::hdrop.fetch_add(1);errno=entry;return fn(request,remaining);}ow::Health h{};h.tid=ow::tid();h.caller=caller;ow::HealthLink link{i,ow::chain.call_id,ow::chain.parent_id,ow::chain.ack_id,h.tid,ow::chain.phase};auto c=ow::context;h.send_id=c.id;h.transport=c.transport;h.iterator=reinterpret_cast<uint64_t>(c.iterator);h.end_iterator=reinterpret_cast<uint64_t>(c.end);h.send_caller=c.caller;if(request){h.request_sec=request->tv_sec;h.request_nsec=request->tv_nsec;}
 // Public const iterator interface only, on its owning thread. No pointer offsets.
 try {if(c.iterator&&c.end&&*c.iterator!=*c.end){const auto& loc=**c.iterator;h.locator_kind=loc.kind;h.locator_port=loc.port;memcpy(h.locator_address,loc.address,16);h.locator_covered=1;}}catch(...){h.locator_covered=-1;}
 h.begin=ow::now();h.cpu_begin=ow::now(CLOCK_THREAD_CPUTIME_ID);errno=entry;int rc=fn(request,remaining);int e=errno;h.cpu_end=ow::now(CLOCK_THREAD_CPUTIME_ID);h.end=ow::now();h.result=rc;h.error=e;ow::health[i]=h;ow::healthlinks[i]=link;ow::hlr[i].store(true,std::memory_order_release);ow::health_link_count.fetch_add(1);ow::hr[i].store(true,std::memory_order_release);errno=e;return rc;}
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* publisher,const void* message,rmw_publisher_allocation_t* allocation){auto fn=ow::require(ow::real_pub,"rcl_publish");static thread_local const uint64_t thread_id=ow::tid();if(thread_id!=ow::owner||!ow::enabled.load(std::memory_order_relaxed))return fn(publisher,message,allocation);ow::Active a;if(!a.yes)return fn(publisher,message,allocation);int e=errno;auto begin=ow::now();errno=e;auto rc=fn(publisher,message,allocation);e=errno;auto finish=ow::now();auto i=ow::rn.fetch_add(1);if(i<ow::RCAP){ow::pubs[i]={begin,finish,reinterpret_cast<uint64_t>(publisher),reinterpret_cast<uint64_t>(message),rc,0};ow::pr[i].store(true,std::memory_order_release);}else ow::rdrop.fetch_add(1);errno=e;return rc;}

void dump_ack_observations() noexcept {
 ow::rows("ackspans.bin",ow::ackspans,ow::asr,ow::asn.load());ow::rows("ackunlocks.bin",ow::ackunlocks,ow::aur,ow::aun.load());if(ow::join_rc==0)ow::dump("ackpolls.bin",ow::ackpolls,(ow::apn.load()<ow::APCAP?ow::apn.load():ow::APCAP)*sizeof(ow::AckPoll));
 char b[1800];int n=snprintf(b,sizeof(b),"{\"schema\":\"owner_wait_ack2/v1\",\"ack_calls\":%lu,\"parallel_miss\":%lu,\"nested_miss\":%lu,\"snapshot_miss\":%lu,\"poll_inconsistent\":%lu,\"stale_arm\":%lu,\"span_count\":%lu,\"span_drop\":%lu,\"poll_count\":%lu,\"poll_drop\":%lu,\"unlock_count\":%lu,\"unlock_drop\":%lu,\"span_size\":%zu,\"poll_size\":%zu,\"unlock_size\":%zu,\"active_id_at_cutoff\":%lu,\"observer_join_rc\":%d,\"sampling_generation_is_ACK_call_id\":true,\"continuous_ownership_proven\":false}\n",ow::ack_calls.load(),ow::ack_parallel_miss.load(),ow::ack_nested_miss.load(),ow::ack_snapshot_miss.load(),ow::ack_poll_inconsistent.load(),ow::ack_stale_arm.load(),ow::asn.load(),ow::asdrop.load(),ow::apn.load(),ow::apdrop.load(),ow::aun.load(),ow::audrop.load(),sizeof(ow::AckSpan),sizeof(ow::AckPoll),sizeof(ow::Unlock),ow::ack_id.load(),ow::join_rc);if(n>0&&size_t(n)<sizeof(b))ow::dump("ackstate.json",b,size_t(n));
}

void dump_chain_observations() noexcept {
 ow::rows("chainspans.bin",ow::chainspans,ow::cr,ow::chain_eligible.load());ow::rows("healthlinks.bin",ow::healthlinks,ow::hlr,ow::hn.load());
 char b[2400];size_t used=0;int n=snprintf(b,sizeof(b),"{\"schema\":\"owner_wait_chain/v1\",\"capacity\":32768,\"first_per_phase\":256,\"slow_threshold_ns\":1000000,\"span_size\":%zu,\"health_link_size\":%zu,\"eligible\":%lu,\"retained\":%lu,\"drop\":%lu,\"health_link_count\":%lu,\"phases\":[",sizeof(ow::ChainSpan),sizeof(ow::HealthLink),ow::chain_eligible.load(),ow::chain_retained.load(),ow::chain_drop.load(),ow::health_link_count.load());if(n<=0||size_t(n)>=sizeof(b))return;used=size_t(n);
 for(unsigned phase=1;phase<=4;++phase){n=snprintf(b+used,sizeof(b)-used,"%s{\"phase\":%u,\"total\":%lu,\"selected\":%lu,\"eligible\":%lu,\"retained\":%lu,\"drop\":%lu}",phase==1?"":",",phase,ow::phase_total[phase].load(),ow::phase_selected[phase].load(),ow::phase_eligible[phase].load(),ow::phase_retained[phase].load(),ow::phase_drop[phase].load());if(n<=0||size_t(n)>=sizeof(b)-used)return;used+=size_t(n);}
 n=snprintf(b+used,sizeof(b)-used,"],\"find_port_covered\":false,\"open_port_covered\":false,\"health_is_exact_sleep_not_complete_check\":true,\"first_retained_is_not_cache_miss\":true}\n");if(n<=0||size_t(n)>=sizeof(b)-used)return;used+=size_t(n);ow::dump("chainstate.json",b,used);
}
__attribute__((constructor)) void initialize_owner_wait() noexcept {int saved=errno;const char* p=getenv("OWNER_WAIT_PREFIX");if(!p||!*p){errno=saved;return;}if(strlen(p)>=sizeof(ow::prefix))ow::fail("prefix too long");strcpy(ow::prefix,p);ow::owner=getpid();auto f=ow::require(ow::real_sh,SHMSEND);ow::require(ow::real_unlock,"pthread_mutex_unlock");ow::require(ow::real_sleep,"nanosleep");ow::require(ow::real_pub,"rcl_publish");auto ack=ow::require(ow::real_ack,ACK2);auto sync=ow::require(ow::real_sync,SYNC);auto getbuf=ow::require(ow::real_getbuf,GETBUF);void* functions[]={reinterpret_cast<void*>(f),reinterpret_cast<void*>(ack),reinterpret_cast<void*>(sync),reinterpret_cast<void*>(getbuf)};uintptr_t offsets[]={PRIVATE_DDS_SHM_OFFSET,PRIVATE_DDS_ACK_OFFSET,PRIVATE_DDS_SYNC_OFFSET,PRIVATE_DDS_GETBUF_OFFSET};ow::validate(functions,offsets,4);ow::enabled.store(true,std::memory_order_release);if(pthread_create(&ow::observer,nullptr,ow::observe,nullptr)!=0)ow::fail("observer create");ow::started=true;errno=saved;}
__attribute__((destructor)) void finish_owner_wait() noexcept {if(!ow::started)return;int saved=errno;ow::closing.store(true,std::memory_order_release);ow::enabled.store(false,std::memory_order_release);timespec join_deadline{};clock_gettime(CLOCK_REALTIME,&join_deadline);++join_deadline.tv_sec;ow::join_rc=pthread_timedjoin_np(ow::observer,nullptr,&join_deadline);for(unsigned i=0;i<100&&ow::inflight.load();++i){timespec t{0,1000000};syscall(SYS_nanosleep,&t,nullptr);}auto pending=ow::inflight.load();dump_ack_observations();dump_chain_observations();ow::save_maps();if(ow::join_rc==0)ow::dump("polls.bin",ow::polls,(ow::pn.load()<ow::PCAP?ow::pn.load():ow::PCAP)*sizeof(ow::Poll));ow::rows("unlocks.bin",ow::unlocks,ow::ur,ow::un.load());ow::rows("health.bin",ow::health,ow::hr,ow::hn.load());ow::rows("publish.bin",ow::pubs,ow::pr,ow::rn.load());char b[2400];int n=snprintf(b,sizeof(b),"{\"schema\":\"owner_wait_probe/v1\",\"owner_tid\":%lu,\"guard_ok\":%d,\"poll_count\":%lu,\"poll_drop\":%lu,\"unlock_count\":%lu,\"unlock_drop\":%lu,\"health_count\":%lu,\"health_drop\":%lu,\"publish_count\":%lu,\"publish_drop\":%lu,\"inflight_at_cutoff\":%lu,\"observer_join_rc\":%d,\"poll_size\":%zu,\"unlock_size\":%zu,\"health_size\":%zu,\"publish_size\":%zu,\"poll_interval_ns\":20000000,\"maps_bytes\":%lu,\"maps_truncated\":%d,\"maps_error\":%d,\"participant_S_covered\":false,\"user_stack_covered\":false,\"unlock_timestamp_is_post_return\":true,\"locator_is_current_iterator_not_peer_guarantee\":true}\n",ow::owner,int(ow::guard_ok.load()),ow::pn.load(),ow::pdrop.load(),ow::un.load(),ow::udrop.load(),ow::hn.load(),ow::hdrop.load(),ow::rn.load(),ow::rdrop.load(),pending,ow::join_rc,sizeof(ow::Poll),sizeof(ow::Unlock),sizeof(ow::Health),sizeof(ow::Pub),ow::maps_bytes,ow::maps_truncated,ow::maps_error);if(n>0&&size_t(n)<sizeof(b))ow::dump("state.json",b,size_t(n));errno=saved;}
