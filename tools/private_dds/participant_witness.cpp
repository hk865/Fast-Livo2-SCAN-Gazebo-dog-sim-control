// Creation-only witness: no DDS DT_NEEDED, QoS changes, or message-path hooks.
// Genuine public factory/getter ABI. No guessed object offsets or old build ID.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
// Factory/participant/QoS headers instantiate static Time_t constants and would
// force libfastrtps constructors at load time. Only GUID storage is dereferenced.
#include <fastdds/rtps/common/Guid.h>
#include <openssl/evp.h>
#include <atomic>
#include <mutex>
#include <dlfcn.h>
#include <link.h>
#include <elf.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <stdexcept>
#include <bits/cxxabi_forced.h>  // GNU forced unwind must never be swallowed.

// Exact public types from the installed 2.14.6 declarations; these classes are
// only passed as pointers/references. No layout, construction or method call is
// defined locally. DomainId_t is uint32_t in fastrtps/types/TypesBase.h:32.
namespace eprosima { namespace fastdds { namespace dds {
class DomainParticipantFactory;
class DomainParticipant;
class DomainParticipantQos;
class DomainParticipantListener;
class StatusMask;
using DomainId_t = uint32_t;
}}}
static_assert(eprosima::fastrtps::rtps::GuidPrefix_t::size == 12, "GUID prefix ABI");
static_assert(eprosima::fastrtps::rtps::EntityId_t::size == 4, "GUID entity ABI");
namespace dd=eprosima::fastdds::dds;
namespace rt=eprosima::fastrtps::rtps;
namespace witness {
constexpr unsigned CAP=256;
std::atomic<unsigned> ordinal{0};
thread_local unsigned depth=0;
struct Depth {Depth(){++depth;}~Depth(){--depth;}};
[[noreturn]] void fail(const char* reason) {
 const char h[]="private DDS participant witness rejected: ";
 auto a=write(2,h,sizeof(h)-1);auto b=write(2,reason,strlen(reason));auto c=write(2,"\n",1);
 (void)a;(void)b;(void)c;_exit(125);
}
template<class F> auto guarded(F&& operation) -> decltype(operation()) {
 try { return operation(); }
 catch (const __cxxabiv1::__forced_unwind&) { throw; }
 catch (const std::exception& error) { fail(error.what()); }
 catch (...) { fail("non-standard witness exception"); }
}
std::string hex(const unsigned char* p,size_t n) {
 static const char digits[]="0123456789abcdef";std::string s;s.reserve(2*n);
 for(size_t i=0;i<n;++i){s+=digits[p[i]>>4];s+=digits[p[i]&15];}return s;
}
std::string quote(const std::string& s) {
 std::ostringstream out;out<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')out<<'\\'<<char(c);else if(c<32){char b[7];snprintf(b,sizeof(b),"\\u%04x",c);out<<b;}else out<<char(c);}out<<'"';return out.str();
}
std::string canonical(const std::string& path) {
 if(path.empty()||path[0]!='/')fail("required absolute path");char p[PATH_MAX];if(!realpath(path.c_str(),p))fail("realpath");return p;
}
std::string small_file(const std::string& path,size_t cap) {
 std::ifstream in(path,std::ios::binary);if(!in)fail("cannot read required file");
 std::string s;char b[1024];while(in){in.read(b,sizeof(b));s.append(b,size_t(in.gcount()));if(s.size()>cap)fail("bounded file exceeded");}return s;
}
struct Config {std::string library,sha,build_id,directory,descriptor,descriptor_sha,boot;uint64_t ticks=0;struct stat file{};void* handle=nullptr;};
Config config;std::once_flag once;
std::string sha256_file(const std::string& path,struct stat* identity=nullptr) {
 int fd=open(path.c_str(),O_RDONLY|O_CLOEXEC);if(fd<0)fail("cannot open hash input");struct stat before{},after{};
 if(fstat(fd,&before)||!S_ISREG(before.st_mode)||before.st_size<0||before.st_size>128*1024*1024)fail("hash input not bounded regular file");
 EVP_MD_CTX* ctx=EVP_MD_CTX_new();if(!ctx||EVP_DigestInit_ex(ctx,EVP_sha256(),nullptr)!=1)fail("SHA init");
 char b[65536];ssize_t n;while((n=read(fd,b,sizeof(b)))>0){if(EVP_DigestUpdate(ctx,b,size_t(n))!=1)fail("SHA update");}if(n<0)fail("SHA read");
 if(fstat(fd,&after)||before.st_dev!=after.st_dev||before.st_ino!=after.st_ino||before.st_size!=after.st_size||before.st_mtim.tv_sec!=after.st_mtim.tv_sec||before.st_mtim.tv_nsec!=after.st_mtim.tv_nsec)fail("hash input changed");
 unsigned char digest[EVP_MAX_MD_SIZE];unsigned length=0;if(EVP_DigestFinal_ex(ctx,digest,&length)!=1||length!=32)fail("SHA final");EVP_MD_CTX_free(ctx);close(fd);if(identity)*identity=after;return hex(digest,length);
}
bool lower_hex(const std::string& s,size_t size) {if(s.size()!=size)return false;for(char c:s)if(!((c>='0'&&c<='9')||(c>='a'&&c<='f')))return false;return true;}
void initialize() {
 const char* d=getenv("PRIVATE_DDS_WITNESS_DESCRIPTOR");const char* out=getenv("PRIVATE_DDS_WITNESS_DIR");if(!d||!out)fail("descriptor and output directory are required when Factory is invoked");
 config.descriptor=canonical(d);config.directory=canonical(out);struct stat dir{};if(stat(config.directory.c_str(),&dir)||!S_ISDIR(dir.st_mode))fail("output is not directory");
 std::istringstream input(small_file(config.descriptor,8192));std::map<std::string,std::string> fields;std::string line;
 while(std::getline(input,line)){if(!line.empty()&&line.back()=='\r')line.pop_back();if(line.empty()||line[0]=='#')continue;auto pos=line.find('=');if(pos==std::string::npos||!fields.emplace(line.substr(0,pos),line.substr(pos+1)).second)fail("invalid/duplicate descriptor field");}
 if(fields.size()!=4||fields["schema"]!="private_fastdds_descriptor/v1")fail("descriptor schema/fields");
 config.library=canonical(fields["library"]);config.sha=fields["sha256"];config.build_id=fields["build_id"];
 if(!lower_hex(config.sha,64)||config.build_id.empty()||config.build_id.size()>128||config.build_id.size()%2||!lower_hex(config.build_id,config.build_id.size()))fail("required SHA/build ID format");
 if(sha256_file(config.library,&config.file)!=config.sha)fail("private DDS SHA mismatch");config.descriptor_sha=sha256_file(config.descriptor);
 std::string statline=small_file("/proc/self/stat",8192);auto end=statline.rfind(')');if(end==std::string::npos)fail("process stat");std::istringstream st(statline.substr(end+2));std::string field;for(unsigned i=0;i<20;++i){if(!(st>>field))fail("process identity");}config.ticks=std::stoull(field);
 config.boot=small_file("/proc/sys/kernel/random/boot_id",128);while(!config.boot.empty()&&(config.boot.back()=='\n'||config.boot.back()=='\r'))config.boot.pop_back();
 // RTLD_NOLOAD never installs DDS into a non-DDS process; only retain an already loaded handle.
 config.handle=dlopen(config.library.c_str(),RTLD_NOLOAD|RTLD_LAZY);if(!config.handle)fail("expected private DDS is not loaded; RTLD_LOCAL fallback unavailable");
}
void ensure() {int saved=errno;guarded([] {std::call_once(once,initialize);});errno=saved;}
struct Notes {uintptr_t base;std::string id;bool found=false;bool wrong_copy=false;};
int inspect_phdr(dl_phdr_info* info,size_t,void* raw) {
 auto& n=*static_cast<Notes*>(raw);const char* name=info->dlpi_name?info->dlpi_name:"";const char* base=strrchr(name,'/');base=base?base+1:name;
 if(strstr(base,"libfastrtps.so")||strstr(base,"libfastdds.so")){
  char path[PATH_MAX];if(!realpath(name,path)||config.library!=path)n.wrong_copy=true;
 }
 if(uintptr_t(info->dlpi_addr)!=n.base)return 0;
 for(unsigned i=0;i<info->dlpi_phnum;++i){const auto& ph=info->dlpi_phdr[i];if(ph.p_type!=PT_NOTE||ph.p_memsz>65536)continue;auto p=reinterpret_cast<const unsigned char*>(info->dlpi_addr+ph.p_vaddr);size_t left=ph.p_memsz;
  while(left>=sizeof(ElfW(Nhdr))){ElfW(Nhdr) h{};memcpy(&h,p,sizeof(h));size_t a=(size_t(h.n_namesz)+3)&~size_t(3),b=(size_t(h.n_descsz)+3)&~size_t(3);if(a>left-sizeof(h)||b>left-sizeof(h)-a)break;
   if(h.n_type==NT_GNU_BUILD_ID&&h.n_namesz==4&&h.n_descsz>0&&h.n_descsz<=64&&!memcmp(p+sizeof(h),"GNU",4)){n.id=hex(p+sizeof(h)+a,h.n_descsz);n.found=true;}size_t used=sizeof(h)+a+b;p+=used;left-=used;
  }
 }return 0;
}
void verify_function(void* fn) {
 Dl_info info{};if(!fn||!dladdr(fn,&info)||!info.dli_fname||canonical(info.dli_fname)!=config.library)fail("original function is not private DDS");
 Notes n{reinterpret_cast<uintptr_t>(info.dli_fbase),"",false,false};dl_iterate_phdr(inspect_phdr,&n);if(n.wrong_copy||!n.found||n.id!=config.build_id)fail("loaded BuildID mismatch or another DDS copy");
 std::istringstream maps(small_file("/proc/self/maps",1024*1024));std::string line;bool matched=false;
 while(std::getline(maps,line)){unsigned long lo{},hi{},off{},inode{};unsigned maj{},min{};char perm[8]{};
  if(sscanf(line.c_str(),"%lx-%lx %7s %lx %x:%x %lu",&lo,&hi,perm,&off,&maj,&min,&inode)==7&&lo<=n.base&&n.base<hi){matched=inode==uint64_t(config.file.st_ino)&&maj==major(config.file.st_dev)&&min==minor(config.file.st_dev)&&line.find(" (deleted)")==std::string::npos;break;}
 }if(!matched)fail("actual mapped inode/device mismatch");
}
template<class F> F original(std::atomic<F>& slot,const char* symbol) {
 return guarded([&]() -> F {int saved=errno;ensure();F fn=slot.load(std::memory_order_acquire);if(!fn){void* p=dlsym(RTLD_NEXT,symbol);if(!p)p=dlsym(config.handle,symbol);verify_function(p);fn=reinterpret_cast<F>(p);slot.store(fn,std::memory_order_release);}else verify_function(reinterpret_cast<void*>(fn));errno=saved;return fn;});
}
using Guid=const rt::GUID_t&(*)(const dd::DomainParticipant*);
using Domain=dd::DomainId_t(*)(const dd::DomainParticipant*);
std::atomic<Guid> guid_fn{nullptr};std::atomic<Domain> domain_fn{nullptr};
void emit(dd::DomainParticipant* participant,const char* overload) {
 guarded([&] {int saved=errno;unsigned index=ordinal.fetch_add(1);if(index>=CAP)fail("per-process factory-return witness cap exceeded");
 std::string guid;dd::DomainId_t domain=0;if(participant){auto get_guid=original(guid_fn,"_ZNK8eprosima7fastdds3dds17DomainParticipant4guidEv");auto get_domain=original(domain_fn,"_ZNK8eprosima7fastdds3dds17DomainParticipant13get_domain_idEv");const auto& g=get_guid(participant);unsigned char bytes[16];memcpy(bytes,g.guidPrefix.value,12);memcpy(bytes+12,g.entityId.value,4);guid=hex(bytes,16);domain=get_domain(participant);}
 timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t))fail("monotonic clock");
 std::ostringstream out;out<<"{\"schema\":\"private_fastdds_participant_init/v1\",\"pid\":"<<getpid()<<",\"start_ticks\":"<<config.ticks<<",\"boot_id\":"<<quote(config.boot)<<",\"ordinal\":"<<index<<",\"creation_depth\":"<<depth<<",\"mono_ns\":"<<(uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec)<<",\"created\":"<<(participant?"true":"false")<<",\"domain\":"<<domain<<",\"guid_hex\":"<<quote(guid)<<",\"overload\":"<<quote(overload)<<",\"library\":"<<quote(config.library)<<",\"sha256\":"<<quote(config.sha)<<",\"build_id\":"<<quote(config.build_id)<<",\"device\":"<<uint64_t(config.file.st_dev)<<",\"inode\":"<<uint64_t(config.file.st_ino)<<",\"descriptor_sha256\":"<<quote(config.descriptor_sha)<<",\"QoS_modified\":false,\"message_hooks\":false}\n";
 std::string data=out.str();if(data.size()>8192)fail("record cap");char path[PATH_MAX];int length=snprintf(path,sizeof(path),"%s/participant_%d_%lu_%u.json",config.directory.c_str(),getpid(),config.ticks,index);if(length<=0||size_t(length)>=sizeof(path))fail("record path");int fd=open(path,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);if(fd<0)fail("record creation");size_t offset=0;while(offset<data.size()){ssize_t n=write(fd,data.data()+offset,data.size()-offset);if(n<=0)fail("record write");offset+=size_t(n);}if(close(fd))fail("record close");errno=saved;});
}
#define CREATE "_ZN8eprosima7fastdds3dds24DomainParticipantFactory18create_participantEjRKNS1_20DomainParticipantQosEPNS1_25DomainParticipantListenerERKNS1_10StatusMaskE"
#define PROFILE "_ZN8eprosima7fastdds3dds24DomainParticipantFactory31create_participant_with_profileERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEPNS1_25DomainParticipantListenerERKNS1_10StatusMaskE"
#define DOMAIN_PROFILE "_ZN8eprosima7fastdds3dds24DomainParticipantFactory31create_participant_with_profileEjRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEPNS1_25DomainParticipantListenerERKNS1_10StatusMaskE"
using C=dd::DomainParticipant*(*)(dd::DomainParticipantFactory*,dd::DomainId_t,const dd::DomainParticipantQos&,dd::DomainParticipantListener*,const dd::StatusMask&);
using P=dd::DomainParticipant*(*)(dd::DomainParticipantFactory*,const std::string&,dd::DomainParticipantListener*,const dd::StatusMask&);
using D=dd::DomainParticipant*(*)(dd::DomainParticipantFactory*,dd::DomainId_t,const std::string&,dd::DomainParticipantListener*,const dd::StatusMask&);
std::atomic<C> c{nullptr};std::atomic<P> p{nullptr};std::atomic<D> d{nullptr};
}
extern "C" dd::DomainParticipant* witness_create(dd::DomainParticipantFactory*,dd::DomainId_t,const dd::DomainParticipantQos&,dd::DomainParticipantListener*,const dd::StatusMask&) asm(CREATE);
extern "C" dd::DomainParticipant* witness_profile(dd::DomainParticipantFactory*,const std::string&,dd::DomainParticipantListener*,const dd::StatusMask&) asm(PROFILE);
extern "C" dd::DomainParticipant* witness_domain_profile(dd::DomainParticipantFactory*,dd::DomainId_t,const std::string&,dd::DomainParticipantListener*,const dd::StatusMask&) asm(DOMAIN_PROFILE);
extern "C" dd::DomainParticipant* witness_create(dd::DomainParticipantFactory* factory,dd::DomainId_t domain,const dd::DomainParticipantQos& qos,dd::DomainParticipantListener* listener,const dd::StatusMask& mask) {witness::Depth scope;int before=errno;auto fn=witness::original(witness::c,CREATE);errno=before;auto result=fn(factory,domain,qos,listener,mask);int after=errno;witness::emit(result,"explicit_qos");errno=after;return result;}
extern "C" dd::DomainParticipant* witness_profile(dd::DomainParticipantFactory* factory,const std::string& profile,dd::DomainParticipantListener* listener,const dd::StatusMask& mask) {witness::Depth scope;int before=errno;auto fn=witness::original(witness::p,PROFILE);errno=before;auto result=fn(factory,profile,listener,mask);int after=errno;witness::emit(result,"profile");errno=after;return result;}
extern "C" dd::DomainParticipant* witness_domain_profile(dd::DomainParticipantFactory* factory,dd::DomainId_t domain,const std::string& profile,dd::DomainParticipantListener* listener,const dd::StatusMask& mask) {witness::Depth scope;int before=errno;auto fn=witness::original(witness::d,DOMAIN_PROFILE);errno=before;auto result=fn(factory,domain,profile,listener,mask);int after=errno;witness::emit(result,"domain_profile");errno=after;return result;}
