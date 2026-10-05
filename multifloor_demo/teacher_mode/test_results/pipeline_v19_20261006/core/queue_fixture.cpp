#include "ordered_ingress.h"
#include <cassert>
#include <atomic>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <chrono>
struct Packet{uint64_t sequence=0;int kind=0;};
using Ptr=std::shared_ptr<Packet>;using Queue=fastlivo_ingress::OrderedQueue<Ptr>;
Ptr packet(int kind=0){auto p=std::make_shared<Packet>();p->kind=kind;return p;}
int main(){
 unsigned checks=0;
 {bool error=false;try{Queue q(0,1);}catch(const std::invalid_argument&){error=true;}assert(error);++checks;}
 {Queue q(512,512);for(int i=0;i<512;++i)assert(q.admit(packet(),1,0)==uint64_t(i+1));assert(q.stats().bytes==512&&q.stats().pending==512);Ptr p;for(int i=0;i<512;++i){assert(q.pop(p));assert(p->sequence==uint64_t(i+1));assert(q.stats().bytes==size_t(512-i));q.acknowledge(p->sequence);}q.close();assert(q.stats().accepted==q.stats().committed&&q.stats().bytes==0);++checks;}
 {Queue q(2,8);assert(q.admit(packet(),4,0)==1);assert(q.admit(packet(),4,0)==2);assert(!q.admit(packet(),1,0));auto s=q.stats();assert(s.rejected==1&&s.accepted==2&&s.pending==2&&s.bytes==8);assert(q.remaining().size()==2);bool error=false;Ptr p;try{q.pop(p);}catch(const std::runtime_error&){error=true;}assert(error);q.abandon();assert(q.stats().canceled==2&&q.remaining().size()==2);++checks;}
 {Queue q(2,8);assert(!q.admit(packet(),size_t(-1),0));assert(q.stats().accepted==0&&q.stats().rejected==1);++checks;}
 {Queue q(3,30);Ptr p;assert(q.admit(packet(2),10,2)==1);assert(q.admit(packet(3),10,3)==2);assert(q.admit(packet(),10,0)==3);assert(q.take(2,p));q.ready(2);assert(!q.pop(p));assert(q.take(1,p));q.ready(1);for(uint64_t i=1;i<=3;++i){assert(q.pop(p));assert(p->sequence==i);q.acknowledge(i);}q.close();++checks;}
 {Queue q(2,8);Ptr p;assert(q.admit(packet(),8,0)==1);assert(q.pop(p));assert(q.stats().bytes==8&&q.stats().inflight==1);assert(!q.admit(packet(),1,0));assert(q.stats().pending==1);++checks;}
 {Queue q(2,8);q.stop_admission();assert(!q.admit(packet(),1,0));assert(q.stats().closed_rejections==1);++checks;}
 {Queue q(2,8);assert(q.admit(packet(2),4,2));q.stop_admission();Ptr p;assert(q.wait_take(2,p));assert(p->sequence==1);q.ready(p->sequence);assert(!q.wait_take(2,p));assert(q.pop(p));q.acknowledge(1);q.close();++checks;}
 {Queue q(2,8);std::atomic<bool>ended=false;std::thread t([&]{Ptr p;assert(!q.wait_take(2,p));ended=true;});q.fail("decoder exception");t.join();assert(ended&&q.stats().failure=="decoder exception");++checks;}
 {Queue q(512,65536);std::thread cloud([&]{Ptr p;while(q.wait_take(2,p)){std::this_thread::sleep_for(std::chrono::microseconds(10));q.ready(p->sequence);}});std::thread image([&]{Ptr p;while(q.wait_take(3,p)){std::this_thread::sleep_for(std::chrono::microseconds(5));q.ready(p->sequence);}});std::thread receiver([&]{for(int i=0;i<128;++i)for(int kind:{2,3,1})assert(q.admit(packet(kind),16,kind));});receiver.join();q.stop_admission();cloud.join();image.join();assert(q.stats().ready==384);Ptr p;for(uint64_t i=1;i<=384;++i){assert(q.pop(p));assert(p->sequence==i);q.acknowledge(i);}q.close();auto s=q.stats();assert(s.accepted==384&&s.delivered==384&&s.committed==384&&!s.pending&&!s.canceled&&!s.closed_rejections&&!s.bytes);++checks;}
 {Queue q(2,8);Ptr p;q.admit(packet(),1,0);q.pop(p);bool error=false;try{q.acknowledge(2);}catch(const std::logic_error&){error=true;}assert(error);q.acknowledge(1);error=false;try{q.acknowledge(1);}catch(const std::logic_error&){error=true;}assert(error);++checks;}
 {Queue q(2,8);q.admit(packet(2),4,2);Ptr p;q.take(1,p);assert(q.stats().raw==0&&q.stats().inflight==1&&q.stats().pending==1&&q.stats().bytes==4);q.ready(1);assert(q.stats().inflight==0&&q.stats().ready==1);q.pop(p);q.acknowledge(1);++checks;}
 std::cout<<"pure_queue_cases="<<checks<<" accepted_busy_drain=384\n";
}
