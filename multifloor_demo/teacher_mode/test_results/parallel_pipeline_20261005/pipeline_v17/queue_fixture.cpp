#include "ordered_ingress.h"
#include <cassert>
#include <atomic>
#include <iostream>
#include <thread>
#include <vector>
struct Packet { uint64_t sequence=0,stamp=0,receipt=0; std::vector<int> payload; };
int main() {
  using Queue=fastlivo_ingress::OrderedQueue<Packet>;
  { Queue q(4,1024); Packet p; for(int i=0;i<4;++i)q.push({uint64_t(i),0,0,{}},100);
    q.push({},100); assert(q.stats().rejected==1);
    bool threw=false;try{q.pop(p);}catch(const std::runtime_error&){threw=true;}
    assert(threw);q.close();auto s=q.stats();assert(s.canceled==4&&s.delivered==0&&s.pending==0); }
  { Queue q(4,100);q.push({},101);auto s=q.stats();assert(s.accepted==0&&s.rejected==1); }
  { Queue q(4,100);q.push({},80);q.push({},21);assert(q.stats().rejected==1); }
  { Queue q(4,100);q.fail("decoder failure");Packet p;bool threw=false;
    try{q.pop(p);}catch(const std::runtime_error&e){threw=std::string(e.what())=="decoder failure";}
    assert(threw);q.close();q.close();q.push({},1);assert(q.stats().rejected==1); }
  constexpr uint64_t n=40000;
  Queue q(n,64*1024*1024);std::atomic<bool> done{false};
  std::thread producer([&]{for(uint64_t i=1;i<=n;++i){
    Packet p{i,1000000000+i*5,2000000000+i*7,{int(i),int(i+1)}};
    q.push(std::move(p),sizeof(Packet)+2*sizeof(int));
    if(i%64==0)std::this_thread::yield();
  }done=true;});
  uint64_t count=0;Packet p;
  while(!done || q.stats().pending){if(q.pop(p)){
    ++count;assert(p.sequence==count&&p.stamp==1000000000+count*5&&p.receipt==2000000000+count*7);
    assert(p.payload.size()==2&&p.payload[0]==int(count)&&p.payload[1]==int(count+1));
    if(count%131==0)std::this_thread::yield();
  }else std::this_thread::yield();}
  producer.join();q.close();auto s=q.stats();
  assert(count==n&&s.accepted==n&&s.delivered==n&&s.canceled==0&&s.rejected==0&&s.closed);
  std::cout << "PASS capacity_count capacity_bytes fail_closed exception shutdown sequence payload timestamp receipt concurrent_40000\n";
}
