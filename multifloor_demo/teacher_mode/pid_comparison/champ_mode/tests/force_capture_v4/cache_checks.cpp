#include "../../native/force_capture.hh"
#include <cassert>
#include <limits>
#include <iostream>
using champ_compare::ForceCapture;
int main() {
  ForceCapture capture;
  assert(!capture.Matches(1,5000000,5000000));
  for(auto v:capture.ForStep(1,5000000,5000000))assert(!v);
  ForceCapture::Values values;
  values.fill(22.);
  capture.Store(1,5000000,5000000,values);
  values.fill(0.); // Physics clears its command buffer, not the observer's copy.
  assert(capture.Matches(1,5000000,5000000));
  for(auto v:capture.ForStep(1,5000000,5000000))assert(v&&*v==22.);
  for(auto v:capture.ForStep(2,10000000,5000000))assert(!v);
  assert(!capture.Matches(1,5000001,5000000));
  assert(!capture.Matches(1,5000000,1000000));
  values.fill(std::nullopt);values[0]=0.;values[1]=std::numeric_limits<double>::infinity();
  capture.Store(2,10000000,5000000,values);
  assert(capture.Matches(2,10000000,5000000));
  assert(capture.values[0]&&*capture.values[0]==0.); // True zero is available.
  assert(!capture.values[1]&&!capture.values[2]);
  values.fill(24.);
  capture.Store(3,15000000,5000000,values);
  assert(*capture.values[0]==24.); // No clamp hides an exceedance.
  capture.Store(3,15000000,5000000,values);
  assert(!capture.Matches(3,15000000,5000000));
  capture.Store(2,10000000,5000000,values);
  assert(!capture.Matches(2,10000000,5000000));
  capture.Store(4,20000000,0,values);
  assert(!capture.Matches(4,20000000,0));
  std::cout<<"same-step cache: missing/true-zero/physics-clear/no-clamp/time/iteration/dt/repeated/backward PASS\n";
}
