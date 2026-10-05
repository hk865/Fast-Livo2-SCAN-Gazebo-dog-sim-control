#!/usr/bin/env python3
"""Reviewer edge fixtures linked against actual new/baseline GridMap objects.

Requires plan_env's CMake regression targets to have been built. Reuses only
the test peer's map initialization/access helpers, not a reimplementation of
ray traversal. No ROS node or simulator is started.
"""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT/'ros2_ws/build/plan_env'
SOURCE = ROOT/'ros2_ws/src/plan_env'
MAIN = r'''
int main() {
  struct Check {std::string name; bool passed; double before; double after;};
  std::vector<Check> checks;
  auto check=[&](const char *name,bool result,double before,double after) {
    checks.push_back({name,result,before,after});
  };
  {
    GridMap g; GridMapTestPeer::configure(g);
    Eigen::Vector3d origin(.1,.1,.1), a(3.05,1.95,.05), b(3.95,1.05,.95), ghost(2.5,.5,.5);
    g.setOccupied(ghost); double before=GridMapTestPeer::odds(g,ghost);
    for(int i=0;i<4;i++) GridMapTestPeer::frame(g,origin,{a,b});
    check("shared endpoint second real segment clears exclusive voxel",
      g.getOccupancy(ghost)==0,before,GridMapTestPeer::odds(g,ghost));
  }
  {
    GridMap g; GridMapTestPeer::configure(g);
    Eigen::Vector3d origin(.5,.5,.5), ghost(2.5,.5,.5);
    for(int i=0;i<254;i++) GridMapTestPeer::frame(g,origin,{{.5,2.5,.5}});
    g.setOccupied(ghost); double before=GridMapTestPeer::odds(g,ghost);
    GridMapTestPeer::frame(g,origin,{{3.5,.5,.5}});
    double after=GridMapTestPeer::odds(g,ghost);
    check("frame 255 new ray must apply exactly one measured miss",
      std::abs(after-before-logit(.30))<1e-10,before,after);
  }
  {
    GridMap a,b; GridMapTestPeer::configure(a); GridMapTestPeer::configure(b);
    Eigen::Vector3d origin(.5,.5,.5), endpoint(3.5,.5,.5), ghost(1.5,.5,.5);
    a.setOccupied(ghost);b.setOccupied(ghost);
    GridMapTestPeer::frame(a,origin,{endpoint});
    GridMapTestPeer::frame(b,origin,std::vector<Eigen::Vector3d>(1000,endpoint));
    double single=GridMapTestPeer::odds(a,ghost), duplicate=GridMapTestPeer::odds(b,ghost);
    check("duplicate rays do not multiply free evidence",
      std::abs(single-duplicate)<1e-12 && std::abs(single-logit(.98)-logit(.30))<1e-10,single,duplicate);
  }
  bool passed=true;std::cout<<"{\"checks\":[";
  for(size_t i=0;i<checks.size();i++) {
    if(i)std::cout<<",";
    const auto &c=checks[i]; passed&=c.passed;
    std::cout<<"{\"name\":\""<<c.name<<"\",\"passed\":"<<(c.passed?"true":"false")
      <<",\"before\":"<<std::setprecision(17)<<c.before<<",\"after\":"<<c.after<<"}";
  }
  std::cout<<"],\"passed\":"<<(passed?"true":"false")<<"}\n";
  return passed?0:1;
}
'''


def main():
    peer = (SOURCE/'test/raycast_regression.cpp').read_text().split('int main(', 1)[0]
    flags = (BUILD/'CMakeFiles/raycast_regression.dir/flags.make').read_text()
    options = []
    for line in flags.splitlines():
        if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
            options.extend(shlex.split(line.split('=', 1)[1]))
    report = {'scope': 'independent edge assertions executed against actual compiled GridMap, no ROS or Gazebo',
              'fixture_sha256': hashlib.sha256((peer+MAIN).encode()).hexdigest(),
              'production_grid_map_sha256': hashlib.sha256((SOURCE/'src/grid_map.cpp').read_bytes()).hexdigest(),
              'production_static_library_sha256': hashlib.sha256((BUILD/'libplan_env.a').read_bytes()).hexdigest(),
              'variants': {}}
    with tempfile.TemporaryDirectory(prefix='demo_grid_edges_') as temporary:
        tmp = Path(temporary); source = tmp/'review.cpp'; obj = tmp/'review.o'
        source.write_text(peer+MAIN)
        compile_result = subprocess.run(['/usr/bin/c++', *options, '-c', str(source), '-o', str(obj)],
            text=True, capture_output=True, timeout=60)
        if compile_result.returncode:
            raise RuntimeError(compile_result.stderr[-4000:])
        for target in ('raycast_regression', 'raycast_regression_baseline'):
            command = shlex.split((BUILD/f'CMakeFiles/{target}.dir/link.txt').read_text())
            old_obj = f'CMakeFiles/{target}.dir/test/raycast_regression.cpp.o'
            command[command.index(old_obj)] = str(obj)
            binary = tmp/target
            command[command.index('-o')+1] = str(binary)
            subprocess.run(command, cwd=BUILD, capture_output=True, text=True, timeout=30, check=True)
            result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=20)
            data = json.loads(result.stdout.strip().splitlines()[-1])
            data['returncode'] = result.returncode
            data['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
            report['variants'][target] = data
    output = ROOT/'test_results/review_grid_map_edges.json'
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['variants']['raycast_regression']['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
