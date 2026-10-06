"""Copy the reviewed native 200/250 Hz fixture into a new excluded profile test."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PRIOR = HERE.parent / "jtc_effective_response_native"
source = (PRIOR / "fixture.cpp").read_text()
assert hashlib.sha256(source.encode()).hexdigest() == "0ee625eee969cd88c9896872baee85c57dfa0f33c69b4b34efc4ecef46b45f3d"

def replace(old, new):
    global source
    assert old in source, old[:80]
    source = source.replace(old, new, 1)

replace("  std::vector<double> ff_scales() const {return ff_velocity_scale_;}",
"""  std::vector<double> ff_scales() const {return ff_velocity_scale_;}
  std::vector<double> actual_pid_gain(int field) const
  {
    std::vector<double> a;
    for(auto &pid:pids_)
    {
      const auto g=pid->get_gains();
      a.push_back(field==0?g.p_gain_:field==1?g.i_gain_:field==2?g.d_gain_:
                  field==3?g.i_max_:g.i_min_);
    }
    return a;
  }""")
replace('    request->names={"interpolate_from_desired_state","open_loop_control"};',
"""    request->names={"interpolate_from_desired_state","open_loop_control"};
    const std::vector<std::string> gain_fields={"p","i","d","i_clamp","ff_velocity_scale"};
    for(const auto &name:names)for(const auto &field:gain_fields)
      request->names.push_back("gains."+name+"."+field);""")
replace('if(values.size()!=2 || values[0].type', 'if(values.size()!=62 || values[0].type')
anchor='    sample("activation",0);'
replace(anchor,
'''    out<<"{\\"kind\\":\\"all_joint_gain_service\\",\\"joint_values\\":[";
    for(size_t joint=0;joint<names.size();++joint)
    {
      if(joint)out<<",";
      out<<"{\\"joint\\":\\""<<names[joint]<<"\\"";
      for(size_t field=0;field<gain_fields.size();++field)
      {
        const auto &value=values[2+joint*gain_fields.size()+field];
        if(value.type!=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE || !std::isfinite(value.double_value))
          throw std::runtime_error("Actual gain service has wrong type/nonfinite value");
        out<<",\\""<<gain_fields[field]<<"\\":"<<value.double_value;
      }
      out<<"}";
    }
    out<<"]}\\n";
    out<<"{\\"kind\\":\\"actual_pid_gains\\",\\"p\\":";array(out,jtc->actual_pid_gain(0));
    out<<",\\"i\\":";array(out,jtc->actual_pid_gain(1));
    out<<",\\"d\\":";array(out,jtc->actual_pid_gain(2));
    out<<",\\"i_clamp_max\\":";array(out,jtc->actual_pid_gain(3));
    out<<",\\"i_clamp_min\\":";array(out,jtc->actual_pid_gain(4));out<<"}\\n";
    sample("activation",0);''')
old='''    const int64_t origin_ns=r.time_ns,duration_ns=1200000000LL;
    int64_t producer_ns=0,update_ns=4000000LL;int k=0;
    // Separate native event grids: callback-confirmed producer first at ties.
    // No update emits a synthetic reference merely to fill its own cadence.
    while(std::min(producer_ns,update_ns)<=duration_ns)
    {
      if(producer_ns<=update_ns)
      {r.time_ns=origin_ns+producer_ns;r.clock();r.reference(nominal);producer_ns+=5000000LL;}
      else
      {r.tick("quasistatic",++k,origin_ns+update_ns);update_ns+=4000000LL;}
    }'''
new='''    const int64_t origin_ns=r.time_ns,duration_ns=2200000000LL;
    auto changed=nominal,restarted=nominal;
    for(size_t j=0;j<nominal.size();++j)
    {changed[j]+=(j%2?1.:-1.)*.01;restarted[j]+=(j%2?1.:-1.)*.005;}
    int64_t producer_ns=0,update_ns=4000000LL;int k=0;
    // Preserve the separate integer grids: actual callback first at ties.
    // Return q0 is the producer's changed target, never a JTC internal state.
    while(std::min(producer_ns,update_ns)<=duration_ns)
    {
      if(producer_ns<=update_ns)
      {
        auto target=nominal;
        if(producer_ns>1200000000LL && producer_ns<=1500000000LL)target=changed;
        else if(producer_ns>1500000000LL && producer_ns<=1875000000LL)
        {
          const double t=double(producer_ns-1500000000LL)/375000000.;
          const double a=10.*t*t*t-15.*t*t*t*t+6.*t*t*t*t*t;
          for(size_t j=0;j<target.size();++j)target[j]=changed[j]+a*(nominal[j]-changed[j]);
        }
        else if(producer_ns>2000000000LL)target=restarted;
        r.time_ns=origin_ns+producer_ns;r.clock();r.reference(target);producer_ns+=5000000LL;
      }
      else
      {
        std::string stage=update_ns<=1200000000LL?"quasistatic":
          update_ns<=1500000000LL?"changed_target":update_ns<=1875000000LL?"producer_stop_return":
          update_ns<=2000000000LL?"nominal_idle":"rapid_restart";
        r.tick(stage,++k,origin_ns+update_ns);update_ns+=4000000LL;
      }
    }'''
replace(old, new)
(HERE / "fixture.cpp").write_text(source)
parameters = (PRIOR / "parameters.yaml").read_text().replace("p: 100.0", "p: 220.982919").replace("d: 1.0", "d: 3.360637")
parameters = parameters.replace("        i_clamp: 2.5", "        i_clamp: 2.5\n        ff_velocity_scale: 0.0")
(HERE / "parameters.yaml").write_text(parameters)
(HERE / "CMakeLists.txt").write_text((PRIOR / "CMakeLists.txt").read_text().replace(
    "jtc_effective_response_native", "jtc_classic_profile_native"))
receipt = dict(scope=__doc__, original_source_sha256=hashlib.sha256((PRIOR / "fixture.cpp").read_bytes()).hexdigest(),
    selected_profile=dict(interpolate_from_desired_state=True, p=220.982919, i=.2, d=3.360637,
        i_clamp=2.5, ff_velocity_scale=0., external_CM_enforce_command_limits=True),
    external_CM_limits_not_in_bare_JTC_fixture=True,
    generated_sha256={name:hashlib.sha256((HERE / name).read_bytes()).hexdigest()
                      for name in ("fixture.cpp", "parameters.yaml", "CMakeLists.txt")})
(HERE / "preparation.json").write_text(json.dumps(receipt, indent=2) + "\n")
print(json.dumps(receipt, indent=2))
