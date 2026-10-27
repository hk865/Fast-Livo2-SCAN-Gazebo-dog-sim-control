"""Current moving-obstacle scene parameters, without route-version pins."""
import json,math
from pathlib import Path

def configuration(profile=None):
    if profile is None:profile=json.loads((Path(__file__).resolve().parents[1]/'config/navigation.json').read_text())
    value=dict(profile['dynamic_scene'])
    for key in ('park_world_xyz','blocking_world_xyz','obstacle_size_m','body_to_world_translation'):
        v=value[key]
        if not isinstance(v,list) or len(v)!=3 or any(type(x)not in(int,float)or not math.isfinite(x)for x in v):raise ValueError('Invalid dynamic scene vector: '+key)
    if any(x<=0 for x in value['obstacle_size_m']):raise ValueError('Positive obstacle dimensions required')
    if type(value['encounter_route_point_index'])is not int or value['encounter_route_point_index']<1:raise ValueError('Invalid route encounter index')
    if any(type(value[k])not in(int,float)or not math.isfinite(value[k])or value[k]<=0 for k in ('speed_mps','blocking_s')):raise ValueError('Positive obstacle timing required')
    if value['speed_mps']>.25:raise ValueError('Obstacle speed safety maximum exceeded')
    return value

def target_at(elapsed_s,scene=None):
    if type(elapsed_s)not in(int,float)or not math.isfinite(elapsed_s)or elapsed_s<0:raise ValueError('Invalid obstacle clock')
    s=configuration()if scene is None else scene
    a=s['park_world_xyz'];b=s['blocking_world_xyz'];v=s['speed_mps']
    distance=math.dist(a,b);enter=distance/v
    if distance<=0:raise ValueError('Distinct park and crossing positions required')
    if elapsed_s<enter:return [x+(y-x)*elapsed_s/enter for x,y in zip(a,b)],'entering'
    if elapsed_s<enter+s['blocking_s']:return list(b),'blocking'
    if elapsed_s<2*enter+s['blocking_s']:
        f=(elapsed_s-enter-s['blocking_s'])/enter
        return [y+(x-y)*f for x,y in zip(a,b)],'leaving'
    return list(a),'clear'
