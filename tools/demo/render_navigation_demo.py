#!/usr/bin/env python3
"""Render an explicitly labelled, timestamp-driven replay of a completed run.

No simulator, localization, controller, or navigation process is started. Scene
pose/joints come from recorded Gazebo telemetry; the map view uses recorded
FAST-LIVO2 corrected poses and saved LiDAR keyframes, without trajectory fitting.
Requires Python numpy/scipy/Pillow/VTK and ffmpeg; an OpenGL context is needed.
"""
import argparse, json, os, subprocess, time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image, ImageDraw, ImageFont
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

W,H,VH=1440,810,576
BG=(12,19,30);WHITE=(231,239,248);MUTED=(151,170,190);CYAN=(55,218,229);GREEN=(135,215,129);ORANGE=(255,188,94)
NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}

def pose(xyz=(0,0,0),rpy=(0,0,0)):
    t=np.eye(4);t[:3,3]=xyz;t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();return t

def origin(node):
    return pose(np.fromstring(node.get('xyz','0 0 0'),sep=' '),np.fromstring(node.get('rpy','0 0 0'),sep=' ')) if node is not None else np.eye(4)

def mesh_data(points,faces=None):
    p=vtk.vtkPoints();p.SetData(numpy_to_vtk(np.ascontiguousarray(points,dtype=np.float64),deep=True))
    poly=vtk.vtkPolyData();poly.SetPoints(p);cells=vtk.vtkCellArray()
    if faces is None:
        ids=np.column_stack([np.ones(len(points),dtype=np.int64),np.arange(len(points),dtype=np.int64)]).ravel()
        cells.SetCells(len(points),numpy_to_vtkIdTypeArray(ids,deep=True));poly.SetVerts(cells)
    else:
        faces=np.asarray(faces,dtype=np.int64);ids=np.column_stack([np.full(len(faces),faces.shape[1]),faces]).ravel()
        cells.SetCells(len(faces),numpy_to_vtkIdTypeArray(ids,deep=True))
        if faces.shape[1]==2:poly.SetLines(cells)
        else:poly.SetPolys(cells)
    return poly

def actor(renderer,poly,color,opacity=1.,width=2.):
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(poly);mapper.ScalarVisibilityOff()
    a=vtk.vtkActor();a.SetMapper(mapper);p=a.GetProperty();p.SetColor(np.array(color)/255.);p.SetOpacity(opacity);p.SetLineWidth(width);p.SetPointSize(width);p.SetAmbient(.3);p.SetDiffuse(.7);renderer.AddActor(a);return a

def line(renderer,xyz,color,width=3.,opacity=1.):
    xyz=np.asarray(xyz);return actor(renderer,mesh_data(xyz,np.column_stack([np.arange(len(xyz)-1),np.arange(1,len(xyz))])),color,opacity,width)

def replace_points(a,xyz):
    a.GetMapper().SetInputData(mesh_data(xyz))

def set_matrix(a,m):
    v=vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):v.SetElement(i,j,float(m[i,j]))
    a.SetUserMatrix(v)

def dae(path):
    # The supplied Velodyne DAE uses a literal '<STL_BINARY>' XML attribute.
    # Repair only that label in memory; leave the asset and geometry untouched.
    root=ET.fromstring(path.read_text().replace('<STL_BINARY>','STL_BINARY'));geometries={}
    for g in root.findall('c:library_geometries/c:geometry',NS):
        mesh=g.find('c:mesh',NS);sources={}
        for s in mesh.findall('c:source',NS):
            a=s.find('c:float_array',NS);access=s.find('c:technique_common/c:accessor',NS)
            if a is not None:sources[s.get('id')]=np.fromstring(a.text,sep=' ').reshape(-1,int(access.get('stride','1')))
        vertices={v.get('id'):v.find("c:input[@semantic='POSITION']",NS).get('source')[1:]for v in mesh.findall('c:vertices',NS)}
        pieces=[]
        for primitive in list(mesh):
            kind=primitive.tag.split('}')[-1]
            if kind not in ('triangles','polylist'):continue
            inputs=primitive.findall('c:input',NS);stride=max(int(x.get('offset','0'))for x in inputs)+1
            v=next(x for x in inputs if x.get('semantic')=='VERTEX');src=sources[vertices[v.get('source')[1:]]][:,:3]
            indices=np.fromstring(primitive.find('c:p',NS).text,sep=' ',dtype=np.int64).reshape(-1,stride)[:,int(v.get('offset','0'))]
            if kind=='triangles':tris=indices.reshape(-1,3)
            else:
                counts=np.fromstring(primitive.find('c:vcount',NS).text,sep=' ',dtype=int);tris=[];offset=0
                for n in counts:
                    for k in range(1,n-1):tris.append([indices[offset],indices[offset+k],indices[offset+k+1]])
                    offset+=n
                tris=np.asarray(tris)
            pieces.append(src[tris])
        geometries[g.get('id')]=np.concatenate(pieces)
    output=[]
    def visit(n,parent):
        t=parent.copy()
        for v in n:
            kind=v.tag.split('}')[-1]
            if kind=='matrix':t=t@np.fromstring(v.text,sep=' ').reshape(4,4)
            elif kind=='translate':t=t@pose(np.fromstring(v.text,sep=' '))
            elif kind=='scale':t=t@np.diag([*np.fromstring(v.text,sep=' '),1])
            elif kind=='rotate':
                a=np.fromstring(v.text,sep=' ');m=np.eye(4);m[:3,:3]=Rotation.from_rotvec(a[:3]*np.deg2rad(a[3])).as_matrix();t=t@m
        for ins in n.findall('c:instance_geometry',NS):
            a=geometries[ins.get('url')[1:]];output.append(a@t[:3,:3].T+t[:3,3])
        for child in n.findall('c:node',NS):visit(child,t)
    for n in root.findall('c:library_visual_scenes/c:visual_scene/c:node',NS):visit(n,np.eye(4))
    scale=float(root.find('c:asset/c:unit',NS).get('meter','1'));tri=np.concatenate(output)*scale
    return mesh_data(tri.reshape(-1,3),np.arange(tri.size//3).reshape(-1,3))

def read_jsonl(p):
    with p.open()as f:return [json.loads(l)for l in f]

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--project',type=Path,required=True);ap.add_argument('--run',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--preview',type=float);ap.add_argument('--fps',type=int,default=20);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=a.project.resolve();run=a.run.resolve();a.output.parent.mkdir(parents=True,exist_ok=True)
    result=json.loads((run/'slam_navigation_result.json').read_text());score=json.loads((run/'slam_backend_score.json').read_text())
    if not result['full_run_pass']:raise ValueError('Select a completely passed run')
    telemetry=read_jsonl(run/'telemetry.jsonl');times=np.array([x['state_physics_world_time']for x in telemetry]);positions=np.array([x['position']for x in telemetry]);quaternions=np.array([x['quaternion_wxyz']for x in telemetry]);qs=np.array([x['q']for x in telemetry]);speeds=np.array([np.linalg.norm(x['body_lin_vel'][:2])for x in telemetry]);states=[x['state']for x in telemetry]
    slam=read_jsonl(run/'slam_poses.jsonl');st=np.array([x['stamp_ns']*1e-9 for x in slam]);sp=np.array([x['T_world_body']for x in slam]);sg=np.array([x['generation']for x in slam])
    route=np.asarray(json.loads((run/'PCT_route.json').read_text())['world_body_points'])
    hp=next((run/'backend_history').glob('*/history.json'));history=json.loads(hp.read_text());kf=history['keyframes'];kt=np.array([x['stamp_ns']*1e-9 for x in kf]);clouds=[]
    for k in kf:
        i=int(np.argmin(np.abs(st-k['stamp_ns']*1e-9)));t=sp[i]@np.asarray(k['T_body_lidar']);p=np.load(hp.parent/k['cloud'],allow_pickle=False);clouds.append(p[:,:3]@t[:3,:3].T+t[:3,3])
    with (root/'assets/building/building2_9.pcd').open('rb')as f:
        while not f.readline().startswith(b'DATA'):pass
        pointmap=np.fromfile(f,dtype='<f4').reshape(-1,6)[::20,:3].copy()
    pointmap+=np.array([-.000002,-.000001,-.1])
    triangles=np.load(root/'assets/building/building_triangles.npz',allow_pickle=False)['triangles']
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);horizontal=np.abs(normals[:,2])/np.maximum(np.linalg.norm(normals,axis=1),1e-12)>.7
    win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(W,VH);win.SetMultiSamples(4)
    left=vtk.vtkRenderer();right=vtk.vtkRenderer();left.SetViewport(0,0,.61,1);right.SetViewport(.61,0,1,1)
    for r in [left,right]:r.SetBackground(np.array(BG)/255.);r.SetBackground2(.08,.12,.18);r.GradientBackgroundOn();win.AddRenderer(r)
    terrain=actor(left,mesh_data(triangles.reshape(-1,3),np.arange(len(triangles)*3).reshape(-1,3)),(176,190,201))
    line(left,route,(142,199,128),2.2,.5)
    pm=mesh_data(pointmap);z=np.clip((pointmap[:,2]+.1)/5.,0,1);colors=np.column_stack([60+100*z,100+70*z,145+60*z]).astype('uint8');pm.GetPointData().SetScalars(numpy_to_vtk(colors,deep=True,array_type=vtk.VTK_UNSIGNED_CHAR));ma=actor(right,pm,(100,150,190),.42,1.3);ma.GetMapper().ScalarVisibilityOn()
    line(right,route,GREEN,2.,.7);trail=line(right,sp[::10,:3,3],CYAN,3.)
    current_cloud=actor(right,mesh_data(clouds[0]),ORANGE,.9,2.)
    marker=vtk.vtkSphereSource();marker.SetRadius(.16);marker.SetThetaResolution(16);marker.SetPhiResolution(12);marker.Update();dot=actor(right,marker.GetOutput(),(255,245,170))
    tip=line(right,np.array([[0,0,0],[.5,0,0]]),(255,245,170),3.)
    urdf=ET.parse(root/'assets/world/go2_measured.urdf').getroot();joints=urdf.findall('joint');order=['rf_hip_joint','rf_upper_leg_joint','rf_lower_leg_joint','lf_hip_joint','lf_upper_leg_joint','lf_lower_leg_joint','rh_hip_joint','rh_upper_leg_joint','rh_lower_leg_joint','lh_hip_joint','lh_upper_leg_joint','lh_lower_leg_joint'];joint_index={n:i for i,n in enumerate(order)}
    visuals=[];cache={}
    for link in urdf.findall('link'):
        for v in link.findall('visual'):
            g=v.find('geometry');m=g.find('mesh');poly=None
            if m is not None:
                filename=Path(m.get('filename')).name;p=root/'assets/meshes'/filename
                if not p.exists():continue
                if filename not in cache:
                    if p.suffix=='.dae':cache[filename]=dae(p)
                    else:s=vtk.vtkSTLReader();s.SetFileName(str(p));s.Update();cache[filename]=s.GetOutput()
                poly=cache[filename];local=origin(v.find('origin'))@np.diag([*np.fromstring(m.get('scale','1 1 1'),sep=' '),1])
            elif g.find('sphere')is not None:
                s=vtk.vtkSphereSource();s.SetRadius(float(g.find('sphere').get('radius')));s.Update();poly=s.GetOutput();local=origin(v.find('origin'))
            elif g.find('box')is not None:
                size=np.fromstring(g.find('box').get('size'),sep=' ');s=vtk.vtkCubeSource();s.SetXLength(size[0]);s.SetYLength(size[1]);s.SetZLength(size[2]);s.Update();poly=s.GetOutput();local=origin(v.find('origin'))
            if poly is not None:
                color=(78,89,101)if link.get('name')=='trunk'else(113,124,136)
                if 'foot' in link.get('name'):color=(32,39,47)
                act=actor(left,poly,color);act.GetProperty().SetSpecular(.4);act.GetProperty().SetSpecularPower(25);visuals.append((link.get('name'),local,act))
    cam=right.GetActiveCamera();cam.SetPosition(25,-28,26);cam.SetFocalPoint(0,0,2.2);cam.SetViewUp(0,0,1);cam.ParallelProjectionOn();cam.SetParallelScale(15.)
    capture=vtk.vtkWindowToImageFilter();capture.SetInput(win);capture.SetInputBufferTypeToRGB();capture.ReadFrontBufferOff()
    fontpath='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';boldpath='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
    fonts={k:ImageFont.truetype(fontpath,k)for k in [17,19,21,23,26]};bold=ImageFont.truetype(boldpath,32)
    parking_start=float(result['parking']['T0_s'])
    # Time selection changes playback speed only, never the recorded state.
    stair_centres=[]
    for level in [1.,2.4,3.9]:
        ids=np.flatnonzero(positions[:,2]>level)
        if len(ids):stair_centres.append(float(times[ids[0]]))
    end=min(float(times[-1]),402.2);frames=[(0.,0.)]*int(a.fps*3);t=0.
    while t<end:
        speed=1. if t>=parking_start or any(abs(t-c)<4 for c in stair_centres)else 4.
        frames.append((t,speed));t+=speed/a.fps
    frames += [(end,0.)]*int(a.fps*3)
    if a.preview is not None:frames=[(a.preview,1.)]
    encoder=None
    if a.preview is None:
        encoder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-n','-f','rawvideo','-pixel_format','rgb24','-video_size',f'{W}x{H}','-framerate',str(a.fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','24','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)],stdin=subprocess.PIPE)
    last_k=-1;last_terrain=-999.;began=time.monotonic()
    for fi,(t,rate)in enumerate(frames):
        i=max(0,min(len(times)-1,int(np.searchsorted(times,t,side='right')-1)));si=max(0,min(len(st)-1,int(np.searchsorted(st,t,side='right')-1)));ki=max(0,min(len(kt)-1,int(np.searchsorted(kt,t,side='right')-1)))
        body=np.eye(4);body[:3,3]=positions[i];body[:3,:3]=Rotation.from_quat(np.roll(quaternions[i],-1)).as_matrix();links={'base_link':body}
        pending=list(joints)
        while pending:
            ready=[j for j in pending if j.find('parent').get('link')in links]
            if not ready:raise ValueError('Disconnected URDF')
            for j in ready:
                rel=origin(j.find('origin'))
                if j.get('name')in joint_index:
                    rot=np.eye(4);rot[:3,:3]=Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*qs[i,joint_index[j.get('name')]]).as_matrix();rel=rel@rot
                links[j.find('child').get('link')]=links[j.find('parent').get('link')]@rel;pending.remove(j)
        for name,local,act in visuals:set_matrix(act,links[name]@local)
        # Reveal the current storey; hide ceilings/upper walls for visibility.
        ceiling=positions[i,2]+.55
        if abs(ceiling-last_terrain)>.1:
            use=triangles[:,:,2].min(1)<ceiling;use&=(horizontal|(triangles[:,:,2].max(1)<ceiling));tri=triangles[use];terrain.GetMapper().SetInputData(mesh_data(tri.reshape(-1,3),np.arange(len(tri)*3).reshape(-1,3)));last_terrain=ceiling
        p=positions[i];cam=left.GetActiveCamera();cam.SetPosition(*(p+[-1.45,-1.9,1.3]));cam.SetFocalPoint(*(p+[.1,0,-.05]));cam.SetViewUp(0,0,1);cam.SetViewAngle(44);cam.SetClippingRange(.05,35)
        xyz=sp[si,:3,3];dot.SetPosition(*xyz);tipp=np.array([xyz,xyz+sp[si,:3,0]*.6]);tip.GetMapper().SetInputData(mesh_data(tipp,np.array([[0,1]])))
        for act in [dot,tip,trail]:act.SetVisibility(t>=st[0])
        current_cloud.SetVisibility(t>=kt[0])
        tr=sp[:si+1:6,:3,3];trail.GetMapper().SetInputData(mesh_data(tr,np.column_stack([np.arange(max(0,len(tr)-1)),np.arange(1,len(tr))])))
        if ki!=last_k:replace_points(current_cloud,clouds[ki]);last_k=ki
        win.Render();capture.Modified();capture.Update();pixels=vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(VH,W,3)[::-1].copy()
        image=Image.new('RGB',(W,H),BG);image.paste(Image.fromarray(pixels),(0,104));d=ImageDraw.Draw(image)
        d.text((28,12),'FAST-LIVO2  ·  Building 多楼层导航',font=bold,fill=WHITE)
        d.text((30,59),'85.70 m 已验收运行  |  已知出生先验  |  原始时间戳日志回放，非实时录屏',font=fonts[21],fill=MUTED)
        d.rounded_rectangle((1180,22,1410,70),radius=12,fill=(31,58,51));d.text((1200,29),'日志回放  /  REPLAY',font=fonts[21],fill=GREEN)
        d.rectangle((0,104,878,140),fill=(19,32,46));d.rectangle((879,104,W,140),fill=(19,32,46))
        d.text((22,107),'场景视图：Gazebo记录姿态与关节 / 楼层剖切显示',font=fonts[19],fill=WHITE)
        d.text((900,107),'地图视图：外部PCD + 实测SLAM轨迹',font=fonts[19],fill=WHITE)
        d.line((878,104,878,680),fill=(63,80,98),width=2)
        d.rectangle((14,640,860,676),fill=(16,28,40));d.text((28,643),f'仿真时间 {t:7.2f} s    '+('暂停画面'if rate==0 else f'{rate:.0f}×回放')+'    场景状态仅供可视化',font=fonts[19],fill=WHITE)
        d.text((897,639),'绿：PCT参考   青：SLAM   橙：当前关键帧',font=fonts[17],fill=MUTED)
        d.line((24,694,1416,694),fill=(51,67,85),width=1)
        phase='初始化 / 等待跟踪'if t<20 else('停车验证  10 Hz · 5 s · 51点'if t>=parking_start else'自主行走 / 多楼层路线跟踪')
        d.text((28,707),phase,font=fonts[23],fill=GREEN if t>=parking_start else WHITE)
        d.text((28,748),f'实测速度 {speeds[i]:.2f} m/s    高度 {positions[i,2]:.2f} m    已存关键帧 {ki+1 if t>=kt[0]else 0}/{len(kf)}    修正序号 {sg[si] if t>=st[0]else 0}',font=fonts[19],fill=MUTED)
        d.text((875,705),'本轮独立评分  PASS',font=fonts[23],fill=GREEN)
        d.text((875,743),f'3D RMSE {score["position_RMSE_m"]*100:.2f} cm   终点XY {result["terminal_xy_error_m"]*100:.2f} cm',font=fonts[19],fill=WHITE)
        d.text((875,773),'GT仅离线评分/场景回放；仿真速度内环保留',font=fonts[17],fill=MUTED)
        if a.preview is not None:image.save(a.output)
        else:encoder.stdin.write(image.tobytes())
        if fi%100==0:print(json.dumps(dict(frame=fi,total=len(frames),source_s=t,render_wall_s=round(time.monotonic()-began,1))),flush=True)
    if encoder is not None:
        encoder.stdin.close()
        if encoder.wait()!=0:raise RuntimeError('ffmpeg failed')
        receipt=dict(source_run=run.name,view='offline telemetry and same-stamp SLAM replay; not live Gazebo/RViz recording',fps=a.fps,frames=len(frames),duration_s=len(frames)/a.fps,source_interval_s=[0,end],trajectory_alignment='none',scene_state_source='recorded Gazebo pose and joint positions',map_pose_source='recorded FAST-LIVO2 plus external map correction',keyframe_cloud_source='saved original LiDAR keyframes with matching source stamps',full_run_pass=True,known_spawn_prior=True,full_run_3D_RMSE_m=score['position_RMSE_m'])
        a.output.with_suffix('.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n');print(json.dumps(receipt,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
