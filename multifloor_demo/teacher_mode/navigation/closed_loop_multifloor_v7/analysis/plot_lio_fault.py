#!/usr/bin/env python3
"""Scientific plots of saved actual LIO evidence and finite counterfactuals."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
 p=argparse.ArgumentParser();p.add_argument('--results',type=Path,required=True);a=p.parse_args();agg=json.loads((a.results/'aggregate_iterations.json').read_text());rows=[x for x in agg if 125<=x['t_s']<=136];term=[x for x in rows if x['stop']];first=[x for x in rows if x['iteration']==0];details=[x for x in json.loads((a.results/'constraint_details.json').read_text())if 125<=x['t_s']<=136 and x['iteration']==0];cf=json.loads((a.results/'single_iteration_counterfactual.json').read_text())
 plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.25,'figure.dpi':120})
 fig,axs=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
 ax=axs[0,0];ax.plot([x['t_s']for x in term],[x['state_after_z']for x in term],color='navy');ax.set_ylabel('LIO estimated position z (m)');ax.set_xlabel('Simulation time (s)');ax.set_title('Actual terminal state; no counterfactual trajectory')
 ax=axs[0,1];ax.plot([x['t_s']for x in first],[100*x['measurement_vz']for x in first],color='black',alpha=.65,label='Actual first-iteration measurement');ax.axhline(0,color='.4',lw=.8);ax.set_ylabel('Single-iteration velocity z correction (cm/s)');ax.set_xlabel('Simulation time (s)');ax.set_title('Actual measurement term, not physical velocity');ax.legend(fontsize=8)
 ax=axs[1,0]
 for group,color,label in [('near_horizontal_absnz_ge085','tab:red','Near-horizontal stored normals'),('vertical_absnz_le020','tab:blue','Near-vertical stored normals'),('sloped_or_other','tab:green','Other normals')]:
  ax.plot([x['t_s']for x in details],[100*x['groups'][group]['vz_measurement_contribution']for x in details],color=color,label=label)
 ax.axhline(0,color='.4',lw=.8);ax.set_xlabel('Simulation time (s)');ax.set_ylabel('First-iteration measurement contribution (cm/s)');ax.set_title('Accepted-row grouped contribution with actual K');ax.legend(fontsize=8)
 ax=axs[1,1];x=np.arange(len(cf['cases']));width=.25
 ax.bar(x-width,[100*q['actual_solution19'][9]for q in cf['cases']],width,color='black',label='Actual')
 ax.bar(x,[100*q['variants'][0]['solution_vz']for q in cf['cases']],width,color='tab:orange',label='Remove evidenced normal-conflict groups')
 ax.bar(x+width,[100*q['variants'][2]['solution_vz']for q in cf['cases']],width,color='tab:purple',label='Broad ratio <.05 sensitivity (unsafe gate)')
 ax.axhline(0,color='.4',lw=.8);ax.set_xticks(x,[f"{q['t_s']:.1f} s"for q in cf['cases']]);ax.set_ylabel('Single-iteration solution vz (cm/s)');ax.set_title('Frozen P/state/vec; recomputed K/G; no rematching');ax.legend(fontsize=8)
 fig.suptitle('Actual 20261005_175143 r1: LIO height failure and matched-row evidence',fontsize=13);fig.savefig(a.results/'lio_height_matching_counterfactual.png');plt.close(fig)
 case=cf['cases'][0];ev={q['plane_id']:q for q in case['historical_rank1_near_horizontal_plane_evidence']};fig=plt.figure(figsize=(12,6),layout='constrained')
 for j,pid in enumerate([871,872]):
  q=ev[pid];pts=np.array(q['actual_accepted_point_world']);pc=pts.mean(0);hc=np.array(q['historical_center']);hn=np.array(q['historical_normal']);pn=np.array(q['actual_accepted_min_eigenvector']);ax=fig.add_subplot(1,2,j+1,projection='3d');ax.grid(True)
  ax.scatter(pts[:,0],pts[:,1],pts[:,2],color='black',s=35,label='Actual accepted points')
  ys=np.linspace(pts[:,1].min()-.03,pts[:,1].max()+.03,4);zs=np.linspace(min(pts[:,2].min(),hc[2])-.03,max(pts[:,2].max(),hc[2])+.03,4);Y,Z=np.meshgrid(ys,zs);X=pc[0]-(pn[1]*(Y-pc[1])+pn[2]*(Z-pc[2]))/pn[0];ax.plot_surface(X,Y,Z,color='tab:green',alpha=.2)
  xx=np.linspace(pc[0]-.13,pc[0]+.13,4);X,Y=np.meshgrid(xx,ys);Z=hc[2]-(hn[0]*(X-hc[0])+hn[1]*(Y-hc[1]))/hn[2];ax.plot_surface(X,Y,Z,color='tab:red',alpha=.25)
  ax.quiver(*pc,*(hn*.17),color='tab:red',linewidth=2,label='Stored historical normal')
  if pn[0]<0:pn=-pn
  ax.quiver(*pc,*(pn*.17),color='tab:green',linewidth=2,label='Current accepted-point PCA normal')
  ax.set_xlim(pc[0]-.13,pc[0]+.22);ax.set_ylim(ys[0],ys[-1]);ax.set_zlim(min(hc[2],pts[:,2].min())-.03,pts[:,2].max()+.18);ax.set_box_aspect((1,1,1));ax.set_xlabel('LIO x (m)');ax.set_ylabel('LIO y (m)');ax.set_zlabel('LIO z (m)');ax.set_title(f"Plane {pid}, t=132.0 s\nhistoric mid/max {q['historical_midmax_ratio']:.4f}; normal |dot| {q['normal_agreement_abs_dot']:.3f}");ax.view_init(elev=23,azim=-56)
  ax.legend(fontsize=8,loc='upper left')
 fig.suptitle('Historical near-horizontal fit versus current thin vertical point face\nAccepted-point geometry shown here; east-wall identity verified in separate offline SDF/native audit',fontsize=12);fig.savefig(a.results/'frozen_plane_current_geometry.png');plt.close(fig)
 print(a.results/'lio_height_matching_counterfactual.png');print(a.results/'frozen_plane_current_geometry.png')

if __name__=='__main__':main()
