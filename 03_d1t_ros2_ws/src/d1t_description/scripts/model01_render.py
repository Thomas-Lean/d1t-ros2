#!/usr/bin/env python3
"""Render engineering views from the SAME exported metre meshes used in RViz."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection,LineCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from model01_geometry import read_stl,profile,profile_x


def color(name):
    if name=='ganten_348_bottle':return '#387bc4'
    if name.startswith('Empty_'):return '#e78029'
    if 'post' in name:return '#646f80'
    return '#83bfc2'


def display_bounds(meshes):
    """Return padded millimetre bounds from the exported scene meshes."""
    points=np.vstack([m.vertices for m in meshes.values()])*1000
    low,high=points.min(0),points.max(0)
    padding=np.maximum((high-low)*.07,5.)
    return low-padding,high+padding


def section(mesh,axis,coordinate):
    segments=[]
    for tri in mesh.vertices[mesh.faces]*1000:
        hits=[]
        for a,b in [(tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])]:
            if (a[axis]-coordinate)*(b[axis]-coordinate)<0:
                hits.append(a+(b-a)*(coordinate-a[axis])/(b[axis]-a[axis]))
        if len(hits)==2:segments.append(np.array(hits))
    return np.array(segments)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    data=json.loads(args.result.read_text());c=data['parameters'];r=c['reported'];d=c['draft']
    meshes={f.stem:read_stl(f) for f in sorted((args.result.parent/'meshes').glob('*.stl'))}
    low,high=display_bounds(meshes)
    order=sorted(meshes,key=lambda n:(n=='ganten_348_bottle',n.startswith('Empty_')))
    # Explicit draw order: rack, cylinder, hand.
    order=[n for n in order if n.startswith('ganten_rack')]+['ganten_348_bottle']+[n for n in order if n.startswith('Empty_')]
    fig,axs=plt.subplots(1,3,figsize=(16,5),layout='constrained')
    for ax,axes,title in zip(axs,[(0,2),(0,1),(1,2)],['Front: X-Z','Top: X-Y','Right: Y-Z']):
        for name in order:
            m=meshes[name];tri=(m.vertices[m.faces]*1000)[:,:,axes]
            ax.add_collection(PolyCollection(tri,facecolors=color(name),edgecolors='none',alpha=.75))
        ax.autoscale_view();ax.set_aspect('equal');ax.grid(alpha=.15)
        ax.set_xlabel('XYZ'[axes[0]]+' [mm]');ax.set_ylabel('XYZ'[axes[1]]+' [mm]')
        ax.set_title(title)
        g=np.array(data['grip_reference_m'])*1000
        ax.plot(g[axes[0]],g[axes[1]],'r+',ms=9)
    fig.suptitle(f'MODEL-01 {data["status"]} | rack {r["rack_width"]:g} x {r["rack_depth"]:g} x {r["rack_height"]:g} mm | '
                 f'cylinder {r["bottle_diameter"]:g} x {r["bottle_height"]:g} mm\n'
                 f'Teal: {r["plate_thickness"]:g} mm rack plate   Blue: cylinder   Orange: existing URDF hand, '
                 f'gap {d["open_gap"]:g} mm   Red: h={r["grip_h"]:g} mm reference',fontsize=12)
    fig.savefig(args.out/'three_views.png',dpi=160);plt.close(fig)

    fig=plt.figure(figsize=(12,8));ax=fig.add_subplot(projection='3d',computed_zorder=False)
    for name in order:
        m=meshes[name]
        ax.add_collection3d(Poly3DCollection(m.vertices[m.faces]*1000,facecolors=color(name),
                                            edgecolors='none',alpha=.9 if name.startswith('Empty_') else .65))
    ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),zlim=(low[2],high[2]),xlabel='X [mm]',ylabel='Y [mm]',zlabel='Z [mm]')
    ax.set_box_aspect(high-low);ax.view_init(elev=22,azim=-63)
    ax.set_title(f'MODEL-01 {data["status"]} | continuous upper {len(d["upper_centers"])} / lower {len(d["lower_centers"])} grooves\n'
                 'Local hand overlay only; camera, arm and installation pose not covered')
    fig.savefig(args.out/'overview.png',dpi=160,bbox_inches='tight');plt.close(fig)

    fig,ax=plt.subplots(figsize=(12,7),layout='constrained')
    xs=profile_x(c);zs=profile(c,xs,'upper')
    ax.fill_between(xs,zs-r['plate_thickness'],zs,color=color('rack'),label=f'{r["plate_thickness"]:g} mm continuous plate')
    y=data['grip_reference_m'][1]*1000
    for name in ['Empty_Link_L','Empty_Link_R','Empty_Link6']:
        lines=section(meshes[name],1,y)
        if len(lines):ax.add_collection(LineCollection(lines[:,:,[0,2]],colors=color(name),linewidths=1.5))
    center=np.array(data['grip_reference_m'])*1000
    theta=np.linspace(0,2*np.pi,400)
    ax.plot(center[0]+r['bottle_diameter']/2*np.cos(theta),center[2]+r['bottle_diameter']/2*np.sin(theta),color=color('ganten_348_bottle'),lw=2,label=f'{r["bottle_diameter"]:g} mm cylinder')
    ax.plot(center[0],center[2],'+',color='red',ms=12,label=f'grip reference h={r["grip_h"]:g} mm')
    wrist=np.array(data['wrist_matrix_m']);tcp=wrist@np.r_[np.array(data['gripper']['tcp_xyz_mm'])/1000,1]
    ax.plot(tcp[0]*1000,tcp[2]*1000,'kx',ms=8,label='unchanged TCP')
    contacts=np.array(data['support_points_m'])*1000
    ax.scatter(contacts[:,0],contacts[:,2],s=6,color='#244a53',label='polyhedron support witnesses')
    for phase in ['approach','close']:
        point=np.array(data['phases'][phase]['witness_m'][0])*1000
        ax.plot(point[0],point[2],'rs',ms=5)
    ax.axhline(r['upper_bottom'],color='gray',ls='--',lw=.7)
    ax.axhline(r['upper_bottom']+r['slot_depth'],color='gray',ls=':',lw=.7)
    ax.set(xlim=(205,380),ylim=(115,265),xlabel='rack X [mm]',ylabel='rack Z [mm]')
    ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(loc='upper left',fontsize=8)
    ax.set_title(f'Gripper section at Y={y:.1f} mm | orange: actual URDF mesh section, gap {d["open_gap"]:g} mm\n'
                 f'W={r["slot_width"]:g}, D={r["slot_depth"]:g} mm | red squares: projected collision witnesses\n'
                 'DRAFT: nominal top approach intersects upper plate; zero clearance is NOT a penetration depth')
    fig.savefig(args.out/'gripper_section.png',dpi=180);plt.close(fig)

    fig,axs=plt.subplots(1,3,figsize=(14,4),layout='constrained')
    for ax,phase in zip(axs,['approach','close','lift']):
        ss=[x for x in data['sampling']['all_sample_summaries'] if x['phase']==phase]
        x=[a['value_mm'] for a in ss];gap=[a['hand_rack']['minimum_mm'] for a in ss]
        ax.plot(x,gap,'o-',ms=2,color='#26797d')
        blocked=[a for a in ss if a['hand_rack']['blocked_pairs']]
        ax.scatter([a['value_mm'] for a in blocked],[0]*len(blocked),c='red',s=12,label='intersection')
        ax.axhline(30,ls='--',color='#b27613',label='30 mm margin comparison')
        ax.set_title(phase);ax.set_ylabel('minimum hand-rack separation [mm]')
        ax.set_xlabel({'approach':'remaining approach [mm]','close':'jaw gap [mm]','lift':'vertical lift [mm]'}[phase])
        ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.suptitle('Finite samples of the nominal local path; obstructed early stages invalidate an end-to-end pick')
    fig.savefig(args.out/'phase_clearance.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
