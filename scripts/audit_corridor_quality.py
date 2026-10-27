#!/usr/bin/env python3
"""Independent sampled voxel audit and width comparison; sampling is NOT a certificate."""
import json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
baseline=json.loads((root/'results/engineering/final_regression.json').read_text())
def sections(data,n=401):
    values=np.polynomial.chebyshev.chebval(np.linspace(-1,1,n),np.array(data['coefficients']).T)
    output=[]
    for a,b,c,dx,dy in values.T:
        E=np.array([[a,c],[c,b]]);d=np.array([dx,dy]);ev,rot=np.linalg.eigh(E)
        centre=-.5*np.linalg.solve(E,d);radii=np.sqrt((1+centre@E@centre)/ev)
        output.append((centre,radii,rot))
    return output
def stats(data):
    es=sections(data);r=np.array([sorted(e[1]) for e in es]);area=np.pi*r.prod(axis=1)
    return dict(min_minor_m=float(r[:,0].min()),median_minor_m=float(np.median(r[:,0])),
                median_major_m=float(np.median(r[:,1])),max_major_m=float(r[:,1].max()),mean_area_m2=float(area.mean()))
def audit(directory):
    data=json.loads((directory/'metrics.json').read_text());request=json.loads((directory/'request.json').read_text())
    file=Path(request['cloud_file'])
    if str(root).startswith('/home/'):
        file=Path(str(file).replace('/root/ws/','/home/diffbot/ros1_ws/'))
    cloud=np.loadtxt(file);origin=np.array(request['origin']);size=np.array(request['size']);res=request['resolution']
    inflation=request['inflation']+request['robot_radius']+request['safety_margin']
    dims=np.ceil(size/res).astype(int);raw=np.zeros(tuple(dims),dtype=bool)
    cells=np.floor((cloud-origin)/res).astype(int);cells=cells[((cells>=0)&(cells<dims)).all(axis=1)]
    raw[tuple(cells.T)]=True;occupied=raw.copy();radius=int(np.ceil(inflation/res))+1 if inflation>0 else 0
    for dx in range(-radius,radius+1):
      for dy in range(-radius,radius+1):
       for dz in range(-radius,radius+1):
        delta=(dx,dy,dz)
        if sum((max(abs(v)-1,0)*res)**2 for v in delta)>inflation**2+1e-12:continue
        source=tuple(slice(max(0,-v),min(n,n-v)) for v,n in zip(delta,dims))
        target=tuple(slice(max(0,v),min(n,n+v)) for v,n in zip(delta,dims))
        occupied[target]|=raw[source]
    for axis,n in enumerate(dims):
      for i in range(n):
        if i*res<inflation-1e-12 or (i+1)*res>size[axis]-inflation+1e-12:
          sl=[slice(None)]*3;sl[axis]=i;occupied[tuple(sl)]=True
    controls=np.loadtxt(directory/'reference_bezier.csv',delimiter=',').reshape(-1,4,3)
    up=np.loadtxt(directory/'frame_axis.xyz');checks=0
    for u,(centre,radii,rot) in zip(np.linspace(0,1,401),sections(data)):
        t=u*len(controls);i=min(int(t),len(controls)-1);t-=i;p=controls[i]
        position=(1-t)**3*p[0]+3*(1-t)**2*t*p[1]+3*(1-t)*t*t*p[2]+t**3*p[3]
        tangent=(1-t)**2*(p[1]-p[0])+2*(1-t)*t*(p[2]-p[1])+t*t*(p[3]-p[2]);tangent/=np.linalg.norm(tangent)
        normal=np.cross(up,tangent);normal/=np.linalg.norm(normal);B=np.column_stack((normal,np.cross(tangent,normal)))
        angles=np.linspace(0,2*np.pi,64,endpoint=False)
        edge=rot@(radii[:,None]*np.array([np.cos(angles),np.sin(angles)]))
        for r in [0.,.25,.5,.75,1.]:
            points=position[:,None]+B@(centre[:,None]+r*edge)
            c=np.floor((points.T-origin)/res).astype(int)
            assert ((c>=0)&(c<dims)).all(),'Outside domain'
            assert not occupied[tuple(c.T)].any(),'Sample intersects inflated occupied voxel'
            checks+=len(c)
    return checks
report=[]
for replay in json.loads((root/'results/quality/latest.json').read_text()):
    scene=replay['scene']
    directory=root/replay['new_source']
    before=next(x for x in baseline if x['scene']==scene and x['request']==1)
    after=json.loads((directory/'metrics.json').read_text())
    row=dict(scene=scene,baseline_source=before['source'],new_source=str(directory.relative_to(root)),
             before=stats(before),after=stats(after),sample_checks=audit(directory),
             total_ms=after['total_ms'],shrink_factor=after['shrink_factor'])
    row['mean_area_ratio']=row['after']['mean_area_m2']/row['before']['mean_area_m2'];report.append(row)
(root/'results/quality/comparison.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
