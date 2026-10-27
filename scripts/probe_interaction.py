#!/usr/bin/env python3
"""Exercise the actual RViz input topics without requiring a desktop."""
import argparse
import json
import math
import numpy as np
import time
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String, Empty
from visualization_msgs.msg import MarkerArray, Marker

parser=argparse.ArgumentParser()
parser.add_argument('scene')
parser.add_argument('--load-only',action='store_true')
args=parser.parse_args()
rospy.init_node('corrgen_interaction_probe',anonymous=True)
state={'status':'','astar':None,'reference':None,'corridor':None}
def capture(key):
    def callback(msg): state[key]=msg.data if key=='status' else msg
    return callback
subs=[rospy.Subscriber('/corrgen_demo/status',String,capture('status')),
      rospy.Subscriber('/corrgen_demo/astar_path',RosPath,capture('astar')),
      rospy.Subscriber('/corrgen_demo/reference_path',RosPath,capture('reference')),
      rospy.Subscriber('/corrgen_demo/corridor',MarkerArray,capture('corridor'))]
goals=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
initials=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
reset=rospy.Publisher('/corrgen_demo/reset',Empty,queue_size=1)
def wait(predicate,timeout=180):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():return
        if state['status'].startswith(('FAIL','MAP_ERROR')):raise RuntimeError(state['status'])
        time.sleep(.05)
    raise RuntimeError('Timeout: '+state['status'])
wait(lambda:state['status']=='WAITING_FOR_START',30)
cloud=rospy.wait_for_message('/corrgen_demo/map',PointCloud2,timeout=10)
assert not state['astar'].poses and not state['reference'].poses,'Map load must not auto-plan'
if args.load_only:
    print(args.scene+': PASS map loaded, points='+str(cloud.width*cloud.height)+', waiting for user input',flush=True)
    raise SystemExit(0)
origin=rospy.get_param('/corrgen_demo/origin')
size=rospy.get_param('/corrgen_demo/size')
resolution=rospy.get_param('/corrgen_demo/resolution')
inflation=rospy.get_param('/corrgen_demo/inflation')+rospy.get_param('/corrgen_demo/robot_radius',0)+rospy.get_param('/corrgen_demo/safety_margin',0)+rospy.get_param('/corrgen_demo/reference_clearance',0)
inset=math.ceil(inflation/resolution)*resolution+.5*resolution
dims=[int(math.ceil(v/resolution)) for v in size]
occupied=np.zeros(tuple(dims),dtype=bool)
for x,y,z in point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=True):
    cell=tuple(int(math.floor((v-o)/resolution)) for v,o in zip((x,y,z),origin))
    if all(0<=v<n for v,n in zip(cell,dims)):occupied[cell]=True
raw=occupied.copy()
radius=math.ceil(inflation/resolution)+1 if inflation>0 else 0
for dx in range(-radius,radius+1):
 for dy in range(-radius,radius+1):
  for dz in range(-radius,radius+1):
   delta=(dx,dy,dz)
   if sum((max(abs(v)-1,0)*resolution)**2 for v in delta)>inflation**2+1e-12:continue
   source=tuple(slice(max(0,-v),min(n,n-v)) for v,n in zip(delta,dims))
   target=tuple(slice(max(0,v),min(n,n+v)) for v,n in zip(delta,dims))
   occupied[target] |= raw[source]
for axis in range(3):
 for index in range(dims[axis]):
  if index*resolution<inflation-1e-12 or (index+1)*resolution>size[axis]-inflation+1e-12:
   section=[slice(None)]*3;section[axis]=index;occupied[tuple(section)]=True
def free_near(p):
    cells=[(x,y,z) for x in range(1,dims[0]-1) for y in range(1,dims[1]-1) for z in [int((p[2]-origin[2])/resolution)] if not occupied[x,y,z]]
    k=min(cells,key=lambda c:sum(((v+.5)*resolution+o-t)**2 for v,o,t in zip(c,origin,p)))
    return tuple((v+.5)*resolution+o for v,o in zip(k,origin))
starts={'balls':(.3,4.1,1.5),'multi_gap':(.3,2.1,1.5),'maze':(.5,.5,1.5)}
ends={'balls':(19.7,6.3,1.5),'multi_gap':(19.5,8.1,1.5),'maze':(19.3,9.3,1.5)}
start=free_near(starts.get(args.scene,(origin[0]+1,origin[1]+1,origin[2]+.75)))
goal=free_near(ends.get(args.scene,(origin[0]+size[0]-1,origin[1]+size[1]-1,origin[2]+size[2]-.75)))
arrow=rospy.get_param('/corrgen_demo/height_mode')=='arrow'
def pose(p):
    msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=rospy.Time.now();msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=p
    if arrow:
        q=(p[2]-origin[2]-inset)/(size[2]-2*inset);msg.pose.orientation.z=q;msg.pose.orientation.w=math.sqrt(1-q*q)
    else:msg.pose.orientation.w=1
    return msg
def send(p):
    wait(lambda:goals.get_num_connections()>0,10);goals.publish(pose(p))
def endpoint(msg,index):
    p=msg.poses[index].pose.position;return (p.x,p.y,p.z)
def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
# Bad selection before the first pair cannot start planning.
send((origin[0]-resolution,origin[1]-resolution,origin[2]+1.5));wait(lambda:state['status'].startswith('INVALID_POINT'),10)
reports=[]
for request,(a,b) in enumerate([(start,goal),(goal,start)],1):
    send(a);wait(lambda:state['status']=='WAITING_FOR_GOAL',10)
    wait(lambda:not state['reference'].poses and all(m.action==Marker.DELETEALL for m in state['corridor'].markers),10)
    send(b);wait(lambda:state['status'].startswith('PASS request='+str(request)),15)
    wait(lambda:bool(state['reference'].poses) and len(state['corridor'].markers)>10,10)
    assert distance(endpoint(state['astar'],0),a)<1e-6
    assert distance(endpoint(state['astar'],-1),b)<1e-6
    assert distance(endpoint(state['reference'],0),a)<1e-6
    assert distance(endpoint(state['reference'],-1),b)<1e-6
    report=dict(request=request,start=a,goal=b,astar_points=len(state['astar'].poses),reference_samples=len(state['reference'].poses),rings=len(state['corridor'].markers),status='PASS')
    reports.append(report);print(args.scene+': '+json.dumps(report),flush=True)
# New invalid request removes the old corridor; initialpose can explicitly restart.
send((origin[0]-resolution,origin[1]-resolution,origin[2]+1.5));wait(lambda:state['status'].startswith('INVALID_POINT'),10)
wait(lambda:not state['reference'].poses and all(m.action==Marker.DELETEALL for m in state['corridor'].markers),10)
msg=PoseWithCovarianceStamped();msg.header=pose(start).header;msg.pose.pose=pose(start).pose
initials.publish(msg);wait(lambda:state['status']=='WAITING_FOR_GOAL',10)
reset.publish(Empty());wait(lambda:state['status']=='WAITING_FOR_START',10)
output=Path(rospy.get_param('/corrgen_demo/session_dir'))
output.mkdir(parents=True,exist_ok=True)
(output/'interaction_checks.json').write_text(json.dumps(dict(scene=args.scene,requests=reports,map_wait=True,invalid_selection=True,clears_stale_results=True,initialpose=True,reset=True),indent=2)+'\n')
