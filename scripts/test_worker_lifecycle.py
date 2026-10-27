#!/usr/bin/env python3
"""Cancellation/map-version/deadline regression; own master 11349, saved real dense map."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

root = Path(__file__).resolve().parents[1]
source = max((root/'results/interactive/dense').glob('run_*'), key=lambda p:p.stat().st_mtime)
request = json.loads((source/'request_1/request.json').read_text())
output = root/'results/engineering/lifecycle'
output.mkdir(parents=True, exist_ok=True)
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11349', ROS_HOSTNAME='127.0.0.1')
with socket.socket() as sock:
    sock.bind(('127.0.0.1',11349))
log = (output/'ros.log').open('w')
master = subprocess.Popen(['roscore','-p','11349'], stdout=log, stderr=log, start_new_session=True)
node = None
report = {'source':str(source), 'status':'RUNNING'}
def wait(predicate, seconds=10):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if predicate(): return
        time.sleep(.005)
    raise RuntimeError('Timeout; '+str(state))
def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait()
try:
    import rospy
    from std_msgs.msg import String, Empty, Header
    from geometry_msgs.msg import PointStamped
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.point_cloud2 import create_cloud_xyz32
    state={'status':''}
    wait(lambda: subprocess.run(['rosnode','list'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode==0)
    rospy.init_node('lifecycle_probe', anonymous=True)
    sub=rospy.Subscriber('/corrgen_demo/status',String,lambda m:state.update(status=m.data))
    maps=rospy.Publisher('/corrgen_demo/cloud',PointCloud2,queue_size=1,latch=True)
    points=rospy.Publisher('/clicked_point',PointStamped,queue_size=1)
    resets=rospy.Publisher('/corrgen_demo/reset',Empty,queue_size=1)
    xyz=[tuple(map(float,line.split())) for line in Path(request['cloud_file']).read_text().splitlines()]
    cloud=create_cloud_xyz32(Header(frame_id='map'),xyz)
    def start(budget, solver="proxqp"):
        global node
        state['status']=''
        params={k:request[k] for k in ['origin','size','resolution','inflation','robot_radius','safety_margin','reference_clearance','degree']}
        params.update(output_dir=str(output), solver=solver,job_budget_ms=budget,solver_budget_ms=1000)
        for k,v in params.items():rospy.set_param('/corrgen_demo/'+k,v)
        node=subprocess.Popen(['rosrun','corrgen_cpp','corrgen_demo_node','__name:=corrgen_demo'],stdout=log,stderr=log,start_new_session=True)
        wait(lambda:maps.get_num_connections()>0)
        maps.publish(cloud)
        wait(lambda:state['status']=='WAITING_FOR_START')
        return Path(rospy.get_param('/corrgen_demo/session_dir'))
    def point(v):
        m=PointStamped();m.header.frame_id='map';m.point.x,m.point.y,m.point.z=v;points.publish(m)
    def submit(directory, number):
        wait(lambda:points.get_num_connections()>0 and resets.get_num_connections()>0)
        point(request['start']);wait(lambda:state['status']=='WAITING_FOR_GOAL')
        point(request['goal']);wait(lambda:(directory/f'request_{number}/request.json').exists())
    directory=start(3000)
    submit(directory,1)
    t=time.monotonic();resets.publish(Empty());wait(lambda:state['status']=='WAITING_FOR_START')
    report['reset_ms']=(time.monotonic()-t)*1000
    done=json.loads((directory/'request_1/completion.json').read_text());assert done['status']=='CANCELLED',done
    submit(directory,2)
    t=time.monotonic();maps.publish(create_cloud_xyz32(Header(frame_id='map'),xyz+[(0.,0.,4.8)]))
    wait(lambda:(directory/'request_2/completion.json').exists())
    report['map_cancel_ms']=(time.monotonic()-t)*1000
    done=json.loads((directory/'request_2/completion.json').read_text());assert done['status']=='STALE_MAP',done
    wait(lambda:state['status']=='WAITING_FOR_START')
    time.sleep(.3);assert state['status']=='WAITING_FOR_START',state
    report['no_stale_publish']=True
    # The supervisor must reap the actual process, not detach its computation.
    children=Path(f'/proc/{node.pid}/task/{node.pid}/children').read_text().strip()
    assert not children,children
    report['no_worker_orphans']=True
    stop(node);node=None
    directory=start(1)
    submit(directory,1)
    wait(lambda:state['status'].startswith('FAIL request=1: TIMEOUT'))
    done=json.loads((directory/'request_1/completion.json').read_text())
    assert done['timeout'] and done['first_publish_ms']==0,done
    report['deadline']=done
    stop(node);node=None
    directory=start(1000, 'sdlp')
    submit(directory,1)
    wait(lambda:state['status'].startswith(('PASS request=1','FAIL request=1')), 5)
    done=json.loads((directory/'request_1/completion.json').read_text())
    assert done['timeout'],done
    report['sdlp_budget']=done
    assert not Path(f'/proc/{node.pid}/task/{node.pid}/children').read_text().strip()
    report['status']='PASS'
except Exception as error:
    report.update(status='FAIL',error=str(error));raise
finally:
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    stop(node);stop(master);log.close()
print(json.dumps(report,indent=2))
