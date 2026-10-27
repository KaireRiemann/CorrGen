#!/usr/bin/env python3
"""Interactive flow regression on isolated ROS master 11347, inside ROS1 shell."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
parser=argparse.ArgumentParser()
parser.add_argument('--scenes',nargs='+',default=['balls','multi_gap','maze','medium'])
parser.add_argument('--load-only',action='store_true')
parser.add_argument('--solver',default='highs',choices=['proxqp','highs','glpk','sdlp'])
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
output=root/('results/interactive' if args.solver=='proxqp' else 'results/interactive_'+args.solver);output.mkdir(parents=True,exist_ok=True)
env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:11347',ROS_HOSTNAME='127.0.0.1')
with socket.socket() as probe:
    probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    probe.bind(('127.0.0.1',11347))
for scene in args.scenes:
    log_path=output/(scene+('_load_launch.log' if args.load_only else '_launch.log'))
    report_path=output/(scene+('_load_run.json' if args.load_only else '_run.json'))
    report=dict(scene=scene,started_unix=time.time(),status='RUNNING',solver=args.solver,load_only=args.load_only)
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    with log_path.open('w') as log:
        launch=subprocess.Popen(['roslaunch','-p','11347','corrgen_cpp',scene+'_demo.launch','rviz:=false','solver:='+args.solver,'output_dir:='+str(output/scene)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                if launch.poll() is not None: raise RuntimeError('roslaunch exited; see '+str(log_path))
                ready=subprocess.run(['rosnode','list'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
                if ready:break
                time.sleep(.25)
            if not ready: raise RuntimeError('ROS master did not start')
            command=['python3',str(root/'scripts/probe_interaction.py'),scene]
            if args.load_only:command.append('--load-only')
            subprocess.run(command,env=env,check=True,timeout=650)
            report['status']='PASS'
        except Exception as error:
            report.update(status='FAIL',error=str(error))
            raise
        finally:
            report['finished_unix']=time.time()
            report_path.write_text(json.dumps(report,indent=2)+'\n')
            if launch.poll() is None: os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGKILL);launch.wait()
print('PASS: requested interactive map launch regressions')
