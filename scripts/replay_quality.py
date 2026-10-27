#!/usr/bin/env python3
"""Replay preserved before/after inputs; never overwrite prior runs."""
import hashlib
import json
from pathlib import Path
import subprocess
import time
root=Path(__file__).resolve().parents[1]
old=json.loads((root/'results/engineering/final_regression.json').read_text())
run=root/'results/quality'/('run_'+str(time.time_ns()));run.mkdir(parents=True)
summary=[]
for scene in ['balls','dense']:
    before=next(x for x in old if x['scene']==scene and x['request']==1)
    request=json.loads((root/before['source']).with_name('request.json').read_text())
    request.update(solver='highs',job_budget_ms=5000,clearance_weight=4.)
    request.pop('config_hash',None)
    request['config_hash']=hashlib.sha256(json.dumps(request,sort_keys=True).encode()).hexdigest()
    directory=run/scene;directory.mkdir()
    (directory/'request.json').write_text(json.dumps(request,indent=2)+'\n')
    command=[str(root.parents[1]/'devel/lib/corrgen_cpp/corrgen_worker'),str(directory)]
    started=time.monotonic()
    try:
        with (directory/'worker.log').open('w') as log:
            result=subprocess.run(command,stdout=log,stderr=log,timeout=20)
        row=dict(scene=scene,baseline_source=before['source'],new_source=str(directory.relative_to(root)),
                 command=command,returncode=result.returncode,wall_ms=(time.monotonic()-started)*1000)
    except subprocess.TimeoutExpired:
        row=dict(scene=scene,status='TIMEOUT',new_source=str(directory.relative_to(root)))
    summary.append(row);(run/'replay.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(row,flush=True)
(root/'results/quality/latest.json').write_text(json.dumps(summary,indent=2)+'\n')
if any(x.get('returncode')!=0 for x in summary):raise SystemExit(1)
