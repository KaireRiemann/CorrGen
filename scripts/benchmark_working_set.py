#!/usr/bin/env python3
"""Same saved dense LP, independent backends and a full-matrix reference."""
import json
from pathlib import Path
import subprocess
import time
root=Path(__file__).resolve().parents[1]
binary=root.parents[1]/'build/CorrGen/corrgen_replay'
output=root/'results/engineering/working_set_comparison.json'
results=[]
for solver,mode,budget in [('proxqp','working',1000),('highs','working',1000),('highs','full',10000)]:
    for repeat in range(3 if mode=='working' else 1):
        command=[str(binary),str(root/'results/performance_dense'),solver,'6',mode,str(budget)]
        started=time.monotonic()
        try:
            result=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=20)
            row=dict(command=command,repeat=repeat,wall_ms=(time.monotonic()-started)*1000,returncode=result.returncode,output=result.stdout)
        except subprocess.TimeoutExpired:
            row=dict(command=command,repeat=repeat,status='TIMEOUT',wall_ms=(time.monotonic()-started)*1000)
        results.append(row);output.write_text(json.dumps(results,indent=2)+'\n');print(row,flush=True)
