"""Bounded independent reproducible cases in parallel; each has separate outputs."""
import sys,os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'vendor'))
from concurrent.futures import ProcessPoolExecutor,as_completed
from simulate_full import CASES,run,OUT
import argparse,json

def worker(name):
 return run(name,CASES[name])
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--workers',type=int,default=3);ap.add_argument('--cases',default='all');a=ap.parse_args()
 names=list(CASES) if a.cases=='all' else a.cases.split(',')
 with ProcessPoolExecutor(max_workers=a.workers) as pool:
  fut={pool.submit(worker,n):n for n in names}
  for f in as_completed(fut):
   try:f.result()
   except Exception as e:print('FAILED',fut[f],repr(e),flush=True);raise
 print('SUITE_COMPLETE',','.join(names),flush=True)
