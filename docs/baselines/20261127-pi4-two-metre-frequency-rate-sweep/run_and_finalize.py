import pathlib, subprocess, sys, json, traceback
here=pathlib.Path(__file__).resolve().parent
with (here/'controller.log').open('w') as log:
    run=subprocess.run([sys.executable,'-u',str(here/'run_sweep.py')],stdout=log,stderr=subprocess.STDOUT)
with (here/'finalize.log').open('w') as log:
    final=subprocess.run([sys.executable,'-u',str(here/'finalize_sweep.py')],stdout=log,stderr=subprocess.STDOUT)
    out=pathlib.Path((here/'latest-run.txt').read_text().strip())
    verify=subprocess.run([sys.executable,str(here/'verify_matrix.py'),str(out)],stdout=log,stderr=subprocess.STDOUT)
    comparison=(subprocess.run([sys.executable,str(here/'compare_reference.py'),str(out)],stdout=log,stderr=subprocess.STDOUT).returncode if verify.returncode==0 else None)
(here/'completion.json').write_text(json.dumps(dict(acquisition_exit=run.returncode,finalization_exit=final.returncode,matrix_verification_exit=verify.returncode,comparison_exit=comparison,out=str(out)),indent=2)+'\n')
