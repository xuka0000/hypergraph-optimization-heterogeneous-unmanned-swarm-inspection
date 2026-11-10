"""Compile the delivered LaTeX without editor services or account tools."""
from pathlib import Path
import argparse, os, shutil, subprocess
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--xelatex');a=p.parse_args()
exe=a.xelatex or shutil.which('xelatex')
if not exe:raise SystemExit('Install XeLaTeX and add it to PATH, or use --xelatex <executable>.')
book=ROOT/'book';build=book/'build';build.mkdir(exist_ok=True)
env=os.environ.copy();env['MIKTEX_ENABLE_INSTALLER']='no'
for run in range(3):
    result=subprocess.run([exe,'-no-shell-escape','-interaction=nonstopmode','-halt-on-error','-output-directory=build','main.tex'],cwd=book,env=env,capture_output=True)
    (build/f'pass{run+1}.txt').write_bytes(result.stdout+result.stderr)
    if result.returncode:raise SystemExit('XeLaTeX failed; see '+str(build/f'pass{run+1}.txt'))
print(build/'main.pdf')
