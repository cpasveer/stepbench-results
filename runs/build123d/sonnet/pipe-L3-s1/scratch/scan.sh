for foot in "[-1.3,-6.0]" "[-1.3,-2.0]" "[-3.0,0.0]" "[1.0,-2.0]"; do for s in 13 15; do
P_OVR="{\"RB\":42.0,\"FOOT\":$foot,\"BEAM_S\":$s}" OUT=scratch/t.step uv run --project ../../agent-env python submission.py >/dev/null 2>&1
echo foot=$foot s=$s; sed -i "s#import_step('output.step')#import_step('scratch/t.step')#" scratch/chk.py; uv run --project ../../agent-env python scratch/chk.py 2>&1 | grep -E 'B[1-4] col'|tr '\n' ' '; echo; sed -i "s#import_step('scratch/t.step')#import_step('output.step')#" scratch/chk.py
done; done
