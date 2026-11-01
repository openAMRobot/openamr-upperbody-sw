#!/usr/bin/env bash
S=${SCRATCH:-/tmp/m02_bisect}
for round in 1 2; do
  for v in b_mass3 c_y060 c_x000 a_acm_all_disabled d_open_inside; do
    OVERLAYS=$S/ovl/$v $S/run_m02.sh ${v}_r$round
    leftovers=$(wc -l < $S/m02_${v}_r$round.leftover); [ "$leftovers" -gt 0 ] && echo "NOTE ${v}_r$round: gz left over: $(cat $S/m02_${v}_r$round.leftover | cut -c1-60)" >> $S/m02_results.txt
  done
done
echo ALLDONE >> $S/m02_results.txt
