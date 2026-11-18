Diagnostics only, do not merge. Inputs are finished runs; nothing here reruns a mission.

extract.py <job logs>         per-run numbers and Nav2 controller events from baseline1-acceptance job logs
parking_analysis.py <dirs>    decomposition from the bag and baseline1_mission.json of extracted artifacts
                              (run by .github/workflows/diag-parking-variability.yml on this branch)
amcl_landscape.py <dir>       offline: Nav2 likelihood-field score at the work-shelf station on the generated map;
                              <dir> holds scene.py as scene_mod.py and config/scene.yaml as scene.yaml
landscape_variants.py <dir>   offline what-if of map resolution and extra LiDAR-plane structure (owner decisions)
