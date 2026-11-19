# Simulation videos (scratch, not for merge)

Renders the two videos of release `sim-videos-2026-11-19` from recorded run artifacts only. Nothing is re-simulated.

| Video | Source run | Profile |
|---|---|---|
| video_b_grocery.mp4 | 44299879412, demo/grocery-dark-store 6170e6a, artifact grocery-demo-attempt1 | firmware_sil devices, sim-oracle poses |
| video_c_engineering.mp4 | 44309791416, integration/baseline1 d63e9ed, artifact baseline1-acceptance-attempt1 | firmware_sil devices, sim-oracle poses |

`make_all.sh` downloads both artifacts and runs every step: bag extraction (`extract_*.py`, rosbags), candump parsing
(`parse_can.py`, host time mapped to Gazebo time through the mission trace), CAN decoding (`candecode.py`), per-frame
rendering with matplotlib (`render_c.py`, `render_b2.py`), slides and cards (`slides_c.py`, `cards.py`) and the
final encode (`assemble.sh`: H.264 High, yuv420p, 1920x1080, 30 fps, faststart, no audio).

Simulation evidence only: not hardware, perception or safety validation.
