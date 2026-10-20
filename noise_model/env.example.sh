# Copy to env.sh and adjust. Then: source env.sh
# Arena SDK (LUCID) installed without the sudo ld.so.conf script: point the loader at its libraries.
export ARENA_ROOT=$HOME/ArenaSDK_Linux_x64
export LD_LIBRARY_PATH=$ARENA_ROOT/lib64:$ARENA_ROOT/GenICam/library/lib/Linux64_x64:$ARENA_ROOT/ffmpeg:$ARENA_ROOT/Metavision/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
# venv with arena_api (wheel in $ARENA_ROOT/OutputDirectory/Linux/x64Release/ArenaPy), numpy, scipy, matplotlib.
# In that venv set ARENAC_CUSTOM_PATHS['python64_lin'] = $ARENA_ROOT/lib64/libarenac.so (arena_api_config.py)
# and SAVEC_CUSTOM_PATHS['python64_lin'] = $ARENA_ROOT/lib64/libsavec.so.
source "$(dirname "${BASH_SOURCE[0]}")/.venv/bin/activate"
# BFS scripts use the Spinnaker C library (/opt/spinnaker) and must run WITHOUT this LD_LIBRARY_PATH:
#   env -u LD_LIBRARY_PATH python scripts/bfs_ptc_capture.py ...
