#source /cvmfs/sw-nightlies.hsf.org/key4hep/setup.sh -r 2025-12-04   #Outdated. Boost 1.88
source /cvmfs/sw-nightlies.hsf.org/key4hep/setup.sh -r 2026-02-08
# LUXE compact
source /data/dust/user/wangyufe/luxegeo/install/bin/thisluxegeo.sh
export LD_LIBRARY_PATH=/data/dust/user/wangyufe/luxegeo/install/lib:$LD_LIBRARY_PATH
export DD4hep_LIBRARY_PATH=/data/dust/user/wangyufe/luxegeo/install/lib64:$DD4hep_LIBRARY_PATH

export LD_LIBRARY_PATH=/data/dust/user/wangyufe/playground/k4ActsTracking/install/lib:$LD_LIBRARY_PATH
export GAUDI_PLUGIN_PATH=/data/dust/user/wangyufe/playground/k4ActsTracking/install/lib:$GAUDI_PLUGIN_PATH
export PYTHONPATH=/data/dust/user/wangyufe/playground/k4ActsTracking/install/python:$PYTHONPATH
