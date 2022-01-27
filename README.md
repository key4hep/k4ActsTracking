# k4ActsTracking


This repository contains the necessary tools to use ACTS functionality in Key4hep


## Dependencies

* Acts

* DD4hep

* ROOT

* EDM4hep

* Gaudi

* k4FWCore

## Compilation and testing


```
source /cvmfs/sw-nightlies.hsf.org/key4hep/setup.sh
mkdir build install
cd build;
cmake .. -DCMAKE_INSTALL_PREFIX=../install -G Ninja
ninja
ctest
```

