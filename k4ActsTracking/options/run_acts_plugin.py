import os
from pprint import pprint
from Gaudi.Configuration import *

from Configurables import ActsAlg, GeoSvc

algList = []

geosvc = GeoSvc("GeoSvc")
# using this instead of CLIC_o3_v14 because it is much faster to instantiate
geosvc.detectors = [os.environ["LCGEO"] + "/CLIC/compact/CLIC_o2_v04/CLIC_o2_v04.xml"]
#  geosvc.detectors = [os.environ["LCGEO"] + "/CLIC/compact/CLIC_o3_v14/CLIC_o3_v14.xml"]

a = ActsAlg("MyActsAlg", OutputLevel=VERBOSE)
algList.append(a)

from Configurables import ApplicationMgr

ApplicationMgr(
    TopAlg=algList, EvtSel="NONE", EvtMax=2, ExtSvc=[geosvc], OutputLevel=INFO
)
