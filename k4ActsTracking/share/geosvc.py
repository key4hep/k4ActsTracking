import os
import sys
from pprint import pprint
from Gaudi.Configuration import *

from Configurables import ActsGeoSvc

print("\n".join(os.environ["LD_LIBRARY_PATH"].split(":")))

algList = []

a = ActsGeoSvc("ActsGeoSvc")
a.detectors = [os.path.join(os.environ["OPENDATADETECTOR"], 'xml/OpenDataDetector.xml')]
a.debugGeometry = True
a.outputFileName = "MyObjFile"
from Configurables import ApplicationMgr

ApplicationMgr(TopAlg=algList, EvtSel="NONE", EvtMax=2, ExtSvc=[a], OutputLevel=INFO)
