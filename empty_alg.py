import os
from pprint import pprint
from Gaudi.Configuration import *

from Configurables import EmptyAlg

algList = []


a = EmptyAlg("MyEmptyAlg")
algList.append(a)

from Configurables import ApplicationMgr

ApplicationMgr(TopAlg=algList, EvtSel="NONE", EvtMax=2, ExtSvc=[], OutputLevel=INFO)
