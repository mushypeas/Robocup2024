import sys
sys.path.append('task/gpsr_repo/')

import rospy

from gpsr_parser import ultimateFollowupParser
from gpsr_followup import *

# Fetch a dish from the refrigerator / and deliver it to the lying person in the bedroom
# Get a drink from the storage rack / and deliver it to the person pointing to the right in the office
# Get a cleaning supply from the tv stand / and bring it to me
# Take a cola from the desk / and put it on the sofa
# "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {followup}",
def takeObjFromPlcmt(g, params):
    print('Start takeObjFromPlcmt')
    params = {'takeVerb': 'Get', 'obj_singCat': 'drink', 'fromLocPrep': 'from', 'plcmtLoc': 'storage rack', 'followup': 'and bring it to me'}
    
    # [0] Extract parameters
    takeVerb, obj, plcmtLoc, followup = params['takeVerb'], params['obj_singCat'], params['plcmtLoc'], params['followup']
    # print(takeVerb, '\n', obj, '\n', fromLocPrep, '\n', plcmtLoc, '\n', cmd)

    # [1] Move to the specified space
    g.move(plcmtLoc)

    # [2] Take the object from the specified location
    g.pick(obj)

    # [3] Follow-up command
    followupName, params = ultimateFollowupParser(followup)
    followUpFunc = g.followupName2followupFunc[followupName]
    followUpFunc(g, params)