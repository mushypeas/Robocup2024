import sys
sys.path.append('task/gpsr_repo/gpsr_followup/')

def findObj(g, params):
    from findObj import findObj
    findObj(g, params)

def findPrs(g, params):
    from findPrs import findPrs
    findPrs(g, params)
    
def meetName(g, params):
    from meetName import meetName
    meetName(g, params)
    
def placeObjOnPlcmt(g, params):
    from placeObjOnPlcmt import placeObjOnPlcmt
    placeObjOnPlcmt(g, params)

def deliverObjToMe(g, params):
    from deliverObjToMe import deliverObjToMe
    deliverObjToMe(g, params)
    
def deliverObjToPrsInRoom(g, params):
    from deliverObjToPrsInRoom import deliverObjToPrsInRoom
    deliverObjToPrsInRoom(g, params)
    
def deliverObjToNameAtBeac(g, params):
    from deliverObjToNameAtBeac import deliverObjToNameAtBeac
    deliverObjToNameAtBeac(g, params)
    
def talkInfo(g, params):
    from talkInfo import talkInfo
    talkInfo(g, params)
    
def answerQuestion(g, params):
    from answerQuestion import answerQuestion
    answerQuestion(g, params)
    
def followPrs(g, params):
    from followPrs import followPrs
    followPrs(g, params)
    
def followPrsToRoom(g, params):
    from followPrsToRoom import followPrsToRoom
    followPrsToRoom(g, params)
    
def guidePrsToBeacon(g, params):
    from guidePrsToBeacon import guidePrsToBeacon
    guidePrsToBeacon(g, params)
    
def takeObj(g, params):
    from takeObj import takeObj
    takeObj(g, params)