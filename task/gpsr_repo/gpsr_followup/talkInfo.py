# "talkInfo": "{talkVerb} {talk}"
def talkInfo(g, params):
    print("Start talkInfo")
    
    # [0] Extract the parameters
    talk = params['talk']
    talk = g.cluster(talk, g.talk_list)
    
    # [1] Talk about the information
    g.talk(talk)
