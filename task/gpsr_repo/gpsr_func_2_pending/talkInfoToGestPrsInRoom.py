from datetime import datetime, timedelta

# "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
def talkInfoToGestPrsInRoom(g, params):
    # Tell the day of the week to the person pointing to the left in the office
    # Tell something about yourself to the person raising their right arm in the kitchen
    # Tell the time to the person raising their right arm in the living room
    # Tell your teams affiliation to the person pointing to the right in the bathroom
    print("Start talkInfoToGestPrsInRoom")
    
    # [0] Extract parameters
    try:
        talk = params['talk']
    except KeyError:
        talk = 'the time'
        
    try:
        gestPers = params['gestPers']
    except KeyError:
        gestPers = 'waving person'
        
    try:
        room = params['room']
    except KeyError:
        room = 'gpsr_instruction_point'

    # [1] Move to the specified room
    g.move(room)

    # [2] Find the person in the room
    g.identifyGestPose(gestPers)

    # [3] Talk to the person
    if talk == 'something about yourself':
        g.say('I am a robot designed to help people in their daily lives')
        
    elif talk == 'the time':
        g.say(f'The time is {datetime.now().time()}')
        
    elif talk == 'what day is today':
        g.say(f'Today is {datetime.now().date()}')
        
    elif talk == 'what day is tomorrow':
        g.say(f'Tomorrow is {datetime.now().date() + timedelta(days=1)}')
    
    elif talk == 'your teams name':
        g.say('My team name is Tidy Boy')