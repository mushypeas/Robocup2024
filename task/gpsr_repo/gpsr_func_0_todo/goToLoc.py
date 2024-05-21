# "goToLoc": "{goVerb} {toLocPrep} the {loc_room} then {followup}",
def goToLoc(g, params):
    # Go to the storage rack then look for a dish and take it and bring it to me
    # Go to the bedroom then find a food and get it and bring it to the waving person in the kitchen
    # Go to the living room then find a fruit and get it and give it to me
    # Go to the kitchen then look for a cleaning supply and grasp it and bring it to the standing person in the kitchen
    
    # [0] Extract parameters
    goVerb, toLocPrep, loc_room, cmd = params['goVerb'], params['toLocPrep'], params['loc_room'], params['followup']
    
    # [1] Move to the specified room
    print(f"I'm moving to the {loc_room}")
    agent.say(f"I'm moving to the {loc_room}")
    move_gpsr(agent, loc_room)

    # [2] Handling the follow-up action
    followup(cmd)