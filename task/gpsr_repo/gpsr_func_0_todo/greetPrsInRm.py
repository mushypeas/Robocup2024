# "greetNameInRm": "{greetVerb} {name} {inLocPrep} the {room} and {followup}",
def greetNameInRm(g, params):
    # Salute Morgan in the living room and follow them to the storage rack
    # Say hello to Jules in the living room and tell the time
    # Say hello to Robin in the office and say your teams country
    # Greet Angel in the living room and say your teams name

    # params = {'greetVerb': 'Salute', 'name': 'Morgan', 'inLocPrep': 'in', 'room': 'living room', 'followup': 'follow them to the storage rack'}
    # [0] Extract parameters
    greet, name, room, cmd = params['greetVerb'], params['name'], params['room'], params['followup']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2][TODO] Find the person in the room
    print(f"[FIND] {name}")

    # [3] Generate the followup command
    # [3-1][TODO] Team Country, Name, Time...
    followup(cmd)