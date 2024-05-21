# "countClothPrsInRoom": "{countVerb} people {inLocPrep} the {room} are wearing {colorClothes}",
def countClothPrsInRoom(g, params):
    # Tell me how many people in the kitchen are wearing red jackets
    # Tell me how many people in the living room are wearing black jackets
    # Tell me how many people in the bathroom are wearing white jackets
    
    # [0] Extract parameters
    room, color = params['room'], params['colorClothes']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] Check the number of people wearing the specified color
    # [TODO] Color Detection, Clothes Detection
    count = 0
    print(f"[COUNT] {count} people in the {room} are wearing {color}")

    # params = {countVerb: 'Tell me how many', room: 'kitchen', colorClothes: 'red jackets'}
    