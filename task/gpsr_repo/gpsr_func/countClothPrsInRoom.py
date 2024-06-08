# "countClothPrsInRoom": "{countVerb} people {inLocPrep} the {room} are wearing {colorClothes}",
def countClothPrsInRoom(g, params):
    # Tell me how many people in the kitchen are wearing red jackets
    # Tell me how many people in the living room are wearing black jackets
    # Tell me how many people in the bathroom are wearing white jackets
    print("Start countClothPrsInRoom")

    # [0] Extract parameters
    try:
        room = params["room"]
        colorClothes = params["colorClothes"]
    except KeyError as e:
        g.cmdError()
        return
    
    colorClothes_list = []

    for color in g.color_list:
        for clothes in g.clothes_list:
            colorClothes_list.append(color + " " + clothes)

    colorClothes = g.cluster(colorClothes, colorClothes_list)

    # [1] Move to the specified room
    g.move(room)

    # [2] Check the number of people wearing the specified color
    count = g.countColorClothesPers(colorClothes)

    # [3] Output the count
    # TODO : Fix the grammar for singular and plural
    g.say(f"There are {count} people in the {room} wearing {colorClothes}")

    g.task_finished_count += 1