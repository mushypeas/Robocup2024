# "guideClothPrsFromBeacToBeac": "{guideVerb} the person wearing a {colorClothe} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}"
def guideClothPrsFromBeacToBeac(g, params):
    # Take the person wearing a white shirt from the entrance to the trashbin
    # Escort the person wearing a yellow shirt from the pantry to the kitchen
    # Lead the person wearing a yellow t shirt from the kitchen table to the sofa
    # Escort the person wearing a blue sweater from the kitchen table to the office
    # Take the person wearing a gray sweater from the storage rack to the living room
    print("Start guideClothPrsFromBeacToBeac")

    # [0] Extract parameters
    try:
        colorClothe = params['colorClothe']
        loc = params['loc']
        loc2 = params['loc_room']
    except:
        g.cmdError()
        return
    
    colorClothes_list = []

    for color in g.color_list:
        for clothes in g.clothes_list:
            colorClothes_list.append(color + " " + clothes)

    colorClothe = g.cluster(colorClothe, colorClothes_list)
    loc = g.cluster(loc, g.loc_list)
    loc2 = g.cluster(loc2, g.loc_list)

    # [1] Move to the specified location
    g.move(loc)

    # [2] Find the person in the location
    g.identifyByClothing(colorClothe)

    # [3] Make the person to follow HSR to the room
    g.guide(loc2)

    g.task_finished_count += 1