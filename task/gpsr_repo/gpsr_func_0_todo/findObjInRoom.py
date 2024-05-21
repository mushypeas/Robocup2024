# "findObjInRoom": "{findVerb} {art} {obj_singCat} {inLocPrep} the {room} then {followup}",
def findObjInRoom(g, params):
    
    # params = {'findVerb': 'Find', 'art': 'a', 'obj_singCat': 'food', 'inLocPrep': 'in', 'room': 'kitchen', 'followup': 'take it and bring it to me'}
    
    # [0] Extract parameters
    findVerb, art, obj, room = params['findVerb'], params['art'], params['obj_singCat'], params['room']
    cmd = params['followup']
    
    # [1] Move to the specified room
    print(f"I'm moving to the {room}")
    move_gpsr(agent, room)
    
    # [2] Find the object in the room
    print(f"Let me find {art} {obj} in the {room}")
    
    found = False
    detected_objects = agent.yolo_module.yolo_bbox
    # detected_objects = [{'name': 'food', 'bbox': (10, 20, 30, 40)}, 
    #                     {'name': 'drink', 'bbox': (50, 60, 70, 80)}]
    
    for obj_data in detected_objects:
        obj_name, obj_bbox = obj_data['name'], obj_data['bbox']
        if obj_name == obj:
            print(f"{obj} is found at {obj_bbox}")
            found = True
            break
    
    # [3] Do follow-up action
    if found:
        followup(cmd)
        # Here you would add the code to manipulate the object as described in `followup`
        # This might involve additional robot movement or manipulator actions
    else:
        print(f"{obj} is not found in the {room}")