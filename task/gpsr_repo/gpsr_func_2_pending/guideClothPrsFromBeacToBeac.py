# [TODO] 자세한 시나리오 구성 필요 Ex. 시야 내부에서 해당 cloth를 입은 상대 앞으로 이동한 후 eye gaze 한 상태 등
from module.CLIP.clip_detection import init_clip, detectTopColor, detectTopClothe
from PIL import Image
import torch

# Placeholder for the move_gpsr function
def move_gpsr(agent, loc):
    print("[HSR] : I'm moving to the {}!".format(loc))

# Main function
def guideClothPrsFromBeacToBeac(agent, params):
    # Take the person wearing a white shirt from the entrance to the trashbin
    # Escort the person wearing a yellow shirt from the pantry to the kitchen
    # Lead the person wearing a yellow t-shirt from the kitchen table to the sofa
    # Escort the person wearing a blue sweater from the kitchen table to the office
    # Take the person wearing a gray sweater from the storage rack to the living room
    # Example params = {'guideVerb': 'Take', 'colorClothe': 'white shirt', 'fromLocPrep': 'from', 'loc': 'entrance', 'toLocPrep': 'to', 'loc_room': 'trashbin'}

    # [0] Extract parameters
    guide, color_clothe, from_loc, to_room = params['guideVerb'], params['colorClothe'], params['loc'], params['loc_room']
    parts = color_clothe.split()
    desired_color = parts[0]
    desired_clothe = ' '.join(parts[1:])

    # [1] Move to the specified location
    move_gpsr(agent, from_loc)

    # Initialize the CLIP model and necessary components for detection
    clip_model, preprocess, tokenizer, device = init_clip()

    # Placeholder for capturing images from the specified location
    img_path = 'examples/shoes.jpg'
    img = Image.open(img_path)

    # [3] Process the image with the CLIP model
    img_processed = preprocess(img).unsqueeze(0).to(device)

    # [4] Detect the person in the location
    print(f"[FIND] let me find the person wearing a {color_clothe} in the {from_loc}")
    for img in img_processed:
        # Detect the top color of the clothes
        top_color, top_color_prob = detectTopColor(img, clip_model, preprocess, tokenizer, device)
        # Detect the type of top clothes
        top_clothe, top_clothe_prob = detectTopClothe(img, clip_model, preprocess, tokenizer, device)
        
        print(f"Top color: {top_color} ({top_color_prob}%)")
        print(f"Top clothe: {top_clothe} ({top_clothe_prob}%)")
        
        # [5] Check if the detected color and clothe type match the desired ones
        if top_color == desired_color and top_clothe == desired_clothe:
            # [6] Make the person follow HSR to the room
            # 해당 사람 앞에 서서 따라오라고 말하기
            print(f"[GUIDE] Hi, I'll {guide} you the {from_loc} to the {to_room}")
            move_gpsr(agent, to_room)
        else:
            print(f"[INFO] No person wearing a {color_clothe} found in the {from_loc}")

    print("[HSR] : I'm done !")