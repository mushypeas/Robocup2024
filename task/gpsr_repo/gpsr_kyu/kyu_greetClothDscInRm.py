from module.CLIP.clip_detection import init_clip, detectTopColor, detectTopClothe
from PIL import Image
import torch

# "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
def move_gpsr(agent, room):
    # Placeholder for the move_gpsr function
    print("[HSR] : I'm moving !")

def followup(agent):
    # Placeholder for the move_gpsr function
    print("[HSR] : I'm doing followup function !")

def greetClothDscInRm(agent, params):
    # Salute the person wearing a blue t shirt in the living room and follow them to the kitchen
    # Introduce yourself to the person wearing an orange coat in the bedroom and answer a quiz
    # Greet the person wearing a blue t shirt in the bedroom and answer a question
    # Introduce yourself to the person wearing a gray t shirt in the kitchen and say something about yourself
    # params = {'greetVerb': 'Salute', 'art': 'a', 'colorClothe': 'blue t shirt', 'inLocPrep': 'in', 'room': 'living room', 'followup': 'follow them to the kitchen'}
    
    # [0] Extract parameters
    greet, art, color_clothe, room, cmd = params['greetVerb'], params['art'], params['colorClothe'], params['room'], params['followup']
    parts = color_clothe.split()
    desired_color = parts[0]
    desired_clothe = ' '.join(parts[1:])

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # Initialize the CLIP model and necessary components for detection
    clip_model, preprocess, tokenizer, device = init_clip()

    count = 0
    img_path = 'examples/shoes.jpg'
    img = Image.open(img_path)

    # [3] Process the image with the CLIP model
    img_processed = preprocess(img).to(device)
    

    # [3] Process the image with the CLIP model
    img_processed = preprocess(img).unsqueeze(0).to(device)
    
    for img in img_processed:
        # Detect the top color of the clothes
        top_color, top_color_prob = detectTopColor(img, clip_model, preprocess, tokenizer, device)
        # Detect the type of top clothes
        top_clothe, top_clothe_prob = detectTopClothe(img, clip_model, preprocess, tokenizer, device)
        
        print (f"Top color: {top_color} ({top_color_prob}%)")
        print (f"Top clothe: {top_clothe} ({top_clothe_prob}%)")
        
        # [3] Check if the detected color and clothe type match the desired ones
        if top_color == desired_color and top_clothe == desired_clothe:
            print(f"[GREET] {greet} the person wearing {art} {color_clothe} in the {room}")
            # [4] Generate the followup command
            followup(cmd)
        else:
            print(f"[INFO] No person wearing {art} {color_clothe} found in the {room}")
