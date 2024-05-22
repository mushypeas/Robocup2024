from detectCLIP import init_clip, detectTopColor, detectTopClothe
from PIL import Image
import torch

def move_gpsr(agent, room):
    # Placeholder for the move_gpsr function
    print("[HSR] : I'm moving !")

def countClothPrsInRoom(agent, params):

    # [0] Extract parameters
    room, color = params['room'], params['colorClothes']

    desired_color, desired_clothe = params['colorClothes'].split()

    # Move to the specified room
    move_gpsr(agent, room)

    # [2] Check the number of people wearing the specified color    # Initialize the CLIP model and necessary components for detection
    clip_model, preprocess, tokenizer, device = init_clip()

    count = 0
    img_path = 'examples/shoes.jpg'
    img = Image.open(img_path)

    # [3] Process the image with the CLIP model
    img_processed = preprocess(img).unsqueeze(0).to(device)

    for img in img_processed:
        # Detect the top color of the clothes
        top_color, top_color_prob = detectTopColor(img, clip_model, preprocess, tokenizer, device)
        # Detect the type of top clothes
        top_clothe, top_clothe_prob = detectTopClothe(img, clip_model, preprocess, tokenizer, device)
        print (f"Top color: {top_color} ({top_color_prob})")
        print (f"Top clothe: {top_clothe} ({top_clothe_prob})")
        # Check if the detected color and clothe type match the desired ones
        if top_color == desired_color and top_clothe == desired_clothe:
            count += 1

    # Print the result
    print(f"[COUNT] {count} people in the {room} are wearing {desired_color} {desired_clothe}")
