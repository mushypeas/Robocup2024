import torch
import open_clip

def init_clip():
    # Assuming the necessary OpenFashionClip setup
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    clip_model, _, preprocess = open_clip.create_model_and_transforms('ViT-B/32')
    state_dict = torch.load('module/CLIP/openfashionclip.pt', map_location=device)
    clip_model.load_state_dict(state_dict['CLIP'])
    clip_model = clip_model.eval().requires_grad_(False).to(device)
    tokenizer = open_clip.get_tokenizer('ViT-B-32')
    return clip_model, preprocess, tokenizer, device

def detect_feature(img, feature_list, prompt_prefix, clip_model, preprocess, tokenizer, device):
    prompts = [prompt_prefix + " " + feature for feature in feature_list]
    tokenized_prompt = tokenizer(prompts).to(device)
    
    with torch.no_grad():
        image = preprocess(img).unsqueeze(0).to(device)
        image_features = clip_model.encode_image(image)
        text_features = clip_model.encode_text(tokenized_prompt)
        image_features /= image_features.norm(dim=-1, keepdim=True)
        text_features /= text_features.norm(dim=-1, keepdim=True)
        
        text_probs = (100.0 * image_features @ text_features.T).softmax(dim=-1)
        
    text_probs_percent = text_probs * 100
    text_probs_percent_np = text_probs_percent.cpu().numpy()
    
    top_index = text_probs_percent_np[0].argmax()
    top_feature = feature_list[top_index]
    top_feature_prob = "{:.2f}%".format(text_probs_percent_np[0][top_index])
    
    return top_feature, top_feature_prob

def detectPose(img, clip_model, preprocess, tokenizer, device):
    leg_pose_list = ["standing", "sitting", "lying"]
    prompt_prefix = "There exists a person who is"
    return detect_feature(img, leg_pose_list, prompt_prefix, clip_model, preprocess, tokenizer, device)

def detectGest(img, clip_model, preprocess, tokenizer, device):
    gesture_list = ["raising their left arm", "raising their right arm", "pointing to the left", "pointing to the right", "waving", "without gesture"]
    prompt_prefix = "There exists a person who is"
    return detect_feature(img, gesture_list, prompt_prefix, clip_model, preprocess, tokenizer, device)

def detectColorCloth(img, clip_model, preprocess, tokenizer, device):
    color_list = ["blue", "yellow", "orange", "red", "gray", "black", "white"]
    color_prompt_prefix = "There exists a person who is wearing cloth with color"    
    color, _ = detect_feature(img, color_list, color_prompt_prefix, clip_model, preprocess, tokenizer, device)

    cloth_list = ["t shirt", "shirt", "blouse", "coat", "jacket", "sweater"]
    cloth_prompt_prefix = "There exists a person who is wearing"
    cloth, _ = detect_feature(img, cloth_list, cloth_prompt_prefix, clip_model, preprocess, tokenizer, device)

    return color + " " + cloth

def detectPersonCount(img, clip_model, preprocess, tokenizer, device, type=None, key=None):
    if type == "pose" or type == "gest":
        person_list = ["no person", "one person", "two people", "three people", "four people", "five people", "six people", "many people"]
        prompt_prefix = f"Number of person who is {key} is"
    elif type == "colorCloth":
        person_list = ["no person", "one person", "two people", "three people", "four people", "five people", "six people", "many people"]
        prompt_prefix = f"Number of person who is wearing {key} is"
    else:
        person_list = ["no person", "one person", "two people", "three people", "four people", "five people", "six people", "many people"]
        prompt_prefix = "A photo with"
    return detect_feature(img, person_list, prompt_prefix, clip_model, preprocess, tokenizer, device)