task_iteration = 3
gpsr_human_head_tilt = 0.0
gpsr_identify_head_tilt = 0.5

yolo_weight_path = 'weight/' + '0717v2.pt'
yolo_classnames_path = 'weight/' + 'classnames.cn'

## Official Github

ABS_POSITION = {
    'zero': [0, 0, 0],
    'gpsr_instruction_point': [6.037, -0.028, -0.737], ## living room
    ## locations
    'hallway cabinet': [1.844, -1.643, -3.129],
    'entrance': [-0.375, 0.042, -0.001],
    'desk': [2.2855, 2.9161, -1.5549],
    'shelf': [2.2343, 3.9577, 3.1078],
    'coathanger': [2.702, -0.657, 1.594],
    'exit': [3.889, 6.493, 1.555],
    'TV table': [5.730, -1.832, 3.113],
    'lounge chair': [6.903, -1.614, -0.013],
    'lamp': [6.903, -1.614, -0.013],
    'couch': [9.195, 0.481, -1.526],
    'coffee table': [7.0467, -1.1802, 0.0446],
    'trashcan': [5.279, 1.830, -2.469],
    'kitchen cabinet': [5.6019, 4.9918, 3.127],
    'dinner table': [5.6348, 3.881, -0.0264],
    'dishwasher': [8.5045, 3.6366, 0.0131],
    'kitchen counter': [8.5652, 4.8247, -0.0005],
    
    ## rooms
    'hallway': [1.709, -2.260, 0.707],
    'office': [1.584, 5.545, -1.036],
    'kitchen': [8.569, 1.708, 2.376],
    'living room': [6.037, -0.028, -0.737]
}

objects_file_path = 'CompetitionTemplate/objects/objects.md'

rooms_list = [
    "hallway", 
    "office", 
    "kitchen", 
    "living room"
]

def invert_dict(d):
    return {v: k for k, v in d.items()}

plcmt_dict = {
    'coffee table': 'fruit',
    'kitchen counter': 'food',
    'dishwasher': 'dish',
    'dinner table': 'snack',
    'kitchen cabinet': 'drink',
    'shelf': 'cleaning_supply',
    'desk': 'decoration'
}

cat_to_plcmt_loc = invert_dict(plcmt_dict)

plcmt_locs = list(plcmt_dict.keys())

names_list = [
    "Sophie", "Julia", "Emma", "Sara", "Laura", 
    "Hayley", "Susan", "Fleur", "Gabrielle", "Robin", 
    "John", "Liam", "Lucas", "William", "Kevin", 
    "Jesse", "Noah", "Harrie", "Peter", "Robin"
] #Gabriëlle

############################################################
############################################################
############################################################
############################################################

## Telegram

quiz_dict = {
    "What is the highest mountain in the Netherlands?": "The Vaalserberg is the highest mountain in the Netherlands, although parts of the mountain belong to Belgium and Germany.",
    "Which painter created the Night Watch?": "It was created by the Dutch painter Rembrandt.",
    "What is the largest lake in the Netherlands?": "The largest lake in the Netherlands is the Ijsselmeer.",
    "Who is the current Baron of Eindhoven?": "King Willem-Alexander of the Netherlands.",
    "When was Eindhoven first chartered?": "In 1232, by the duke of Brabant, Henry I.",
    "How many people live in Eindhoven?": "More than 200,000 people currently live in Eindhoven.",
    "What is the mascot for the 2024 RoboCup called?": "The official mascot for this year's RoboCup is called Robin.",
    "How low is the lowest point in the Netherlands?": "The lowest point of the Netherlands is -6.67m below sea level. It is located close to the A20.",
    "What was the Dutch currency before the Euro?": "The guilder was the currency of the Netherlands before the euro was introduced in 2002."
}

quiz_list = list(quiz_dict.keys())

############################################################
############################################################
############################################################
############################################################

## Command Generator

gesture_person_list = ["waving person", "person raising their left arm", "person raising their right arm",
                           "person pointing to the left", "person pointing to the right"]
pose_person_list = ["sitting person", "standing person", "lying person"]

gesture_person_plural_list = ["waving persons", "persons raising their left arm", "persons raising their right arm",
                                "persons pointing to the left", "persons pointing to the right"]
pose_person_plural_list = ["sitting persons", "standing persons", "lying persons"]

person_info_list = ["name", "pose", "gesture"]
object_comp_list = ["biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"]

talk_list = ["something about yourself", "the time", "what day is today", "what day is tomorrow", "your teams name", "your teams country", "your teams affiliation", "the day of the week", "the day of the month"]
question_list = ["question", "quiz"]

color_list = ["blue", "yellow", "black", "white", "red", "orange", "gray"]
clothe_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket"]
clothes_list = ["t shirts", "shirts", "blouses", "sweaters", "coats", "jackets"]