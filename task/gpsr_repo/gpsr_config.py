task_iteration = 3
gpsr_human_head_tilt = 0.0

yolo_weight_path = 'weight/gpsr_pnu.pt'
yolo_classnames_path = 'weight/gpsr_pnu.cn'

## Official Github

ABS_POSITION = {
    ## locations
    'hallway cabinet': [0, 0, 0],
    'entrance': [0, 0, 0],
    'desk': [0, 0, 0],
    'shelf': [0, 0, 0],
    'coathanger': [0, 0, 0],
    'exit': [0, 0, 0],
    'TV table': [0, 0, 0],
    'lounge chair': [0, 0, 0],
    'lamp': [0, 0, 0],
    'couch': [0, 0, 0],
    'coffee table': [0, 0, 0],
    'trashcan': [0, 0, 0],
    'kitchen cabinet': [0, 0, 0],
    'dinner table': [0, 0, 0],
    'dishwasher': [0, 0, 0],
    'kitchen counter': [0, 0, 0],
    
    ## rooms
    'hallway': [0, 0, 0],
    'office': [0, 0, 0],
    'kitchen': [0, 0, 0],
    'living room': [0, 0, 0]
}

objects_file_path = 'CompetitionTemplate/objects/objects.md'

rooms_list = ["hallway", "office", "kitchen", "living room"]
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

## Command Generator

gesture_person_list = ["waving person", "person raising their left arm", "person raising their right arm",
                           "person pointing to the left", "person pointing to the right"]
pose_person_list = ["sitting person", "standing person", "lying person"]

gesture_person_plural_list = ["waving persons", "persons raising their left arm", "persons raising their right arm",
                                "persons pointing to the left", "persons pointing to the right"]
pose_person_plural_list = ["sitting persons", "standing persons", "lying persons"]

person_info_list = ["name", "pose", "gesture"]
object_comp_list = ["biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"]

talk_list = ["something about yourself", "the time", "what day is today", "what day is tomorrow", "your teams name",
                "your teams country", "your teams affiliation", "the day of the week", "the day of the month"]
question_list = ["question", "quiz"]

color_list = ["blue", "yellow", "black", "white", "red", "orange", "gray"]
clothe_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket"]
clothes_list = ["t shirts", "shirts", "blouses", "sweaters", "coats", "jackets"]