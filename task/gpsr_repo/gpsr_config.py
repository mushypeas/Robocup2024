task_iteration = 3

ABS_POSITION = {
    "gpsr_instruction_point": [2.130, -0.151, 0.097],
    "bed": [6.837, 2.962, -1.556],
    "bedside table": [6.837, 2.962, -1.556],
    "shelf": [2.169, -0.714, -3.107],
    "trashbin": [7.419, 3.242, 1.655],
    "dishwasher": [2.264, -1.519, -3.113],
    "potted plant": [2.264, -1.519, -3.113],
    "kitchen table": [3.040, -0.081, -1.048],
    "chairs": [4.936, 1.975, -3.097],
    "pantry": [2.561, -2.073, -3.108],
    "refrigerator": [2.561, -2.073, -3.108], # = pantry
    "sink": [2.561, -2.073, -1.508], # = pantry
    "cabinet": [2.473, -0.666, -3.033],
    "coatrack": [2.473, -0.666, -3.033], # = cabinet
    "desk": [4.841, 1.402, -3.128],
    "armchair": [4.841, 1.402, -3.128],
    "desk lamp": [4.841, 1.402, -3.128], # = armchair, desk
    "waste basket": [7.419, 3.242, 1.655],
    "tv stand": [6.498, -0.274, 1.212],
    "storage rack": [2.169, -0.714, -3.107],
    "lamp": [4.841, 1.402, -3.128],
    "side tables": [4.841, 1.402, -3.128],
    "sofa": [5.888, -0.125, -0.491],
    "bookshelf": [2.169, -0.714, -3.107],
    "entrance": [0, 0, 0],
    "exit": [0, 0, 0],
    "bedroom": [6.153, 3.546, -0.617],
    "kitchen": [2.491, -0.317, -1.564],
    "office": [4.841, 1.402, -3.128],
    "living room": [5.888, -0.125, -0.491],
    "bathroom": [5.888, -0.125, -0.491]
}

objects_file_path = 'CompetitionTemplate/objects/test.md'
rooms_list = ["living room", "bedroom", "kitchen", "office", "bathroom"]
names_list = ['Adel', 'Angel', 'Axel', 'Charlie', 'Jane', 'Jules', 'Morgan', 'Paris', 'Robin', 'Simone']




gesture_person_list = ["waving person", "person raising their left arm", "person raising their right arm",
                           "person pointing to the left", "person pointing to the right"]
pose_person_list = ["sitting person", "standing person", "lying person"]
# Ugly...
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