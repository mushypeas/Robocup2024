task_iteration = 3
gpsr_human_head_tilt = 0.0

yolo_weight_path = 'weight/gpsr_pnu.pt'
yolo_classnames_path = 'weight/gpsr_pnu.cn'

ABS_POSITION = {
    "gpsr_instruction_point": [2.024, -2.309, -0.018],
    "bed": [5.532, -6.343, -0.716],
    "bedside table": [5.532, -6.343, -0.716],
    "shelf": [2.565, -2.458, -3.072],
    "trashbin": [6.243, 0.243, 0.852],
    "dishwasher": [6.421, -2.626, -1.449],
    "potted plant": [6.421, -2.626, -1.449],
    "kitchen table": [4.979, -1.283, -0.088],
    "chairs": [3.505, -4.892, -3.091],
    "pantry": [5.288, -0.300, 1.606],
    "refrigerator": [6.285, -6.537, -0.094],
    "sink": [5.288, -0.300, 1.606], 
    "cabinet": [5.288, -0.300, 1.606],
    "coatrack": [5.288, -0.300, 1.606],
    "desk": [3.505, -4.892, -3.091],
    "armchair": [3.505, -4.892, -3.091],
    "desk lamp": [3.505, -4.892, -3.091],
    "waste basket": [6.243, 0.243, 0.852],
    "tv stand": [2.896, -2.714, -2.395],
    "storage rack": [5.288, -0.300, 1.606],
    "lamp": [3.505, -4.892, -3.091],
    "side tables": [3.505, -4.892, -3.091],
    "sofa": [3.129, -1.708, 1.596],
    "bookshelf": [3.306, -6.430, -1.552],
    "entrance": [0, 0, 0],
    "exit": [0, 0, 0],

    "bedroom": [4.665, -4.642, -0.752],
    "kitchen": [4.375, -3.410, 1.028],
    "office": [3.863, -4.498, -2.332],
    "living room": [2.873, -2.994, 1.622],
    "bathroom": [2.873, -2.994, 1.622],
}

objects_file_path = 'CompetitionTemplate/objects/objects.md'

rooms_list = ["living room", "bedroom", "kitchen", "office", "bathroom"]
names_list = ['Adel', 'Angel', 'Axel', 'Charlie', 'Jane', 'Jules', 'Morgan', 'Paris', 'Robin', 'Simone']

############################################################
############################################################
############################################################
############################################################

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