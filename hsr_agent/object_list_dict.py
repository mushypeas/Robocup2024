# generated from gen_object_list_dict()
# name_to_itemtype = {'water': 4, 'milk': 4, 'coke': 4, 'tonic': 4, 'bubble_tea': 0, 'ice tea': 4, 'cereal': 5, 'tuna_can': 2, 'coffee_jar': 2, 'sugar': 0, 'mustard': 0, 'apple': 1, 'peach': 1, 'orange': 1, 'banana': 1, 'strawberry': 1, 'pockys': 4, 'pringles': 0, 'spoon': 2, 'fork': 2, 'plate': 2, 'bowl': 2, 'mug': 2, 'knife': 2, 'paperbag': 1, 'tab': 0, 'cracker': 0, 'jello_red': 2, 'jello_black': 2, 'coffee_can': 2, 'tomato_soup': 2, 'pear': 1, 'plum': 1, 'lemon': 1, 'tennis_ball': 4, 'golf_ball': 4, 'base_ball': 4, 'soccer_ball': 4, 'soft_ball': 4, 'cube': 5, 'dice': 4, 'wine': 1, 'ice_tea': 1, 'orange_juice': 1, 'tropical_juice': 1, 'juice_pack': 1, 'cereal_red': 1, 'cereal_yellow': 1, 'sponge': 0, 'scrub': 0, 'spam': 0, 'shopping_bag_1': 7, 'shopping_bag_2': 7, 'shopping_bag_3': 7, 'cereal_black': 7, 'dishwasher_tablet': 7, 'jello red': 0, 'jello black': 0, 'coffee can': 0, 'tuna can': 0, 'tomato soup': 0, 'tennis ball': 3, 'golf ball': 3, 'base ball': 3, 'bubble tea': 4, 'shopping bag': 5, 'dishwasher tablet': 5} 

# generated from gen_object_list_dict()
name_to_grasping_type = {'water': 0, 'milk': 0, 'coke': 0, 'tonic': 0, 'bubble_tea': 0, 'ice tea': 0, 'cereal': 0, 'tuna_can': 0, 'coffee_jar': 0, 'sugar': 0, 'mustard': 0, 'apple': 0, 'peach': 0, 'orange': 0, 'banana': 0, 'strawberry': 0, 'pockys': 0, 'pringles': 0, 'spoon': 1, 'fork': 1, 'plate': 3, 'bowl': 2, 'mug': 0, 'knife': 1, 'paperbag': 1, 'tab': 0, 'cracker': 0, 'jello_red': 0, 'jello_black': 0, 'coffee_can': 0, 'tomato_soup': 0, 'pear': 0, 'plum': 0, 'lemon': 0, 'tennis_ball': 0, 'golf_ball': 0, 'base_ball': 0, 'soccer_ball': 0, 'soft_ball': 0, 'cube': 0, 'dice': 0, 'wine': 0, 'ice_tea': 0, 'orange_juice': 0, 'tropical_juice': 0, 'juice_pack': 0, 'cereal_red': 0, 'cereal_yellow': 0, 'sponge': 1, 'scrub': 0, 'spam': 0, 'shopping_bag_1': 0, 'shopping_bag_2': 0, 'shopping_bag_3': 0, 'cereal_black': 0, 'dishwasher_tablet': 0, 'jello red': 0, 'jello black': 0, 'coffee can': 0, 'tuna can': 0, 'tomato soup': 0, 'tennis ball': 0, 'golf ball': 0, 'base ball': 0, 'bubble tea': 0, 'shopping bag': 0, 'dishwasher tablet': 1}

def gen_object_list_dict():
    # Append all (Legacy) OBJECT_LIST
    all_objects = [
        ['water', 0, 0, 0],
        ['milk', 1, 0, 0],
        ['coke', 2, 0, 0],
        ['tonic', 3, 0, 0],
        ['bubble_tea', 4, 0, 0],
        ['ice tea', 5, 0, 0],
        ['cereal', 6, 2, 0],
        ['tuna_can', 7, 2, 0],
        ['coffee_jar', 8, 2, 0],
        ['sugar', 9, 2, 0],
        ['mustard', 10, 2, 0],
        ['apple', 11, 3, 0],
        ['peach', 12, 3, 0],
        ['orange', 13, 3, 0],
        ['banana', 14, 3, 0],
        ['strawberry', 15, 3, 0],
        ['pockys', 16, 4, 0],
        ['pringles', 17, 4, 0],
        ['spoon', 18, 5, 1],
        ['fork', 19, 5, 1],
        ['plate', 20, 5, 3],
        ['bowl', 21, 5, 2],
        ['mug', 22, 5, 0],
        ['knife', 23, 5, 1],
        ['paperbag', 24, 1, 1],
        ['tab', 25, 0, 0],
        ['cracker', 0, 5, 0],
        ['sugar', 1, 2, 0],
        ['jello_red', 2, 2, 0],
        ['jello_black', 3, 2, 0],
        ['coffee_can', 4, 2, 0],
        ['tuna_can', 5, 2, 0],
        ['pringles', 6, 5, 0],
        ['mustard', 7, 2, 0],
        ['tomato_soup', 8, 2, 0],
        ['pear', 9, 3, 0],
        ['peach', 10, 3, 0],
        ['apple', 11, 3, 0],
        ['strawberry', 12, 3, 0],
        ['orange', 13, 3, 0],
        ['banana', 14, 3, 0],
        ['plum', 15, 3, 0],
        ['lemon', 16, 3, 0],
        ['bowl', 5, 6, 2],
        ['mug', 0, 6, 0],
        ['plate', 4, 6, 3],
        ['knife', 1, 6, 1],
        ['fork', 3, 6, 1],
        ['spoon', 2, 6, 1],
        ['tennis_ball', 23, 4, 0],
        ['golf_ball', 24, 4, 0],
        ['base_ball', 25, 4, 0],
        ['soccer_ball', 26, 4, 0],
        ['soft_ball', 27, 4, 0],
        ['cube', 28, 4, 0],
        ['dice', 29, 4, 0],
        ['wine', 30, 1, 0],
        ['ice_tea', 31, 1, 0],
        ['orange_juice', 32, 1, 0],
        ['milk', 33, 1, 0],
        ['tropical_juice', 34, 1, 0],
        ['juice_pack', 35, 1, 0],
        ['cereal_red', 36, 1, 0],
        ['cereal_yellow', 37, 1, 0],
        ['coke', 38, 1, 0],
        ['sponge', 39, 0, 1],
        ['scrub', 40, 0, 0],
        ['spam', 41, 2, 0],
        ['shopping_bag_1', 42, 7, 0],
        ['shopping_bag_2', 43, 7, 0],
        ['shopping_bag_3', 44, 7, 0],
        ['cereal_black', 45, 7, 0],
        ['dishwasher_tablet', 46, 7, 0],
        ['cracker', 0, 0, 0],
        ['sugar', 1, 0, 0],
        ['jello red', 2, 0, 0],
        ['jello black', 3, 0, 0],
        ['spam', 4, 0, 0],
        ['coffee can', 5, 0, 0],
        ['tuna can', 6, 0, 0],
        ['pringles', 7, 0, 0],
        ['mustard', 8, 0, 0],
        ['tomato soup', 9, 0, 0],
        ['pear', 10, 1, 0],
        ['peach', 11, 1, 0],
        ['apple', 12, 1, 0],
        ['strawberry', 13, 1, 0],
        ['orange', 14, 1, 0],
        ['plum', 15, 1, 0],
        ['lemon', 16, 1, 0],
        ['bowl', 17, 2, 2],
        ['mug', 18, 2, 0],
        ['plate', 19, 2, 3],
        ['knife', 20, 2, 1],
        ['fork', 21, 2, 1],
        ['spoon', 22, 2, 1],
        ['tennis ball', 23, 3, 0],
        ['golf ball', 24, 3, 0],
        ['base ball', 25, 3, 0],
        ['water', 26, 4, 0],
        ['bubble tea', 27, 4, 0],
        ['tonic', 28, 4, 0],
        ['coke', 29, 4, 0],
        ['ice tea', 30, 4, 0],
        ['milk', 31, 4, 0],
        ['cereal', 32, 5, 0],
        ['shopping bag', 33, 5, 0],
        ['dishwasher tablet', 34, 5, 1],
        ['cube', 35, 5, 0],
        ['banana', 36, 1, 0]
    ]

    name_to_itemtype = {}
    name_to_grasping_type = {}

    for name, _, itemtype, grasping_type in all_objects:
        name_to_itemtype[name] = itemtype
        name_to_grasping_type[name] = grasping_type


    print(name_to_itemtype, '\n')
    print(name_to_grasping_type)
    
    return name_to_itemtype, name_to_grasping_type

if __name__ == '__main__':
    name_to_itemtype, name_to_grasping_type = gen_object_list_dict()