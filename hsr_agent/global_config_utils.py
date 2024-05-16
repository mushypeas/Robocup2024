import os
from object_list_dict import name_to_grasping_type

def make_object_list(yolo_classname_path):
    # yolo_classname path
    file_path = os.path.join('..', 'module', 'yolov7', yolo_classname_path)

    with open(file_path, 'r', encoding='utf-8') as f:
        class_names = [line.strip() for line in f.readlines()]
        
    OBJECT_LIST = []
        
    for idx, class_name in enumerate(class_names):
        # I thk we don't need itemtype anymore!
        # try:
        #     itemtype = name_to_itemtype[class_name]
        # except KeyError:
        #     itemtype = 0
            
        try:
            grasping_type = name_to_grasping_type[class_name]
        
        except KeyError:
            # 이름 좀 달라도 알잘딱으로 grasping type 찾아줌 
            class_name_replace = [class_name.replace(' ', '_')]
            class_name_replace += [class_name.replace(' ', '')]
            class_name_replace += [class_name.replace('_', ' ')]
            class_name_replace += [class_name.replace('_', '')]
            
            for alt_class_name in class_name_replace:
                if alt_class_name in name_to_grasping_type:
                    grasping_type = name_to_grasping_type[alt_class_name]
                    break
                
            for arb_class_name in name_to_grasping_type:
                if arb_class_name.replace(' ', '_') in class_name:
                    grasping_type = name_to_grasping_type[arb_class_name]
                    break
                elif arb_class_name.replace(' ', '') in class_name:
                    grasping_type = name_to_grasping_type[arb_class_name]
                    break
                elif arb_class_name.replace('_', ' ') in class_name:
                    grasping_type = name_to_grasping_type[arb_class_name]
                    break
                elif arb_class_name.replace('_', '') in class_name:
                    grasping_type = name_to_grasping_type[arb_class_name]
                    break
                
            grasping_type = 0
            
        OBJECT_LIST.append([idx, class_name, 0, grasping_type])
        
    return OBJECT_LIST

if __name__ == '__main__':
    OBJECT_LIST = make_object_list('weight/test.cn')
    print(OBJECT_LIST)