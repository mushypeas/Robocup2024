import os

# 이전 class_names와 이후 class_names 리스트

old_class_names = [
    'yellow_bag','white_bag'
]

new_class_names = [
   'friss', 'water', 'fanta', 'coke_can', 'milk', 'ice_tea', 'coke_big',
    'banana', 'apple', 'strawberry', 'pear', 'peach', 'lemon', 'orange', 'plum',
    'pringles', 'crisps_red', 'candy_red', 'candy_purple', 'liquorice', 'stroopwaffles',
    'tictac', 'pea_soup', 'cornflakes', 'mayonnaise', 'curry', 'hagelslag_blue', 
    'hagelslag_red', 'pancake_mix', 'sausage', 'plate', 'fork', 'bowl', 'spoon',
    'mug', 'knife', 'soap', 'candle', '_apple_red', '_apple_yellow', '__tulip', 
    '__pastilles','yellow_bag','white_bag'
]

# 이전 class index를 이후 class index로 변환하는 매핑 딕셔너리 생성
class_mapping = {old_class_names.index(name): new_class_names.index(name) for name in old_class_names if name in new_class_names}

# class_mapping[22] = 35 # 병주: index는 22를 35로 변환 (부산대에서 cereal를 21,22 에다가 둘다 맵핑했기 때문에 발생한 문제를 땜빵한것임.)
print(class_mapping)

# 폴더 경로
input_folder = "./old/"
output_folder = "./new/"

# output 폴더가 없으면 생성
os.makedirs(output_folder, exist_ok=True)

# input 폴더의 모든 .txt 파일을 처리
for filename in os.listdir(input_folder):
    if filename.endswith(".txt"):
        input_path = os.path.join(input_folder, filename)
        output_path = os.path.join(output_folder, filename)

        # 파일 읽기
        with open(input_path, "r") as file:
            lines = file.readlines()

        # 변환된 내용 저장
        output_lines = []
        for line in lines:
            parts = line.strip().split()
            class_id = int(parts[0])
            if class_id in class_mapping:
                new_class_id = class_mapping[class_id]
                parts[0] = str(new_class_id)
                output_lines.append(" ".join(parts))

        # 파일 쓰기
        with open(output_path, "w") as file:
            file.write("\n".join(output_lines))

print("모든 파일이 성공적으로 업데이트되었습니다.")