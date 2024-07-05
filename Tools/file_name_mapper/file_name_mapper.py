import os
import shutil

plus_num = 1000

def rename_files(old_directory, new_directory, num):
    if not os.path.exists(new_directory):
        os.makedirs(new_directory)
        
    for filename in os.listdir(old_directory):
        name, extension = os.path.splitext(filename)
        
        if not name.isdigit():
            raise ValueError(f"Filename '{filename}' is not a number.")
            
        new_filename = str(int(name) + num) + extension
        shutil.copy2(os.path.join(old_directory, filename), os.path.join(new_directory, new_filename))

# Example usage
old_directory = "./old"
new_directory = "./new"

rename_files(old_directory, new_directory, plus_num)