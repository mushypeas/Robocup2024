from face_id import FaceId
from PIL import Image
import glob

image_name = glob.glob('/face_detection/user_face_database/jk_mask.jpg')[0]
img = Image.open(image_name)

face_id = FaceId()
checked_id = face_id.check_id(img)
print(checked_id)