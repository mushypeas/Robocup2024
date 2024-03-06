import torchvision.models as models
import torchvision.transforms as transforms
from PIL import Image
import torch
from torch.autograd import Variable
import pandas as pd
import numpy as np
import cv2

class HumanId:
    def __init__(self):
        self.scaler = transforms.Resize((640, 320))
        self.normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        self.to_tensor = transforms.ToTensor()

        self.device = torch.device('cuda')

        # feature extract model
        # self.model = models.resnet152(pretrained=True)

        #self.model = models.resnet50(pretrained=True)
        #self.model = models.resnet101(pretrained=True)
        #self.model = models.resnext101_32x8d(pretrained=True)
        #self.model = models.resnext50_32x4d(weights='DEFAULT')
        self.model = models.efficientnet_b7(weights='DEFAULT')
        # self.model = models.regnet_y_32gf(weights='DEFAULT')


        self.model.to(self.device)
        self.layer = self.model._modules.get('avgpool')
        self.model.eval()
        # self.full_database_features = self.get_all_database_tensor(self.database_df)

    def get_vector(self, im2=None, tensor=False):
        print('start feature extraction')
        # convert from BGR to RGB
        color_coverted = cv2.cvtColor(im2, cv2.COLOR_BGR2RGB)
        # convert from openCV2 to PIL
        im2 = Image.fromarray(color_coverted)
        t_img = Variable(self.normalize(self.to_tensor(self.scaler(im2))).unsqueeze(0))
        t_img = t_img.to(self.device)

        resnet_model_size = 2048
        efficientnet_b7_size = 2560
        regnet_y_32gf_size = 3712

        model_size = efficientnet_b7_size
        my_embedding = torch.zeros(model_size)

        def copy_data(m, i, o):
            my_embedding.copy_(o.data.reshape(o.data.size(1)))
        h = self.layer.register_forward_hook(copy_data)
        self.model(t_img)
        h.remove()
        if tensor:
            return my_embedding

        return my_embedding.numpy()

    def get_multi_vector(self, rgb_img, human_info_arrays, target_feature):
        full_tensor = None

        for human in human_info_arrays:
            x, y, w, h = int(human.tlwh[0]), int(human.tlwh[1]), int(human.tlwh[2]), int(human.tlwh[3])
            if x < 0: x = 0
            if y < 0: y = 0

            im2 = rgb_img[y:y + h, x:x + w]
            feature = self.get_vector(im2=im2, tensor=True)
            feature = torch.reshape(feature, (1, -1))
            if full_tensor is None:
                full_tensor = feature
                continue
            full_tensor = torch.cat([full_tensor, feature], dim=0)

        sim_np = self.cos_sim(full_tensor, target_feature)
        checked_id = np.argmax(sim_np)
        return checked_id

    def cos_sim(self, A, B):
        return [np.dot(i, B) / (np.linalg.norm(i)*np.linalg.norm(B)) for i in A]
