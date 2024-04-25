import os
import numpy as np
import matplotlib.pyplot as plt

from PIL import Image
from sklearn.decomposition import PCA
from sklearn.manifold import TSNE
from transformers import CLIPProcessor, CLIPModel, CLIPConfig
import torch
from torchvision import transforms

# Available Config modes: shoes, drink
class CLIPDetectorConfig:
    def __init__(self, name: str, labels: list, positive_texts: list, negative_texts: list, neutral_texts: list, threshold: float):
        self.positive_index = len(positive_texts)
        self.negative_index = len(positive_texts + negative_texts)
        self.texts = positive_texts + negative_texts + neutral_texts
        self.threshold = threshold
        # Used for local mode only
        self.name = name
        self.labels = labels


class CLIPDetector:
    def __init__(self, config: CLIPDetectorConfig, mode,):
        self.mode = mode
        self.set_config(config)
        self.pretrained_model = "openai/clip-vit-large-patch14"
        # self.pretrained_model = "openai/clip-vit-base-patch32"
        
        self.model = CLIPModel.from_pretrained(self.pretrained_model)
        self.processor = CLIPProcessor.from_pretrained(self.pretrained_model)

        # GPU
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print('Device: ', self.device)
        self.model.to(self.device)

    def set_config(self, config: CLIPDetectorConfig):
        self.name = config.name
        self.labels = config.labels
        self.positive_index = config.positive_index
        self.negative_index = config.negative_index
        self.texts = config.texts
        self.threshold = config.threshold


    def evaluate(self, images, image_names=None, labels=None):
        
        # GPU
        transform = transforms.Compose([transforms.ToTensor()])
        images = transform(images).unsqueeze(0).to(self.device)

        inputs = self.processor(
            text=self.texts,
            images=images,
            return_tensors="pt",
            padding=True
        )

        # GPU
        inputs['input_ids'] = inputs['input_ids'].to(self.device)
        inputs['attention_mask'] = inputs['attention_mask'].to(self.device)
        inputs['pixel_values'] = inputs['pixel_values'].to(self.device)

        outputs = self.model(**inputs)

        logits_per_image = outputs.logits_per_image  # this is the image-text similarity score
        # probs = logits_per_image.softmax(dim=1).detach().numpy()  # we can take the softmax to get the label probabilities

        # GPU
        probs = logits_per_image.softmax(dim=1).detach().cpu().numpy()

        # For testing on the HSR
        if self.mode == "HSR":
            prob = probs[0].round(3)
            positive_prob = np.sum(prob[:self.positive_index], dtype=np.float64).round(3) 
            negative_prob = np.sum(prob[self.positive_index:self.negative_index], dtype=np.float64).round(3) 
            neutral_prob = np.sum(prob[self.negative_index:], dtype=np.float64).round(3)
            print(f'PROBS: {prob} \n    {"Positive" if (positive_prob + neutral_prob) > negative_prob else "Negative"}')
            print(f'Positive prob: {positive_prob.round(3)}    Negative prob: {negative_prob.round(3)}    Neutral prob: {neutral_prob.round(3)}')
            return positive_prob, negative_prob, neutral_prob
        
        # For testing on the local dataset to calculate the accuracy
        elif self.mode == "local":
            correct = 0
            for image_name, label, prob in list(zip(image_names, labels, probs)):
                print(f"Image: {image_name}, Label: {label}\n")
                positive_prob = np.sum(prob[:self.positive_index])
                negative_prob = np.sum(prob[self.positive_index:])
                if label == "positive":
                    if positive_prob > negative_prob:
                        print(f'PROBS: {prob} \nCorrect:   True Positive')
                        correct += 1
                    else:
                        print(f'PROBS: {prob} \nIncorrect: False Negative')
                else:
                    if positive_prob <= negative_prob:
                        print(f'PROBS: {prob} \nCorrect:   True Negative')
                        correct += 1
                    else:
                        print(f'PROBS: {prob} \nIncorrect: False Positive')
                print()

            return correct, len(labels)


    # def plot(self, image_embeds, labels):
    #     pca = PCA(n_components=50)
    #     pca_result = pca.fit_transform(image_embeds)
    #     tsne = TSNE(n_components=2, verbose=1, perplexity=40, n_iter=300)
    #     tsne_results = tsne.fit_transform(pca_result)

    #     plt.figure(figsize=(10, 10))
    #     for i in range(len(labels)):
    #         plt.scatter(tsne_results[i, 0], tsne_results[i, 1], marker='o' if labels[i] == 0 else '*')
    #     plt.show()

    # Return True if target object is detected.
    def detect(self, images):
        # CLIP detection for the HSR during the actual task operation
        if self.mode == "HSR":
            return self.evaluate(images)

        # CLIP detection for local datasets
        if self.mode == "local":
            image_names ,images, labels = [], [], []
            markers = ['o', '*']
            for i, label in enumerate(self.labels):
                image_dir = f"/home/mushypeas/workspace/images/{self.name}/{label}"
                image_files = [os.path.join(image_dir, file) for file in os.listdir(image_dir) if file.endswith((".jpg", ".jpeg", ".png"))]
                image_files.sort()
                image_names += image_files
                for image_file in image_files:
                    image = Image.open(image_file)
                    image = image.resize((480, 640))
                    images.append(image)
                    labels.append(label)

            correct, total = self.evaluate(images, image_names, labels)

            print("===============================================")
            print(f"Total Score:    {correct}/{len(labels)}")
            print(f"Total Accuracy: {round(correct / len(labels), 3)}")


if __name__ == "__main__":
    SHOES = CLIPDetectorConfig(
        name="shoes",
        labels=["negative", "positive"],
        # positive_texts=[
        #     "a photo of a person wearing socks",
        #     "a photo of a person barefoot",
        # ],
        # negative_texts=[
        #     "a photo of a person wearing shoes",
        #     "a photo of a person wearing sandals",
        # ],
        positive_texts=[
            "a photo of a person wearing socks or being barefoot",
        ],
        negative_texts=[
            "a photo of a person wearing shoes or sandals",
            "a photo of a person wearing shoes or sandals with socks",
        ],
        neutral_texts=[
            "a photo of a background with no people in it",
        ],
        threshold=0
    )

    DRINK = CLIPDetectorConfig(
        name="drink",
        labels=["negative", "positive"],
        positive_texts=[
            "a photo of a person standing with a cup in their hand",
            "a photo of a person standing with a bottle in their hand",
            "a photo of a person standing with a soda can in their hand",
            "a photo of a person standing with a beverage in their hand",
        ],
        negative_texts=[
            "a photo of a person standing without any objects in their hand",
            "a photo of a person standing without any objects in their hand with a drink in the background",
        ],
        threshold=0
    )
    # clip = CLIPDetector(SHOES)
    clip = CLIPDetector(DRINK)

    clip.detect_local_dataset()