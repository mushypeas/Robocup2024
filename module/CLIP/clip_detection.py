import os
import numpy as np
import matplotlib.pyplot as plt

from PIL import Image
from sklearn.decomposition import PCA
from sklearn.manifold import TSNE
from transformers import CLIPProcessor, CLIPModel, CLIPConfig

# Available Config modes: shoes, drink
class CLIPDetectorConfig:
    def __init__(self, name: str, labels: list, positive_texts: list, negative_texts: list, threshold: float):
        self.name = name
        self.labels = labels
        self.text_index = len(positive_texts)
        self.texts = positive_texts + negative_texts
        self.threshold = threshold


class CLIPDetector:
    def __init__(self, config: CLIPDetectorConfig):
        self.name = config.name
        self.labels = config.labels
        self.text_index = config.text_index
        self.texts = config.texts
        self.threshold = config.threshold
        self.pretrained_model = "openai/clip-vit-base-patch32"
        self.model = CLIPModel.from_pretrained(self.pretrained_model)
        self.processor = CLIPProcessor.from_pretrained(self.pretrained_model)


    def detect(self, image):
        inputs = self.processor(
            text=self.config["texts"],
            images=image,
            return_tensors="pt",
            padding=True
        )
        outputs = self.model(**inputs)
        logits_per_image = outputs.logits_per_image  # this is the image-text similarity score
        probs = logits_per_image.softmax(dim=1)  # we can take the softmax to get the label probabilities
        return probs


    def evaluate(self, images, image_names, labels):
        inputs = self.processor(
            text=self.texts,
            images=images,
            return_tensors="pt",
            padding=True
        )
        outputs = self.model(**inputs)

        logits_per_image = outputs.logits_per_image  # this is the image-text similarity score
        probs = logits_per_image.softmax(dim=1).detach().numpy()  # we can take the softmax to get the label probabilities

        correct = 0
        for image_name, label, prob in list(zip(image_names, labels, probs)):
            print(f"Image: {image_name}, Label: {label}\n")
            positive_prob = np.sum(prob[:self.text_index])
            negative_prob = np.sum(prob[self.text_index:])
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

        print("===============================================")
        print(f"Total Score:    {correct}/{len(labels)}")
        print(f"Total Accuracy: {round(correct / len(labels), 3)}")


    def plot(self, image_embeds, labels):
        pca = PCA(n_components=50)
        pca_result = pca.fit_transform(image_embeds)
        tsne = TSNE(n_components=2, verbose=1, perplexity=40, n_iter=300)
        tsne_results = tsne.fit_transform(pca_result)

        plt.figure(figsize=(10, 10))
        for i in range(len(labels)):
            plt.scatter(tsne_results[i, 0], tsne_results[i, 1], marker='o' if labels[i] == 0 else '*')
        plt.show()

    def clip_detect(self):

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

        self.evaluate(images, image_names, labels)

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
        "a photo of a person wearing socks or being barefoot, without shoes or sandals",
    ],
    negative_texts=[
        "a photo of a person wearing shoes or sandals",
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

if __name__ == "__main__":
    # clip = CLIPDetector(SHOES)
    clip = CLIPDetector(DRINK)

    clip.clip_detect()