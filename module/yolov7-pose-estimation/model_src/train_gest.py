import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset, random_split
import pandas as pd
import numpy as np

# 데이터셋 클래스 정의
class GestKeypointDataset(Dataset):
    def __init__(self, file_path):
        self.data = []
        self.labels = []
        self.label_map = {
            'raising left': 0,
            'raising right': 1,
            'pointing right': 2,
            'pointing left': 3
        }
        
        with open(file_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                features = list(map(float, parts[:-2]))
                label = self.label_map[parts[-2] + ' ' + parts[-1]]
                self.data.append(features)
                self.labels.append(label)
        
        self.data = np.array(self.data, dtype=np.float32)
        self.labels = np.array(self.labels, dtype=np.int64)
    
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, idx):
        return self.data[idx], self.labels[idx]

# 데이터셋 로드
file_path = 'gest_keypoint_labels_1.txt'
dataset = GestKeypointDataset(file_path)

# 데이터셋 섞기
dataset_size = len(dataset)
indices = list(range(dataset_size))
np.random.shuffle(indices)

# 데이터셋 분할 (10:1 비율)
train_size = int(0.9 * dataset_size)
test_size = dataset_size - train_size
train_indices, test_indices = indices[:train_size], indices[train_size:]

train_dataset = torch.utils.data.Subset(dataset, train_indices)
test_dataset = torch.utils.data.Subset(dataset, test_indices)

train_dataloader = DataLoader(train_dataset, batch_size=32, shuffle=True)
test_dataloader = DataLoader(test_dataset, batch_size=32, shuffle=False)

# 모델 정의 (입력 피처 수를 데이터의 피처 수에 맞추기)
input_size = len(dataset[0][0])  # 데이터의 피처 수
class GestClassifier(nn.Module):
    def __init__(self, input_size):
        super(GestClassifier, self).__init__()
        self.fc1 = nn.Linear(input_size, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, 4)  # 4개의 클래스
        self.relu = nn.ReLU()
    
    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# 하이퍼파라미터 설정
learning_rate = 0.001
num_epochs = 100

# 모델, 손실 함수, 옵티마이저 초기화
model = GestClassifier(input_size)
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# 학습 루프
for epoch in range(num_epochs):
    model.train()
    for data, labels in train_dataloader:
        # 모델 예측
        outputs = model(data)
        loss = criterion(outputs, labels)
        
        # 역전파 및 옵티마이저 스텝
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
    
    print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')

# 테스트
model.eval()
correct = 0
total = 0
with torch.no_grad():
    for data, labels in test_dataloader:
        outputs = model(data)
        _, predicted = torch.max(outputs.data, 1)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()

        print(f'predicted: {predicted}')
        print(f'labels: {labels}')

print(f'Test Accuracy: {100 * correct / total:.2f}%')

# 모델 저장
torch.save(model.state_dict(), 'gest_classifier.pth')
