import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import transforms, models
from torch.utils.data import DataLoader, Dataset
from PIL import Image
import os

# Custom dataset
class WindowDataset(Dataset):
    def __init__(self, image_folder, annotations_file, transform=None):
        self.image_folder = image_folder
        self.transform = transform
        # annotations_file is a path to a file which contains image names and corresponding points
        self.annotations = pd.read_csv(annotations_file)

    def __len__(self):
        return len(self.annotations)

    def __getitem__(self, idx):
        img_name = os.path.join(self.image_folder, self.annotations.iloc[idx, 0])
        image = Image.open(img_name)
        points = self.annotations.iloc[idx, 1:].values.astype('float').reshape(-1, 2)
        sample = {'image': image, 'points': points}

        if self.transform:
            sample['image'] = self.transform(sample['image'])

        return sample

# Model definition
class WindowPointsCNN(nn.Module):
    def __init__(self):
        super(WindowPointsCNN, self).__init__()
        # Use a pre-trained model as the feature extractor
        self.features = models.resnet18(pretrained=True)
        # Replace the classifier with a new one for regression
        self.features.fc = nn.Linear(self.features.fc.in_features, 128)
        self.regressor = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 16),
            nn.ReLU(),
            nn.Linear(16, 8)  # 8 coordinates, (x,y) for 4 points
        )

    def forward(self, x):
        x = self.features(x)
        x = self.regressor(x)
        return x

# Hyperparameters
batch_size = 4
learning_rate = 1e-3
num_epochs = 10

# Transformations
transform = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

# Dataset
train_dataset = WindowDataset(image_folder='path_to_images', 
                              annotations_file='annotations.csv',
                              transform=transform)

train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True)

# Model
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = WindowPointsCNN().to(device)

# Loss and optimizer
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# Training loop
for epoch in range(num_epochs):
    for i, data in enumerate(train_loader):
        inputs = data['image'].to(device)
        targets = data['points'].to(device)

        # Forward pass
        outputs = model(inputs)
        loss = criterion(outputs, targets.view(-1, 8))  # Reshape if necessary

        # Backward and optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if (i+1) % 10 == 0:
            print(f'Epoch [{epoch+1}/{num_epochs}], Step [{i+1}/{len(train_loader)}], Loss: {loss.item():.4f}')

# Save the model checkpoint
torch.save(model.state_dict(), 'model.ckpt')
