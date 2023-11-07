import torch
from torchvision import transforms
from PIL import Image

# Function to load the model
def load_model(checkpoint_path):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = WindowPointsCNN().to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()
    return model

# Function to predict the points for a single image
def predict(model, image_path, device):
    # Same transformations as used during training
    transform = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    # Load and transform the image
    image = Image.open(image_path)
    image = transform(image).unsqueeze(0).to(device)  # Add batch dimension and send to device

    # Predict the points
    with torch.no_grad():
        points = model(image).cpu().numpy().flatten()

    # TODO: Post-process points if necessary (e.g., scaling them back to the original image size)
    # This step depends on how your annotations were scaled during training

    return points

# Usage
checkpoint_path = 'model.ckpt'  # Path to your model checkpoint
image_path = 'path_to_your_test_image.jpg'  # Path to the image you want to predict on
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Load the model
model = load_model(checkpoint_path)

# Predict points
predicted_points = predict(model, image_path, device)
print(predicted_points)  # Print or process the predicted points
