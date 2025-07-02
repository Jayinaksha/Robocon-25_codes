import os
import shutil
from sklearn.model_selection import train_test_split

frames_dir = "frames"
labels_dir = "labels"  # Where your annotation tool saves .txt files

# Create YOLO directory structure
for split in ["train", "val"]:
    os.makedirs(f"dataset/images/{split}", exist_ok=True)
    os.makedirs(f"dataset/labels/{split}", exist_ok=True)

# List all annotated images
images = [f for f in os.listdir(frames_dir) if f.endswith(".jpg")]
train_imgs, val_imgs = train_test_split(images, test_size=0.2, random_state=42)

for img_list, split in [(train_imgs, "train"), (val_imgs, "val")]:
    for img_name in img_list:
        # Copy image
        shutil.copy(os.path.join(frames_dir, img_name), f"dataset/images/{split}/{img_name}")
        # Copy label
        label_name = img_name.replace(".jpg", ".txt")
        shutil.copy(os.path.join(labels_dir, label_name), f"dataset/labels/{split}/{label_name}")

print("Dataset organized for YOLO")