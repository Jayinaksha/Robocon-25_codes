import cv2
import os

# Parameters
IMAGES_DIR = 'frames'
ANNOTATIONS_DIR = 'labels'
CLASS_NAME = 'bot'  # Only one class
CLASS_ID = 0        # YOLO expects class IDs starting from 0

os.makedirs(ANNOTATIONS_DIR, exist_ok=True)

def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, img, img_copy, bbox, current_filename

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        img_copy = img.copy()

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            img = img_copy.copy()
            cv2.rectangle(img, (ix, iy), (x, y), (0, 255, 0), 2)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        x1, y1 = min(ix, x), min(iy, y)
        x2, y2 = max(ix, x), max(iy, y)
        bbox = (x1, y1, x2, y2)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        save_yolo_annotation(current_filename, bbox, img.shape)

def save_yolo_annotation(filename, bbox, img_shape):
    x1, y1, x2, y2 = bbox
    img_h, img_w = img_shape[:2]
    # Convert to YOLO format: class x_center y_center width height (all normalized)
    x_center = ((x1 + x2) / 2) / img_w
    y_center = ((y1 + y2) / 2) / img_h
    width = (x2 - x1) / img_w
    height = (y2 - y1) / img_h
    annotation = f"{CLASS_ID} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n"
    base = os.path.splitext(os.path.basename(filename))[0]
    with open(os.path.join(ANNOTATIONS_DIR, base + ".txt"), "a") as f:
        f.write(annotation)
    print(f"Saved annotation for {filename}: {annotation.strip()}")

if __name__ == "__main__":
    global ix, iy, drawing, img, img_copy, bbox, current_filename
    image_files = [os.path.join(IMAGES_DIR, f) for f in os.listdir(IMAGES_DIR) if f.lower().endswith(('.jpg', '.png', '.jpeg'))]
    for current_filename in image_files:
        img = cv2.imread(current_filename)
        img_copy = img.copy()
        bbox = None
        cv2.namedWindow('Annotate')
        cv2.setMouseCallback('Annotate', draw_rectangle)
        print(f"Annotating: {current_filename}")
        while True:
            cv2.imshow('Annotate', img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('n'):  # next image
                break
            elif key == ord('r'):  # reset
                img = img_copy.copy()
                print("Reset annotation.")
            elif key == ord('q'):
                exit(0)
        cv2.destroyAllWindows()
    print("Annotation complete. Use the 'labels' folder with YOLO training.")