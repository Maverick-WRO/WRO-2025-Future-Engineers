import cv2

def show_image(winname, img, scale=0.5):
    h, w = img.shape[:2]
    resized = cv2.resize(img, (int(w*scale), int(h*scale)))
    cv2.imshow(winname, resized)
