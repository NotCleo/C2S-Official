import numpy as np
import cv2
import pandas as pd
import os

def main():
    print("=========================================")
    print("  ARTIX-7 YOLOv8 - SMART BOX DECODER")
    print("=========================================")

    img_file = "input_test.bin"
    csv_file = "coordinates.csv"
    out_file = "final_detection.png"

    if not os.path.exists(img_file) or not os.path.exists(csv_file):
        print(f"[ERROR] Missing files. Need '{img_file}' and '{csv_file}'.")
        return

    # 1. SMART IMAGE LOADER
    print(f"[*] Analyzing '{img_file}'...")
    file_size = os.path.getsize(img_file)
    print(f"[*] File Size: {file_size} bytes.")

    img_bgr = None

    test_decode = cv2.imread(img_file)
    if test_decode is not None:
        print("[*] Image was successfully decoded as a standard compressed image format.")
        img_bgr = test_decode
        img_bgr = cv2.resize(img_bgr, (320, 240))
    else:
        if file_size == 230400: 
            print("[*] Detected standard 8-bit RAW RGB image.")
            raw_img = np.fromfile(img_file, dtype=np.uint8)
            img_rgb = raw_img.reshape((240, 320, 3))
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            
        elif file_size == 921600: 
            print("[*] Detected 32-bit FLOAT RAW image. Converting to 8-bit...")
            raw_img = np.fromfile(img_file, dtype=np.float32)
            if raw_img.max() <= 1.0:
                raw_img = raw_img * 255.0
            img_rgb = raw_img.astype(np.uint8).reshape((240, 320, 3))
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        else:
            print(f"[ERROR] Cannot decipher image format. File size {file_size} does not match any known 320x240 uncompressed layout.")
            return

    # 2. READ COORDINATES
    print("[*] Reading coordinates from CSV...")
    try:
        # STRIP THE WHITESPACE FROM THE C-GENERATED CSV
        df = pd.read_csv(csv_file, skipinitialspace=True)
    except Exception as e:
        print(f"[ERROR] Failed to read CSV: {e}")
        return

    # 3. DRAW BOUNDING BOXES
    box_count = 0
    for index, row in df.iterrows():
        if row['confidence'] == 0.0:
            continue
            
        x_min = int(row['x_min'])
        y_min = int(row['y_min'])
        x_max = int(row['x_max'])
        y_max = int(row['y_max'])
        conf = float(row['confidence'])

        x_min = max(0, min(320, x_min))
        y_min = max(0, min(240, y_min))
        x_max = max(0, min(320, x_max))
        y_max = max(0, min(240, y_max))

        cv2.rectangle(img_bgr, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        
        label = f"Target: {conf*100:.1f}%"
        (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
        cv2.rectangle(img_bgr, (x_min, y_min - h - 4), (x_min + w, y_min), (0, 255, 0), -1)
        cv2.putText(img_bgr, label, (x_min, y_min - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        
        box_count += 1

    if box_count == 0:
         print("[!] CSV contained no valid target coordinates.")

    # 4. SAVE OUTPUT
    display_img = cv2.resize(img_bgr, (640, 480), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(out_file, display_img)
    
    print(f"[SUCCESS] Drew {box_count} detections!")
    print(f"[*] Saved high-res output to '{out_file}'.")

if __name__ == "__main__":
    main()
