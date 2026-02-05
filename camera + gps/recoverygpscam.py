import numpy as np
from PIL import Image, ImageEnhance, ImageOps
import os

INPUT_FILE = "image.bin"
OUTPUT_FILE = "final_geotagged.jpg"
WIDTH = 320
HEIGHT = 240

def auto_balance_white_patch(img_array, percentile=98):
    r_max = np.percentile(img_array[:, :, 0], percentile)
    g_max = np.percentile(img_array[:, :, 1], percentile)
    b_max = np.percentile(img_array[:, :, 2], percentile)
    
    target = 240.0
    scale_r = target / r_max if r_max > 0 else 1.0
    scale_g = target / g_max if g_max > 0 else 1.0
    scale_b = target / b_max if b_max > 0 else 1.0
    
    result = img_array.astype(float)
    result[:, :, 0] *= scale_r
    result[:, :, 1] *= scale_g
    result[:, :, 2] *= scale_b
    
    return np.clip(result, 0, 255).astype(np.uint8)

def extract_metadata(raw_bytes, expected_size):
    """
    Extracts data appended after the image data
    """
    if len(raw_bytes) <= expected_size:
        print("[INFO] No metadata found (File size matches image exactly).")
        return

    meta_bytes = raw_bytes[expected_size:]
    try:
        # Decode bytes to string
        meta_string = meta_bytes.decode('utf-8', errors='ignore')
        print("\n" + "="*40)
        print("       RECOVERED GPS METADATA")
        print("="*40)
        print(f"RAW DATA: {meta_string}")
        
        # Pretty print if format matches our C code
        if "Lat:" in meta_string:
            parts = meta_string.split(',')
            for p in parts:
                print(f" -> {p}")
        print("="*40 + "\n")
    except Exception as e:
        print(f"[WARN] Could not decode metadata: {e}")

def main():
    if not os.path.exists(INPUT_FILE):
        print(f"[ERROR] {INPUT_FILE} not found.")
        return

    print(f"Loading {INPUT_FILE}...")
    with open(INPUT_FILE, "rb") as f:
        raw_file_content = f.read()

    # Calculate where image data ends
    expected_image_size = WIDTH * HEIGHT * 2 # YUV422 is 2 bytes per pixel
    
    # 1. Extract Metadata first
    extract_metadata(raw_file_content, expected_image_size)

    # 2. Extract Image Data
    if len(raw_file_content) < expected_image_size:
        print("[ERROR] File too small for specified resolution.")
        # Pad with zeros to prevent crash, though image will be corrupt
        raw_file_content += b'\x00' * (expected_image_size - len(raw_file_content))
    
    image_data = raw_file_content[:expected_image_size]

    # 3. Process Image (Existing Logic)
    arr = np.frombuffer(image_data, dtype=np.uint8)
    clusters = arr.reshape((HEIGHT, WIDTH // 2, 4))
    
    v, y0, u, y1 = clusters[:,:,0], clusters[:,:,1], clusters[:,:,2], clusters[:,:,3]

    y0 = y0.astype(np.float32)
    y1 = y1.astype(np.float32)
    u  = u.astype(np.float32) - 128
    v  = v.astype(np.float32) - 128
    
    def convert_pixel(y, u, v):
        r = y + 1.402 * v
        g = y - 0.34414 * u - 0.71414 * v
        b = y + 1.772 * u
        return np.dstack((r, g, b))

    rgb0 = convert_pixel(y0, u, v)
    rgb1 = convert_pixel(y1, u, v)
    
    raw_rgb = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    raw_rgb[:, 0::2, :] = np.clip(rgb0, 0, 255)
    raw_rgb[:, 1::2, :] = np.clip(rgb1, 0, 255)
    
    print("Applying 'White Patch' Auto-Calibration...")
    tuned_rgb = auto_balance_white_patch(raw_rgb)
    
    img = Image.fromarray(tuned_rgb, 'RGB')

    enhancer = ImageEnhance.Contrast(img)
    img = enhancer.enhance(1.2)
    
    enhancer = ImageEnhance.Color(img)
    img = enhancer.enhance(1.3)

    img.save(OUTPUT_FILE, quality=98)
    print(f"[SUCCESS] Saved image to {OUTPUT_FILE}")
    img.show()

if __name__ == "__main__":
    main()
