import numpy as np
from PIL import Image, ImageEnhance, ImageOps
import os

# ==========================================
# CONFIGURATION
# ==========================================
INPUT_FILE = "image.bin"
OUTPUT_FILE = "final_auto_tuned.jpg"
WIDTH = 320
HEIGHT = 240

def auto_balance_white_patch(img_array, percentile=98):
    """
    Finds the brightest pixels (top 2%) and assumes they should be white.
    Scales each channel (R, G, B) so that the brightest pixels equal 255.
    """
    # 1. Find the max brightness for each channel (ignoring outliers)
    r_max = np.percentile(img_array[:, :, 0], percentile)
    g_max = np.percentile(img_array[:, :, 1], percentile)
    b_max = np.percentile(img_array[:, :, 2], percentile)
    
    print(f"Detected Max Levels -> R:{r_max:.1f}, G:{g_max:.1f}, B:{b_max:.1f}")
    
    # 2. Calculate Scaling Factors (Avoid division by zero)
    # We target '240' instead of '255' to leave room for highlights
    target = 240.0
    scale_r = target / r_max if r_max > 0 else 1.0
    scale_g = target / g_max if g_max > 0 else 1.0
    scale_b = target / b_max if b_max > 0 else 1.0
    
    print(f"Calculated Correction -> R_SCALE={scale_r:.3f}, G_SCALE={scale_g:.3f}, B_SCALE={scale_b:.3f}")

    # 3. Apply Scaling
    result = img_array.astype(float)
    result[:, :, 0] *= scale_r
    result[:, :, 1] *= scale_g
    result[:, :, 2] *= scale_b
    
    # 4. Clip to valid range
    return np.clip(result, 0, 255).astype(np.uint8)

def main():
    if not os.path.exists(INPUT_FILE):
        print(f"[ERROR] {INPUT_FILE} not found.")
        return

    print(f"Loading {INPUT_FILE}...")
    with open(INPUT_FILE, "rb") as f:
        raw_data = f.read()

    # 1. Pad/Trim
    expected = WIDTH * HEIGHT * 2
    if len(raw_data) < expected:
        raw_data += b'\x00' * (expected - len(raw_data))
    data = raw_data[:expected]

    # 2. Decode Raw Bytes (VYUY Order - Fixes Blue/Red Swap)
    arr = np.frombuffer(data, dtype=np.uint8)
    clusters = arr.reshape((HEIGHT, WIDTH // 2, 4))
    
    # Extract: V, Y0, U, Y1
    v, y0, u, y1 = clusters[:,:,0], clusters[:,:,1], clusters[:,:,2], clusters[:,:,3]

    # 3. YUV to RGB Conversion (Standard BT.601)
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
    
    # Recombine
    raw_rgb = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    raw_rgb[:, 0::2, :] = np.clip(rgb0, 0, 255)
    raw_rgb[:, 1::2, :] = np.clip(rgb1, 0, 255)
    
    # 4. APPLY AUTO TUNING
    print("Applying 'White Patch' Auto-Calibration...")
    tuned_rgb = auto_balance_white_patch(raw_rgb)
    
    img = Image.fromarray(tuned_rgb, 'RGB')

    # 5. Final Polish (Contrast/Saturation)
    # Often YUV conversions look a bit "flat", so we boost it slightly
    enhancer = ImageEnhance.Contrast(img)
    img = enhancer.enhance(1.2)
    
    enhancer = ImageEnhance.Color(img)
    img = enhancer.enhance(1.3)

    img.save(OUTPUT_FILE, quality=98)
    print(f"[SUCCESS] Saved perfectly tuned image to {OUTPUT_FILE}")
    img.show()

if __name__ == "__main__":
    main()
