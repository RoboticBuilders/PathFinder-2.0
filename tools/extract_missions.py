import cv2
import numpy as np
import sys
from pathlib import Path

img_path = Path('wro2025_junior_field.png')
if not img_path.exists():
    print('ERROR: image not found:', img_path)
    sys.exit(2)

img = cv2.imread(str(img_path))
if img is None:
    print('ERROR: failed to read image')
    sys.exit(3)

h, w = img.shape[:2]
print(f'Image size: {w}x{h}')
# Preprocess
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5,5), 0)
edges = cv2.Canny(blur, 50, 150)

# Dilate edges to close gaps
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
dil = cv2.dilate(edges, kernel, iterations=2)

contours, _ = cv2.findContours(dil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter contours by area and rectangularity
rects = []
for cnt in contours:
    area = cv2.contourArea(cnt)
    if area < (w*h)*0.002:  # small areas ignored
        continue
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
    x,y,ww,hh = cv2.boundingRect(approx)
    aspect = ww/float(hh)
    rects.append((area, x,y,ww,hh, len(approx)))

# Sort by x then y
rects = sorted(rects, key=lambda r: (r[1], r[2]))

# Convert pixel to cm using field dims used in code
FIELD_W_CM = 236.0
FIELD_H_CM = 114.0

results = []
for area,x,y,ww,hh,nv in rects:
    x_cm = x * FIELD_W_CM / w
    y_cm = (h - (y+hh)) * FIELD_H_CM / h  # convert image origin top-left to plotting origin bottom-left
    # y conversion: plotting uses 0 at bottom; image has 0 at top -> invert
    # but path_drawer and path_planner use (0,0) bottom-left? They used extent=(0, field_width, 0, field_height) when showing image.
    # When annotating we'll keep y as measured from bottom.
    w_cm = ww * FIELD_W_CM / w
    h_cm = hh * FIELD_H_CM / h
    results.append({'x_px': x, 'y_px': y, 'w_px': ww, 'h_px': hh, 'x_cm': x_cm, 'y_cm': y_cm, 'w_cm': w_cm, 'h_cm': h_cm, 'vertices': nv, 'area_px': area})

# Print results
import json
print(json.dumps(results, indent=2))

# Draw debug output
dbg = img.copy()
for r in results:
    x,y,ww,hh = int(r['x_px']), int(r['y_px']), int(r['w_px']), int(r['h_px'])
    cv2.rectangle(dbg, (x,y), (x+ww, y+hh), (0,255,0), 2)

out = Path('mission_debug.png')
cv2.imwrite(str(out), dbg)
print('Wrote debug image to', out)

# Also write a python-friendly missions list (cm coords)
mission_list = []
for i,r in enumerate(results):
    mission_list.append((f'M{i+1}', round(r['x_cm'],1), round(r['y_cm'],1), round(r['w_cm'],1), round(r['h_cm'],1)))

print('\nSuggested mission list (name, x_cm, y_cm, w_cm, h_cm):')
for m in mission_list:
    print(m)

with open('tools/extracted_missions.json', 'w') as f:
    import json
    json.dump({'image_size_px': [w,h], 'field_cm': [FIELD_W_CM, FIELD_H_CM], 'rects': results, 'suggested': mission_list}, f, indent=2)
print('Saved tools/extracted_missions.json')
