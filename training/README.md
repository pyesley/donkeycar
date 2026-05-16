# YOLOv8n custom training

Trains a YOLOv8n model on custom-labeled images (banana peels by default).
The output `best.pt` is a drop-in replacement for the model run by
`ROS2/yolo_speaker` on the Raspberry Pi 5 -- same architecture, just
different class names.

## What this expects

- **Hardware**: a CUDA GPU (designed for an RTX 5090, but any modern NVIDIA GPU works)
- **Labels**: YOLO `.txt` format (one file per image, lines: `class_id cx cy w h` normalized 0-1)
- **Images**: `.jpg`, `.jpeg`, `.png`, or `.bmp`

If your labels are Pascal VOC `.xml` or COCO `.json`, convert first --
Roboflow and CVAT both export YOLO format.

## Setup (RTX 5090 / Blackwell)

```bash
git clone https://github.com/pyesley/donkeycar.git
cd donkeycar/training
python -m venv .venv
. .venv/bin/activate              # Linux/macOS
# .venv\Scripts\activate          # Windows

# RTX 5090 (sm_120) needs CUDA 12.8 nightly torch:
pip install --pre torch torchvision --index-url https://download.pytorch.org/whl/nightly/cu128
pip install -r requirements.txt
```

For an older GPU (RTX 30/40 series) replace the torch line with:
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

## Train

```bash
# 1. Put your images and labels somewhere -- e.g. D:\banana_data\
#    Either as D:\banana_data\images\*.jpg + D:\banana_data\labels\*.txt
#    or mixed in one folder.
python prepare_dataset.py --src "D:\banana_data" --val-frac 0.2

# 2. Train (uses dataset.yaml -- edit class names there if you add more)
python train.py --epochs 100

# Output:
#   runs/detect/banana_peel/weights/best.pt   <-- the trained model
#   runs/detect/banana_peel/weights/last.pt   <-- last epoch (usually worse)
```

For a ~1k image single-class dataset, 100 epochs at imgsz=640 should hit
decent mAP in under 30 min on an RTX 5090. Watch the `mAP50-95` column in
the training output -- it should climb steadily.

## Verify before shipping to the Pi

```bash
python predict.py --weights runs/detect/banana_peel/weights/best.pt \
                  --source dataset/images/val/
# Look in runs/detect/predict/ for annotated images with bounding boxes.
```

## Deploy to the Pi

```bash
scp runs/detect/banana_peel/weights/best.pt mdyesley@<pi-ip>:/home/mdyesley/ros2_ws/banana_peel.pt
```

On the Pi, in `~/fruit_detector.sh`, change the yolo launch line to point at
the new model:

```bash
ros2 launch yolo_bringup yolov8.launch.py \
    model:=/home/mdyesley/ros2_ws/banana_peel.pt \
    input_image_topic:=/camera/image_raw \
    image_reliability:=2 \
    device:=cpu
```

And update the announce target classes (either bake into the script or
override at run time):

```bash
ros2 run yolo_speaker announce_node --ros-args \
    -p "target_classes:=['banana_peel']"
```

## Why this stays compatible with the Pi

| Concern | Why it works |
|---|---|
| Model architecture | Both training and inference use YOLOv8n (nano) |
| File format | `.pt` PyTorch checkpoint -- Ultralytics loads any 8.x version |
| Classes | The model embeds class names; Ultralytics on the Pi reads them and yolo_ros publishes `class_name` accordingly |
| Inference path | Same `ultralytics.YOLO(...).predict(...)` used in `yolo_ros` |

The Pi currently runs `ultralytics==8.4.6`. Training with a newer
Ultralytics version (8.4.x or 8.5+) is fine -- the resulting `.pt` is
backward-compatible for inference.

## Multi-class extension (if you add classes later)

1. Edit `dataset.yaml`:
   ```yaml
   names:
     0: banana_peel
     1: banana_whole
   ```
2. Make sure your label `.txt` files use the matching `class_id` integers.
3. Re-run `prepare_dataset.py` and `train.py`.
4. On the Pi: `target_classes:=['banana_peel','banana_whole']`.
