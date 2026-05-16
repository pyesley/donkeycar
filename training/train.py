"""Fine-tune YOLOv8n on a custom dataset (banana peels by default).

The output `runs/detect/<name>/weights/best.pt` is a drop-in replacement for
the yolov8n.pt used by ROS2/yolo_speaker on the Pi -- same architecture, just
custom classes.

Run:
    python train.py                          # uses dataset.yaml, 100 epochs, GPU 0
    python train.py --epochs 200 --batch 32  # tweak
"""

import argparse
from pathlib import Path

from ultralytics import YOLO


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", default="dataset.yaml")
    p.add_argument("--model", default="yolov8n.pt",
                   help="base model. yolov8n.pt = nano (matches the Pi). "
                        "Use yolov8s.pt for slightly better mAP at ~3x compute.")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--device", default="0",
                   help="'0' = first GPU, 'cpu' = CPU, '0,1' = multi-GPU")
    p.add_argument("--name", default="banana_peel", help="run name under runs/detect/")
    p.add_argument("--patience", type=int, default=30,
                   help="early stop if no improvement in this many epochs")
    args = p.parse_args()

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        name=args.name,
        patience=args.patience,
    )

    best = Path(f"runs/detect/{args.name}/weights/best.pt").resolve()
    print(f"\nbest weights: {best}")
    print("\nto use on the Pi:")
    print(f"  scp {best.name} mdyesley@<pi>:/home/mdyesley/ros2_ws/banana_peel.pt")
    print("then on the Pi, edit ~/fruit_detector.sh and change the yolo launch line to:")
    print("  ros2 launch yolo_bringup yolov8.launch.py \\")
    print("      model:=/home/mdyesley/ros2_ws/banana_peel.pt \\")
    print("      input_image_topic:=/camera/image_raw image_reliability:=2 device:=cpu")
    print("and update announce_node target_classes:")
    print("  ros2 run yolo_speaker announce_node --ros-args \\")
    print("      -p \"target_classes:=['banana_peel']\"")


if __name__ == "__main__":
    main()
