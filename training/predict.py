"""Sanity-check a trained model on a single image or a folder.

Run:
    python predict.py --weights runs/detect/banana_peel/weights/best.pt \
                      --source dataset/images/val/                       \
                      --conf 0.25
"""

import argparse
from pathlib import Path

from ultralytics import YOLO


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--weights", required=True)
    p.add_argument("--source", required=True, help="image path or folder")
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--device", default="0")
    args = p.parse_args()

    model = YOLO(args.weights)
    results = model.predict(
        source=args.source,
        conf=args.conf,
        device=args.device,
        save=True,
    )
    out_dir = Path(results[0].save_dir).resolve() if results else None
    print(f"\nannotated images saved to: {out_dir}")


if __name__ == "__main__":
    main()
