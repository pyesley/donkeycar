"""Split raw images + YOLO labels into train/val folders for Ultralytics.

Source layout (flexible — picks whichever exists):
    SOURCE/images/*.{jpg,jpeg,png}  + SOURCE/labels/*.txt
    SOURCE/*.{jpg,jpeg,png,txt}             (images + labels mixed)

Output layout (what dataset.yaml expects):
    dataset/
      images/{train,val}/<file>.<ext>
      labels/{train,val}/<file>.txt

Run:
    python prepare_dataset.py --src /path/to/usb/banana_peels --val-frac 0.2
"""

import argparse
import random
import shutil
from pathlib import Path

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def find_pairs(src: Path):
    """Return list of (image_path, label_path) pairs, skipping unmatched files."""
    images_dir = src / "images" if (src / "images").is_dir() else src
    labels_dir = src / "labels" if (src / "labels").is_dir() else src

    pairs, missing = [], []
    for img in sorted(images_dir.rglob("*")):
        if img.suffix.lower() not in IMG_EXTS:
            continue
        label = labels_dir / f"{img.stem}.txt"
        if label.is_file():
            pairs.append((img, label))
        else:
            missing.append(img)
    return pairs, missing


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--src", required=True, help="folder with raw images + YOLO .txt labels")
    p.add_argument("--out", default="dataset", help="output dataset root")
    p.add_argument("--val-frac", type=float, default=0.2)
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()

    src = Path(args.src).resolve()
    out = Path(args.out).resolve()
    if not src.is_dir():
        raise SystemExit(f"--src not found: {src}")

    pairs, missing = find_pairs(src)
    if not pairs:
        raise SystemExit(f"no (image, label) pairs found under {src}")

    print(f"found {len(pairs)} image/label pairs")
    if missing:
        print(f"warning: {len(missing)} images had no matching .txt label (skipped)")

    random.Random(args.seed).shuffle(pairs)
    n_val = max(1, int(len(pairs) * args.val_frac))
    val_pairs = pairs[:n_val]
    train_pairs = pairs[n_val:]
    print(f"train={len(train_pairs)}  val={len(val_pairs)}")

    for split, items in (("train", train_pairs), ("val", val_pairs)):
        img_dst = out / "images" / split
        lbl_dst = out / "labels" / split
        img_dst.mkdir(parents=True, exist_ok=True)
        lbl_dst.mkdir(parents=True, exist_ok=True)
        for img, label in items:
            shutil.copy2(img, img_dst / img.name)
            shutil.copy2(label, lbl_dst / label.name)

    print(f"\nwrote dataset to {out}")
    print(f"point dataset.yaml's `path:` at this directory (default already does).")


if __name__ == "__main__":
    main()
