#!/usr/bin/env python3

from pathlib import Path
from PIL import Image
import sys


TARGET_W = 550
TARGET_H = 400


def prepare_layer(input_path: Path, output_path: Path):
    img = Image.open(input_path).convert("RGBA")

    # Keep aspect ratio. Do not stretch robot.
    img.thumbnail((TARGET_W, TARGET_H), Image.Resampling.LANCZOS)

    canvas = Image.new("RGBA", (TARGET_W, TARGET_H), (0, 0, 0, 0))

    x = (TARGET_W - img.width) // 2
    y = TARGET_H - img.height

    canvas.alpha_composite(img, (x, y))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path)

    print(f"saved {output_path}")
    print(f"size={canvas.size}, mode={canvas.mode}")


def main():
    if len(sys.argv) != 3:
        print("Usage:")
        print("  python3 scripts/prepare_robot_layer.py input.png output.png")
        raise SystemExit(1)

    prepare_layer(Path(sys.argv[1]), Path(sys.argv[2]))


if __name__ == "__main__":
    main()
