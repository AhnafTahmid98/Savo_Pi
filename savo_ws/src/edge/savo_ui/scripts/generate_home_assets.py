#!/usr/bin/env python3

from pathlib import Path
from PIL import Image, ImageDraw, ImageFont, ImageFilter
import math


ROOT = Path(__file__).resolve().parents[1]
ROBOT360_DIR = ROOT / "assets" / "images" / "robot360"
OUT_DIR = ROOT / "assets" / "images" / "generated"

W, H = 800, 480
TOP_BAR_H = 56

# Fixed top-bar slots shared with draw_live_top_bar_overlay() in ui_node.cpp.
# Dynamic IP/time strings are intentionally absent from the generated asset.
TOP_WIFI_X = 580
TOP_WIFI_Y = 21
TOP_IP_X = 608
TOP_SEPARATOR_X = 740
TOP_TIME_RIGHT = 790

# Final homepage robot placement standard.
# Transparent robot PNGs should be prepared to fit this box.
ROBOT_X = 150
ROBOT_Y = 80
ROBOT_W = 550
ROBOT_H = 400

ROBOT_LAYER_DIR = ROOT / "assets" / "images" / "robot_layers"

# Home layout tuned for 800x480 DSI display.
# The robot source frames are full-screen renders, so we crop the robot area,
# resize it, and place it in the clean center-right visual area.
ROBOT_CROP_BOX = (95, 70, 735, 465)
ROBOT_TARGET_BOX = (245, 78, 615, 452)

TITLE_X = 124
TITLE_Y = 74

LEFT_MENU_X = 14
LEFT_MENU_Y = 72
LEFT_MENU_W = 74
LEFT_MENU_H = 374

STATUS_X = 600
STATUS_Y = 100
STATUS_W = 190
STATUS_H = 238


def font(size: int, bold: bool = False):
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]

    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size=size)

    return ImageFont.load_default()


FONT_SMALL = font(13)
FONT_MED = font(16)
FONT_MED_BOLD = font(16, bold=True)
FONT_BIG = font(25, bold=True)
FONT_TITLE = font(56, bold=True)

# Runtime C++ font atlases.
# Each glyph cell stores its proportional advance in the first pixel.
RUNTIME_FONT_COLUMNS = 16
RUNTIME_FONT_PADDING = 2
RUNTIME_FONT_CHARS = "".join(chr(code) for code in range(32, 127))


CYAN = (40, 230, 255)
CYAN_SOFT = (70, 210, 255)
WHITE = (235, 245, 255)
TEXT_DIM = (180, 205, 220)
PANEL = (2, 16, 36)
PANEL_2 = (4, 26, 56)


def load_robot_layer(name: str) -> Image.Image | None:
    if not name:
        return None

    path = ROBOT_LAYER_DIR / name
    if not path.exists():
        return None

    img = Image.open(path).convert("RGBA")

    # Keep aspect ratio. Never stretch the robot.
    img.thumbnail((ROBOT_W, ROBOT_H), Image.Resampling.LANCZOS)

    canvas = Image.new("RGBA", (ROBOT_W, ROBOT_H), (0, 0, 0, 0))
    x = (ROBOT_W - img.width) // 2
    y = ROBOT_H - img.height
    canvas.alpha_composite(img, (x, y))

    return canvas



def load_robot_frame(name: str) -> Image.Image:
    path = ROBOT360_DIR / name
    if not path.exists():
        raise FileNotFoundError(f"Missing robot frame: {path}")

    img = Image.open(path).convert("RGB")
    if img.size != (W, H):
        img = img.resize((W, H), Image.Resampling.LANCZOS)
    return img


def glow_text(base: Image.Image, xy, text, fnt, fill, glow=(50, 180, 255), blur=8, strength=2):
    glow_layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    gd = ImageDraw.Draw(glow_layer)

    x, y = xy
    for _ in range(strength):
        gd.text((x, y), text, font=fnt, fill=glow + (180,))

    glow_layer = glow_layer.filter(ImageFilter.GaussianBlur(blur))
    base.alpha_composite(glow_layer)

    d = ImageDraw.Draw(base)
    d.text((x, y), text, font=fnt, fill=fill + (255,))


def line(draw, pts, fill=CYAN, width=2):
    draw.line(pts, fill=fill, width=width, joint="curve")


def draw_home_icon(draw, cx, cy, color):
    roof = [(cx - 18, cy), (cx, cy - 17), (cx + 18, cy)]
    draw.line(roof, fill=color, width=4, joint="curve")
    draw.rounded_rectangle((cx - 13, cy, cx + 13, cy + 20), radius=3, outline=color, width=4)
    draw.rectangle((cx - 4, cy + 9, cx + 4, cy + 20), fill=color)


def draw_mic_icon(draw, cx, cy, color):
    draw.rounded_rectangle((cx - 7, cy - 20, cx + 7, cy + 8), radius=7, outline=color, width=3)
    draw.arc((cx - 18, cy - 6, cx + 18, cy + 24), 20, 160, fill=color, width=3)
    draw.line((cx, cy + 23, cx, cy + 35), fill=color, width=3)
    draw.line((cx - 11, cy + 35, cx + 11, cy + 35), fill=color, width=3)


def draw_nav_icon(draw, cx, cy, color):
    pts = [(cx - 18, cy - 8), (cx + 20, cy - 23), (cx + 6, cy + 18), (cx - 2, cy + 4)]
    draw.line(pts + [pts[0]], fill=color, width=4, joint="curve")
    draw.line((cx - 2, cy + 4, cx + 20, cy - 23), fill=color, width=3)


def draw_status_icon(draw, cx, cy, color):
    draw.rounded_rectangle((cx - 18, cy - 15, cx + 18, cy + 15), radius=7, outline=color, width=3)
    pts = [(cx - 13, cy), (cx - 6, cy), (cx - 2, cy - 8), (cx + 4, cy + 9), (cx + 8, cy), (cx + 14, cy)]
    draw.line(pts, fill=color, width=3, joint="curve")


def draw_power_icon(draw, cx, cy, color):
    draw.arc((cx - 18, cy - 18, cx + 18, cy + 18), 35, 325, fill=color, width=4)
    draw.line((cx, cy - 22, cx, cy + 2), fill=color, width=4)


def draw_wifi_icon(draw, cx, cy, color):
    draw.arc((cx - 18, cy - 10, cx + 18, cy + 26), 220, 320, fill=color, width=3)
    draw.arc((cx - 12, cy - 4, cx + 12, cy + 20), 220, 320, fill=color, width=3)
    draw.ellipse((cx - 2, cy + 13, cx + 2, cy + 17), fill=color)


def draw_bluetooth_icon(draw, cx, cy, color):
    pts1 = [(cx, cy - 18), (cx + 10, cy - 8), (cx - 8, cy + 8), (cx, cy + 18), (cx, cy - 18)]
    pts2 = [(cx - 8, cy - 8), (cx + 10, cy + 8)]
    draw.line(pts1, fill=color, width=3, joint="curve")
    draw.line(pts2, fill=color, width=3)


def draw_battery_icon(draw, cx, cy, color):
    draw.rounded_rectangle((cx - 17, cy - 10, cx + 14, cy + 10), radius=3, outline=color, width=3)
    draw.rectangle((cx + 16, cy - 4, cx + 20, cy + 4), fill=color)
    draw.rectangle((cx - 11, cy - 5, cx - 4, cy + 5), fill=color)
    draw.rectangle((cx - 2, cy - 5, cx + 5, cy + 5), fill=color)
    draw.rectangle((cx + 7, cy - 5, cx + 11, cy + 5), fill=color)


def panel_glow(base, box, radius=16, color=(0, 210, 255), alpha=90):
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    d.rounded_rectangle(box, radius=radius, outline=color + (alpha,), width=2)
    blur = layer.filter(ImageFilter.GaussianBlur(5))
    base.alpha_composite(blur)
    base.alpha_composite(layer)


def draw_top_bar(base, page_title="HOME"):
    d = ImageDraw.Draw(base)

    bar_h = 56

    # Top rail background.
    d.rectangle((0, 0, W, bar_h), fill=(0, 8, 21, 225))
    d.line((14, bar_h - 1, W - 14, bar_h - 1), fill=(0, 170, 220, 120), width=1)

    # Left brand/title.
    d.text((22, 20), "SAVO", font=FONT_MED_BOLD, fill=WHITE)
    d.line((82, 18, 82, 34), fill=(0, 170, 220, 120), width=1)
    d.text((118, 18), page_title.upper(), font=FONT_MED_BOLD, fill=CYAN)

    # Small aligned Wi-Fi icon.
    def tiny_wifi_icon(cx, cy):
        d.arc((cx - 10, cy - 8, cx + 10, cy + 12), 215, 325, fill=CYAN, width=2)
        d.arc((cx - 6, cy - 4, cx + 6, cy + 8), 215, 325, fill=CYAN, width=2)
        d.arc((cx - 3, cy, cx + 3, cy + 6), 215, 325, fill=CYAN, width=2)
        d.ellipse((cx - 2, cy + 12, cx + 2, cy + 16), fill=CYAN)

    # Fixed layout: [Wi-Fi] [dynamic IP] | [dynamic time].
    # The runtime C++ overlay owns both text slots.
    tiny_wifi_icon(TOP_WIFI_X, TOP_WIFI_Y)

    d.line(
        (TOP_SEPARATOR_X, 16, TOP_SEPARATOR_X, 38),
        fill=(0, 150, 190, 90),
        width=1,
    )



def draw_left_menu(base, active_item="Home"):
    d = ImageDraw.Draw(base)
    x, y, w, h = LEFT_MENU_X, LEFT_MENU_Y, LEFT_MENU_W, LEFT_MENU_H

    panel_glow(base, (x, y, x + w, y + h), radius=14, alpha=95)
    d.rounded_rectangle(
        (x, y, x + w, y + h),
        radius=14,
        fill=(0, 10, 28, 190),
        outline=(0, 210, 255, 130),
        width=1,
    )

    # Dedicated compact icons for the side menu.
    def left_home_icon(cx, cy, color):
        d.line((cx - 13, cy - 1, cx, cy - 13, cx + 13, cy - 1), fill=color, width=3)
        d.rectangle((cx - 8, cy - 1, cx + 8, cy + 11), outline=color, width=3)
        d.rectangle((cx - 3, cy + 6, cx + 3, cy + 11), fill=color)

    def left_mic_icon(cx, cy, color):
        d.rounded_rectangle((cx - 5, cy - 12, cx + 5, cy + 4), radius=5, outline=color, width=3)
        d.arc((cx - 13, cy - 3, cx + 13, cy + 17), 25, 155, fill=color, width=2)
        d.line((cx, cy + 13, cx, cy + 19), fill=color, width=2)
        d.line((cx - 8, cy + 19, cx + 8, cy + 19), fill=color, width=2)

    def left_nav_icon(cx, cy, color):
        pts = [
            (cx - 14, cy - 6),
            (cx + 14, cy - 16),
            (cx + 5, cy + 16),
            (cx - 2, cy + 5),
        ]
        d.line(pts + [pts[0]], fill=color, width=3, joint="curve")
        d.line((cx - 2, cy + 5, cx + 14, cy - 16), fill=color, width=2)

    def left_status_icon(cx, cy, color):
        d.rounded_rectangle((cx - 14, cy - 10, cx + 14, cy + 10), radius=5, outline=color, width=3)
        d.line(
            (cx - 9, cy, cx - 5, cy, cx - 2, cy - 5, cx + 4, cy + 6, cx + 8, cy, cx + 10, cy),
            fill=color,
            width=2,
        )

    def left_power_icon(cx, cy, color):
        d.arc((cx - 14, cy - 14, cx + 14, cy + 14), 35, 325, fill=color, width=4)
        d.line((cx, cy - 17, cx, cy - 2), fill=color, width=4)

    items = [
        ("Home", left_home_icon),
        ("Voice", left_mic_icon),
        ("Navigate", left_nav_icon),
        ("Status", left_status_icon),
        ("Power", left_power_icon),
    ]

    # 5 equal boxes spread evenly inside the vertical rail.
    # This keeps the bottom gap after Power balanced.
    item_h = 58
    top_pad = 8
    bottom_pad = 8
    item_gap = (h - top_pad - bottom_pad - (len(items) * item_h)) // (len(items) - 1)
    first_y = y + top_pad

    item_x0 = x + 6
    item_x1 = x + w - 6

    for i, (label, icon_func) in enumerate(items):
        active = label == active_item
        item_y0 = first_y + i * (item_h + item_gap)
        item_y1 = item_y0 + item_h

        cx = x + w // 2
        icon_cy = item_y0 + 22
        label_y = item_y0 + 41

        box = (item_x0, item_y0, item_x1, item_y1)

        if active:
            panel_glow(base, box, radius=12, alpha=120)
            d.rounded_rectangle(
                box,
                radius=12,
                fill=(0, 65, 105, 165),
                outline=(0, 230, 255, 210),
                width=1,
            )
            icon_color = WHITE
            text_color = WHITE
        else:
            d.rounded_rectangle(
                box,
                radius=12,
                fill=(0, 14, 34, 115),
                outline=(0, 155, 210, 85),
                width=1,
            )
            icon_color = (188, 220, 235)
            text_color = WHITE

        icon_func(cx, icon_cy, icon_color)

        tw = d.textlength(label, font=FONT_SMALL)
        d.text((x + (w - tw) / 2, label_y), label, font=FONT_SMALL, fill=text_color)



def draw_status_panel(base):
    d = ImageDraw.Draw(base)
    x, y, w, h = STATUS_X, STATUS_Y, STATUS_W, STATUS_H

    panel_glow(base, (x, y, x + w, y + h), radius=16, alpha=100)
    d.rounded_rectangle(
        (x, y, x + w, y + h),
        radius=16,
        fill=(0, 12, 31, 205),
        outline=(0, 210, 255, 145),
        width=1,
    )

    title = "SYSTEM STATUS"
    tw = d.textlength(title, font=FONT_SMALL)
    d.text((x + (w - tw) / 2, y + 15), title, font=FONT_SMALL, fill=CYAN)
    d.line((x + 18, y + 40, x + w - 18, y + 40), fill=(0, 145, 190, 90), width=1)

    def small_status_icon(icon, cx, cy):
        if icon == "gear":
            d.ellipse((cx - 7, cy - 7, cx + 7, cy + 7), outline=CYAN_SOFT, width=3)
            d.ellipse((cx - 2, cy - 2, cx + 2, cy + 2), fill=CYAN_SOFT)

        elif icon == "mic":
            d.rounded_rectangle((cx - 4, cy - 11, cx + 4, cy + 4), radius=4, outline=CYAN_SOFT, width=3)
            d.arc((cx - 11, cy - 3, cx + 11, cy + 14), 25, 155, fill=CYAN_SOFT, width=2)
            d.line((cx, cy + 11, cx, cy + 17), fill=CYAN_SOFT, width=2)
            d.line((cx - 7, cy + 17, cx + 7, cy + 17), fill=CYAN_SOFT, width=2)

        elif icon == "nav":
            pts = [
                (cx - 9, cy - 4),
                (cx + 11, cy - 12),
                (cx + 4, cy + 12),
                (cx - 1, cy + 3),
            ]
            d.line(pts + [pts[0]], fill=CYAN_SOFT, width=3, joint="curve")
            d.line((cx - 1, cy + 3, cx + 11, cy - 12), fill=CYAN_SOFT, width=2)

        elif icon == "battery":
            d.rounded_rectangle((cx - 12, cy - 7, cx + 9, cy + 7), radius=2, outline=CYAN_SOFT, width=2)
            d.rectangle((cx + 11, cy - 3, cx + 14, cy + 3), fill=CYAN_SOFT)
            d.rectangle((cx - 8, cy - 4, cx - 4, cy + 4), fill=CYAN_SOFT)
            d.rectangle((cx - 2, cy - 4, cx + 2, cy + 4), fill=CYAN_SOFT)
            d.rectangle((cx + 4, cy - 4, cx + 7, cy + 4), fill=CYAN_SOFT)

    rows = [
        ("Mode", "Idle", "gear"),
        ("Voice", "Ready", "mic"),
        ("Nav", "Standby", "nav"),
        ("Battery", "Good", "battery"),
    ]

    row_y = y + 50
    row_h = 40

    for label, value, icon in rows:
        row_box = (x + 10, row_y, x + w - 10, row_y + row_h)

        d.rounded_rectangle(
            row_box,
            radius=9,
            fill=(2, 24, 52, 155),
            outline=(0, 145, 190, 95),
            width=1,
        )

        icx = x + 28
        icy = row_y + row_h // 2

        label_x = x + 55
        value_w = d.textlength(value, font=FONT_SMALL)
        value_x = x + w - 18 - value_w

        small_status_icon(icon, icx, icy)

        d.text((label_x, row_y + 12), label, font=FONT_SMALL, fill=WHITE)
        d.text((value_x, row_y + 12), value, font=FONT_SMALL, fill=CYAN)

        row_y += 47



def draw_title_area(base):
    d = ImageDraw.Draw(base)

    title_x = 120
    title_y = 86

    d.text((title_x, title_y), "SAVO", font=FONT_TITLE, fill=(210, 238, 255))
    d.text((title_x, title_y + 63), "Autonomous Guide Robot", font=FONT_MED, fill=WHITE)

    # Clean readiness line.
    ready_y = title_y + 95
    line_x0 = title_x
    line_x1 = title_x + 30
    text_x = line_x1 + 10

    d.line(
        (line_x0, ready_y + 8, line_x1, ready_y + 8),
        fill=(0, 210, 255, 180),
        width=1,
    )

    d.text(
        (text_x, ready_y),
        "Ready to Assist",
        font=FONT_MED,
        fill=CYAN,
    )



def darken_edges(base):
    overlay = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)
    d.rectangle((0, 0, W, H), outline=(0, 70, 110, 100), width=2)
    d.rounded_rectangle((0, 0, W - 1, H - 1), radius=13, outline=(0, 100, 160, 120), width=2)
    base.alpha_composite(overlay)


def compose_home(robot_frame_name: str) -> Image.Image:
    # Clean generated dashboard background.
    # No old robot360 full-frame image is used here.
    base = Image.new("RGBA", (W, H), (0, 6, 18, 255))
    d = ImageDraw.Draw(base)

    # Smooth dark blue vertical gradient.
    for y in range(H):
        t = y / max(1, H - 1)
        r = int(0 + 2 * t)
        g = int(7 + 12 * t)
        b = int(20 + 34 * t)
        d.line((0, y, W, y), fill=(r, g, b, 255))

    # Subtle futuristic blue glow, not a robot image.
    glow = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    gd = ImageDraw.Draw(glow)

    gd.ellipse((-120, 300, 920, 650), outline=(0, 115, 255, 80), width=5)
    gd.ellipse((-80, 322, 880, 620), outline=(0, 210, 255, 42), width=2)
    gd.ellipse((145, 88, 690, 515), fill=(0, 80, 160, 24))

    for x in range(105, 760, 54):
        gd.line((x, 80, x - 80, 470), fill=(0, 120, 180, 13), width=1)

    glow = glow.filter(ImageFilter.GaussianBlur(10))
    base.alpha_composite(glow)

    # Prefer clean transparent robot layer.
    layer_name = None
    if robot_frame_name == "robot_045.ppm":
        layer_name = "robot_front_right.png"
    elif robot_frame_name == "robot_000.ppm":
        layer_name = "robot_front.png"
    elif robot_frame_name == "robot_315.ppm":
        layer_name = "robot_front_left.png"

    robot_layer = load_robot_layer(layer_name) if layer_name else None

    if robot_layer is not None:
        base.alpha_composite(robot_layer, (ROBOT_X, ROBOT_Y))
    else:
        # No fallback to old full robot scene.
        # If a layer is missing, show a clean placeholder only.
        d = ImageDraw.Draw(base)
        box = (ROBOT_X, ROBOT_Y, ROBOT_X + ROBOT_W, ROBOT_Y + ROBOT_H)
        d.rounded_rectangle(box, radius=24, outline=(0, 160, 220, 120), width=2)
        d.text((ROBOT_X + 145, ROBOT_Y + 178), "ROBOT LAYER MISSING", font=FONT_MED_BOLD, fill=CYAN)

    # Soft dark protection behind title and right panel.
    soft = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    sd = ImageDraw.Draw(soft)
    sd.rectangle((92, 62, 350, 235), fill=(0, 8, 24, 115))
    sd.rectangle((600, 78, 800, 370), fill=(0, 8, 24, 96))
    soft = soft.filter(ImageFilter.GaussianBlur(18))
    base.alpha_composite(soft)

    draw_top_bar(base, "HOME")
    draw_left_menu(base, "Home")
    draw_status_panel(base)
    draw_title_area(base)
    darken_edges(base)

    return base.convert("RGB")



def compose_page_shell(page_title: str, active_item: str) -> Image.Image:
    """Create the common shell used by Voice, Navigate, Status and Power."""

    base = Image.new("RGBA", (W, H), (0, 6, 18, 255))
    d = ImageDraw.Draw(base)

    # Same dark-blue background family as the Home page.
    for y in range(H):
        t = y / max(1, H - 1)
        r = int(0 + 2 * t)
        g = int(7 + 12 * t)
        b = int(20 + 34 * t)
        d.line((0, y, W, y), fill=(r, g, b, 255))

    # Subtle shared background glow.
    glow = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    gd = ImageDraw.Draw(glow)

    gd.ellipse((80, 210, 850, 650), outline=(0, 115, 255, 46), width=4)
    gd.ellipse((145, 95, 790, 520), fill=(0, 80, 160, 18))

    for x in range(120, 780, 60):
        gd.line((x, 75, x - 90, 470), fill=(0, 120, 180, 11), width=1)

    glow = glow.filter(ImageFilter.GaussianBlur(11))
    base.alpha_composite(glow)

    # Shared page content panel.
    panel = (112, 72, 782, 458)

    panel_glow(base, panel, radius=18, alpha=75)

    d.rounded_rectangle(
        panel,
        radius=18,
        fill=(0, 11, 29, 215),
        outline=(0, 190, 235, 120),
        width=1,
    )

    # Common content heading area.
    d.text(
        (138, 88),
        page_title.upper(),
        font=FONT_MED_BOLD,
        fill=CYAN,
    )

    d.text(
        (138, 111),
        "ROBOT SAVO INTERFACE",
        font=FONT_SMALL,
        fill=(188, 220, 235),
    )

    d.line(
        (138, 132, 758, 132),
        fill=(0, 145, 190, 85),
        width=1,
    )

    # The static shell owns layout only.
    # Live page data will be drawn by the C++ runtime below this line.
    draw_top_bar(base, page_title)
    draw_left_menu(base, active_item)
    darken_edges(base)

    return base.convert("RGB")



def generate_runtime_font_atlas(
    file_name: str,
    runtime_font,
    cell_width: int,
    cell_height: int,
):
    """Generate a proportional antialiased glyph atlas for C++ rendering."""

    rows = math.ceil(len(RUNTIME_FONT_CHARS) / RUNTIME_FONT_COLUMNS)

    atlas = Image.new(
        "RGB",
        (RUNTIME_FONT_COLUMNS * cell_width, rows * cell_height),
        (0, 0, 0),
    )

    draw = ImageDraw.Draw(atlas)
    ascent, _descent = runtime_font.getmetrics()
    baseline = RUNTIME_FONT_PADDING + ascent

    for index, character in enumerate(RUNTIME_FONT_CHARS):
        column = index % RUNTIME_FONT_COLUMNS
        row = index // RUNTIME_FONT_COLUMNS

        cell_x = column * cell_width
        cell_y = row * cell_height

        # Store proportional advance in the first pixel:
        # red = advance, green = metadata marker.
        advance = max(
            1,
            min(
                255,
                int(round(draw.textlength(character, font=runtime_font))) + 1,
            ),
        )

        atlas.putpixel(
            (cell_x, cell_y),
            (advance, 255, 0),
        )

        if character == " ":
            continue

        bbox = draw.textbbox(
            (0, baseline),
            character,
            font=runtime_font,
            anchor="ls",
        )

        draw_x = cell_x + RUNTIME_FONT_PADDING - bbox[0]
        draw_y = cell_y + baseline

        draw.text(
            (draw_x, draw_y),
            character,
            font=runtime_font,
            fill=(255, 255, 255),
            anchor="ls",
        )

    output_path = OUT_DIR / file_name
    atlas.save(output_path)

    print(
        f"generated {output_path} "
        f"size={atlas.width}x{atlas.height}"
    )


def save_ppm(img: Image.Image, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    generate_runtime_font_atlas(
        "ui_font_small.ppm",
        FONT_SMALL,
        18,
        22,
    )

    generate_runtime_font_atlas(
        "ui_font_medium.ppm",
        FONT_MED,
        20,
        24,
    )

    generate_runtime_font_atlas(
        "ui_font_large_bold.ppm",
        FONT_BIG,
        32,
        36,
    )

    frames = [
        ("home_idle_000", "robot_045.ppm"),  # front-right default
        ("home_idle_001", "robot_000.ppm"),  # front
        ("home_idle_002", "robot_315.ppm"),  # front-left
        ("home_idle_003", "robot_000.ppm"),  # front
        ("home_idle_004", "robot_045.ppm"),  # front-right return
    ]

    for stem, robot_name in frames:
        img = compose_home(robot_name)
        save_ppm(img, OUT_DIR / f"{stem}.ppm")
        img.save(OUT_DIR / f"{stem}.png")
        print(f"generated {OUT_DIR / f'{stem}.ppm'} from {robot_name}")

    page_shells = [
        ("voice_shell", "VOICE", "Voice"),
        ("navigate_shell", "NAVIGATE", "Navigate"),
        ("status_shell", "STATUS", "Status"),
        ("power_shell", "POWER", "Power"),
    ]

    for stem, page_title, active_item in page_shells:
        img = compose_page_shell(page_title, active_item)
        save_ppm(img, OUT_DIR / f"{stem}.ppm")
        img.save(OUT_DIR / f"{stem}.png")
        print(f"generated {OUT_DIR / f'{stem}.ppm'}")

    (OUT_DIR / "README.txt").write_text(
        "Generated home UI assets. Runtime C++ should load home_idle_000/001/002.ppm.\n"
    )

    print("done")


if __name__ == "__main__":
    main()
