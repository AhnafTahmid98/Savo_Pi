#!/usr/bin/env python3
"""
Robot Face (Pi DSI, fullscreen, SDL auto-driver)

- Prefers kmsdrm (direct KMS on the Pi DSI)
- Falls back to wayland or x11 if available
- Non-blocking blink, clean SIGTERM handling
- Uses a logical 800x480 layout, but scales to the real screen size
  (for the DFRobot 7" DSI this is usually 1024x600).

Author: Robot Savo
"""

from __future__ import annotations

import os
import sys
import time
import math
import random
import signal
from typing import List, Tuple

import pygame

# ---------------------------------------------------------------------------
# SDL / video backend hints
# ---------------------------------------------------------------------------
os.environ.setdefault("SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR", "0")
os.environ.setdefault("SDL_RENDER_VSYNC", "1")

# Logical design size (we scale to whatever the real screen is)
NATIVE_W, NATIVE_H = 800, 480
FPS = 30

BLACK     = (0, 0, 0)
BLUE      = (0, 180, 255)
DARK_BLUE = (0, 100, 180)
GREY      = (80, 80, 80)

LEFT_EYE_CENTER  = (250, 200)
RIGHT_EYE_CENTER = (550, 200)
EYE_RADIUS       = 70
MOUTH_XYWH       = (300, 350, 200, 25)

# --- scaling helpers (GLOBAL after init) ---
_SX, _SY = 1.0, 1.0
def Sx(v: float) -> int: return int(v * _SX)
def Sy(v: float) -> int: return int(v * _SY)
def S2(pt: Tuple[float, float]) -> Tuple[int, int]: return (Sx(pt[0]), Sy(pt[1]))
def Sr() -> float: return min(_SX, _SY)


# ---------------------------------------------------------------------------
# SDL driver selection — mirror the logic of display_manager_node
# ---------------------------------------------------------------------------

def _choose_driver() -> str:
    """
    Try SDL video drivers and pick one that works.

    Order:
      1) whatever SDL_VIDEODRIVER is currently set to (if any)
      2) kmsdrm
      3) wayland
      4) x11
    """
    tried: List[str] = []
    candidates: List[str] = []

    env_drv = os.environ.get("SDL_VIDEODRIVER")
    if env_drv:
        candidates.append(env_drv)

    for drv in ("kmsdrm", "wayland", "x11"):
        if drv not in candidates:
            candidates.append(drv)

    for drv in candidates:
        if drv in tried:
            continue
        tried.append(drv)

        try:
            os.environ["SDL_VIDEODRIVER"] = drv
            pygame.display.init()
            info = pygame.display.get_desktop_sizes()
            if info:
                print(
                    f"[face.py] SDL driver '{drv}' OK, desktop sizes: {info}",
                    flush=True,
                )
                pygame.display.quit()
                return drv
        except Exception as exc:
            print(
                f"[face.py] SDL driver '{drv}' failed: {exc}",
                flush=True,
            )
        finally:
            try:
                pygame.display.quit()
            except Exception:
                pass

    raise RuntimeError(
        "No usable SDL video driver found (tried: " + ", ".join(tried) + ")."
    )


def init_pygame():
    pygame.init()

    drv = _choose_driver()
    os.environ["SDL_VIDEODRIVER"] = drv

    # Now open the real fullscreen window
    flags = pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF
    screen = pygame.display.set_mode((NATIVE_W, NATIVE_H), flags)
    w, h = screen.get_size()
    pygame.display.set_caption(f"Robot Face [{drv}] {w}x{h}")
    pygame.mouse.set_visible(False)
    clock = pygame.time.Clock()

    # set global scales
    global _SX, _SY
    _SX, _SY = (w / float(NATIVE_W)), (h / float(NATIVE_H))

    return screen, clock, (w, h), drv


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

def draw_mouth(surface, mood):
    x, y, width, height = MOUTH_XYWH
    X, Y, W, H = Sx(x), Sy(y), Sx(width), Sy(height)
    if mood == "neutral":
        pygame.draw.rect(
            surface,
            BLUE,
            (X, Y, W, H),
            border_radius=max(8, int(12 * Sr())),
        )
    elif mood == "happy":
        rect = (X, Y, W, max(H, Sy(100)))
        pygame.draw.arc(
            surface,
            BLUE,
            rect,
            math.pi,
            2 * math.pi,
            max(4, int(8 * Sr())),
        )
    elif mood == "sad":
        rect = (X, Y - Sy(60), W, max(H, Sy(100)))
        pygame.draw.arc(
            surface,
            BLUE,
            rect,
            0,
            math.pi,
            max(4, int(8 * Sr())),
        )


def draw_eyes(surface, eyes_closed):
    L = S2(LEFT_EYE_CENTER)
    R = S2(RIGHT_EYE_CENTER)
    ER = Sx(EYE_RADIUS)
    pupil = max(6, int(20 * Sr()))
    if not eyes_closed:
        pygame.draw.circle(surface, BLUE, L, ER)
        pygame.draw.circle(surface, BLUE, R, ER)
        pygame.draw.circle(surface, DARK_BLUE, L, pupil)
        pygame.draw.circle(surface, DARK_BLUE, R, pupil)
    else:
        pygame.draw.rect(
            surface,
            BLUE,
            (L[0] - ER, L[1] - max(3, Sy(5)), 2 * ER, max(6, Sy(10))),
        )
        pygame.draw.rect(
            surface,
            BLUE,
            (R[0] - ER, R[1] - max(3, Sy(5)), 2 * ER, max(6, Sy(10))),
        )


def draw_head(surface):
    rect = (Sx(100), Sy(50), Sx(600), Sy(380))
    pygame.draw.rect(
        surface,
        GREY,
        rect,
        border_radius=max(24, int(60 * Sr())),
    )


def draw_face(surface, mood, eyes_closed):
    surface.fill(BLACK)
    draw_head(surface)
    draw_eyes(surface, eyes_closed)
    draw_mouth(surface, mood)
    pygame.display.flip()


# ---------------------------------------------------------------------------
# main loop
# ---------------------------------------------------------------------------

def main():
    screen, clock, _, driver = init_pygame()
    print(f"[face.py] Using SDL driver '{driver}'", flush=True)

    mood = "neutral"
    eyes_closed = False

    now = time.monotonic()
    next_blink_at = now + random.uniform(2.0, 5.0)
    blink_end_at = None
    next_mood_at = now + random.uniform(4.0, 8.0)

    running = True

    def handle_sigterm(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, handle_sigterm)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (
                pygame.K_ESCAPE,
                pygame.K_q,
            ):
                running = False

        t = time.monotonic()

        # blink
        if not eyes_closed and t >= next_blink_at:
            eyes_closed = True
            blink_end_at = t + 0.15
        if eyes_closed and blink_end_at and t >= blink_end_at:
            eyes_closed = False
            next_blink_at = t + random.uniform(3.0, 6.0)
            blink_end_at = None

        # mood
        if t >= next_mood_at:
            mood = random.choice(["neutral", "happy", "sad"])
            next_mood_at = t + random.uniform(4.0, 8.0)

        draw_face(screen, mood, eyes_closed)
        clock.tick(FPS)

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as e:
        print("Display init failed:", e)
        sys.exit(1)
