#!/usr/bin/env python3
"""
Robot Face (Wayland-first, Pi DSI 800x480)
- Prefers Wayland; falls back to KMSDRM then X11 automatically
- Fullscreen, double-buffered, vsynced
- Non-blocking blink timing
- Clean exits on ESC/Q or SIGTERM (systemd-friendly)
- Auto-fit to actual screen size (keeps proportions)

Author: Savo Copilot
"""

import os, sys, time, math, random, signal
import pygame

# ---------- Display/SDL hints ----------
# Prefer Wayland on your Pi desktop
os.environ.setdefault("SDL_VIDEODRIVER", "wayland")
# Disable client-side window decorations under Wayland (kiosk-like)
os.environ.setdefault("SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR", "0")
# Try to keep vsync on to avoid tearing
os.environ.setdefault("SDL_RENDER_VSYNC", "1")

# 7" DFRobot DSI native
NATIVE_W, NATIVE_H = 800, 480
FPS = 30

# Colors
BLACK     = (0, 0, 0)
BLUE      = (0, 180, 255)
DARK_BLUE = (0, 100, 180)
GREY      = (80, 80, 80)

# Base (design-space) geometry — will be scaled to actual screen
LEFT_EYE_CENTER  = (250, 200)
RIGHT_EYE_CENTER = (550, 200)
EYE_RADIUS       = 70
MOUTH_XYWH       = (300, 350, 200, 25)

def _choose_driver():
    """Try current SDL_VIDEODRIVER, then kmsdrm, then x11."""
    tried = []
    for drv in (os.environ.get("SDL_VIDEODRIVER") or "wayland", "kmsdrm", "x11"):
        if drv in tried:  # avoid duplicate attempts
            continue
        tried.append(drv)
        try:
            os.environ["SDL_VIDEODRIVER"] = drv
            pygame.display.init()
            # headless drivers don't have any video modes; need a basic probe
            info = pygame.display.get_desktop_sizes()
            if info:
                return drv
        except Exception:
            pass
        finally:
            try:
                pygame.display.quit()
            except Exception:
                pass
    raise RuntimeError("No usable SDL video driver found (wayland/kmsdrm/x11).")

def _scaled(val, sx, sy):
    """Scale scalar or 2-tuple from design-space (800x480) to actual screen."""
    if isinstance(val, (list, tuple)) and len(val) == 2:
        return (int(val[0] * sx), int(val[1] * sy))
    return int(val * (sx if isinstance(val, (int, float)) else 1))

def init_pygame():
    pygame.init()
    driver = _choose_driver()

    # Fullscreen, double-buffered
    flags = pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF
    screen = pygame.display.set_mode((NATIVE_W, NATIVE_H), flags)

    # If compositor reports a different desktop size, refit
    # (Some Wayland setups ignore requested size; we adapt anyway.)
    try:
        w, h = screen.get_size()
    except Exception:
        w, h = NATIVE_W, NATIVE_H

    pygame.display.set_caption(f"Robot Face [{driver}] {w}x{h}")
    pygame.mouse.set_visible(False)
    clock = pygame.time.Clock()

    # Compute scale from design space to actual
    sx = w / NATIVE_W
    sy = h / NATIVE_H

    return screen, clock, sx, sy, (w, h), driver

def draw_mouth(surface, mood, sx, sy):
    x, y, width, height = MOUTH_XYWH
    X, Y = _scaled((x, y), sx, sy)
    W, H = _scaled(width, sx), _scaled(height, sy)

    if mood == "neutral":
        pygame.draw.rect(surface, BLUE, (X, Y, W, H), border_radius=max(8, int(12*sx)))
    elif mood == "happy":
        # Smile arc: use a taller rect to draw arc
        rect = (X, Y, W, max(H, _scaled(100, sy)))
        pygame.draw.arc(surface, BLUE, rect, math.pi, 2 * math.pi, max(4, int(8*sx)))
    elif mood == "sad":
        rect = (X, Y - _scaled(60, sy), W, max(H, _scaled(100, sy)))
        pygame.draw.arc(surface, BLUE, rect, 0, math.pi, max(4, int(8*sx)))

def draw_eyes(surface, eyes_closed, sx, sy):
    L = _scaled(LEFT_EYE_CENTER, sx, sy)
    R = _scaled(RIGHT_EYE_CENTER, sx, sy)
    ER = _scaled(EYE_RADIUS, sx)  # keep roughly proportional in X
    pupil = max(6, int(20 * (sx + sy) * 0.5))

    if not eyes_closed:
        pygame.draw.circle(surface, BLUE, L, ER)
        pygame.draw.circle(surface, BLUE, R, ER)
        pygame.draw.circle(surface, DARK_BLUE, L, pupil)
        pygame.draw.circle(surface, DARK_BLUE, R, pupil)
    else:
        # horizontal blink lines
        pygame.draw.rect(surface, BLUE, (L[0] - ER, L[1] - max(3, int(5*sy)), 2*ER, max(6, int(10*sy))))
        pygame.draw.rect(surface, BLUE, (R[0] - ER, R[1] - max(3, int(5*sy)), 2*ER, max(6, int(10*sy))))

def draw_head(surface, sx, sy):
    rect = ( _scaled(100, sx), _scaled(50, sy), _scaled(600, sx), _scaled(380, sy) )
    pygame.draw.rect(surface, GREY, rect, border_radius=max(24, int(60*min(sx, sy))))

def draw_face(surface, mood, eyes_closed, sx, sy):
    surface.fill(BLACK)
    draw_head(surface, sx, sy)
    draw_eyes(surface, eyes_closed, sx, sy)
    draw_mouth(surface, mood, sx, sy)
    pygame.display.flip()

def main():
    screen, clock, sx, sy, (w, h), driver = init_pygame()

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
        # events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False

        t = time.monotonic()

        # blink timing (non-blocking)
        if not eyes_closed and t >= next_blink_at:
            eyes_closed = True
            blink_end_at = t + 0.15  # 150 ms
        if eyes_closed and blink_end_at and t >= blink_end_at:
            eyes_closed = False
            next_blink_at = t + random.uniform(3.0, 6.0)
            blink_end_at = None

        # mood rotation
        if t >= next_mood_at:
            mood = random.choice(["neutral", "happy", "sad"])
            next_mood_at = t + random.uniform(4.0, 8.0)

        draw_face(screen, mood, eyes_closed, sx, sy)
        clock.tick(FPS)

    pygame.quit()
    sys.exit(0)

if __name__ == "__main__":
    try:
        main()
    except RuntimeError as e:
        print("Display init failed:", e)
        sys.exit(1)
