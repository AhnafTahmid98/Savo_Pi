#!/usr/bin/env python3
"""
Robot Face (Wayland-first, Pi DSI 800x480)
- Prefers Wayland; works under Weston
- Non-blocking blink, clean SIGTERM handling
- Proper scaling helpers (fixed)
"""

import os, sys, time, math, random, signal
import pygame

# Wayland first (Weston)
os.environ.setdefault("SDL_VIDEODRIVER", "wayland")
os.environ.setdefault("SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR", "0")
os.environ.setdefault("SDL_RENDER_VSYNC", "1")

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
def Sx(v): return int(v * _SX)
def Sy(v): return int(v * _SY)
def S2(pt): return (Sx(pt[0]), Sy(pt[1]))
def Sr():  return min(_SX, _SY)  # for border radii / thickness

def _choose_driver():
    """Confirm we can init a video driver (already preferring wayland)."""
    tried = []
    for drv in (os.environ.get("SDL_VIDEODRIVER") or "wayland", "kmsdrm", "x11"):
        if drv in tried: 
            continue
        tried.append(drv)
        try:
            os.environ["SDL_VIDEODRIVER"] = drv
            pygame.display.init()
            info = pygame.display.get_desktop_sizes()
            if info:
                return drv
        except Exception:
            pass
        finally:
            try: pygame.display.quit()
            except Exception: pass
    raise RuntimeError("No usable SDL video driver found (wayland/kmsdrm/x11).")

def init_pygame():
    pygame.init()
    driver = _choose_driver()

    flags = pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF
    screen = pygame.display.set_mode((NATIVE_W, NATIVE_H), flags)
    w, h = screen.get_size()
    pygame.display.set_caption(f"Robot Face [{driver}] {w}x{h}")
    pygame.mouse.set_visible(False)
    clock = pygame.time.Clock()

    # set global scales
    global _SX, _SY
    _SX, _SY = (w / NATIVE_W), (h / NATIVE_H)
    return screen, clock, (w, h), driver

def draw_mouth(surface, mood):
    x, y, width, height = MOUTH_XYWH
    X, Y, W, H = Sx(x), Sy(y), Sx(width), Sy(height)
    if mood == "neutral":
        pygame.draw.rect(surface, BLUE, (X, Y, W, H), border_radius=max(8, int(12 * Sr())))
    elif mood == "happy":
        rect = (X, Y, W, max(H, Sy(100)))
        pygame.draw.arc(surface, BLUE, rect, math.pi, 2 * math.pi, max(4, int(8 * Sr())))
    elif mood == "sad":
        rect = (X, Y - Sy(60), W, max(H, Sy(100)))
        pygame.draw.arc(surface, BLUE, rect, 0, math.pi, max(4, int(8 * Sr())))

def draw_eyes(surface, eyes_closed):
    L = S2(LEFT_EYE_CENTER)
    R = S2(RIGHT_EYE_CENTER)
    ER = Sx(EYE_RADIUS)
    pupil = max(6, int(20 * (Sr())))
    if not eyes_closed:
        pygame.draw.circle(surface, BLUE, L, ER)
        pygame.draw.circle(surface, BLUE, R, ER)
        pygame.draw.circle(surface, DARK_BLUE, L, pupil)
        pygame.draw.circle(surface, DARK_BLUE, R, pupil)
    else:
        pygame.draw.rect(surface, BLUE, (L[0] - ER, L[1] - max(3, Sy(5)), 2*ER, max(6, Sy(10))))
        pygame.draw.rect(surface, BLUE, (R[0] - ER, R[1] - max(3, Sy(5)), 2*ER, max(6, Sy(10))))

def draw_head(surface):
    rect = ( Sx(100), Sy(50), Sx(600), Sy(380) )
    pygame.draw.rect(surface, GREY, rect, border_radius=max(24, int(60 * Sr())))

def draw_face(surface, mood, eyes_closed):
    surface.fill(BLACK)
    draw_head(surface)
    draw_eyes(surface, eyes_closed)
    draw_mouth(surface, mood)
    pygame.display.flip()

def main():
    screen, clock, _, driver = init_pygame()

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
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
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
