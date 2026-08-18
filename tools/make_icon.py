#!/usr/bin/env python3
"""Генератор иконки для can_gui.py.

Мотив: статор с тремя катушками (BLDC) и две линии шины CAN.
Запуск: python3 tools/make_icon.py  ->  tools/icon.png (256x256)
"""
import math
import os
from PIL import Image, ImageDraw

S = 256                       # размер иконки
BG      = (31, 39, 51, 255)   # фон, как тёмная тема GUI
EDGE    = (58, 70, 88, 255)   # кромка
STATOR  = (52, 152, 219, 255) # синий - статор
COIL    = (46, 204, 113, 255) # зелёный - катушки
ROTOR   = (236, 240, 241, 255)
BUS     = (241, 196, 15, 255) # жёлтый - линии CAN


def make(size=S):
    k = size / 256.0
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    def px(v):
        return int(round(v * k))

    # фон со скруглением
    d.rounded_rectangle([0, 0, size - 1, size - 1], radius=px(52), fill=BG,
                        outline=EDGE, width=max(1, px(3)))

    cx, cy, r = size / 2, px(112), px(66)

    # статор
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=STATOR, width=max(2, px(13)))

    # три катушки под 120 градусов
    coil_r = px(20)
    for a in (-90, 30, 150):
        ax = cx + r * math.cos(math.radians(a))
        ay = cy + r * math.sin(math.radians(a))
        d.ellipse([ax - coil_r, ay - coil_r, ax + coil_r, ay + coil_r], fill=COIL)

    # ротор с меткой угла
    rr = px(27)
    d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=ROTOR)
    d.line([cx, cy, cx + rr * 0.95, cy - rr * 0.95], fill=BG, width=max(2, px(7)))

    # две линии шины CAN
    y1, y2 = px(196), px(220)
    for y in (y1, y2):
        d.line([px(34), y, size - px(34), y], fill=BUS, width=max(2, px(10)))
    # узлы на шине
    for x in (px(64), size / 2, size - px(64)):
        d.ellipse([x - px(9), y1 - px(9), x + px(9), y1 + px(9)], fill=BG)
    return img


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icon.png")
    make().save(out)
    # контроль читаемости в мелких размерах
    for s in (16, 24, 32, 48, 64, 128):
        make(s).save(out.replace("icon.png", f".preview_{s}.png"))
    print("готово:", out)
