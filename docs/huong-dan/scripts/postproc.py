#!/usr/bin/env python3
"""Hậu kỳ ảnh chụp màn hình + video mô phỏng cho sổ tay (cắt, đánh số chú thích, ghép video)."""
import json
import os
import subprocess

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "data", "sim_raw")   # ảnh chụp + video thô (không đưa vào git)
IMG = os.path.join(HERE, "..", "img")
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
WIN = (66, 69, 1920, 1080)          # vùng cửa sổ ứng dụng (bỏ dock trái, thanh trên, thanh tiêu đề)
ACC = (196, 127, 12)
META = {}


def save_webp(im, name, max_w=1600, q=84):
    if im.width > max_w:
        im = im.resize((max_w, round(im.height * max_w / im.width)), Image.LANCZOS)
    path = os.path.join(IMG, name)
    im.convert("RGB").save(path, "WEBP", quality=q, method=6)
    META[name] = list(im.size)
    print(name, im.size, os.path.getsize(path) // 1024, "KB")


def markers(im, pts, offset=(0, 0), r=19):
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(FONT_B, 21)
    for i, (x, y) in enumerate(pts, 1):
        x, y = x - offset[0], y - offset[1]
        d.ellipse((x - r - 3, y - r - 3, x + r + 3, y + r + 3), fill=(255, 255, 255))
        d.ellipse((x - r, y - r, x + r, y + r), fill=ACC)
        t = str(i)
        w = d.textlength(t, font=f)
        d.text((x - w / 2, y - 13), t, font=f, fill=(255, 255, 255))
    return im


def shot(name):
    return Image.open(os.path.join(SIM, name)).convert("RGB")


def screenshots():
    # 1. Toàn cảnh khi bấm Docking Sim (2 cửa sổ cạnh nhau)
    save_webp(shot("shot_overview.png").crop(WIN), "s01-overview.webp")
    # 2. Gazebo có chú thích
    g = shot("shot_gazebo.png").crop(WIN)
    markers(g, [(425, 690), (640, 640), (1112, 545), (790, 300), (1250, 268), (1275, 760),
                (245, 1032), (1650, 518), (1318, 1048), (545, 140)], offset=WIN[:2])
    save_webp(g, "s02-gazebo.webp")
    # 3. RViz có chú thích
    r = shot("shot_rviz3.png").crop(WIN)
    markers(r, [(345, 250), (1565, 600), (1425, 300), (1168, 470), (1088, 805), (1215, 652),
                (470, 912)], offset=WIN[:2])
    save_webp(r, "s03-rviz.webp")
    # 4. rqt_graph: chuỗi node/topic chính
    save_webp(shot("shot_rqt2.png").crop((290, 598, 1912, 812)), "s04-rqt-graph.webp", max_w=1622)
    # 5. Biểu tượng trên taskbar (6 launcher do dự án ghim)
    strip = shot("shot_rqt2.png").crop((0, 440, 66, 820)).resize((132, 760), Image.LANCZOS)
    canvas = Image.new("RGB", (720, 760), (255, 255, 255))
    canvas.paste(strip, (0, 0))
    d = ImageDraw.Draw(canvas)
    f = ImageFont.truetype(FONT, 26)
    fb = ImageFont.truetype(FONT_B, 26)
    labels = [(60, "RViz2", False), (186, "rqt", False), (314, "Gazebo", False),
              (440, "PlotJuggler", False), (570, "CloudCompare", False),
              (700, "Docking Sim ← mở mô phỏng", True)]
    for y, t, bold in labels:
        d.line((140, y, 168, y), fill=(150, 160, 170), width=2)
        d.text((178, y - 16), t, font=fb if bold else f, fill=ACC if bold else (27, 36, 48))
    save_webp(canvas, "s05-launchers.webp", max_w=720)


def two_panes(src):
    """Ghép vùng 3D của Gazebo (trái) và RViz (phải) từ ảnh bố cục quay video."""
    im = shot(src)
    gz = im.crop((66, 118, 948, 1080)).resize((660, 720), Image.LANCZOS)
    rv = im.crop((1382, 132, 1905, 1048)).resize((412, 720), Image.LANCZOS)
    out = Image.new("RGB", (1072 + 8, 720), (255, 255, 255))
    out.paste(gz, (0, 0))
    out.paste(rv, (668, 0))
    return out


def label(im, text, xy, size=24):
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(FONT_B, size)
    w = d.textlength(text, font=f)
    x, y = xy
    d.rounded_rectangle((x - 8, y - 6, x + w + 8, y + size + 8), radius=6, fill=(20, 26, 32))
    d.text((x, y), text, font=f, fill=(255, 255, 255))


def before_after():
    a = two_panes("shot_layout3.png")
    b = two_panes("shot_docked.png")
    for im, t in ((a, "TRƯỚC: robot ở điểm xuất phát"), (b, "SAU: DOCKED, cách đích 1,0 cm")):
        label(im, t, (16, 14))
    out = Image.new("RGB", (a.width, a.height * 2 + 12), (255, 255, 255))
    out.paste(a, (0, 0))
    out.paste(b, (0, a.height + 12))
    save_webp(out, "s06-before-after.webp", max_w=1080)


def video():
    raw = os.path.join(SIM, "raw_run1.mp4")
    out = os.path.join(IMG, "dock-run.mp4")
    font = FONT_B.replace(":", r"\:")
    vf = ("[0:v]split=2[a][b];"
          "[a]crop=882:962:66:118,scale=660:720[g];"
          "[b]crop=523:916:1382:132,scale=412:720[r];"
          "[g][r]hstack=inputs=2,"
          f"drawtext=fontfile={font}:text='Gazebo':x=14:y=12:fontsize=22:fontcolor=white:box=1:boxcolor=0x141A20CC:boxborderw=7,"
          f"drawtext=fontfile={font}:text='RViz':x=674:y=12:fontsize=22:fontcolor=white:box=1:boxcolor=0x141A20CC:boxborderw=7[v]")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", "2.0", "-t", "19.5", "-i", raw,
                    "-filter_complex", vf, "-map", "[v]", "-r", "15", "-c:v", "libx264",
                    "-preset", "slow", "-crf", "27", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                    out], check=True)
    print("dock-run.mp4", os.path.getsize(out) // 1024, "KB")
    # ảnh poster + 4 khung theo trạng thái (thời điểm tính từ đầu video đã cắt 2 s)
    rows = [json.loads(line) for line in open(os.path.join(HERE, "data", "run1.jsonl"))]
    ctl = [(r["t"], r["state"].split(":")[0]) for r in rows if r["k"] == "ctl"]
    t_start = next(t for t, s in ctl if s == "WAIT_DETECTION")
    t_blind = next(t for t, s in ctl if s == "BLIND")
    t_dock = next(t for t, s in ctl if s == "DOCKED")
    picks = [(t_start + 0.3, f"0,0 s · APPROACH bắt đầu"),
             ((t_start + t_blind) / 2, f"{(t_blind - t_start) / 2:.1f} s · APPROACH (đang cua)"),
             ((t_blind + t_dock) / 2, f"{(t_blind + t_dock) / 2 - t_start:.1f} s · BLIND (pose giữ)"),
             (t_dock + 0.8, f"{t_dock - t_start:.1f} s · DOCKED")]
    frames = []
    for k, (t, cap) in enumerate(picks):
        png = os.path.join(SIM, f"frame{k}.png")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{max(0.0, t - 2.0):.2f}", "-i", out,
                        "-frames:v", "1", png], check=True)
        im = Image.open(png).convert("RGB")
        label(im, cap, (12, im.height - 50), size=22)
        frames.append(im)
        if k == 1:
            save_webp(im, "dock-run-poster.webp", max_w=1072)
    w, h = frames[0].size
    grid = Image.new("RGB", (w * 2 + 10, h * 2 + 10), (255, 255, 255))
    for k, im in enumerate(frames):
        grid.paste(im, ((k % 2) * (w + 10), (k // 2) * (h + 10)))
    save_webp(grid, "s07-filmstrip.webp", max_w=1400)
    META["timeline"] = dict(t_start=t_start, t_blind=t_blind, t_dock=t_dock)


if __name__ == "__main__":
    import sys
    os.makedirs(IMG, exist_ok=True)
    screenshots()
    if "--shots-only" not in sys.argv:
        before_after()
        video()
    json.dump(META, open(os.path.join(SIM, "postproc_meta.json"), "w"), indent=1)
    print(json.dumps(META.get("timeline")))
