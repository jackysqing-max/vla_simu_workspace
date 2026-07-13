import cv2
import mediapipe as mp
import time
import math
import csv
import numpy as np
import nibabel as nib
import pyvista as pv
from pyvistaqt import BackgroundPlotter

# ============================================================
# 0) SENSITIVITY (UN SOLO PARAMETRO)
# ============================================================
SENSITIVITY = 1.0  # 0.6 meno sensibile, 1.0 default, 1.5-2.0 più sensibile

def make_config(sens: float):
    sens = max(0.1, float(sens))
    return {
        # soglie z (palm.z - tip.z): più alto => più difficile attivare
        "z_shift": 0.1 / sens,         # indice avanti per SHIFT
        "z_rotation": 0.05 / sens,      # medio avanti per ROTATE
        "z_margin": 0.012 / sens,        # margine per decidere chi domina (anti-interferenze)

        # filtri jitter
        "frame_move_thresh_px": 1.0 / sens,
        "pinch_deadzone_px": 2.0 / sens,

        # guadagni
        "pan_scale_world_per_px": 1.0 * sens,  # pan
        "rot_deg_per_px": 1 * sens,         # rotazione (deg/px)
        "zoom_gain_per_px": 0.004 * sens,      # zoom: factor = exp(delta*gain)

        # CSV
        "csv_path": f"gesture_metrics_{time.strftime('%Y%m%d_%H%M%S')}.csv",
    }

CFG = make_config(SENSITIVITY)

# ============================================================
# 1) MRI LOAD + PYVISTA (NON BLOCCANTE)
# ============================================================
nifti_path = "t1_150116AR_20150115.nii"  # <-- cambia qui

img = nib.load(nifti_path)
data = img.get_fdata().astype(np.float32)
if data.ndim == 4:
    data = data[..., 0]

volume = pv.wrap(data)

plotter = BackgroundPlotter(title="MRI 3D - Gesture Controlled")
plotter.add_volume(volume, cmap="gray", opacity="sigmoid", shade=False)
plotter.add_axes()
plotter.show_grid()

# ============================================================
# 2) CAMERA CONTROLS: PAN / ROTATE / ZOOM
# ============================================================
def pan_screenshot(dx_pix, dy_pix):
    cam = plotter.camera

    pos = np.array(cam.position, dtype=float)
    focal = np.array(cam.focal_point, dtype=float)
    view_up = np.array(cam.up, dtype=float)

    view_dir = focal - pos
    view_dir /= (np.linalg.norm(view_dir) + 1e-9)

    right = np.cross(view_dir, view_up)
    right /= (np.linalg.norm(right) + 1e-9)

    up = view_up / (np.linalg.norm(view_up) + 1e-9)

    scale = CFG["pan_scale_world_per_px"]
    offset = right * (dx_pix * scale) + up * (-dy_pix * scale)

    cam.position = tuple(pos + offset)
    cam.focal_point = tuple(focal + offset)

def rotate_camera(dx_pix, dy_pix):
    # FIX robusto (VTK methods)
    cam = plotter.camera
    deg = CFG["rot_deg_per_px"]
    cam.Azimuth(float(dx_pix) * deg)
    cam.Elevation(float(-dy_pix) * deg)
    cam.OrthogonalizeViewUp()

def zoom_by_pinch(delta_dist_pix):
    cam = plotter.camera
    gain = CFG["zoom_gain_per_px"]
    factor = math.exp(float(delta_dist_pix) * gain)
    cam.Zoom(factor)

# ============================================================
# 3) MEDIAPIPE + HELPERS
# ============================================================
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

def lm_px(lm, idx, w, h):
    p = lm.landmark[idx]
    return int(p.x * w), int(p.y * h)

def is_extended(lm, tip_id, pip_id):
    # euristica: esteso se tip sopra pip (y minore)
    return lm.landmark[tip_id].y < lm.landmark[pip_id].y

def z_diff(palm, tip):
    return palm.z - tip.z

def dist_px(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

# ============================================================
# 4) CSV LOG + METRICHE (solo quando MODE != OFF)
# ============================================================
csv_f = open(CFG["csv_path"], "w", newline="", encoding="utf-8")
fields = [
    "t", "mode", "action",
    "x_i", "y_i", "x_m", "y_m", "x_t", "y_t",
    "z_i", "z_m", "pinch_dist",
    "dx", "dy", "delta_pinch",
    "proc_ms", "render_ms"
]
writer = csv.DictWriter(csv_f, fieldnames=fields)
writer.writeheader()

metrics = {
    "on_frames": 0,
    "on_time_s": 0.0,
    "commands": 0,
    "mode_switches": 0,
    "sum_proc_ms": 0.0,
    "sum_render_ms": 0.0,
    "sum_dx2": 0.0,   # per RMS jitter
    "sum_dy2": 0.0,
    "t_on_start": None,
}

# ===== stats per modalità =====
per_mode = {
    "SHIFT":  {"frames": 0, "action_frames": 0, "sum_proc_ms": 0.0, "sum_render_ms": 0.0, "sum_dx2": 0.0, "sum_dy2": 0.0},
    "ROTATE": {"frames": 0, "action_frames": 0, "sum_proc_ms": 0.0, "sum_render_ms": 0.0, "sum_dx2": 0.0, "sum_dy2": 0.0},
    "ZOOM":   {"frames": 0, "action_frames": 0, "sum_proc_ms": 0.0, "sum_render_ms": 0.0, "sum_dx2": 0.0, "sum_dy2": 0.0},
}



def metrics_on_enter():
    if metrics["t_on_start"] is None:
        metrics["t_on_start"] = time.perf_counter()

def metrics_on_exit():
    if metrics["t_on_start"] is not None:
        metrics["on_time_s"] += (time.perf_counter() - metrics["t_on_start"])
        metrics["t_on_start"] = None

# ============================================================
# 5) STATO
# ============================================================
mode = "OFF"  # OFF / SHIFT / ROTATE / ZOOM
prev_i = None
prev_m = None
prev_pinch = None

# ============================================================
# 6) LOOP
# ============================================================
cap = cv2.VideoCapture(0)
print("Premi 'q' per uscire.")
print("CSV:", CFG["csv_path"])

RED = (0, 0, 255)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
YELLOW = (0, 255, 255)
BLUE = (255, 255, 0)

while True:
    t0 = time.perf_counter()

    ok, frame = cap.read()
    if not ok:
        break

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rgb.flags.writeable = False
    result = hands.process(rgb)

    # defaults (per log)
    x_i = y_i = x_m = y_m = x_t = y_t = None
    z_i = z_m = 0.0
    pinch_dist = 0.0
    dx = dy = 0.0
    delta_pinch = 0.0
    action = ""
    render_ms = 0.0

    if result.multi_hand_landmarks:
        lm = result.multi_hand_landmarks[0]
        mp_drawing.draw_landmarks(frame, lm, mp_hands.HAND_CONNECTIONS)

        palm = lm.landmark[0]

        # tips
        x_i, y_i = lm_px(lm, 8, w, h)    # index tip
        x_m, y_m = lm_px(lm, 12, w, h)   # middle tip
        x_t, y_t = lm_px(lm, 4, w, h)    # thumb tip

        index_tip = lm.landmark[8]
        middle_tip = lm.landmark[12]
        thumb_tip = lm.landmark[4]

        z_i = z_diff(palm, index_tip)
        z_m = z_diff(palm, middle_tip)

        # estensioni
        idx_ext = is_extended(lm, 8, 6)
        mid_ext = is_extended(lm, 12, 10)
        ring_ext = is_extended(lm, 16, 14)
        pinky_ext = is_extended(lm, 20, 18)

        ring_closed = not ring_ext
        pinky_closed = not pinky_ext

        # thumb fuori (euristica): tip sopra IP
        thumb_ext = lm.landmark[4].y < lm.landmark[3].y

        pinch_dist = dist_px((x_i, y_i), (x_t, y_t))

        # -----------------------------
        # CONDIZIONI MODALITÀ
        # -----------------------------
        z_margin = CFG["z_margin"]

        # ZOOM: indice+pollice fuori, anulare+mignolo chiusi, OFF quando alzi medio
        zoom_cond = idx_ext and thumb_ext and ring_closed and pinky_closed and (not mid_ext)

        # SHIFT: indice avanti e domina il medio
        shift_cond = (z_i > CFG["z_shift"]) and idx_ext and (z_i > z_m + z_margin)

        # ROTATE: medio avanti e domina l'indice
        rotate_cond = (z_m > CFG["z_rotation"]) and mid_ext and (z_m > z_i + z_margin)

        # priorità
        new_mode = "OFF"
        if zoom_cond:
            new_mode = "ZOOM"
        elif rotate_cond:
            new_mode = "ROTATE"
        elif shift_cond:
            new_mode = "SHIFT"
        else:
            new_mode = "OFF"

        # mode switch
        if new_mode != mode:
            metrics["mode_switches"] += 1
            # chiudi periodo ON se uscivo da ON
            if mode != "OFF" and new_mode == "OFF":
                metrics_on_exit()
            # apri periodo ON se entravo in ON
            if mode == "OFF" and new_mode != "OFF":
                metrics_on_enter()

            mode = new_mode
            prev_i = prev_m = prev_pinch = None

        # -----------------------------
        # ESECUZIONE COMANDI
        # -----------------------------
        render_t0 = None

        if mode == "SHIFT":
            if prev_i is not None:
                dx = x_i - prev_i[0]
                dy = y_i - prev_i[1]
                # jitter stats anche se non mando comando
                metrics["sum_dx2"] += dx * dx
                metrics["sum_dy2"] += dy * dy

                if abs(dx) > CFG["frame_move_thresh_px"] or abs(dy) > CFG["frame_move_thresh_px"]:
                    render_t0 = time.perf_counter()
                    pan_screenshot(dx, dy)
                    plotter.render()
                    render_ms = (time.perf_counter() - render_t0) * 1000.0
                    action = "PAN"
                    metrics["commands"] += 1
            prev_i = (x_i, y_i)

        elif mode == "ROTATE":
            if prev_m is not None:
                dx = x_m - prev_m[0]
                dy = y_m - prev_m[1]
                metrics["sum_dx2"] += dx * dx
                metrics["sum_dy2"] += dy * dy

                if abs(dx) > CFG["frame_move_thresh_px"] or abs(dy) > CFG["frame_move_thresh_px"]:
                    render_t0 = time.perf_counter()
                    rotate_camera(dx, dy)
                    plotter.render()
                    render_ms = (time.perf_counter() - render_t0) * 1000.0
                    action = "ROT"
                    metrics["commands"] += 1
            prev_m = (x_m, y_m)

        elif mode == "ZOOM":
            if prev_pinch is not None:
                delta_pinch = pinch_dist - prev_pinch
                # per jitter usiamo delta come dx (solo per un indicatore)
                metrics["sum_dx2"] += delta_pinch * delta_pinch

                if abs(delta_pinch) > CFG["pinch_deadzone_px"]:
                    render_t0 = time.perf_counter()
                    zoom_by_pinch(delta_pinch)
                    plotter.render()
                    render_ms = (time.perf_counter() - render_t0) * 1000.0
                    action = "ZOOM"
                    metrics["commands"] += 1
            prev_pinch = pinch_dist

        else:
            prev_i = prev_m = prev_pinch = None

        # -----------------------------
        # COLORI (come richiesto)
        # -----------------------------
        idx_col = RED
        mid_col = RED
        thb_col = RED

        if mode == "SHIFT":
            idx_col = GREEN
        elif mode == "ROTATE":
            mid_col = GREEN
        elif mode == "ZOOM":
            idx_col = GREEN
            thb_col = GREEN

        cv2.circle(frame, (x_i, y_i), 10, idx_col, -1)
        cv2.circle(frame, (x_m, y_m), 10, mid_col, -1)
        cv2.circle(frame, (x_t, y_t), 10, thb_col, -1)

    else:
        # no hand
        if mode != "OFF":
            metrics_on_exit()
        mode = "OFF"
        prev_i = prev_m = prev_pinch = None

    proc_ms = (time.perf_counter() - t0) * 1000.0

    # log solo quando ON
    if mode != "OFF":
        metrics["on_frames"] += 1
        metrics["sum_proc_ms"] += proc_ms
        metrics["sum_render_ms"] += render_ms

            # ===== accumulo per-mode =====
        if mode in per_mode:
            per_mode[mode]["frames"] += 1
            per_mode[mode]["sum_proc_ms"] += proc_ms
            per_mode[mode]["sum_render_ms"] += render_ms

            # action non vuota => frame che ha generato comando reale
            if action is not None and action != "":
                per_mode[mode]["action_frames"] += 1

            # jitter per mode: SHIFT/ROTATE usano dx,dy; ZOOM usa delta_pinch come dx
            if mode == "ZOOM":
                per_mode[mode]["sum_dx2"] += (delta_pinch * delta_pinch)
            else:
                per_mode[mode]["sum_dx2"] += (dx * dx)
                per_mode[mode]["sum_dy2"] += (dy * dy)


        writer.writerow({
            "t": time.time(),
            "mode": mode,
            "action": action,
            "x_i": x_i, "y_i": y_i,
            "x_m": x_m, "y_m": y_m,
            "x_t": x_t, "y_t": y_t,
            "z_i": round(z_i, 6),
            "z_m": round(z_m, 6),
            "pinch_dist": round(pinch_dist, 3),
            "dx": round(dx, 3),
            "dy": round(dy, 3),
            "delta_pinch": round(delta_pinch, 3),
            "proc_ms": round(proc_ms, 3),
            "render_ms": round(render_ms, 3),
        })

    # overlay
    cv2.putText(frame, f"MODE: {mode}  {action}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, CYAN, 2)
    cv2.putText(frame, f"z_i:{z_i:.3f}  z_m:{z_m:.3f}  margin:{CFG['z_margin']:.3f}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 1)
    cv2.putText(frame, f"pinch:{pinch_dist:.1f}", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 1)

    cv2.imshow("Webcam + Gesture Controller (SHIFT/ROT/ZOOM)", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ============================================================
# 7) CHIUSURA + SUMMARY METRICHE NEL CSV (GENERALI + PER MODE)
# ============================================================
cap.release()
cv2.destroyAllWindows()

# chiudi periodo ON se ancora aperto
metrics_on_exit()

# -------- summary generale (come avevi) --------
on_frames = max(1, metrics["on_frames"])
avg_proc_ms = metrics["sum_proc_ms"] / on_frames
avg_render_ms = metrics["sum_render_ms"] / on_frames
fps_est = 1000.0 / max(1e-6, avg_proc_ms)

# jitter (RMS) globale
rms_jitter = math.sqrt((metrics["sum_dx2"] + metrics["sum_dy2"]) / on_frames)

cmd_rate = metrics["commands"] / max(1e-6, metrics["on_time_s"])  # comandi/sec
mode_switch_rate = metrics["mode_switches"] / max(1e-6, metrics["on_time_s"])  # switch/sec

writer.writerow({
    "t": time.time(),
    "mode": "SUMMARY",
    "action": (
        f"on_frames={metrics['on_frames']}; "
        f"on_time_s={metrics['on_time_s']:.3f}; "
        f"commands={metrics['commands']}; "
        f"cmd_rate_s={cmd_rate:.3f}; "
        f"mode_switches={metrics['mode_switches']}; "
        f"switch_rate_s={mode_switch_rate:.3f}; "
        f"rms_jitter={rms_jitter:.3f}; "
        f"avg_proc_ms={avg_proc_ms:.3f}; "
        f"avg_render_ms={avg_render_ms:.3f}; "
        f"fps_est={fps_est:.1f}; "
        f"sensitivity={SENSITIVITY}"
    ),
    "x_i": "", "y_i": "", "x_m": "", "y_m": "", "x_t": "", "y_t": "",
    "z_i": "", "z_m": "", "pinch_dist": "",
    "dx": "", "dy": "", "delta_pinch": "",
    "proc_ms": "", "render_ms": "",
})

# -------- summary PER MODE: SHIFT / ROTATE / ZOOM --------
for m in ["SHIFT", "ROTATE", "ZOOM"]:
    frames = max(1, per_mode[m]["frames"])
    action_frames = per_mode[m]["action_frames"]

    ratio_action = action_frames / frames  # frame con comando / frame in quella mode
    avg_proc_m = per_mode[m]["sum_proc_ms"] / frames
    avg_render_m = per_mode[m]["sum_render_ms"] / frames
    fps_m = 1000.0 / max(1e-6, avg_proc_m)

    # jitter RMS per modalità
    # SHIFT/ROTATE: sqrt(mean(dx^2 + dy^2))
    # ZOOM: usiamo sqrt(mean(delta_pinch^2)) (solo 1D)
    if m == "ZOOM":
        rms_m = math.sqrt(per_mode[m]["sum_dx2"] / frames)
    else:
        rms_m = math.sqrt((per_mode[m]["sum_dx2"] + per_mode[m]["sum_dy2"]) / frames)

    writer.writerow({
        "t": time.time(),
        "mode": f"SUMMARY_{m}",
        "action": (
            f"mode_frames={per_mode[m]['frames']}; "
            f"action_frames={action_frames}; "
            f"ratio_action={ratio_action:.3f}; "
            f"rms_jitter={rms_m:.3f}; "
            f"avg_proc_ms={avg_proc_m:.3f}; "
            f"avg_render_ms={avg_render_m:.3f}; "
            f"fps_est={fps_m:.1f}; "
            f"sensitivity={SENSITIVITY}"
        ),
        "x_i": "", "y_i": "", "x_m": "", "y_m": "", "x_t": "", "y_t": "",
        "z_i": "", "z_m": "", "pinch_dist": "",
        "dx": "", "dy": "", "delta_pinch": "",
        "proc_ms": "", "render_ms": "",
    })

csv_f.close()
print("CSV salvato:", CFG["csv_path"])
