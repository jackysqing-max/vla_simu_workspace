import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# =========================
# CONFIG
# =========================
XLSX_PATH = "resume.xlsx"
SHEET_NAME = 3
INDEX_COL = 0

METRICS = ["ratio_action", "fps_est", "rms_jitter", "avg_proc_ms", "avg_render_ms"]


# Range per normalizzare in [0,1]
NORM_RANGES = {
    #"switch_rate_s": (0.0, 5.0),
    "ratio_action":  (0, 1),
    "rms_jitter":    (0, 0.035),
    "avg_proc_ms":   (0, 60.0),
    "fps_est":       (0, 30.0),
    "avg_render_ms": (0, 30.0),
}

# Banda di "fluidità" (in unità originali)
FLUID_BAND = {
    #"switch_rate_s": (2, 5),
    "ratio_action":  (0.8, 1.00),
    "rms_jitter":    (0.005, 0.03),
    "avg_proc_ms":   (10, 50),
    "fps_est":       (20.0, 1e9),
    "avg_render_ms": (10, 25),
}

# Etichette assi
LABELS = [
    #"Switching rate", 
    "CMD-Gen ratio",    # switch_rate_s
    "FPS",              # fps_est
    "RMS jitter",      # rms_jitter
    "Proc latency",         # avg_proc_ms
    "Render latency",       # avg_render_ms
]

# Palette IEEE-friendly (Okabe–Ito) -> un colore per metrica
METRIC_COLORS = [
    "#000000",  # blue
    "#000000",  # vermillion
    "#000000",  # bluish green
    "#000000",  # reddish purple
    "#000000",  # orange
    "#000000",  # sky blue
]

# Fluid ring color (rosso chiaro)
FLUID_RING_COLOR = "#3FD164"   # light red

# =========================
# HELPERS
# =========================
def clamp01(x):
    return np.clip(x, 0.0, 1.0)

def normalize(values, metric):
    lo, hi = NORM_RANGES[metric]
    v = (values - lo) / (hi - lo + 1e-99)
    v = clamp01(v)
    # inverti se lower-is-better così "più alto = meglio" per tutte
    #if metric in LOWER_IS_BETTER:
    #    v = 1.0 - v
    return v

def normalize_single(value, metric):
    return float(normalize(np.array([value], dtype=float), metric)[0])

def close(vals):
    return np.r_[vals, vals[0]]

# =========================
# LOAD DATA
# =========================
df = pd.read_excel(XLSX_PATH, sheet_name=SHEET_NAME, index_col=INDEX_COL)
df.columns = df.columns.astype(str).str.strip()

df_tests = df[~df.index.astype(str).str.lower().isin(["mean", "std"])].copy()

# conversione virgole decimali (se Excel IT)
for c in METRICS:
    if df_tests[c].dtype == object:
        df_tests[c] = df_tests[c].astype(str).str.replace(",", ".", regex=False).astype(float)

mean_vals = df_tests[METRICS].mean()
std_vals  = df_tests[METRICS].std(ddof=1)

# mean / std normalizzati
mean_norm = np.array([normalize_single(mean_vals[m], m) for m in METRICS], dtype=float)
std_low_norm  = np.array([normalize_single(mean_vals[m] - std_vals[m], m) for m in METRICS], dtype=float)
std_high_norm = np.array([normalize_single(mean_vals[m] + std_vals[m], m) for m in METRICS], dtype=float)

# Fluid band normalizzato
fluid_low = np.array([normalize_single(FLUID_BAND[m][0], m) for m in METRICS], dtype=float)
fluid_high = np.array([normalize_single(FLUID_BAND[m][1], m) for m in METRICS], dtype=float)

# =========================
# RADAR PLOT
# =========================
N = len(METRICS)
angles = np.linspace(0, 2*np.pi, N, endpoint=False)
angles_closed = np.r_[angles, angles[0]]

fig = plt.figure(figsize=(9, 9))
ax = plt.subplot(111, polar=True)
ax.set_theta_offset(np.pi / 2)
ax.set_theta_direction(-1)

ax.set_xticks(angles)
ax.set_xticklabels(LABELS, fontsize=2)
for label in ax.get_xticklabels():
    label.set_y(-0.12)
    label.set_fontsize(11)

ax.set_ylim(0, 1)
ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
ax.set_yticklabels(["0.2","0.4","0.6","0.8","1.0"], fontsize=9)
ax.grid(True, alpha=0.35)

# --- Colori (puoi cambiarli a piacere) ---
MEAN_LINE_COLOR = "black"
MEAN_RING_COLOR = "#D6EA2A"      # ring mean±std (nero)
MEAN_RING_ALPHA = 0.18
FLUID_RING_COLOR = "#3FD164"
FLUID_RING_ALPHA = 0.18

# === Mean polygon (linea) ===
#ax.plot(angles_closed, close(mean_norm), linewidth=1.2, color=MEAN_LINE_COLOR, label="Mean")
#ax.scatter(angles, mean_norm, s=10, color=MEAN_LINE_COLOR, zorder=6)


# === Mean ± std come "anello" ===
# anello esterno (mean+std)
ax.fill(angles_closed, close(std_high_norm), color=MEAN_RING_COLOR, alpha=MEAN_RING_ALPHA, label=r"Mean $\pm$ std")
# "buco" interno (mean-std)
ax.fill(angles_closed, close(std_low_norm), color="#FFFFFF", alpha=1.0)

# === Fluid band come "anello" ===
ax.fill(angles_closed, close(fluid_high), color=FLUID_RING_COLOR, alpha=FLUID_RING_ALPHA, label="Fluid band")
ax.fill(angles_closed, close(fluid_low),  color="#FFFFFF", alpha=1)

ax.set_title("ZOOM Performance", fontsize=16, pad=18)
ax.legend(loc="upper right", bbox_to_anchor=(1.28, 1.12))
plt.tight_layout()
plt.show()
