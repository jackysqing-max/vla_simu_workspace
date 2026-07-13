import cv2
import mediapipe as mp
import numpy as np
import nibabel as nib
import pyvista as pv
from pyvistaqt import BackgroundPlotter

# ============================================================
# MRI LOAD + PyVista BackgroundPlotter (NON BLOCCANTE)
# ============================================================
nifti_path = "t1_150116AR_20150115.nii"   # <-- metti il tuo file qui

img = nib.load(nifti_path)
data = img.get_fdata().astype(np.float32)
if data.ndim == 4:
    data = data[..., 0]

volume = pv.wrap(data)

plotter = BackgroundPlotter(title="MRI 3D – Gesture Controlled")
plotter.add_volume(volume, cmap="gray", opacity="sigmoid", shade=False)
plotter.add_axes()
plotter.show_grid()

# ============================================================
# PAN COME "SCREENSHOT" (usa up/right della camera)
# ============================================================
def move_camera_from_pixels(dx_pix, dy_pix):
    """
    dx_pix, dy_pix = spostamento (in pixel) tra due frame consecutivi.
    Effetto: pan nel piano della vista, come trascinare uno screenshot.
    """
    cam = plotter.camera

    pos = np.array(cam.position)
    focal = np.array(cam.focal_point)
    view_up = np.array(cam.up)

    # direzione di vista (dal punto di vista verso il fuoco)
    view_dir = focal - pos
    view_dir /= (np.linalg.norm(view_dir) + 1e-9)

    # vettore right = view_dir × up
    right = np.cross(view_dir, view_up)
    right /= (np.linalg.norm(right) + 1e-9)

    # up normalizzato
    up = view_up / (np.linalg.norm(view_up) + 1e-9)

    # quanto vale un pixel in "unità mondo"
    SCALE = 1.0  # regola la sensibilità

    offset = right * (dx_pix * SCALE) + up * (-dy_pix * SCALE)

    cam.position = tuple(pos + offset)
    cam.focal_point = tuple(focal + offset)

    plotter.render()

# ============================================================
# PARAMETRI GESTO
# ============================================================
# una sola soglia per ON/OFF, diversa per indice e medio
Z_THRESH_INDEX  = 0.035   # z_diff indice > soglia → dito avanti
Z_THRESH_MIDDLE = 0.040   # z_diff medio > soglia → dito avanti

FRAME_MOVE_THRESH = 1.0   # minimo movimento in pixel per applicare pan

# ============================================================
# MEDIAPIPE
# ============================================================
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# ============================================================
# STATO GESTO (CONTINUO)
# ============================================================
gesture_active = False          # True se gesto valido
prev_index_px = None            # posizione indice precedente
prev_middle_px = None           # posizione medio precedente

# ============================================================
# LOOP WEBCAM
# ============================================================
cap = cv2.VideoCapture(0)
print("Premi 'q' per uscire.")

while True:
    ok, frame = cap.read()
    if not ok:
        break

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    mode_text = "MODE: OFF"
    z_index_disp = 0.0
    z_middle_disp = 0.0

    if result.multi_hand_landmarks:
        lm = result.multi_hand_landmarks[0]
        mp_drawing.draw_landmarks(frame, lm, mp_hands.HAND_CONNECTIONS)

        palm_lm = lm.landmark[0]

        # ---- indice ----
        index_tip_lm = lm.landmark[8]
        index_pip_lm = lm.landmark[6]
        index_tip_px = (int(index_tip_lm.x * w), int(index_tip_lm.y * h))
        z_index = palm_lm.z - index_tip_lm.z
        z_index_disp = z_index
        index_extended = index_tip_lm.y < index_pip_lm.y

        # ---- medio ----
        middle_tip_lm = lm.landmark[12]
        middle_pip_lm = lm.landmark[10]
        middle_tip_px = (int(middle_tip_lm.x * w), int(middle_tip_lm.y * h))
        z_middle = palm_lm.z - middle_tip_lm.z
        z_middle_disp = z_middle
        middle_extended = middle_tip_lm.y < middle_pip_lm.y

        # ---- anulare ----
        ring_tip_lm = lm.landmark[16]
        ring_pip_lm = lm.landmark[14]
        ring_extended = ring_tip_lm.y < ring_pip_lm.y
        ring_closed = not ring_extended

        # ---- mignolo ----
        pinky_tip_lm = lm.landmark[20]
        pinky_pip_lm = lm.landmark[18]
        pinky_extended = pinky_tip_lm.y < pinky_pip_lm.y
        pinky_closed = not pinky_extended

        # ---- pollice ----
        thumb_tip_lm = lm.landmark[4]
        thumb_ip_lm  = lm.landmark[3]
        # semplice euristica: se il tip è più in basso del joint, consideriamo il pollice "chiuso"
        thumb_closed = thumb_tip_lm.y > thumb_ip_lm.y

        # condizioni di profondità
        index_forward  = z_index  > Z_THRESH_INDEX
        middle_forward = z_middle > Z_THRESH_MIDDLE

        # forma corretta della mano:
        # - indice esteso
        # - medio esteso
        # - pollice, anulare, mignolo chiusi
        shape_ok = (
            index_extended and
            middle_extended and
            thumb_closed and
            ring_closed and
            pinky_closed
        )

        # ==========================
        # LOGICA MODALITÀ (ON / OFF)
        # ==========================
        if gesture_active:
            # spegni se la forma NON è più corretta
            # oppure se uno dei due dita non è più "avanti"
            if (not shape_ok) or (not (index_forward and middle_forward)):
                gesture_active = False
                prev_index_px = None
                prev_middle_px = None
        else:
            # accendi solo se:
            #  - forma corretta (indice+medio fuori, altre 3 chiuse)
            #  - indice e medio abbastanza avanti
            if shape_ok and index_forward and middle_forward:
                gesture_active = True
                prev_index_px = index_tip_px
                prev_middle_px = middle_tip_px

        # ==========================
        # SE MODE ON → MOVIMENTO CONTINUO
        # ==========================
        index_color = (0, 0, 255)

        if gesture_active:
            mode_text = "MODE: ON"
            index_color = (0, 255, 0)

            if prev_index_px is not None and prev_middle_px is not None:
                # movimento indice
                dx_i = index_tip_px[0] - prev_index_px[0]
                dy_i = index_tip_px[1] - prev_index_px[1]
                # movimento medio
                dx_m = middle_tip_px[0] - prev_middle_px[0]
                dy_m = middle_tip_px[1] - prev_middle_px[1]

                # movimento medio tra le due dita
                dx = 0.5 * (dx_i + dx_m)
                dy = 0.5 * (dy_i + dy_m)

                if abs(dx) > FRAME_MOVE_THRESH or abs(dy) > FRAME_MOVE_THRESH:
                    move_camera_from_pixels(dx, dy)

            prev_index_px = index_tip_px
            prev_middle_px = middle_tip_px
        else:
            mode_text = "MODE: OFF"
            prev_index_px = None
            prev_middle_px = None

        # disegna cerchi su indice e medio
        cv2.circle(frame, index_tip_px, 10, index_color, -1)
        cv2.circle(frame, middle_tip_px, 10, index_color, -1)

    # ==========================
    # OVERLAY TESTO
    # ==========================
    cv2.putText(frame, mode_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.putText(frame, f"z_idx: {z_index_disp:.3f}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
    cv2.putText(frame, f"z_mid: {z_middle_disp:.3f}", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

    cv2.imshow("Webcam + Gesture Controller (2-finger continuous)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
