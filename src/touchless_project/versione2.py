import cv2
import mediapipe as mp
import time
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
    dx_pix, dy_pix = spostamento del dito tra due frame consecutivi (in pixel).
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
    SCALE = 1  # regola la sensibilità (prova 0.05–0.3)

    offset = right * (dx_pix * SCALE) + up * (-dy_pix * SCALE)

    cam.position = tuple(pos + offset)
    cam.focal_point = tuple(focal + offset)

    plotter.render()

# ============================================================
# PARAMETRI GESTO
# ============================================================
START_FORWARD_Z = 0.085     # indice avanti per INIZIARE il gesto (mode ON)
END_FORWARD_Z   = 0.080     # indice torna indietro per FINIRE il gesto (mode OFF)

FRAME_MOVE_THRESH = 1.0     # spostamento minimo in pixel per considerare un movimento


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
gesture_active = False       # true se z_diff > soglia
prev_index = None            # posizione del frame precedente quando gesture_active = True

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
    index_color = (0, 0, 255)   # rosso quando OFF
    z_diff_disp = 0.0

    if result.multi_hand_landmarks:
        lm = result.multi_hand_landmarks[0]
        mp_drawing.draw_landmarks(frame, lm, mp_hands.HAND_CONNECTIONS)

        index_tip_lm = lm.landmark[8]
        index_pip_lm = lm.landmark[6]
        palm_lm      = lm.landmark[0]

        index_tip_px = (int(index_tip_lm.x * w), int(index_tip_lm.y * h))

        # indice esteso?
        index_extended = index_tip_lm.y < index_pip_lm.y

        # altre dita chiuse (medio, anulare, mignolo)
        finger_ids = [(12, 10), (16, 14), (20, 18)]
        other_closed = all(
            lm.landmark[tip].y > lm.landmark[pip].y
            for (tip, pip) in finger_ids
        )

        # profondità (palmo - indice)
        z_diff = palm_lm.z - index_tip_lm.z
        z_diff_disp = z_diff

        # ==========================
        # LOGICA MODALITÀ (ON / OFF)
        # ==========================
        if gesture_active:
            # controlla se dobbiamo spegnere la modalità
            #if (not index_extended) or (not other_closed) or (z_diff < END_FORWARD_Z):
            if (z_diff < END_FORWARD_Z):
                gesture_active = False
                prev_index = None
        else:
            # controlla se dobbiamo accendere la modalità
            #if index_extended and other_closed and z_diff > START_FORWARD_Z:
            if z_diff > START_FORWARD_Z:
                gesture_active = True
                prev_index = index_tip_px  # inizializza

        # ==========================
        # SE MODE ON → TRADUCI OGNI SPOSTAMENTO IN COMANDI
        # ==========================
        if gesture_active:
            mode_text = "MODE: ON"
            index_color = (0, 255, 0)  # verde

            if prev_index is not None:
                dx = index_tip_px[0] - prev_index[0]
                dy = index_tip_px[1] - prev_index[1]

                # se lo spostamento è significativo → muovi camera
                if abs(dx) > FRAME_MOVE_THRESH or abs(dy) > FRAME_MOVE_THRESH:
                    move_camera_from_pixels(dx, dy)

            prev_index = index_tip_px
        else:
            mode_text = "MODE: OFF"
            index_color = (0, 0, 255)  # rosso
            prev_index = None

        # disegna indice
        cv2.circle(frame, index_tip_px, 10, index_color, -1)

    # ==========================
    # OVERLAY TESTO
    # ==========================
    cv2.putText(frame, mode_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.putText(frame, f"z_diff: {z_diff_disp:.3f}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

    cv2.imshow("Webcam + Gesture Controller (continuous)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
