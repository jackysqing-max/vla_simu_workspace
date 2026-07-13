import cv2
import math
import socket
import mediapipe as mp

# ==========================
# PARAMETRI
# ==========================
MIN_DIST = 1e-3
PAN_THRESH = 2.0              # pixel minimi per pan
THUMB_CLOSED_DIST = 60        # distanza thumb-base indice per "pollice chiuso" (aumentata)
SWIPE_THRESH = 60.0           # spostamento orizzontale minimo per swipe slice
SWIPE_COOLDOWN = 10           # frame di pausa dopo uno swipe
PINCH_MAX_DIST = 220.0        # max distanza per considerare pinch
PINCH_MIN_DIST = 20.0         # min distanza per considerare pinch
PINCH_ZOOM_STEP = 8.0         # ogni 8 px di cambio distanza = 1 step di zoom
ZOOM_STEP_FACTOR = 1.1        # fattore per step di zoom

# ==========================
# FUNZIONI
# ==========================
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# ==========================
# SOCKET → IMAGEJ
# ==========================
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 50007))

def send(cmd):
    # print(cmd)  # scommenta per debug
    sock.sendall((cmd + "\n").encode())

# ==========================
# MEDIAPIPE
# ==========================
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# ==========================
# STATO
# ==========================
prev_index = None             # per pan
prev_swipe_center_x = None    # per swipe
swipe_cooldown = 0

# stato per zoom a step
base_pinch_dist = None        # distanza "zero" da cui misurare passi
prev_pinch_steps = 0          # numero di passi già applicati

# ==========================
# LOOP WEBCAM
# ==========================
cap = cv2.VideoCapture(0)
print("Premi 'q' per uscire.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    mode_text = "MODE: NONE"

    thumb_tip = None
    index_tip = None
    index_mcp = None
    tips4 = []

    if result.multi_hand_landmarks:
        lm = result.multi_hand_landmarks[0]
        mp_drawing.draw_landmarks(frame, lm, mp_hands.HAND_CONNECTIONS)

        # punti principali
        thumb_tip = (lm.landmark[4].x * w, lm.landmark[4].y * h)
        index_tip = (lm.landmark[8].x * w, lm.landmark[8].y * h)
        index_mcp = (lm.landmark[5].x * w, lm.landmark[5].y * h)

        tips4 = [
            (lm.landmark[8].x * w,  lm.landmark[8].y * h),   # index tip
            (lm.landmark[12].x * w, lm.landmark[12].y * h),  # middle
            (lm.landmark[16].x * w, lm.landmark[16].y * h),  # ring
            (lm.landmark[20].x * w, lm.landmark[20].y * h),  # pinky
        ]

        cv2.circle(frame, (int(thumb_tip[0]), int(thumb_tip[1])), 6, (0,0,255), -1)
        cv2.circle(frame, (int(index_tip[0]), int(index_tip[1])), 6, (0,255,0), -1)

    # ====== STATO MANO ======
    thumb_closed = False
    open_hand_4 = False
    pinch_mode = False

    # pollice chiuso: thumb vicino alla base dell'indice
    if thumb_tip is not None and index_mcp is not None:
        if distance(thumb_tip, index_mcp) < THUMB_CLOSED_DIST:
            thumb_closed = True

    # mano aperta: 4 dita estese
    if result.multi_hand_landmarks:
        lm = result.multi_hand_landmarks[0]
        extended = 0
        finger_ids = [(8, 6), (12, 10), (16, 14), (20, 18)]
        for tip_id, pip_id in finger_ids:
            tip = lm.landmark[tip_id]
            pip = lm.landmark[pip_id]
            if tip.y < pip.y:
                extended += 1
        if extended >= 4:
            open_hand_4 = True

    # pinch: pollice + indice vicini, non pollice chiuso, non mano aperta
    if thumb_tip is not None and index_tip is not None and not thumb_closed and not open_hand_4:
        dist_ti = distance(thumb_tip, index_tip)
        if PINCH_MIN_DIST < dist_ti < PINCH_MAX_DIST:
            pinch_mode = True

    # ====== LOGICA GESTI ======

    # 1) SLICE: mano aperta (4 dita) → swipe orizzontale
    if open_hand_4 and len(tips4) == 4:
        mode_text = "MODE: SLICE (4-finger swipe)"

        center_x = sum(p[0] for p in tips4) / 4.0

        if swipe_cooldown > 0:
            swipe_cooldown -= 1

        if prev_swipe_center_x is not None and swipe_cooldown == 0:
            dx = center_x - prev_swipe_center_x

            if dx > SWIPE_THRESH:
                send("SLICE_D 1")
                swipe_cooldown = SWIPE_COOLDOWN
            elif dx < -SWIPE_THRESH:
                send("SLICE_D -1")
                swipe_cooldown = SWIPE_COOLDOWN

        prev_swipe_center_x = center_x
        prev_index = None
        base_pinch_dist = None
        prev_pinch_steps = 0

    # 2) SHIFT: pollice chiuso → pan con indice
    elif thumb_closed and index_tip is not None:
        mode_text = "MODE: SHIFT (thumb closed + index)"

        if prev_index is not None:
            dx = index_tip[0] - prev_index[0]
            dy = index_tip[1] - prev_index[1]

            if abs(dx) > PAN_THRESH or abs(dy) > PAN_THRESH:
                send(f"SHIFT_D {dx:.3f} {dy:.3f}")

        prev_index = index_tip
        prev_swipe_center_x = None
        base_pinch_dist = None
        prev_pinch_steps = 0

    # 3) ZOOM: pinch → step multipli 1.1x / 1/1.1x
    elif pinch_mode and thumb_tip is not None and index_tip is not None:
        mode_text = "MODE: ZOOM (pinch thumb+index)"

        dist = distance(thumb_tip, index_tip)

        if base_pinch_dist is None:
            # prima volta che entri in pinch: definisci dist di riferimento
            base_pinch_dist = dist
            prev_pinch_steps = 0
        else:
            # quanti "step" da 8px hai fatto rispetto alla distanza base
            steps_float = (dist - base_pinch_dist) / PINCH_ZOOM_STEP
            steps_now = int(round(steps_float))

            delta_steps = steps_now - prev_pinch_steps

            if delta_steps > 0:
                # apertura -> zoom in
                for _ in range(delta_steps):
                    send(f"ZOOM_D {ZOOM_STEP_FACTOR:.3f}")
                prev_pinch_steps = steps_now

            elif delta_steps < 0:
                # chiusura -> zoom out
                inv = 1.0 / ZOOM_STEP_FACTOR
                for _ in range(-delta_steps):
                    send(f"ZOOM_D {inv:.3f}")
                prev_pinch_steps = steps_now

        prev_index = None
        prev_swipe_center_x = None

        cv2.line(frame,
                 (int(thumb_tip[0]), int(thumb_tip[1])),
                 (int(index_tip[0]), int(index_tip[1])),
                 (255, 0, 0), 2)

    # nessuna modalità
    else:
        mode_text = "MODE: NONE"
        prev_index = None
        prev_swipe_center_x = None
        base_pinch_dist = None
        prev_pinch_steps = 0
        if swipe_cooldown > 0:
            swipe_cooldown -= 1

    # ====== DEBUG VISIVO ======
    cv2.putText(frame, mode_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow("Touchless Gestures → ImageJ (SHIFT/SLICE/ZOOM)", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
sock.close()
