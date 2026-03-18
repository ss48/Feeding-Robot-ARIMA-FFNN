"""
Perception Demo — Standalone sensor-fusion test for the feeding robot.

Visualises a plate with coloured fruits and an animated mouth that opens
and closes.  Simulates noisy camera and sonar sensors, then fuses them
with a linear Kalman filter.  Three-panel display shows raw camera, raw
sonar, and fused estimates with live RMSE / confidence metrics.

Controls:
    q  — quit
    o  — toggle outlier storm (forces sonar outliers)
    s  — toggle sonar failure  (disables sonar input)

Dependencies: numpy, opencv-python  (no ROS required)

NOTE — for production mouth-corner detection consider MediaPipe Face Mesh
(landmarks 61 / 291 for corners, 13 / 14 for open/close state) or dlib
(landmarks 48 / 54).
"""

import math
import time

import cv2
import numpy as np

# ───────────────────────────────────────────────────────────────────────
# Fruit definitions
# ───────────────────────────────────────────────────────────────────────
FRUITS = {
    'apple':      {'color_bgr': (30,  30, 220), 'radius': 22},
    'banana':     {'color_bgr': (0,  230, 255), 'radius': 18},
    'grape':      {'color_bgr': (180, 50, 130), 'radius': 10},
    'strawberry': {'color_bgr': (50,  30, 180), 'radius': 14},
    'orange':     {'color_bgr': (0,  140, 255), 'radius': 20},
    'kiwi':       {'color_bgr': (30, 140,  50), 'radius': 16},
}

# Area-to-distance constant (same as fusion_node.py)
AREA_TO_DIST_K = 5000.0


# ===================================================================
# Fruit Plate Renderer
# ===================================================================
class FruitPlateRenderer:
    """Draws a plate with coloured fruits arranged in a circle."""

    def __init__(self, plate_center, plate_radius):
        self.cx, self.cy = plate_center
        self.pr = plate_radius
        # Pre-compute ground-truth positions evenly around the plate
        n = len(FRUITS)
        self.fruit_gt = {}
        for i, (name, props) in enumerate(FRUITS.items()):
            angle = 2.0 * math.pi * i / n - math.pi / 2
            fx = int(self.cx + 0.55 * self.pr * math.cos(angle))
            fy = int(self.cy + 0.55 * self.pr * math.sin(angle))
            area = math.pi * props['radius'] ** 2
            self.fruit_gt[name] = {
                'pos': (fx, fy),
                'area': area,
                'radius': props['radius'],
                'color': props['color_bgr'],
            }

    def render(self, frame, highlight_name=None):
        """Draw the plate and all fruits.  Returns ground-truth dict."""
        # Plate (grey ellipse with rim)
        cv2.ellipse(frame, (self.cx, self.cy), (self.pr, int(self.pr * 0.6)),
                     0, 0, 360, (200, 200, 200), -1)
        cv2.ellipse(frame, (self.cx, self.cy), (self.pr, int(self.pr * 0.6)),
                     0, 0, 360, (160, 160, 160), 2)

        for name, gt in self.fruit_gt.items():
            thickness = -1
            cv2.circle(frame, gt['pos'], gt['radius'], gt['color'], thickness)
            # Label
            tx, ty = gt['pos']
            cv2.putText(frame, name, (tx - 20, ty + gt['radius'] + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1,
                        cv2.LINE_AA)
        return self.fruit_gt


# ===================================================================
# Mouth Animator
# ===================================================================
class MouthAnimator:
    """Draws a simple face with a mouth that cycles open / closed."""

    def __init__(self, face_center, face_radius, period=3.0):
        self.cx, self.cy = face_center
        self.fr = face_radius
        self.period = period
        self.mouth_w = int(face_radius * 0.5)
        self.mouth_h = int(face_radius * 0.25)

    def openness(self, t):
        """Returns mouth openness in [0, 1]."""
        return 0.5 + 0.5 * math.sin(2.0 * math.pi * t / self.period)

    def get_mouth_corners(self, t):
        """Ground-truth mouth corner positions (left, right)."""
        omy = self.cy + int(self.fr * 0.35)
        left  = (self.cx - self.mouth_w, omy)
        right = (self.cx + self.mouth_w, omy)
        return left, right

    def render(self, frame, t):
        """Draw face and animated mouth.  Returns openness."""
        op = self.openness(t)

        # Face outline
        cv2.circle(frame, (self.cx, self.cy), self.fr, (180, 200, 220), 2)

        # Eyes
        eye_y = self.cy - int(self.fr * 0.2)
        eye_dx = int(self.fr * 0.3)
        cv2.circle(frame, (self.cx - eye_dx, eye_y), 5, (80, 60, 40), -1)
        cv2.circle(frame, (self.cx + eye_dx, eye_y), 5, (80, 60, 40), -1)

        # Nose
        nose_y = self.cy + int(self.fr * 0.05)
        cv2.circle(frame, (self.cx, nose_y), 3, (150, 130, 120), -1)

        # Mouth — ellipse arc whose vertical extent depends on openness
        mouth_cy = self.cy + int(self.fr * 0.35)
        half_open = max(1, int(self.mouth_h * op))
        axes = (self.mouth_w, half_open)

        if op < 0.08:
            # Closed — draw a line
            left, right = self.get_mouth_corners(t)
            cv2.line(frame, left, right, (60, 60, 180), 2)
        else:
            # Open — draw filled ellipse arc
            cv2.ellipse(frame, (self.cx, mouth_cy), axes, 0, 0, 360,
                        (60, 60, 180), -1)
            # Lip outline
            cv2.ellipse(frame, (self.cx, mouth_cy), axes, 0, 0, 360,
                        (40, 40, 140), 2)

        # Corner markers (ground truth)
        left, right = self.get_mouth_corners(t)
        cv2.circle(frame, left, 3, (0, 255, 0), -1)
        cv2.circle(frame, right, 3, (0, 255, 0), -1)

        # Openness text
        state_str = "OPEN" if op > 0.5 else "CLOSED" if op < 0.15 else "PARTIAL"
        cv2.putText(frame, f"Mouth: {state_str} ({op:.0%})",
                    (self.cx - 70, self.cy + self.fr + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1,
                    cv2.LINE_AA)
        return op


# ===================================================================
# Sensor Simulator
# ===================================================================
class SensorSimulator:
    """Generates noisy camera and sonar measurements."""

    def __init__(self, cam_pos_sigma=5.0, cam_area_sigma=50.0,
                 sonar_sigma=2.0, mouth_sigma=3.0):
        self.cam_pos_sigma = cam_pos_sigma
        self.cam_area_sigma = cam_area_sigma
        self.sonar_sigma = sonar_sigma
        self.mouth_sigma = mouth_sigma
        self.rng = np.random.default_rng(42)

    def simulate_camera(self, fruit_gt):
        """Return dict {name: (noisy_x, noisy_y, noisy_area)}."""
        readings = {}
        for name, gt in fruit_gt.items():
            nx = gt['pos'][0] + self.rng.normal(0, self.cam_pos_sigma)
            ny = gt['pos'][1] + self.rng.normal(0, self.cam_pos_sigma)
            na = max(10.0, gt['area'] + self.rng.normal(0, self.cam_area_sigma))
            readings[name] = (nx, ny, na)
        return readings

    def simulate_sonar(self, plate_dist_true, mouth_dist_true,
                       outlier_mode=False):
        """Return (plate_dist_noisy, mouth_dist_noisy)."""
        sigma = self.sonar_sigma
        if outlier_mode:
            sigma *= 3.0
        # 5 % random outlier chance (or 100 % in outlier mode)
        if outlier_mode or self.rng.random() < 0.05:
            plate = plate_dist_true + self.rng.normal(0, sigma * 3)
            mouth = mouth_dist_true + self.rng.normal(0, sigma * 3)
        else:
            plate = plate_dist_true + self.rng.normal(0, sigma)
            mouth = mouth_dist_true + self.rng.normal(0, sigma)
        return max(1.0, plate), max(1.0, mouth)

    def simulate_mouth_corners(self, left_true, right_true):
        """Return noisy (left, right) corner positions."""
        lx = left_true[0] + self.rng.normal(0, self.mouth_sigma)
        ly = left_true[1] + self.rng.normal(0, self.mouth_sigma)
        rx = right_true[0] + self.rng.normal(0, self.mouth_sigma)
        ry = right_true[1] + self.rng.normal(0, self.mouth_sigma)
        return (lx, ly), (rx, ry)


# ===================================================================
# Kalman Filter (linear, 6-state)
# ===================================================================
class KalmanFilterDemo:
    """
    6-state linear Kalman filter fusing camera and sonar.

    State: [food_x, food_y, plate_distance, mouth_distance,
            mouth_corner_L_x, mouth_corner_R_x]

    Camera provides:  food_x, food_y, plate_distance (via area)
    Sonar  provides:  plate_distance, mouth_distance
    Vision provides:  mouth_corner_L_x, mouth_corner_R_x
    """

    N = 6  # state dimension

    def __init__(self):
        self.x = np.zeros(self.N)
        self.P = np.diag([100.0, 100.0, 50.0, 50.0, 100.0, 100.0])
        self.Q = np.diag([0.5, 0.5, 0.2, 0.2, 0.5, 0.5])
        self.R_base = np.diag([25.0, 25.0, 4.0, 4.0, 9.0, 9.0])
        self._initialised = False

    def predict(self, dt):
        F = np.eye(self.N)
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, z_cam, z_sonar, z_mouth, sonar_ok=True):
        """
        z_cam   = (food_x, food_y, plate_dist_from_area)  or None
        z_sonar = (plate_dist, mouth_dist)                 or None
        z_mouth = (corner_L_x, corner_R_x)                or None
        """
        rows_H = []
        rows_z = []
        rows_R = []

        # Camera measurements → state indices 0, 1, 2
        if z_cam is not None:
            for local_i, state_i in enumerate([0, 1, 2]):
                h = np.zeros(self.N)
                h[state_i] = 1.0
                rows_H.append(h)
                rows_z.append(z_cam[local_i])
                rows_R.append(self.R_base[state_i, state_i])

        # Sonar measurements → state indices 2, 3
        if z_sonar is not None and sonar_ok:
            for local_i, state_i in enumerate([2, 3]):
                h = np.zeros(self.N)
                h[state_i] = 1.0
                rows_H.append(h)
                rows_z.append(z_sonar[local_i])
                rows_R.append(self.R_base[state_i, state_i])

        # Mouth corner measurements → state indices 4, 5
        if z_mouth is not None:
            for local_i, state_i in enumerate([4, 5]):
                h = np.zeros(self.N)
                h[state_i] = 1.0
                rows_H.append(h)
                rows_z.append(z_mouth[local_i])
                rows_R.append(self.R_base[state_i, state_i])

        if not rows_H:
            return

        H = np.array(rows_H)
        z = np.array(rows_z)
        R = np.diag(rows_R)

        # First measurement → snap state
        if not self._initialised:
            for i, row in enumerate(H):
                idx = np.argmax(row)
                self.x[idx] = z[i]
            self._initialised = True
            return

        # Innovation
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        self.P = (np.eye(self.N) - K @ H) @ self.P


# ===================================================================
# Metrics Tracker
# ===================================================================
class MetricsTracker:
    """Rolling RMSE and confidence from Kalman filter state."""

    def __init__(self, window=100):
        self.window = window
        self.errors = {i: [] for i in range(6)}

    def record(self, state_idx, truth, estimate):
        buf = self.errors[state_idx]
        buf.append((truth - estimate) ** 2)
        if len(buf) > self.window:
            buf.pop(0)

    def rmse(self, state_idx):
        buf = self.errors[state_idx]
        if not buf:
            return 0.0
        return math.sqrt(sum(buf) / len(buf))

    @staticmethod
    def confidence_from_P(P, idx):
        return 1.0 / (1.0 + math.sqrt(max(0.0, P[idx, idx])))


# ===================================================================
# Drawing helpers
# ===================================================================
def draw_panel_title(frame, x, y, title):
    cv2.putText(frame, title, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)


def draw_bar(frame, x, y, w, h, value, max_val, color, label):
    """Draw a horizontal bar chart element."""
    bar_w = int(w * min(value / max_val, 1.0))
    cv2.rectangle(frame, (x, y), (x + bar_w, y + h), color, -1)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (150, 150, 150), 1)
    cv2.putText(frame, f"{label}: {value:.1f} cm", (x + 5, y + h - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1,
                cv2.LINE_AA)


def draw_dashed_circle(frame, center, radius, color, dash_len=6):
    """Draw a dashed circle approximation."""
    cx, cy = int(center[0]), int(center[1])
    n_pts = max(12, int(2 * math.pi * radius / dash_len))
    for i in range(0, n_pts, 2):
        a1 = 2 * math.pi * i / n_pts
        a2 = 2 * math.pi * (i + 1) / n_pts
        p1 = (int(cx + radius * math.cos(a1)), int(cy + radius * math.sin(a1)))
        p2 = (int(cx + radius * math.cos(a2)), int(cy + radius * math.sin(a2)))
        cv2.line(frame, p1, p2, color, 1, cv2.LINE_AA)


# ===================================================================
# Main demo loop
# ===================================================================
def main():
    # Layout constants
    W, H = 1200, 600
    PANEL_W = W // 3

    # Ground-truth distances (cm)
    TRUE_PLATE_DIST = 25.0
    TRUE_MOUTH_DIST = 35.0

    # Create scene objects
    plate = FruitPlateRenderer(
        plate_center=(PANEL_W // 2, H // 2 + 30),
        plate_radius=110,
    )
    mouth = MouthAnimator(
        face_center=(PANEL_W // 2, 120),
        face_radius=60,
        period=3.0,
    )
    sensors = SensorSimulator()
    kf = KalmanFilterDemo()
    metrics = MetricsTracker()

    outlier_mode = False
    sonar_ok = True
    t = 0.0
    dt = 1.0 / 30.0
    prev_time = time.monotonic()

    print("Perception Demo started.")
    print("  q = quit | o = toggle outlier storm | s = toggle sonar failure")

    while True:
        now = time.monotonic()
        dt = now - prev_time
        prev_time = now
        t += dt

        canvas = np.zeros((H, W, 3), dtype=np.uint8)

        # ── Panel dividers ──
        cv2.line(canvas, (PANEL_W, 0), (PANEL_W, H), (60, 60, 60), 1)
        cv2.line(canvas, (2 * PANEL_W, 0), (2 * PANEL_W, H), (60, 60, 60), 1)

        # ==============================================================
        # LEFT PANEL — Raw Camera View
        # ==============================================================
        left = canvas[:, :PANEL_W]
        draw_panel_title(left, 10, 25, "Raw Camera")

        # Draw face + mouth
        openness_val = mouth.render(left, t)
        left_corner_gt, right_corner_gt = mouth.get_mouth_corners(t)

        # Draw plate + fruits (ground truth)
        fruit_gt = plate.render(left)

        # Simulate noisy camera detections and overlay
        cam_readings = sensors.simulate_camera(fruit_gt)
        for name, (nx, ny, na) in cam_readings.items():
            r = FRUITS[name]['radius']
            draw_dashed_circle(left, (nx, ny), r + 3,
                               (100, 255, 100))

        # Simulate noisy mouth corners
        mcl_noisy, mcr_noisy = sensors.simulate_mouth_corners(
            left_corner_gt, right_corner_gt)
        cv2.circle(left, (int(mcl_noisy[0]), int(mcl_noisy[1])), 4,
                   (0, 200, 255), -1)
        cv2.circle(left, (int(mcr_noisy[0]), int(mcr_noisy[1])), 4,
                   (0, 200, 255), -1)

        # ==============================================================
        # CENTER PANEL — Raw Sonar
        # ==============================================================
        mid = canvas[:, PANEL_W:2 * PANEL_W]
        draw_panel_title(mid, 10, 25, "Raw Sonar")

        sonar_plate, sonar_mouth = sensors.simulate_sonar(
            TRUE_PLATE_DIST, TRUE_MOUTH_DIST, outlier_mode=outlier_mode)

        bar_x, bar_w = 30, PANEL_W - 60
        draw_bar(mid, bar_x, 60, bar_w, 35, sonar_plate, 60.0,
                 (0, 180, 255), "Plate dist")
        draw_bar(mid, bar_x, 110, bar_w, 35, sonar_mouth, 60.0,
                 (255, 140, 0), "Mouth dist")

        # True distance reference lines
        cv2.putText(mid, f"True plate: {TRUE_PLATE_DIST:.0f} cm",
                    (bar_x, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (150, 150, 150), 1, cv2.LINE_AA)
        cv2.putText(mid, f"True mouth: {TRUE_MOUTH_DIST:.0f} cm",
                    (bar_x, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (150, 150, 150), 1, cv2.LINE_AA)

        # Mode indicators
        if outlier_mode:
            cv2.putText(mid, "OUTLIER STORM ON", (bar_x, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2,
                        cv2.LINE_AA)
        if not sonar_ok:
            cv2.putText(mid, "SONAR DISABLED", (bar_x, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2,
                        cv2.LINE_AA)

        # Sonar distance history visualisation
        sonar_cy = H // 2 + 40
        cv2.putText(mid, "Sonar Signal Visualisation", (bar_x, sonar_cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1,
                    cv2.LINE_AA)
        # Simple radial sonar graphic
        sc = (PANEL_W // 2, sonar_cy + 80)
        for r_ring in [40, 80, 120]:
            cv2.circle(mid, sc, r_ring, (40, 60, 40), 1)
        # Plate blip
        blip_r = int(40 + 80 * sonar_plate / 60.0)
        cv2.circle(mid, (sc[0] - 20, sc[1] - blip_r // 2), 5,
                   (0, 255, 0), -1)
        cv2.putText(mid, "plate", (sc[0] - 40, sc[1] - blip_r // 2 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1,
                    cv2.LINE_AA)
        # Mouth blip
        blip_m = int(40 + 80 * sonar_mouth / 60.0)
        cv2.circle(mid, (sc[0] + 20, sc[1] - blip_m // 2), 5,
                   (255, 140, 0), -1)
        cv2.putText(mid, "mouth", (sc[0] + 5, sc[1] - blip_m // 2 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 140, 0), 1,
                    cv2.LINE_AA)

        # ==============================================================
        # KALMAN FILTER UPDATE
        # ==============================================================
        # Pick the first fruit (apple) as the primary tracked food
        apple_gt = fruit_gt['apple']
        apple_cam = cam_readings['apple']

        cam_plate_dist = AREA_TO_DIST_K / math.sqrt(max(10.0, apple_cam[2]))

        z_cam = (apple_cam[0], apple_cam[1], cam_plate_dist)
        z_sonar = (sonar_plate, sonar_mouth)
        z_mouth = (mcl_noisy[0], mcr_noisy[0])

        kf.predict(dt)
        kf.update(z_cam, z_sonar, z_mouth, sonar_ok=sonar_ok)

        # Record metrics
        metrics.record(0, apple_gt['pos'][0], kf.x[0])
        metrics.record(1, apple_gt['pos'][1], kf.x[1])
        metrics.record(2, TRUE_PLATE_DIST, kf.x[2])
        metrics.record(3, TRUE_MOUTH_DIST, kf.x[3])
        metrics.record(4, left_corner_gt[0], kf.x[4])
        metrics.record(5, right_corner_gt[0], kf.x[5])

        # ==============================================================
        # RIGHT PANEL — Fused Estimate
        # ==============================================================
        right_panel = canvas[:, 2 * PANEL_W:]
        draw_panel_title(right_panel, 10, 25, "Fused Estimate (Kalman)")

        # Draw face + mouth (same as left)
        mouth.render(right_panel, t)

        # Draw fused mouth corners
        fused_lcx = int(kf.x[4])
        fused_rcx = int(kf.x[5])
        mouth_y = mouth.cy + int(mouth.fr * 0.35)
        cv2.circle(right_panel, (fused_lcx, mouth_y), 5, (255, 0, 255), -1)
        cv2.circle(right_panel, (fused_rcx, mouth_y), 5, (255, 0, 255), -1)
        cv2.putText(right_panel, "fused", (fused_lcx - 15, mouth_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 255), 1,
                    cv2.LINE_AA)

        # Draw plate + fruits at fused positions
        plate.render(right_panel)

        # Overlay fused primary food position (apple)
        fused_fx, fused_fy = int(kf.x[0]), int(kf.x[1])
        cv2.circle(right_panel, (fused_fx, fused_fy), 8, (255, 0, 255), 2)
        cv2.putText(right_panel, "fused food",
                    (fused_fx + 10, fused_fy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1,
                    cv2.LINE_AA)

        # Metrics overlay
        my = H - 180
        cv2.putText(right_panel, "--- RMSE ---", (10, my),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1,
                    cv2.LINE_AA)
        labels = ["Food X", "Food Y", "Plate Dist", "Mouth Dist",
                  "Mouth L", "Mouth R"]
        for i, lbl in enumerate(labels):
            rmse = metrics.rmse(i)
            conf = metrics.confidence_from_P(kf.P, i)
            color = (0, 255, 0) if rmse < 5.0 else (0, 200, 255) if rmse < 15.0 else (0, 0, 255)
            cv2.putText(right_panel,
                        f"{lbl}: RMSE={rmse:.1f}  conf={conf:.0%}",
                        (10, my + 20 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1,
                        cv2.LINE_AA)

        # Fused distance readout
        cv2.putText(right_panel,
                    f"Fused plate dist: {kf.x[2]:.1f} cm  "
                    f"(true: {TRUE_PLATE_DIST:.0f})",
                    (10, my + 20 + 6 * 18 + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 220), 1,
                    cv2.LINE_AA)
        cv2.putText(right_panel,
                    f"Fused mouth dist: {kf.x[3]:.1f} cm  "
                    f"(true: {TRUE_MOUTH_DIST:.0f})",
                    (10, my + 20 + 7 * 18 + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 220), 1,
                    cv2.LINE_AA)

        # ==============================================================
        # Show and handle keys
        # ==============================================================
        cv2.imshow("Feeding Robot — Perception Demo", canvas)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('o'):
            outlier_mode = not outlier_mode
            print(f"Outlier storm: {'ON' if outlier_mode else 'OFF'}")
        elif key == ord('s'):
            sonar_ok = not sonar_ok
            print(f"Sonar: {'ENABLED' if sonar_ok else 'DISABLED'}")

    cv2.destroyAllWindows()
    print("Demo finished.")


if __name__ == '__main__':
    main()
