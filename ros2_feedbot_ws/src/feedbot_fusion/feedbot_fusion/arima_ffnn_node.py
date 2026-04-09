"""
ARIMA-FFNN Hybrid Predictive Controller Node

Implements the combined predictive model from the paper:
  Yt = Tt + St + ARIMA(Rt) + FFNN(Rt)

Where:
  - Time-series decomposition separates Trend (Tt), Seasonality (St), Residual (Rt)
  - ARIMA models linear patterns in residuals
  - FFNN captures nonlinear dependencies in residuals

Subscribes to sensor time-series data and publishes predicted next states
for feeding timing, force, and positioning.

Reference: Algorithm 1 & Equations 1-4 in the paper.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

import numpy as np
from collections import deque


# ---------------------------------------------------------------------------
# Time-series decomposition (moving-average based)
# ---------------------------------------------------------------------------
def decompose_series(series, period=10):
    """Decompose a 1-D series into trend, seasonality, and residual.

    Uses a centred moving average for the trend and averages over full
    periods for the seasonal component (additive model).

    Args:
        series: 1-D numpy array of observations.
        period: Expected cycle length (default 10 – ~1 s at 10 Hz).

    Returns:
        (trend, seasonal, residual) arrays of the same length as *series*.
        Leading/trailing values that cannot be computed are filled with the
        nearest valid value so the arrays always match the input length.
    """
    n = len(series)
    if n < period:
        zeros = np.zeros(n)
        return series.copy(), zeros, zeros

    # Trend via centred moving average
    half = period // 2
    trend = np.full(n, np.nan)
    for i in range(half, n - half):
        trend[i] = np.mean(series[i - half:i + half + 1])
    # Fill edges
    first_valid = trend[~np.isnan(trend)][0] if np.any(~np.isnan(trend)) else 0.0
    last_valid = trend[~np.isnan(trend)][-1] if np.any(~np.isnan(trend)) else 0.0
    for i in range(n):
        if np.isnan(trend[i]):
            trend[i] = first_valid if i < half else last_valid

    # Detrended
    detrended = series - trend

    # Seasonal component – average over each position within the period
    seasonal = np.zeros(n)
    for i in range(period):
        indices = list(range(i, n, period))
        seasonal[indices] = np.mean(detrended[indices])

    residual = series - trend - seasonal
    return trend, seasonal, residual


# ---------------------------------------------------------------------------
# Lightweight ARIMA(p, d, q) estimator
# ---------------------------------------------------------------------------
class SimpleARIMA:
    """Minimal ARIMA implementation for real-time residual forecasting.

    Uses ordinary-least-squares to fit AR coefficients on differenced data
    and keeps a small MA correction buffer.  This avoids heavy dependencies
    (statsmodels) while matching the paper's formulation (Eq. 2).

    Parameters:
        p: AR order (number of lagged residual terms).
        d: Differencing order (typically 0 or 1).
        q: MA order (number of past forecast-error terms).
    """

    def __init__(self, p=3, d=1, q=1):
        self.p = p
        self.d = d
        self.q = q
        self.phi = np.zeros(p)       # AR coefficients
        self.theta = np.zeros(q)     # MA coefficients
        self.errors = deque(maxlen=max(q, 1))
        self._fitted = False

    def _difference(self, series):
        """Apply differencing *d* times."""
        s = series.copy()
        for _ in range(self.d):
            s = np.diff(s)
        return s

    def fit(self, residuals):
        """Estimate AR coefficients via OLS on the (differenced) residuals."""
        data = self._difference(residuals)
        n = len(data)
        if n <= self.p:
            return

        # Build design matrix for AR(p)
        X = np.zeros((n - self.p, self.p))
        y = data[self.p:]
        for lag in range(self.p):
            X[:, lag] = data[self.p - lag - 1: n - lag - 1]

        # OLS: phi = (X^T X)^{-1} X^T y
        try:
            self.phi = np.linalg.lstsq(X, y, rcond=None)[0]
        except np.linalg.LinAlgError:
            self.phi = np.zeros(self.p)

        # Bootstrap MA errors from in-sample residuals
        y_hat = X @ self.phi
        in_sample_errors = y - y_hat
        self.errors.clear()
        for e in in_sample_errors[-self.q:]:
            self.errors.append(e)

        self._fitted = True

    def predict(self, residuals):
        """One-step-ahead prediction of the next residual value."""
        if not self._fitted:
            return 0.0

        data = self._difference(residuals)
        if len(data) < self.p:
            return 0.0

        # AR component
        recent = data[-self.p:][::-1]  # most recent first
        ar_pred = float(np.dot(self.phi, recent))

        # MA component
        ma_pred = 0.0
        errors_list = list(self.errors)
        for j in range(min(self.q, len(errors_list))):
            ma_pred += self.theta[j] * errors_list[-(j + 1)]

        prediction = ar_pred + ma_pred

        # Undo differencing (add back last value of undifferenced series)
        undiff = prediction
        tail = residuals.copy()
        for _ in range(self.d):
            undiff += tail[-1]
            tail = np.diff(tail)

        # Track forecast error
        self.errors.append(0.0)  # updated externally when actual arrives

        return float(undiff)


# ---------------------------------------------------------------------------
# Lightweight Feed-Forward Neural Network (FFNN)
# ---------------------------------------------------------------------------
class SimpleFFNN:
    """Two-hidden-layer FFNN for nonlinear residual refinement (Eq. 3).

    Architecture: input(k) -> hidden1(h1) -> hidden2(h2) -> output(1)
    Activation: tanh (hidden), linear (output)
    Training: mini-batch SGD with learning rate decay.

    Parameters:
        input_size: Window of past residuals used as input (k in the paper).
        hidden1: Number of neurons in first hidden layer.
        hidden2: Number of neurons in second hidden layer.
        lr: Learning rate.
    """

    def __init__(self, input_size=10, hidden1=16, hidden2=8, lr=0.005):
        self.input_size = input_size
        self.lr = lr

        # Xavier initialisation
        self.W1 = np.random.randn(input_size, hidden1) * np.sqrt(2.0 / input_size)
        self.b1 = np.zeros(hidden1)
        self.W2 = np.random.randn(hidden1, hidden2) * np.sqrt(2.0 / hidden1)
        self.b2 = np.zeros(hidden2)
        self.W3 = np.random.randn(hidden2, 1) * np.sqrt(2.0 / hidden2)
        self.b3 = np.zeros(1)

        self._train_count = 0

    def _forward(self, x):
        """Forward pass returning intermediate activations for backprop."""
        z1 = x @ self.W1 + self.b1
        a1 = np.tanh(z1)
        z2 = a1 @ self.W2 + self.b2
        a2 = np.tanh(z2)
        z3 = a2 @ self.W3 + self.b3
        return z3, (x, z1, a1, z2, a2)

    def predict(self, residual_window):
        """Predict refined residual from a window of past residuals."""
        x = np.array(residual_window[-self.input_size:]).reshape(1, -1)
        if x.shape[1] < self.input_size:
            x = np.pad(x, ((0, 0), (self.input_size - x.shape[1], 0)))
        out, _ = self._forward(x)
        return float(out[0, 0])

    def train_step(self, residual_window, target):
        """Single SGD step (online learning as described in Section 2.5)."""
        x = np.array(residual_window[-self.input_size:]).reshape(1, -1)
        if x.shape[1] < self.input_size:
            x = np.pad(x, ((0, 0), (self.input_size - x.shape[1], 0)))

        out, (x_in, z1, a1, z2, a2) = self._forward(x)
        error = out[0, 0] - target

        # Backpropagation
        d3 = error * np.ones((1, 1))
        dW3 = a2.T @ d3
        db3 = d3.sum(axis=0)

        d2 = (d3 @ self.W3.T) * (1 - a2 ** 2)
        dW2 = a1.T @ d2
        db2 = d2.sum(axis=0)

        d1 = (d2 @ self.W2.T) * (1 - a1 ** 2)
        dW1 = x_in.T @ d1
        db1 = d1.sum(axis=0)

        # Gradient clipping
        for g in [dW1, dW2, dW3]:
            np.clip(g, -1.0, 1.0, out=g)

        # Update
        lr = self.lr / (1.0 + self._train_count * 1e-4)
        self.W3 -= lr * dW3
        self.b3 -= lr * db3
        self.W2 -= lr * dW2
        self.b2 -= lr * db2
        self.W1 -= lr * dW1
        self.b1 -= lr * db1

        self._train_count += 1
        return error ** 2


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------
class ARIMAFFNNNode(Node):
    """Hybrid ARIMA+FFNN predictive controller node.

    Collects time-series windows from joint states, force sensor, and vision,
    performs decomposition + ARIMA + FFNN prediction, and publishes the
    predicted next state as well as the combined prediction error used by
    downstream controllers (fusion_node / feeding_fsm).

    Topics published:
        /predicted_state        (Float64MultiArray) – [joint1..4 predicted positions]
        /prediction_error       (Float64)           – combined model error
        /mouth_ready_prediction (Bool)              – predicted mouth-readiness

    Topics subscribed:
        /joint_states           (JointState)
        /spoon_force            (Float64)
        /food_visible           (Bool)
        /food_center            (Point)
    """

    # Algorithm parameters (Section 2.3 / 2.4)
    WINDOW_SIZE = 50         # k – past observations to keep
    DECOMPOSE_PERIOD = 10    # ~1 s at 10 Hz
    ARIMA_P = 3
    ARIMA_D = 1
    ARIMA_Q = 1
    FFNN_INPUT = 10          # window fed to FFNN
    PREDICT_HZ = 10.0        # prediction rate

    def __init__(self):
        super().__init__('arima_ffnn_node')

        # ------ data buffers (time windows) ------
        self.joint_history = {f'joint{i}': deque(maxlen=self.WINDOW_SIZE)
                              for i in range(1, 5)}
        self.force_history = deque(maxlen=self.WINDOW_SIZE)
        self.food_x_history = deque(maxlen=self.WINDOW_SIZE)

        self.food_visible = False
        self.food_center = (0.0, 0.0, 0.0)
        self.current_force = 0.0
        self.current_joint_positions = [0.0] * 4

        # ------ models per joint ------
        self.arima_models = {f'joint{i}': SimpleARIMA(self.ARIMA_P, self.ARIMA_D, self.ARIMA_Q)
                             for i in range(1, 5)}
        self.ffnn_models = {f'joint{i}': SimpleFFNN(self.FFNN_INPUT)
                            for i in range(1, 5)}

        # Force predictor
        self.arima_force = SimpleARIMA(self.ARIMA_P, self.ARIMA_D, self.ARIMA_Q)
        self.ffnn_force = SimpleFFNN(self.FFNN_INPUT)

        # Iteration / performance tracking (Section 2.5)
        self.iteration = 0
        self.cumulative_mse = 0.0

        # ------ subscribers ------
        self.create_subscription(JointState, '/joint_states',
                                 self.joint_cb, 10)
        self.create_subscription(Float64, '/spoon_force',
                                 self.force_cb, 10)
        self.create_subscription(Bool, '/food_visible',
                                 self.food_visible_cb, 10)
        self.create_subscription(Point, '/food_center',
                                 self.food_center_cb, 10)

        # ------ publishers ------
        self.pred_state_pub = self.create_publisher(
            Float64MultiArray, '/predicted_state', 10)
        self.pred_error_pub = self.create_publisher(
            Float64, '/prediction_error', 10)
        # mouth_ready_prediction is now published by face_node (camera-based)

        # ------ timer ------
        self.timer = self.create_timer(
            1.0 / self.PREDICT_HZ, self.predict_step)

        self.get_logger().info(
            'ARIMA-FFNN predictive node started '
            f'(p={self.ARIMA_P}, d={self.ARIMA_D}, q={self.ARIMA_Q}, '
            f'ffnn_input={self.FFNN_INPUT}, window={self.WINDOW_SIZE})')

    # ---- callbacks ----
    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_history:
                self.joint_history[name].append(pos)
        self.current_joint_positions = [
            self.joint_history[f'joint{i}'][-1]
            if self.joint_history[f'joint{i}']
            else 0.0
            for i in range(1, 5)
        ]

    def force_cb(self, msg):
        self.current_force = msg.data
        self.force_history.append(msg.data)

    def food_visible_cb(self, msg):
        self.food_visible = msg.data

    def food_center_cb(self, msg):
        self.food_center = (msg.x, msg.y, msg.z)
        self.food_x_history.append(msg.x)

    # ---- main prediction loop (Eq. 4) ----
    def predict_step(self):
        """Execute one prediction cycle: decompose -> ARIMA -> FFNN -> publish."""
        predicted_positions = []
        total_error = 0.0
        n_predictions = 0

        for jname in [f'joint{i}' for i in range(1, 5)]:
            history = self.joint_history[jname]
            if len(history) < self.DECOMPOSE_PERIOD + 2:
                predicted_positions.append(
                    history[-1] if history else 0.0)
                continue

            series = np.array(history)

            # Step 3: Time-series decomposition (Algorithm 1)
            trend, seasonal, residual = decompose_series(
                series, self.DECOMPOSE_PERIOD)

            # Step 4: ARIMA on residuals (Eq. 2)
            arima = self.arima_models[jname]
            arima.fit(residual)
            arima_pred = arima.predict(residual)

            # FFNN refinement on residuals (Eq. 3)
            ffnn = self.ffnn_models[jname]
            ffnn_pred = ffnn.predict(residual)

            # Combined prediction (Eq. 4):
            # Y_{t+1} = T_t + S_t + ARIMA(R) + FFNN(R)
            combined = trend[-1] + seasonal[-1] + arima_pred + ffnn_pred
            predicted_positions.append(combined)

            # Online learning: train FFNN on actual vs predicted residual
            if len(residual) > 1:
                mse = ffnn.train_step(
                    residual[:-1].tolist(), residual[-1])
                total_error += mse
                n_predictions += 1

        # Force prediction (same pipeline)
        if len(self.force_history) >= self.DECOMPOSE_PERIOD + 2:
            f_series = np.array(self.force_history)
            f_trend, f_season, f_resid = decompose_series(
                f_series, self.DECOMPOSE_PERIOD)
            self.arima_force.fit(f_resid)
            force_pred = (f_trend[-1] + f_season[-1]
                          + self.arima_force.predict(f_resid)
                          + self.ffnn_force.predict(f_resid))
            if len(f_resid) > 1:
                self.ffnn_force.train_step(
                    f_resid[:-1].tolist(), f_resid[-1])
        else:
            force_pred = self.current_force

        # --- Publish predicted joint state ---
        state_msg = Float64MultiArray()
        state_msg.data = predicted_positions
        self.pred_state_pub.publish(state_msg)

        # --- Publish prediction error ---
        avg_error = total_error / max(n_predictions, 1)
        self.cumulative_mse = (0.95 * self.cumulative_mse + 0.05 * avg_error)
        err_msg = Float64()
        err_msg.data = self.cumulative_mse
        self.pred_error_pub.publish(err_msg)

        # mouth_ready_prediction removed — now handled by face_node

        self.iteration += 1
        if self.iteration % 50 == 0:
            self.get_logger().info(
                f'Iteration {self.iteration}: '
                f'MSE={self.cumulative_mse:.6f}, '
                f'pred_joints={[f"{p:.3f}" for p in predicted_positions]}')


def main(args=None):
    rclpy.init(args=args)
    node = ARIMAFFNNNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
