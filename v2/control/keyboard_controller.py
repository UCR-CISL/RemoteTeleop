class RobotKeyboardController:
    """Tracks WASD key state and exposes robot teleop control values."""

    FORWARD_KEYS = {"w"}
    REVERSE_KEYS = {"s"}
    LEFT_KEYS = {"a"}
    RIGHT_KEYS = {"d"}

    def __init__(self):
        self._pressed_keys = set()

    def press(self, key):
        normalized_key = self._normalize_key(key)
        if normalized_key in self._control_keys():
            self._pressed_keys.add(normalized_key)

    def release(self, key):
        self._pressed_keys.discard(self._normalize_key(key))

    def set_pressed_keys(self, keys):
        self._pressed_keys = {
            normalized_key
            for normalized_key in (self._normalize_key(key) for key in keys)
            if normalized_key in self._control_keys()
        }

    def controls(self):
        throttle = 0.0
        steering_angle = 0.0

        if self._pressed_keys & self.FORWARD_KEYS:
            throttle += 1.0
        if self._pressed_keys & self.REVERSE_KEYS:
            throttle -= 1.0
        if self._pressed_keys & self.LEFT_KEYS:
            steering_angle -= 0.5
        if self._pressed_keys & self.RIGHT_KEYS:
            steering_angle += 0.5

        return throttle, steering_angle

    @classmethod
    def _control_keys(cls):
        return cls.FORWARD_KEYS | cls.REVERSE_KEYS | cls.LEFT_KEYS | cls.RIGHT_KEYS

    @staticmethod
    def _normalize_key(key):
        if hasattr(key, "char"):
            key = key.char
        if key is None:
            return ""
        return str(key).lower()
