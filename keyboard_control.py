from pynput import keyboard  # pip install pynput
import time

from ostrich_neck.hardware import OstrichNeckHardwareInterface

# CONFIG

HOME_YAW = 2048
HOME_PITCH = 2048
STEP = 25  # change per arrow key press (speed control)
YAW_MIN, YAW_MAX = 0, 4095
PITCH_MIN, PITCH_MAX = 1024, 3072
TIME_SLEEP = 0.01 # update frequency (seconds)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

def main():
    with OstrichNeckHardwareInterface() as neck:
        yaw = HOME_YAW
        pitch = HOME_PITCH
        neck.move_to_encoder_steps(yaw, pitch)
        print(f"Moved to home position: yaw={yaw}, pitch={pitch}")
        print("Use arrow keys to control the motors. Press ESC to quit.")

        # Track currently pressed keys
        current_keys = set()

        def on_press(key):
            try:
                current_keys.add(key)
            except AttributeError:
                pass

        def on_release(key):
            try:
                current_keys.discard(key)
                if key == keyboard.Key.esc:
                    return False  # Stop listener
            except AttributeError:
                pass

        # Start the keyboard listener in a non-blocking way
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        try:
            while listener.is_alive():
                moved = False
                if keyboard.Key.up in current_keys:
                    pitch = clamp(pitch + STEP, PITCH_MIN, PITCH_MAX)
                    moved = True
                if keyboard.Key.down in current_keys:
                    pitch = clamp(pitch - STEP, PITCH_MIN, PITCH_MAX)
                    moved = True
                if keyboard.Key.right in current_keys:
                    yaw = clamp(yaw - STEP, YAW_MIN, YAW_MAX)
                    moved = True
                if keyboard.Key.left in current_keys:
                    yaw = clamp(yaw + STEP, YAW_MIN, YAW_MAX)
                    moved = True

                if moved:
                    neck.move_to_encoder_steps(yaw, pitch)
                    time.sleep(TIME_SLEEP)  # prevent too fast updates

                time.sleep(0.01)  # small sleep to reduce CPU usage

            print("Exiting...")
        finally:
            listener.stop()
            print("Motors safely disabled and port closed.")

if __name__ == "__main__":
    main()
