import subprocess
import os
import time


class Robot:
    def __init__(self):
        print("🗣️ Voice Robot initialized")

        self.audio_dir = os.path.join(
            os.path.dirname(__file__),
            "robot_audio"
        )

        self.current_destination = None
        self.last_command = None

        self.audio_files = {
            "BATHROOM": "heading_bathroom.wav",
            "STOP": "stop.wav",
            "RESUME": "resume.wav",
            "UNKNOWN": "unknown.wav",
            "ARRIVED": "destination_arrived.wav",
            "BASKETBALL": "heading_basketball.wav",
            "CONNECTED": "user_connected.wav",
            "START": "heading_start.wav"
        }

    # -------------------------
    # ENTRY POINT
    # -------------------------
    def send_command(self, command):
        command = command.upper().strip()

        print(f"📩 Received command: {command}")

        if command in ["BATHROOM", "BASKETBALL","START"]:
            self.last_command = command
            self.current_destination = command

            print(f"📤 Heading to {command}")

            self.speak(self.audio_files[command])
            return

        elif command in self.audio_files:
            self.speak(self.audio_files[command])
        else:
            self.speak(self.audio_files["UNKNOWN"])


    # -------------------------
    # AUDIO PLAYER
    # -------------------------
    def speak(self, filename):
        audio_path = os.path.join(self.audio_dir, filename)

        if not os.path.exists(audio_path):
            print(f"⚠️ Audio file missing: {audio_path}")
            return

        print(f"🔊 Playing: {audio_path}")

        try:
            subprocess.run(
                ["aplay", audio_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True
            )
        except Exception as e:
            print(f"❌ Audio playback error: {e}")