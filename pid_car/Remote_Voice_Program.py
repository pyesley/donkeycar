#!/usr/bin/env python3
"""
Minimal Remote Voice Command System (using Vosk)

This script continuously listens for a trigger phrase (e.g., "hey donkey") using Vosk.
When the trigger phrase is detected, the system:
  1. Plays a jingle.
  2. Waits 4 seconds.
  3. Records a follow-on command for a fixed duration.
  4. Calls a remote TTS script on your PC via SSH (passing the follow-on command).
  5. Calls a remote objective detection script (Llama_OBJ.py) on your PC via SSH (passing the same command) and prints its output.
  6. Copies back the generated WAV file from the remote PC via SCP.
  7. Plays the WAV file using simpleaudio.

Usage:
    remote_voice_only_NEW.py (drive) [--trigger=<phrase>] [--mic=<index>] [--ip=<ip>] [--user=<user>] [--myscript=<path>] [--remoteaudio=<path>] [--localaudio=<path>]

Options:
    -h --help               Show this screen.
    --trigger=<phrase>      Trigger phrase to listen for [default: hey donkey].
    --mic=<index>           (Unused with Vosk; kept for compatibility) [default: 1].
    --ip=<ip>               IP of remote PC [default: 192.168.86.29].
    --user=<user>           Username on remote PC [default: mdyesley].
    --myscript=<path>       Remote TTS script path [default: /home/mdyesley/PycharmProjects/Llama/Raspberry_Audio_Llama.py].
    --remoteaudio=<path>    Path to remote WAV on PC [default: /home/mdyesley/Downloads/Sounds/demo_out.wav].
    --localaudio=<path>     Where to copy WAV locally [default: /home/mdyesley/SOUNDFILE/output.wav].
"""

import logging
import subprocess
import time
import json
from docopt import docopt

import simpleaudio as sa
from pydub import AudioSegment
from vosk import Model, KaldiRecognizer
import pyaudio

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def play_wav_file(filepath):
    try:
        wave_obj = sa.WaveObject.from_wave_file(filepath)
        play_obj = wave_obj.play()
        play_obj.wait_done()
        print("Playback finished.")
    except Exception as e:
        logger.error("Error playing WAV file: %s", e)

def play_audio(file_path):
    try:
        if file_path.lower().endswith(".mp3"):
            audio = AudioSegment.from_mp3(file_path)
            file_path = "converted_audio.wav"
            audio.export(file_path, format="wav")
        wave_obj = sa.WaveObject.from_wave_file(file_path)
        play_obj = wave_obj.play()
        play_obj.wait_done()
        print("Playback finished.")
    except Exception as e:
        print(f"Error playing audio: {e}")

class RemoteVoiceCommand:
    def __init__(self, mic_device_index=1, trigger_phrase="hey donkey",
                 pc_ip="192.168.86.29", pc_user="mdyesley",
                 remote_script="/home/mdyesley/PycharmProjects/Llama/Raspberry_Audio_Llama.py",
                 OBJ_script="/home/mdyesley/PycharmProjects/Llama/Llama_OBJ.py",
                 remote_audio_path="/home/mdyesley/Downloads/Sounds/demo_out.wav",
                 local_audio_path="/home/mdyesley/SOUNDFILE/output.wav"):
        self.mic_device_index = int(mic_device_index)
        self.trigger_phrase = trigger_phrase.lower()
        self.pc_ip = pc_ip
        self.pc_user = pc_user
        self.remote_script = remote_script
        self.OBJ_script = OBJ_script
        self.remote_audio_path = remote_audio_path
        self.local_audio_path = local_audio_path
        self.vosk_model_path = "/home/mdyesley/myPIDcar/vosk-model-small-en-us-0.15"

    def run_remote_code(self, follow_text):
        safe_text = follow_text.replace("'", "'\"'\"'")
        command = (f"ssh {self.pc_user}@{self.pc_ip} "
                   f"'/home/mdyesley/miniforge3/bin/conda run -n f5-tts python3 {self.remote_script} \"{safe_text}\"'")
        logger.info("🚀 Running remote TTS command: %s", command)
        subprocess.run(command, shell=True)

    def run_OBJ_code(self, follow_text):
        safe_text = follow_text.replace("'", "'\"'\"'")
        command = (f"ssh {self.pc_user}@{self.pc_ip} "
                   f"'/home/mdyesley/miniforge3/bin/conda run -n f5-tts python3 {self.OBJ_script} \"{safe_text}\"'")
        logger.info("🚀 Running objective detection command: %s", command)
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        objective = result.stdout.strip()
        print("Objective is: " + objective)
        return objective

    def copy_audio_file(self):
        command = f"scp {self.pc_user}@{self.pc_ip}:{self.remote_audio_path} {self.local_audio_path}"
        logger.info("Executing SCP command: %s", command)
        try:
            subprocess.run(command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            logger.error("SCP failed with error: %s", e)

    def record_follow_command(self, stream, model, rate=16000, duration=5):
        chunk_size = 1024
        num_chunks = int(rate / chunk_size * duration)
        rec_follow = KaldiRecognizer(model, rate)
        print("Recording follow-on command...")
        for _ in range(num_chunks):
            try:
                data = stream.read(chunk_size, exception_on_overflow=False)
            except Exception as e:
                logger.error("Error reading follow-on audio: %s", e)
                break
            rec_follow.AcceptWaveform(data)
        result = json.loads(rec_follow.Result())
        follow_text = result.get("text", "").lower()
        print("Recognized follow-on command:", follow_text)
        return follow_text

    def run(self):
        try:
            model = Model(self.vosk_model_path)
        except Exception as e:
            logger.error("Could not load Vosk model from '%s': %s", self.vosk_model_path, e)
            return
        recognizer = KaldiRecognizer(model, 16000)
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000,
                        input=True, frames_per_buffer=1024)
        stream.start_stream()
        logger.info("Microphone stream started; listening for trigger phrase...")
        print("Listening for trigger phrase...")
        while True:
            try:
                data = stream.read(1024, exception_on_overflow=False)
            except Exception as e:
                logger.error("Error reading from microphone: %s", e)
                continue
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text = result.get("text", "").lower()
                if text:
                    print("Heard:", text)
                if self.trigger_phrase in text:
                    logger.info("Trigger phrase detected!")
                    print("Trigger phrase detected!")
                    play_audio("/home/mdyesley/myPIDcar/2_second_jingle_for_200_subs.mp3")
                    time.sleep(4)
                    print("\nListening for follow-on command (speak now)...")
                    follow_text = self.record_follow_command(stream, model, duration=10)
                    if not follow_text:
                        print("No follow-on command detected; please try again.")
                        continue
                    logger.info("Follow-on command: %s", follow_text)
                    # Call objective detection
                    self.run_OBJ_code(follow_text)
                    # Then call TTS generation once
                    self.run_remote_code(follow_text)
                    self.copy_audio_file()
                    play_audio(self.local_audio_path)
                    print("\nListening for trigger phrase...")

def main():
    args = docopt(__doc__)
    trigger_phrase = args['--trigger']
    mic_index = args['--mic']
    pc_ip = args['--ip']
    pc_user = args['--user']
    remote_script = args['--myscript']
    remote_audio = args['--remoteaudio']
    local_audio = args['--localaudio']
    voice_cmd = RemoteVoiceCommand(
        mic_device_index=mic_index,
        trigger_phrase=trigger_phrase,
        pc_ip=pc_ip,
        pc_user=pc_user,
        remote_script=remote_script,
        OBJ_script="/home/mdyesley/PycharmProjects/Llama/Llama_OBJ.py",
        remote_audio_path=remote_audio,
        local_audio_path=local_audio
    )
    try:
        voice_cmd.run()
    except KeyboardInterrupt:
        logger.info("Shutting down remote voice command system.")

if __name__ == '__main__':
    main()
