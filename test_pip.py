import numpy as np
import subprocess
import os
import time

PIPER_MODEL = "/home/vladilena/piper/en_GB-semaine-medium.onnx"
PIPER_BINARY = "/home/vladilena/piper/piper/piper"

def read_text_from_image():


    command = f'echo "OK. Then lets begin" | {PIPER_BINARY} --model {PIPER_MODEL} --output_file sound_stella/start4.wav'
    subprocess.run(command, shell=True, executable="/bin/bash", check=True)

    os.system("cvlc --play-and-exit --quiet /home/vladilena/th_open_ws/src/your_smach/script/sound_stella/start4.wav")


if __name__ == "__main__":
    read_text_from_image()
