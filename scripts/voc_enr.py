#!/usr/bin/env python3
import sounddevice as sd
import vosk
import json
import subprocess
import os

# Chemin vers tes fichiers audio
AUDIO_DIR = "/home/user/catkin_ws/src/museum_nav/audio"  # ex: ~/robot_audio

# Chemin absolu vers le modèle
MODEL_PATH = "/home/user/vosk-models/vosk-model-small-fr-0.22"

# Vérifie que le dossier existe
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"Modèle introuvable : {MODEL_PATH}")

model = vosk.Model()
rec = vosk.KaldiRecognizer(model, 16000)
# Charger le modèle Vosk

# Micro
stream = sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16', channels=1)
stream.start()

print("Prêt à écouter...")

while True:
    data = stream.read(4000)[0]
    if rec.AcceptWaveform(bytes(data)):
        result = json.loads(rec.Result())
        texte = result.get("text", "")
        if texte:
            print(f"Commande reconnue : {texte}")
            

            # Jouer le fichier .wav correspondant si existe
            fichier = os.path.join(AUDIO_DIR, f"{texte}.wav")
            if os.path.exists(fichier):
                subprocess.run(["aplay", fichier])
            else:
                print(f"Aucun fichier audio trouvé pour : {texte}")
