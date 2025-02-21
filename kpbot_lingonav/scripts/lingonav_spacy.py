import speech_recognition as sr
recognizer = sr.Recognizer()

#function for speech to text recognition
import speech_recognition as sr
import requests
import io  # For in-memory file handling
import spacy
from spacy.matcher import PhraseMatcher

def infer_human_speech():
    recognizer = sr.Recognizer()
    mic_index = 9

    while True:
        command = input("Type 'listen' to start recording or 'exit' to exit: ").strip().lower()

        if command == "":
            with sr.Microphone(device_index=mic_index) as source:
                print("Adjusting for ambient noise... Please wait.")
                recognizer.energy_threshold = 300  
                recognizer.dynamic_energy_threshold = True
                recognizer.adjust_for_ambient_noise(source, duration=1)

                print("Listening... Speak now!")

                try:
                    # Listen until user manually stops
                    audio = recognizer.listen(source, timeout=None, phrase_time_limit=None)
                    print("Processing speech-to-text...")

                    # Send directly to API without saving
                    files = {
                        'file': ('command.wav', io.BytesIO(audio.get_wav_data()), 'audio/wav'),
                        'language': (None, 'english'),
                    }

                    response = requests.post('https://asr.iitm.ac.in/internal/asr/decode', files=files)

                    if response.status_code == 200:
                        result = response.json()
                        transcript = result.get("transcript", "") 
                        print("Transcribed Text:", transcript)
                        return transcript
                    else:
                        print("Error:", response.text)

                    break  # Exit after processing one command

                except sr.WaitTimeoutError:
                    print("No speech detected. Try again.")

        elif command == "exit":
            print("Exiting program...")
            break

        else:
            print("Invalid command! Type 'listen' to start recording or 'exit' to quit.")

def infer_human_command(command):

    # command = input("Enter your navigation command: ")
    nlp=spacy.load("en_core_web_md")

    doc=nlp(command)

    # custom_locations=["bakery", "chaat counter", "fast food counter","bill counter", "kitchen","sandwich counter","juice counter", "table one","table two",
    #                   "table three","table four","table five","table six","table seven",
    #                   "table eight","table nine","table ten", "table eleven","table twelve","table thirteen","table fourteen","table fifteen"]

    locations={'table one': (5.18, -1.82, 0.0),
                'table five': (5.18, 0.609, 0.0),
                'table 3': (2.0, 0.0, 0.0)}
    
    matcher=PhraseMatcher(nlp.vocab)
    patterns=[nlp(locs) for locs in list(locations.keys())]

    matcher.add("CUSTOM LOCATIONS", None, *patterns)

    matches=matcher(doc)

    location_seq = [doc[start:end].text for _, start, end in matches]
    coordinate_seq = []

    for i in range(len(location_seq)): 
        coordinate_seq.append(locations[location_seq[i]])

    return location_seq, coordinate_seq

# print(infer_human_command("go to bakery then to table 1"))

