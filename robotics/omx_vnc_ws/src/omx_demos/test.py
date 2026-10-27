import speech_recognition as sr
import pyttsx3   # text-to-speech engine

# Step 1: Initialize recognizer and speaker
r = sr.Recognizer()
engine = pyttsx3.init()   # create TTS engine

# Step 2: Capture audio from microphone
with sr.Microphone() as source:
    print("Say something...")
    audio = r.listen(source)

try:
    # Step 3: Convert speech to text
    text = r.recognize_google(audio)
    print("You said: " + text)

    # Step 4: Convert text back to speech
    engine.say("You said " + text)  # queue text to be spoken
    engine.runAndWait()             # actually speak it aloud

except sr.UnknownValueError:
    print("Sorry, I could not understand your voice.")
    engine.say("Sorry, I could not understand your voice.")
    engine.runAndWait()
except sr.RequestError:
    print("Could not request results, check internet connection.")
    engine.say("Could not request results, check internet connection.")
    engine.runAndWait()