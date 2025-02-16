import speech_recognition as sr
from typing import Optional
import pyttsx3

class UserLanguageInteractionService:

    def __init__(self, language: str = "de-DE", speech_speed: int = 100) -> None:
        self.recognizer = sr.Recognizer()
        self.speaker = pyttsx3.init()
        self.language = language

        self.speaker.setProperty('rate', speech_speed)
        for voice in self.speaker.getProperty('voices'):
            if language[:2] in voice.languages:
                self.speaker.setProperty('voice', voice.id)
                break
    
    def askUserBinary(self, question: str, true_answers: [str], false_answers: [str]) -> Optional[bool]:
        true_answers = list(map(str.lower, true_answers))
        false_answers = list(map(str.lower, false_answers))

        self.speaker.say(question)
        self.speaker.runAndWait()

        answer = self.listen()
        if answer is not None:
            answer = answer.lower().strip()
            if answer in true_answers:
                return True
            elif answer in false_answers:
                return False
            else:
                return None
        return None


    def askUserJaNein(self, question: str) -> Optional[bool]:
        return self.askUserBinary(question, ["ja"], ["nein"])


    def listen(self) -> Optional[str]:
        with sr.Microphone() as source:
            print("Recording...")
            audio = self.recognizer.listen(source)
            try:
                text = self.recognizer.recognize_google(audio, language=self.language)
                print("You said:", text)
                return text
            except sr.UnknownValueError:
                print("Sorry, could not understand the audio.")
                return None
            except sr.RequestError as e:
                print(f"Could not request results from Google Speech Recognition service; {e}")
                return None