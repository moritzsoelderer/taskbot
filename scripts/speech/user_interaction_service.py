import rospy
import speech_recognition as sr
from typing import Optional
from gtts import gTTS
import pygame
from io import BytesIO


class UserLanguageInteractionService:

    def __init__(self, language: str = "en", speech_speed: int = 100) -> None:
        self.recognizer = sr.Recognizer()
        self.language = language
        self.loc_to_beep_sound = rospy.get_param("beep_audio")

        pygame.mixer.init()
    
    
    def askUserBinary(self, question: str, true_answers: [str], false_answers: [str], retries: int = 0, 
                        fallback_question="Sorry I could not understand", response_true="", response_false="") -> Optional[bool]:
        true_answers = list(map(str.lower, true_answers))
        false_answers = list(map(str.lower, false_answers))

        r = 1
        self.speak(question)
        response = None
        answer = self.listen()
        response = self.detect(answer, true_answers, false_answers)

        while r < 1 + retries and response is None: 
            self.speak(fallback_question)
            answer = self.listen()
            response = self.detect(answer, true_answers, false_answers)
            r += 1

        if response is not None:
            if response:
                if response_true != "":
                    self.speak(response_true)
            else:
                if response_false != "":
                    self.speak(response_false)

        return response


    def detect(self, answer: str, true_answers: [str], false_answers: [str]) -> Optional[bool]:
        if answer is not None:
            answer = answer.lower().split()
            for true_answer in true_answers:
                print(answer, ": ", true_answer)
                if true_answer in answer:
                    return True
            for false_answer in false_answers:
                print(answer, ": ", false_answer)
                if false_answer in answer:
                    return False
            else:
                return None
        return None


    def askUserJaNein(self, question: str) -> Optional[bool]:
        return self.askUserBinary(question, ["ja"], ["nein"])


    def speak(self, question):
        tts = gTTS(text=question, lang=self.language, slow=False)
        fp = BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)
        pygame.mixer.music.load(fp, "mp3")
        pygame.mixer.music.play()

        # Wait until the speech is finished
        while pygame.mixer.music.get_busy():
            continue


    def play(self, loc_to_beep_sound: str) -> None:
        pygame.mixer.music.load(loc_to_beep_sound)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            continue
        rospy.sleep(1)


    def listen(self) -> Optional[str]:
        with sr.Microphone() as source:
            rospy.loginfo(f"Source :{source}")
            rospy.loginfo("Recording...")
            self.play(self.loc_to_beep_sound)
            audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
            rospy.loginfo("Voice Recorded.")
            try:
                text = self.recognizer.recognize_google(audio, language=self.language)
                rospy.loginfo(f"You said: {text}")
                return text
            except sr.UnknownValueError:
                rospy.loginfo("Sorry, could not understand the audio. Retrying...")
                return None
            except sr.RequestError as e:
                rospy.loginfo(f"Could not request results from Google Speech Recognition service; {e}")
                return None