#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gtts import gTTS
import os
import speech_recognition as sr

order = []
menu = ["nasi lemak", "pizza", "burger", "fried chicken", "pepsi", "orange juice", "apple juice", "coffee"]

def text2int(textnum, numwords={}):
    if not numwords:
      units = [
        "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
        "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
        "sixteen", "seventeen", "eighteen", "nineteen",
      ]
      for idx, word in enumerate(units):    numwords[word] = (1, idx)

    current = result = 0
    for word in textnum.split():
        if word not in numwords:
          raise Exception("Illegal word: " + word)

        scale, increment = numwords[word]
        current = current * scale + increment
        if scale > 100:
            result += current
            current = 0

    return result + current

def add_order(food, amount):
    food_order = [item[0] for item in order]
    if food in food_order:
        index = food_order.index(food)
        order[index][1] = str(int(order[index][1]) + int(amount))
    else:
        order.append([food, amount])


def get_order(isRepeat):
    order_text = "repeat order " if isRepeat else "Your order consists of "
    if len(order) == 1:
        order_text += order[0][1] + " " + order[0][0]
    elif len(order) == 0:
        return None
    else:
        for i in range(len(order)):
            if i != len(order) - 1:
                order_text += order[i][1] + " " + order[i][0] + " "
            else:
                order_text += "and " + order[i][1] + " " + order[i][0]
    return order_text

def googletts(speech):
    flag = False
    end = False
    speech_text = []
    speech = speech.decode()
    speech = speech.lower()
    # change numeric word to number like one to 1
    for word in speech.split():
        try:
            speech_text.append(text2int(word))
        except:
            speech_text.append(word)
    speech = " ".join(map(str, speech_text))
    items = []
    amount = []
    # check word in speech isit exist in sentence
    for item in menu:
        if item in speech:
            items.append(item)
    # check amount of items required
    for word in speech.split():
        if word.isdigit():
            amount.append(word)
        elif word == "a":
            amount.append('1')
    # add the order to the order list
    for (x, y) in zip(items, amount):
        flag = True
        add_order(x, y)
    # other command to recognize
    if "repeat" in speech:
        tts = get_order(True)
    elif "done" in speech:
        tts = get_order(False)
    elif "thank you" in speech:
        tts = "thank you"
        end = True
    elif "menu" in speech:
        tts = "Sure, this is the menu"
    elif flag:
        tts = "please continue your order"
    else:
        tts = "cannot recognize your order. please repeat your sentence"

    rospy.init_node('googletts', anonymous=True)
    tts_reply = gTTS(tts)
    tts_reply.save("speech.mp3")
    os.system("mpg321 speech.mp3")
    os.remove("speech.mp3")
    return end


def googlesr():
    order = ""
    rospy.init_node('googlesr', anonymous=True)
    pub = rospy.Publisher('result', String, queue_size=10)
    overall = ""
    greet = gTTS("Dear Customer, how I can help you?")
    greet.save("speech.mp3")
    os.system("mpg321 speech.mp3")
    os.remove("speech.mp3")
    while not rospy.is_shutdown():
        # obtain audio from the microphone
        r = sr.Recognizer()

        with sr.Microphone() as source:
            print(">>> Say something!")
            audio = r.record(source, duration=5)

        # recognize speech using Google Speech Recognition
        try:
            result = r.recognize_google(audio)
            print("SR result: " + result)
            end = googletts(result)
            if end:
                break
        except sr.UnknownValueError:
            print("SR could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
        pub.publish(order)


if __name__ == '__main__':
    order = []
    try:
        googlesr()
    except rospy.ROSInterruptException:
        pass

