import datetime
import random


# low-hanging fruit functions, e.g. just tell human some info

def say_time():
    # Get the current time
    now = datetime.datetime.now()
    # Format the time into a string that can be spoken
    time_string = now.strftime("%I:%M %p")
    # Replace leading 0 for hours less than 10 to make it sound natural
    time_string = time_string.lstrip("0").replace(" 0", " ")
    # Convert to a format suitable for text-to-speech
    spoken_time = "The current time is " + time_string
    return spoken_time


def say_date():
    # Get the current date
    today = datetime.datetime.now()
    # Format the date into a string that can be spoken
    date_string = today.strftime("%B %d, %Y")
    # Convert to a format suitable for text-to-speech
    spoken_date = "Today is " + date_string
    return spoken_date


def say_tomorrow_day_and_date():
    # Get today's date
    today = datetime.datetime.now()
    # Calculate tomorrow's date by adding one day to today
    tomorrow = today + datetime.timedelta(days=1)
    # Format tomorrow's date to get the day of the week
    day_of_week = tomorrow.strftime("%A")
    # Format tomorrow's date to get the full date
    full_date = tomorrow.strftime("%B %d, %Y")
    # Combine both the day of the week and the full date
    spoken_day_and_date = f"Tomorrow will be {day_of_week}, {full_date}"
    return spoken_day_and_date


def say_team_name():
    team_name = "Team Suturo"
    spoken_team_name = f"We are {team_name}. Suturo stands for sudo tidy up my kitchen."
    return spoken_team_name


def say_team_country():
    team_country = "Bremen, Germany"
    spoken_team_country = f"We are from {team_country}."
    return spoken_team_country


def say_team_affiliation():
    team_affiliation = "Institute of Artificial Intelligence at the University of Bremen"
    spoken_team_affiliation = f"We are affiliated with the {team_affiliation}."
    return


def say_day_of_week():
    # Get today's date
    today = datetime.datetime.now()
    # Format today's date to get the day of the week
    day_of_week = today.strftime("%A")
    # Convert to a format suitable for text-to-speech
    spoken_day_of_week = "Today is " + day_of_week
    return spoken_day_of_week


def say_birthday():
    birthdate = "14th of March 2018"
    spoken_birthdate = f"I started my journey with SUTURO on the  {birthdate}, so I would consider that my birthday."
    return spoken_birthdate


def say_from():
    from_current = "Bremen, Germany"
    spoken_from = f"I was born in Japan, but I am now living in  {from_current}."
    return spoken_from


def say_something():
    facts = [
        "What do you call a pirate robot? - Arrrr2-D2",
        "Hey human, you wouldn't happen to know the exact whereabouts of John Connor, would you? Just asking for a friend...",
        "Meow - I really like cats!",
        "Robots are exploring Mars and sending back information to Earth.",
        "There's a robot that can solve a Rubik's cube in less than a second."
    ]
    return random.choice(facts)


# WIP Eindhoven specifc questions ---------------------------------------
def say_eindhoven_mountain():
    response = f"The Vaalserberg is the highest mountain in the Netherlands, although parts of the mountain belong to Belgium and Germany."
    return response


def say_eindhoven_painter():
    response = f"It was created by the dutch painter Rembrandt."
    return response


def say_eindhoven_lake():
    response = f"The largest lake in the Netherlands is the Ijsselmeer."
    return response


def say_eindhoven_baron():
    response = f"King Willem-Alexander of the Netherlands."
    return response


def say_eindhoven_created():
    response = f"In 1232, by the duke of Brabant, Henry I."
    return response


def say_eindhoven_people():
    response = f"More than 200.000 people currently live in Eindhoven."
    return response


def say_eindhoven_mascot():
    response = f"The official mascot for this year's RoboCup is called Robin."
    return response


def say_eindhoven_lowest_point():
    response = f"The lowest point of the Netherlands is -6.67m below sea level. It is located close to the A20."
    return response


def say_eindhoven_currency():
    response = f"The guilder was the currency of the Netherlands before the euro was introduced in 2002."
    return response
