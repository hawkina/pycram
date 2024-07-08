import datetime
import random


# low hanging fruit functions, e.g. just tell human some info

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
    birthdate = "first of November 2018"
    spoken_birthdate = f"I started my journey with SUTURO on the  {birthdate}, so I would consider that my birthday."
    return spoken_birthdate


def say_from():
    from_current = "Bremen, Germany"
    spoken_from = f"I was born in Japan, but I am now living in  {from_current}."
    return spoken_from


def say_something():
    facts = [
        "Robots can learn to cook from watching YouTube videos!",
        "Some robots are designed to help protect endangered species.",
        "In Japan, robots can serve as Buddhist priests.",
        "The first robot was created in the 5th century BC and was a mechanical bird.",
        "Robots are exploring Mars and sending back information to Earth.",
        "There's a robot that can solve a Rubik's cube in less than a second."
    ]
    return random.choice(facts)
