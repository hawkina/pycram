from .knowrob_interface import *


# print(f": {get_obj_instance_of_type('http://www.ease-crc.org/ont/SUTURO.owl#')}")
def test_obj_instance_of_type():
    print("test_obj_instance_of_type")
    false_results = []

    objects = [
        "Sprite", "Tea", "Coke", "Pasta", "Cornflakes", "Sweetener", "Creamer",
        "Tomato_Soup", "Instant_Noodles", "Red_Cabbage", "Pear", "Plum", "Apple",
        "Lemon", "Peach", "Orange", "Strawberry", "Banana", "Crisps", "Gum",
        "Chocolate_Peanuts", "Cracker", "Cotton_Pads", "Tissue", "Q_Tips",
        "Toothpaste", "Spoon", "Plate", "Cup", "Fork", "Bowl", "Knife"
    ]

    for obj in objects:
        result = get_obj_instance_of_type(f"http://www.ease-crc.org/ont/SUTURO.owl#{obj}")
        if result is False:
            false_results.append(obj)

    # Print only the ones that returned False
    if false_results:
        print("Objects that returned False:", false_results)
    else:
        print("All objects returned a valid instance.")


def test_get_room_entry_pose_class():
    print("room_entry_pose_class")
    false_results = []

    objects = [
        "living_room", "bedroom", "hallway", "kitchen", "office"
    ]

    for obj in objects:
        result = get_room_entry_pose_class(f"{obj}")
        if result is False:
            false_results.append(obj)
    # Print only the ones that returned False
    if false_results:
        print("Objects that returned False:", false_results)
    else:
        print("All objects returned a valid instance.")


def test_get_room_pose():
    result = []
    for room in rooms:
        result.append(f"{room} Entry:")
        result.append(get_room_pose(f"{room}", 'entry'))
        result.append(f"{room} Exit:")
        result.append(get_room_pose(f"{room}", 'exit'))
        result.append("---")
    print(f"Results: {result}")


def test_get_room_middle_pose():
    result = []
    for room in rooms:
        result.append(f"{room} Entry:")
        result.append(get_room_pose(f"{room}", 'entry'))
        result.append(f"{room} Exit:")
        result.append(get_room_pose(f"{room}", 'exit'))
        result.append("---")
    print(f"Results: {result}")

