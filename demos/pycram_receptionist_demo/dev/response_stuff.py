from demos.pycram_receptionist_demo.utils.ResponseLoader import ResponseLoader


def main():
    # Modell instanziieren und Daten laden
    model = ResponseLoader(json_file='../utils/convo_response.json')
    model.load_data()

    # Ein bekanntes Hobby
    hobby = "Volleyball"
    print(f"Hobby: {hobby}")
    print(f"Antwort: {model.predict_response(hobby)}\n")

    # Ein unbekanntes Hobby
    hobby = "Skydiving"
    print(f"Hobby: {hobby}")
    print(f"Antwort: {model.predict_response(hobby)}\n")

    # Ein weiteres bekanntes Hobby
    hobby = "Cooking"
    print(f"Hobby: {hobby}")
    print(f"Antwort: {model.predict_response(hobby)}\n")

    # Ein weiteres unbekanntes Hobby
    hobby = "Snowboarding"
    print(f"Hobby: {hobby}")
    print(f"Antwort: {model.predict_response(hobby)}\n")

if __name__ == "__main__":
    main()
