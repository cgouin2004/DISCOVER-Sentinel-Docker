import json

def create_data():
    """
    Create sample flight records data.
    
    Each record is represented as a dictionary containing:
    - lat: Latitude (float)
    - long: Longitude (float)
    - alt: Altitude (int)
    - flight_type: Type of flight (int)
    - speed: Speed of flight (int)
    - delay: Delay in minutes (int)
    """
    data = [
        {"lat": 0, "long": 0,  "alt": 8, "flight_type": 0,  "speed": 1, "delay": 3, "tolerance": 0},
        {"lat": 34.58750273155081, "long": -82.80774063317057,  "alt": 0, "flight_type": 1,  "speed": 1, "delay": 1, "tolerance": 2},
        {"lat": 34.587415189177044, "long": -82.80751959733547,  "alt": 0, "flight_type": 1,  "speed": 2, "delay": 2,  "tolerance": 2},
        {"lat": 34.5870407392980, "long": -82.80776852849705,  "alt": 4, "flight_type": 1,  "speed": 3, "delay": 5,  "tolerance": 2},
        {"lat": 34.58705254282278, "long": -82.80807319951298,  "alt": 0, "flight_type": 1,  "speed": 4, "delay": 0,  "tolerance": 2},
        {"lat": 34.58750273155081, "long": -82.80774063317057,  "alt": 0, "flight_type": 1,  "speed": 5, "delay": 3,  "tolerance": 2},
        {"lat": 0, "long": 0,  "alt": 0, "flight_type": 2,  "speed": 0, "delay": 0,  "tolerance": 0}
    ]
    return data

def save_to_json(filename, data):
    """
    Write the provided data to a JSON file with indentation.
    """
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)

def main():
    filename = "/home/ros/ros_ws/src/connorscode/missions/mission.json"  # Output JSON file path
    data = create_data()    # Generate sample data
    save_to_json(filename, data)
    print(f"Data successfully saved to {filename}")

if __name__ == "__main__":
    main()
