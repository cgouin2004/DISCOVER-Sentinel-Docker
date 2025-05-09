from SGFlightKitCore import FlightManager

def main():
    fm = FlightManager()
    waypoints = fm.load_waypoints()

    # Process each record and extract its values into Python variables
    for idx, wp in enumerate(waypoints, start=1):
        print(
            f"Waypoint {idx}: ",
            f"lat={wp['lat']}, ",
            f"long={wp['long']}, ",
            f"alt={wp['alt']}, ",
            f"flight_type={wp['flight_type']}, ",
            f"speed={wp['speed']}, ",
            f"delay={wp['delay']}"
        )


if __name__ == "__main__":
    main()
