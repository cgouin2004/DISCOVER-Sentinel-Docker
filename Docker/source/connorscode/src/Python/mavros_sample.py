

#! /usr/bin/env python


from SGFlightKitCore import FlightManager

flight_manager = FlightManager()

flight_manager.wait_for_device_get_connected()

flight_manager.fly_to_position(0, 0, 0)


flight_manager.wait(1)

flight_manager.offboard_mode_enable()

print(FlightManager.util_is_vehicle_mode_offboard())

flight_manager.wait(1)

flight_manager.arm()

flight_manager.fly_to_position(0, 0, 10)

flight_manager.wait(120)

