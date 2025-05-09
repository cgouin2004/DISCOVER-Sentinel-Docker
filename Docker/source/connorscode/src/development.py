from SGFlightKitCore import *
import rospy


if __name__ == "__main__":
    flight_manager = FlightManager()
    flight_manager.wait_for_device_get_connected()
    print("device is connected!")

    while (not flight_manager.is_vehicle_mode_offboard()):
        print("vehicle is not offboard mode!")



    flight_manager.fly_to_position(0, 0, 1)


    flight_manager.wait(4)


    flight_manager.fly_to_position(0, 0, 0)


    #flight_manager.fly_to_position(0, 0, 0)


    rospy.spin()