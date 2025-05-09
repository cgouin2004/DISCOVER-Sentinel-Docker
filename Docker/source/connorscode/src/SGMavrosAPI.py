# API 
import rospy

from abc import ABC, abstractmethod


# TODO complete this phase to make SGFlightCore completely removed from mavros, and or ros 1

class SGMavrosAbstractAPI(ABC):

    @abstractmethod
    def rate(self):
        pass



class BaseService:
    def __init__(service_name):
        self.service_name = service_name

        
