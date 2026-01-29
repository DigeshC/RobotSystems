from abc import abstractmethod

class Sensing:
    def __init__(self):
        pass

    @abstractmethod
    def read_values(self):
        # Subclasses must implement this method
        pass


    