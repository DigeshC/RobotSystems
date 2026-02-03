from abc import abstractmethod

class Detector:
    def __init__(self):
        pass

    @abstractmethod
    def detect(self):
        # Subclasses must implement this method
        pass