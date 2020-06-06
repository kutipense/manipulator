import serial

class ManipulatorConnection():
    def __init__(self,addr="/dev/ttyUSB0"):
        self.connection = serial.Serial(addr)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.connection.close()

    def yuru(loc1, loc2, loc3, loc4, loc5, duration=100):
        self.connection.write("#3P%d#7P%d#15P%d#23P%d#31P%dT%d\r" %(loc1,loc2,loc3,loc4,loc5,duration))

