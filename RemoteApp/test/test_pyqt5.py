# import sys
# from PyQt5 import QtCore, QtWidgets

# class YourClassName(QtWidgets.QWidget):
#     def __init__(self):
#         super().__init__()

#         # ... Other initialization code ...

#         self.key_timer = QtCore.QTimer(self)
#         self.key_timer.timeout.connect(self.parse_vehicle_keys)
#         self.key_pressed = {}

#     def keyPressEvent(self, event):
#         """
#         Key press event
#         """
#         key = event.key()
#         self.key_pressed[key] = True
#         self.key_timer.start(100)  # Start the timer for continuous key checking

#     def keyReleaseEvent(self, event):
#         """
#         Key release event
#         """
#         key = event.key()
#         if key in self.key_pressed:
#             del self.key_pressed[key]
#         if not self.key_pressed:
#             self.key_timer.stop()

#     def parse_vehicle_keys(self):
#         """
#         Continuously parse vehicle keys
#         """
#         for key in self.key_pressed:
#             if key == QtCore.Qt.Key_Up:
#                 print("UP")
#                 # self._throttle = min(self._throttle + 1, 240)
#                 # Update other values or perform actions here based on keys
#             elif key == QtCore.Qt.Key_Down:
#                 print("Down")
#                 # Handle the down key
#                 pass
#             elif key == QtCore.Qt.Key_Left:
#                 print("LEFT")
#                 # Handle the left key
#                 pass
#             elif key == QtCore.Qt.Key_Right:
#                 # Handle the right key
#                 pass

# if __name__ == "__main__":
#     app = QtWidgets.QApplication(sys.argv)
#     your_class_instance = YourClassName()
#     your_class_instance.show()
#     sys.exit(app.exec_())





import sys
from PyQt5 import QtCore, QtWidgets

class YourClassName(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        # ... Other initialization code ...

        self.key_pressed = set()

        self.key_timer = QtCore.QTimer(self)
        self.key_timer.timeout.connect(self.parse_vehicle_keys)
        self.key_timer.start(100)  # Start the timer for continuous key checking

    def keyPressEvent(self, event):
        """
        Key press event
        """
        key = event.key()
        self.key_pressed.add(key)

    def keyReleaseEvent(self, event):
        """
        Key release event
        """
        key = event.key()
        if key in self.key_pressed:
            self.key_pressed.remove(key)

    def parse_vehicle_keys(self):
        """
        Continuously parse vehicle keys
        """
        for key in self.key_pressed:
            if key == QtCore.Qt.Key_Up:
                print("UP")
                # self._throttle = min(self._throttle + 1, 240)
                # Update other values or perform actions here based on keys
            elif key == QtCore.Qt.Key_Down:
                print("Down")
                # Handle the down key
                pass
            elif key == QtCore.Qt.Key_Left:
                print("LEFT")
                # Handle the left key
                pass
            elif key == QtCore.Qt.Key_Right:
                # Handle the right key
                pass

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    your_class_instance = YourClassName()
    your_class_instance.show()
    sys.exit(app.exec_())
