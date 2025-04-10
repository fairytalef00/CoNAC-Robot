import sys
from PyQt5.QtWidgets import QApplication
from ui import RobotArmControl

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotArmControl()
    window.show()
    sys.exit(app.exec_())
