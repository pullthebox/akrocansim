"""akrocansim is a CAN bus J1939 controller simulator"""

__version__ = '0.6.5'
__app_name__ = 'akro CAN simulator'

from . import gui

def main() -> None:
    gui.AkrocansimGui()
