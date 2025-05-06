import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lunabotics/lunabotics-csu25/lunabotics_workspace/src/manual_drive/install/manual_drive'
