import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robousr/Desktop/Robotica_Semestre-2026-1/Proyecto_Final_ws/install/robot2_control'
