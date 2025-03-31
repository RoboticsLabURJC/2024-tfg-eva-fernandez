import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/evichan/Desktop/2024-tfg-eva-fernandez/pruebas/install/moving_nao'
