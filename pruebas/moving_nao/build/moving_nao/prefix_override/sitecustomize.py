import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/2024-tfg-eva-fernandez/pruebas/moving_nao/install/moving_nao'