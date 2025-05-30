import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/william/ORT_ELECTRICAL/ort_ws/install/ort'
