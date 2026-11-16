import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/src/marrtinorobot2/marrtinorobot2_recognition/install/marrtinorobot2_recognition'
