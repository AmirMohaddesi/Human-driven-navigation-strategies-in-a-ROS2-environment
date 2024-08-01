import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hsr/disaster_response_swarm/install/disaster_response_swarm'
