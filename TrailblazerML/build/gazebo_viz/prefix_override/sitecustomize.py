import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rafal/diff_Trailblazer/TrailblazerML/install/gazebo_viz'
