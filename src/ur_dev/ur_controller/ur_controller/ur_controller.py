from ur_controller import box_mind as bm
import rclpy
from autologging import TRACE
import logging, sys
logging.basicConfig(level=TRACE, stream=sys.stdout, format="\n%(funcName)s:%(message)s\n") # %(levelname)s:%(name)s:%(funcName)s:%(message)s

def main(args=None):
    rclpy.init(args=args)

    mind = bm.BoxMind(debug=True)
    mind.yo()

    # Shut down
    rclpy.shutdown()

if __name__ == '__main__':
    main()