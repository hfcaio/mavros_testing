#!/usr/bin/env python3

import rclpy 
import threading
from mavros_testing.communication import Mav

if __name__ == "__main__":
  # TESTING MAVROS: square path
  rclpy.init()
  mav = Mav(debug=True)
  thread = threading.Thread(target=rclpy.spin, args=(mav,), daemon=True)
  thread.start()

  rate = mav.create_rate(10) # 10 Hz
  try:
    while rclpy.ok():
      mav.takeoff(1.0)
      mav.goto(x=1.0, y=0.0, z=1.0, yaw=0.0, send_time=5)
      mav.goto(x=1.0, y=1.0, z=1.0, yaw=0.0, send_time=5)
      mav.goto(x=0.0, y=1.0, z=1.0, yaw=0.0, send_time=5)
      mav.goto(x=0.0, y=0.0, z=1.0, yaw=0.0, send_time=5)
      if mav.land(): break
      rate.sleep()
  except KeyboardInterrupt:
    pass

  rclpy.shutdown()
  thread.join()