#!/usr/bin/env python3

import rclpy

from ars_msf_slam.ars_msf_slam_ros import ArsMsfSlamRos


def main(args=None):

  rclpy.init(args=args)

  ars_msf_slam_ros = ArsMsfSlamRos()

  ars_msf_slam_ros.open()

  try:
      ars_msf_slam_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_msf_slam_ros.destroy_node()
    rclpy.try_shutdown()

  return 0


''' MAIN '''
if __name__ == '__main__':

  main()
