#!/usr/bin/env python3

import dbus
import dbus.mainloop.glib
from gi.repository import GLib

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from service import Service
from advertisement import Advertisement
from robot import Robot

BLUEZ_SERVICE_NAME = "org.bluez"
GATT_MANAGER_IFACE = "org.bluez.GattManager1"
LE_ADVERTISING_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"


class BLERobotNode(Node):

    def __init__(self):
        super().__init__('ble_robot_node')

        # ROS Publisher
        self.command_publisher = self.create_publisher(
            String,
            '/voice_command',
            10
        )

        self.get_logger().info("🤖 BLE ROS Node Started")


def register_app_success(): 
    print("🚀 BLE Server Registered")


def register_app_failed(error):
    print("Register failed:", error)
    raise SystemExit(1)


def main():

    # 🔹 Start ROS
    rclpy.init()

    ros_node = BLERobotNode()

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    # Robot
    robot = Robot()

    # Pass ROS node into service
    app = Service(bus, robot, ros_node, index=0)

    adapter_path = "/org/bluez/hci0"

    gatt_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter_path),
        GATT_MANAGER_IFACE
    )

    adv_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter_path),
        LE_ADVERTISING_MANAGER_IFACE
    )

    options = dbus.Dictionary({}, signature="sv")

    gatt_manager.RegisterApplication(
        app.get_path(),
        options,
        reply_handler=register_app_success,
        error_handler=register_app_failed
    )

    adv = Advertisement(bus, 0)

    adv_manager.RegisterAdvertisement(
        adv.get_path(),
        options,
        reply_handler=lambda: print("Advertising Started"),
        error_handler=lambda e: print("Advertising Failed:", e)
    )

    # Run both loops
    loop = GLib.MainLoop()

    try:
        loop.run()

    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C received — shutting down")

        loop.quit()          # ← important

    finally:
        robot.shutdown()     # ← if you added shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()