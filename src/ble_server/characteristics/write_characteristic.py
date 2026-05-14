import dbus
import dbus.service

from std_msgs.msg import String

CHAR_UUID = "87654321-4321-4321-4321-BA0987654321"

class WriteCharacteristic(dbus.service.Object):

    def __init__(self, bus, service, robot, ros_node):
        self.path = service.get_path() + "/write0"

        super().__init__(bus, self.path)

        self.bus = bus
        self.service = service
        self.uuid = CHAR_UUID

        self.robot = robot
        self.ros_node = ros_node

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {
            "org.bluez.GattCharacteristic1": {
                "UUID": self.uuid,
                "Service": self.service.get_path(),
                "Flags": ["write"]
            }
        }

    @dbus.service.method(
        "org.bluez.GattCharacteristic1",
        in_signature="aya{sv}",
        out_signature=""
    )
    def WriteValue(self, value, options):

        text = bytes(value).decode("utf-8").strip()

        print("Received from iPhone:", text)

        # 🔹 Send to robot (existing behavior)
        self.robot.send_command(text)

        # 🔹 Publish ROS message
        msg = String()
        msg.data = text

        self.ros_node.command_publisher.publish(msg)

        self.ros_node.get_logger().info(
            f"Published command: {text}"
        )