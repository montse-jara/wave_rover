import dbus
import dbus.service

from characteristics.write_characteristic import WriteCharacteristic
from characteristics.notify_characteristic import NotifyCharacteristic

SERVICE_UUID = "12345678-1234-1234-1234-1234567890AB"


class Service(dbus.service.Object):

    def __init__(self, bus, robot, ros_node, index=0):
        """
        bus        → DBus system bus
        robot      → Robot instance
        ros_node   → ROS2 node (used for publishing commands)
        index      → Service index
        """

        # Create valid DBus object path
        self.path = f"/org/bluez/example/service{index}"

        super().__init__(bus, self.path)

        self.bus = bus
        self.robot = robot
        self.ros_node = ros_node

        self.uuid = SERVICE_UUID
        self.primary = True

        # 🔹 Create characteristics
        self.write_char = WriteCharacteristic(
            bus,
            self,
            robot,
            ros_node  
        )

        self.notify_char = NotifyCharacteristic(
            bus,
            self
        )

    # Required DBus Methods

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {
            "org.bluez.GattService1": {
                "UUID": self.uuid,
                "Primary": self.primary,
                "Characteristics": [
                    self.write_char.get_path(),
                    self.notify_char.get_path()
                ]
            }
        }

    @dbus.service.method(
        "org.freedesktop.DBus.ObjectManager",
        in_signature="",
        out_signature="a{oa{sa{sv}}}"
    )
    def GetManagedObjects(self):
        """
        Returns all managed BLE objects.
        Required by BlueZ.
        """

        return {
            self.get_path(): self.get_properties(),

            self.write_char.get_path():
                self.write_char.get_properties(),

            self.notify_char.get_path():
                self.notify_char.get_properties()
        }