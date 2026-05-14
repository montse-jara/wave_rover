import dbus
import dbus.service

NOTIFY_CHAR_UUID = "12348765-1234-1234-1234-1234567890AB"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"

class NotifyCharacteristic(dbus.service.Object):
    def __init__(self, bus, service):
        self.path = service.get_path() + "/notify0"
        super().__init__(bus, self.path)
        self.bus = bus
        self.service = service
        self.uuid = NOTIFY_CHAR_UUID
        self.notifying = False

    # Required DBus Methods
    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {
            "org.bluez.GattCharacteristic1": {
                "UUID": self.uuid,
                "Service": self.service.get_path(),
                "Flags": ["notify"]
            }
        }

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    @dbus.service.method("org.bluez.GattCharacteristic1",
                         in_signature="",
                         out_signature="")
    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
        print("🔔 Notifications enabled")

    @dbus.service.method("org.bluez.GattCharacteristic1",
                         in_signature="",
                         out_signature="")
    def StopNotify(self):
        self.notifying = False
        print("🔕 Notifications stopped")

    # Send notification to phone
    def send_notification(self, message):
        if not self.notifying:
            return
        value = [dbus.Byte(c) for c in message.encode()]
        self.PropertiesChanged(
            "org.bluez.GattCharacteristic1",
            {"Value": value},
            []
        )
        print("📣 Notified:", message)