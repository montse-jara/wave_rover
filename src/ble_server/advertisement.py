import dbus
import dbus.service

SERVICE_UUID = "12345678-1234-1234-1234-1234567890AB"

class Advertisement(dbus.service.Object):
    PATH_BASE = "/org/bluez/example/advertisement"

    def __init__(self, bus, index):
        self.path = self.PATH_BASE + str(index)
        super().__init__(bus, self.path)
        self.service_uuid = SERVICE_UUID

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {
            "org.bluez.LEAdvertisement1": {
                "Type": "peripheral",
                "ServiceUUIDs": [self.service_uuid],
                "LocalName": "RubikPi"
            }
        }

    @dbus.service.method("org.freedesktop.DBus.Properties",
                         in_signature="ss",
                         out_signature="v")
    def Get(self, interface, prop):
        props = self.get_properties()[interface]
        return props[prop]

    @dbus.service.method("org.freedesktop.DBus.Properties",
                         in_signature="ssv",
                         out_signature="")
    def Set(self, interface, prop, value):
        raise NotImplementedError()

    @dbus.service.method("org.freedesktop.DBus.Properties",
                         in_signature="s",
                         out_signature="a{sv}")
    def GetAll(self, interface):
        return self.get_properties()[interface]