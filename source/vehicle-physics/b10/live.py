"""B10 live vehicle with causal auxiliary commands and fault setters."""
from auxiliaries import make_vehicle_network
from compat import load_b8

base = load_b8("live_viewer_full", "b10_private_live")
base.make_vehicle_network = make_vehicle_network


class LiveVehicle(base.LiveVehicle):
    def valve_commands(self):
        commands = super().valve_commands()
        for name in ("compressor_intake", "compressor_discharge"):
            commands.pop(name, None)
        return commands

    def set_auxiliary_power_failure(self, name, failed):
        if not isinstance(failed, bool) or name not in ("compressor", *self.network.fans):
            raise ValueError("known auxiliary and boolean fault required")
        obj = self.network.pump if name == "compressor" else self.network.fans[name]
        obj.motor.power_failed = failed

    def set_auxiliary_voltage(self, name, fraction):
        if name not in ("compressor", *self.network.fans):
            raise ValueError("known auxiliary required")
        self.network.update(0, valve_commands={name: fraction}, snapshot=False)
        if name == "compressor":
            self.network.automatic_compressor = False

    def telemetry(self):
        return super().telemetry() | dict(version="B10", pump=self.network.pump.snapshot(),
                                         fans={n:f.snapshot() for n,f in self.network.fans.items()},
                                         thermal={n:t.snapshot() for n,t in self.network.thermal.items()})


base.LiveVehicle = LiveVehicle
if __name__ == "__main__":
    raise SystemExit(base.main())
