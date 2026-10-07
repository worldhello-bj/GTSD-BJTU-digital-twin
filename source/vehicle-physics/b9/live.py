"""B9 live controls reuse B8 mechanical inputs with dynamic metering ports.

Use --headless to test without a window. The viewer shows recorded/current
rigid-body mechanics; source meshes do not contain real movable valve internals.
"""
from compat import load_b8
from valve_dynamics import make_vehicle_network

base = load_b8("live_viewer_full", "b9_private_live")
base.make_vehicle_network = make_vehicle_network


class LiveVehicle(base.LiveVehicle):
    def set_valve_coil_failure(self, port: str, failed: bool):
        self.network.set_coil_failure(port, failed)

    def telemetry(self):
        return super().telemetry() | dict(version="B9", spool_audit=self.network.spool_audit(),
                                         dynamic_metering_ports={n:s.snapshot() for n,s in self.network.spools.items()})


base.LiveVehicle = LiveVehicle

if __name__ == "__main__":
    raise SystemExit(base.main())
