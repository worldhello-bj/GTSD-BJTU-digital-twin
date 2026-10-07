"""B11 articulated source cables attached to the force-driven B10/B8 plant."""
from pathlib import Path
import sys
import cables

B10 = Path(__file__).resolve().parents[1]/"b10"
if str(B10) not in sys.path:
    sys.path.append(str(B10))
from auxiliaries import make_vehicle_network
from compat import load_b8

base = load_b8("live_viewer_full", "b11_private_live")
base.build_model = cables.model
base.make_vehicle_network = make_vehicle_network


class LiveVehicle(base.LiveVehicle):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.cable_spec = cables.LAST_SPEC

    def valve_commands(self):
        result = super().valve_commands()
        for name in ("compressor_intake", "compressor_discharge"):
            result.pop(name, None)
        return result

    def release_cable_attachment(self, key):
        if key not in {r["key"] for r in self.cable_spec["routes"]}:
            raise ValueError("unknown cable attachment")
        self.data.eq_active[self.model.equality("cable_"+key+"_attachment").id] = False

    def set_auxiliary_power_failure(self, name, failed):
        if not isinstance(failed, bool) or name not in ('compressor', *self.network.fans):
            raise ValueError('known auxiliary and boolean fault required')
        obj = self.network.pump if name == 'compressor' else self.network.fans[name]
        obj.motor.power_failed = failed

    def set_auxiliary_voltage(self, name, fraction):
        if name not in ('compressor', *self.network.fans):
            raise ValueError('known auxiliary required')
        self.network.update(0, valve_commands={name:fraction}, snapshot=False)
        if name == 'compressor':
            self.network.automatic_compressor = False

    def cable_attachment_errors(self):
        return {r["key"]:float(base.np.linalg.norm(self.data.site("cable_"+r["key"]+"_end").xpos-
                                                  self.data.site("cable_"+r["key"]+"_anchor").xpos))
                for r in self.cable_spec["routes"]}

    def telemetry(self):
        return super().telemetry() | dict(version="B11 cable equivalent", cable_spec=self.cable_spec,
                                         attachment_error_m=self.cable_attachment_errors(), pump=self.network.pump.snapshot(),
                                         fans={n:f.snapshot() for n,f in self.network.fans.items()},
                                         thermal={n:t.snapshot() for n,t in self.network.thermal.items()})


base.LiveVehicle = LiveVehicle
if __name__ == "__main__":
    raise SystemExit(base.main())
