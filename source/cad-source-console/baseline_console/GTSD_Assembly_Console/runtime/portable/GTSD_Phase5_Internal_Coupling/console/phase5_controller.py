"""Add detached observations without modifying the frozen owner/controller code."""
import copy
from controller import Controller
from internal_registry.registry import observe


class RegisteredController(Controller):
    def _state(self, snapshot):
        if snapshot is not None and snapshot.get("binding_subsystem") == "integrated":
            snapshot = copy.deepcopy(snapshot)
            try:
                snapshot["internal_systems"] = observe(snapshot)
            except Exception as exc:
                # Registration is an independent read-only view. A broken
                # catalog must be conspicuous, but must not destroy the owner's
                # error publication path or fabricate a mechanical failure.
                snapshot["internal_systems"] = {
                    "schema": "gtsd-internal-system-observations-v1",
                    "sample_time_s": snapshot.get("time_s"),
                    "geometry_identity_match": False, "available_count": 0,
                    "observations": [], "edges": [], "hardware_connected": False,
                    "calibrated": False, "error": str(exc),
                }
        return super()._state(snapshot)
