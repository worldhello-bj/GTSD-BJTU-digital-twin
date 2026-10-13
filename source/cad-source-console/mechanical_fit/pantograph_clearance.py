"""Exact 2-D circular-fit checks; no silent CAD recentering or physical fit claim.

The same nominal circle can differ from its rendered polygon. This module uses
native CAD circles for dimensions and separately reports polygonal clearance.
Pin radius is a user-selected research parameter, never an inferred real pin.
"""
from pathlib import Path
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parent


def evaluate(pin_diameter_mm=7.9, minimum_radial_clearance_mm=0.0):
    for name, value in (("pin diameter", pin_diameter_mm),
                        ("minimum clearance", minimum_radial_clearance_mm)):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(name + " must be a finite number in mm")
    if pin_diameter_mm <= 0 or pin_diameter_mm > 20:
        raise ValueError("Research pin diameter must be in (0, 20] mm")
    if minimum_radial_clearance_mm < 0 or minimum_radial_clearance_mm > 2:
        raise ValueError("Research clearance must be in [0, 2] mm")
    path = ROOT / "evidence/pantograph_interfaces.json"
    raw = path.read_bytes()
    source = json.loads(raw)
    rows = []
    for item in source["interfaces"]:
        plate, eye = item["outer_hole"], item["tie_hole"]
        offset = math.dist(plate["source_center_root_mm"][:2],
                           eye["source_center_root_mm"][:2])
        if abs(offset - item["source_axis_offset_mm"]) > 1e-9:
            raise ValueError("Native source offset witness differs")
        outer_margin = plate["source_radius_mm"] - offset - pin_diameter_mm / 2
        eye_margin = eye["source_radius_mm"] - pin_diameter_mm / 2
        # Chords bound the radius of an inscribed 128-sided hole. This is a
        # conservative visual-polygon check, not a source diameter correction.
        polygon_margin = min(plate["source_radius_mm"] * math.cos(math.pi / 128) - offset,
                             eye["source_radius_mm"] * math.cos(math.pi / 128)) - pin_diameter_mm / 2
        max_diameter = 2 * min(eye["source_radius_mm"], plate["source_radius_mm"] - offset)
        rows.append({
            "interface": item["id"], "native_center_offset_mm": offset,
            "outer_hole_diameter_mm": 2 * plate["source_radius_mm"],
            "tie_hole_diameter_mm": 2 * eye["source_radius_mm"],
            "pin_axis_choice": "NATIVE_TIE_HOLE_AXIS_RESEARCH_CHOICE",
            "outer_radial_margin_mm": outer_margin,
            "tie_radial_margin_mm": eye_margin,
            "conservative_render_polygon_margin_mm": polygon_margin,
            "maximum_common_pin_diameter_mm": max_diameter,
            "outer_axis_comparison": {
                "axis_choice": "NATIVE_OUTER_HOLE_AXIS",
                "outer_radial_margin_mm": plate["source_radius_mm"] - pin_diameter_mm / 2,
                "tie_radial_margin_mm": eye["source_radius_mm"] - offset - pin_diameter_mm / 2,
                "maximum_common_pin_diameter_mm": 2 * min(plate["source_radius_mm"], eye["source_radius_mm"] - offset),
                "nominal_circles_fit": min(plate["source_radius_mm"], eye["source_radius_mm"] - offset) - pin_diameter_mm / 2 >= minimum_radial_clearance_mm - 1e-12,
                "mapping_note": "Existing H1 C hinge uses the outer plate axis, not the tie-axis study choice. H1 D is an equality closure. This comparison does not change either solver axis or the source meshes.",
            },
            "nominal_circles_fit": min(outer_margin, eye_margin) >= minimum_radial_clearance_mm - 1e-12,
            "polygonal_display_fit": polygon_margin >= minimum_radial_clearance_mm,
            "source_handles": [plate["source_handle"], eye["source_handle"]],
            "source_paths": [plate["source_path"], eye["source_path"]],
            "source_geometry_moved_or_scaled": False,
            "axial_stack_confirmed": False,
            "hole_size_is_not_measured_pin_size": True,
        })
    return {
        "schema": "gtsd-pantograph-circular-fit-review-v1", "units": "mm",
        "pin_diameter_mm": pin_diameter_mm,
        "pin_diameter_evidence": "EXPLICIT_REPLACEABLE_ENGINEERING_PARAMETER_NOT_MEASURED",
        "minimum_radial_clearance_mm": minimum_radial_clearance_mm,
        "source_inputs_sha256": hashlib.sha256(raw).hexdigest(),
        "source_dxf_sha256": source["source_dxf_sha256"], "interfaces": rows,
        "new_physics": False, "as_built": False,
        "conclusion": "Native C/D offsets alone are not proof of material interference. The 8.2 mm outer holes admit an 8 mm nominal pin on the 8 mm tie-hole axes in exact circle geometry; tie clearance, tolerances and axial stack remain unconfirmed.",
    }


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--pin-diameter-mm", type=float, default=7.9)
    parser.add_argument("--minimum-radial-clearance-mm", type=float, default=0.)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.pin_diameter_mm, args.minimum_radial_clearance_mm), indent=2))
