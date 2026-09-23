from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Ax1
from OCC.Display.SimpleGui import init_display
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCC.Core.gp import gp_Trsf, gp_Vec
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
import numpy as np

import OCC
print(OCC.VERSION)

import warnings
warnings.filterwarnings("ignore")

import csv
results = []

# Configuration
HULL_STEP_FILE = "FULL.step"
HEIGHTS_TO_CUT_AT = [0, 180]  # mm
# HEIGHTS_TO_CUT_AT = np.linspace(0, 180, 9+1)
HEIGHTS_TO_CUT_AT = np.concatenate([np.arange(0, 80, 1), np.arange(80, 180+1, 5)]).tolist()
HEIGHTS_TO_CUT_AT = np.arange(180, 180+150+1, 10).tolist()
print(f"{HEIGHTS_TO_CUT_AT=}")
# PITCHES_TO_CUT_AT: rotation about the boat's own COM (com_pnt), axis = lateral (+X, STEP frame).
# Right-hand rule about +X: positive theta rotates +Y toward +Z.
PITCHES_TO_CUT_AT = list(range(-20, 21)) # deg from -20 to 20
# PITCHES_TO_CUT_AT = [10, 20] # deg
print(f"{PITCHES_TO_CUT_AT=}")
HEIGHTS_TO_SAVE     = []  # mm
HEIGHTS_TO_DISPLAY  = []  # mm
# HEIGHTS_TO_DISPLAY  = [HEIGHTS_TO_CUT_AT[0], HEIGHTS_TO_CUT_AT[-1]]  # mm
print(f"{HEIGHTS_TO_DISPLAY=}")
# COM position relative to the STEP-frame origin (measured): 529.9 mm forward,
# 0 mm left/right, 24.8 mm lower (STEP y-back z-up so -529.9 forward, -24.8 down relative to STEP origin)
FORWARD_MM = 529.9
DOWN_MM = 24.8
COM_STEP_MM = gp_Pnt(0, -FORWARD_MM, -DOWN_MM)  # boat's COM, raw (unshifted) STEP frame

# Let the water body be a 3x3 meters wide and long, 1 meter deep centered at the (0, 0, -.5)
BOX_SIZE_XY = 3000     # 3 m wide/long
BOX_DEPTH   = 1000     # 1 m depth

# Reload the hull from scratch every N pitch iterations, to avoid whatever OCC-internal
RELOAD_HULL_EVERY_N_PITCHES = 2

def load_hull():
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Copy
    from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
    from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh

    reader = STEPControl_Reader()
    status = reader.ReadFile(HULL_STEP_FILE)
    if status != 1:
        raise RuntimeError("Nie udało się wczytać pliku STEP.")
    reader.TransferRoots()
    h = reader.OneShape()

    # Copy to detach transient references
    h = BRepBuilderAPI_Copy(h).Shape()
    # Merge coplanar/collinear faces (reduces complexity)
    unifier = ShapeUpgrade_UnifySameDomain(h, True, True, True)
    unifier.Build()
    h = unifier.Shape()
    # Pre-mesh to speed up Boolean ops (tolerance ~1 mm)
    BRepMesh_IncrementalMesh(h, 1.0)
    return h

hull = load_hull()

if HEIGHTS_TO_DISPLAY:
    # Prepare a display
    display, start_display, add_menu, add_function_to_menu = init_display()

# Iterate over pitch angles (outer) and heights (inner)
for pitch_i, theta_deg in enumerate(PITCHES_TO_CUT_AT):
    if pitch_i > 0 and pitch_i % RELOAD_HULL_EVERY_N_PITCHES == 0:
        print(f"[RELOAD] rebuilding hull from STEP after {pitch_i} pitch iterations")
        hull = load_hull()

    theta_rad = np.deg2rad(theta_deg)
    # Geometry uses the negated angle: with +X as the rotation axis, +theta_rad here was
    # observed to dip the bow down - flipped so positive theta_deg raises the bow (matches
    # the Simulink model's pitch convention).
    rot_rad = -theta_rad

    # Rotate the hull about its own COM (com_pnt/COM_STEP_MM), axis = lateral (+X)
    rot = gp_Trsf()
    rot.SetRotation(gp_Ax1(COM_STEP_MM, gp_Dir(1, 0, 0)), rot_rad)
    hull_pitched = BRepBuilderAPI_Transform(hull, rot).Shape()

    for i, z in enumerate(HEIGHTS_TO_CUT_AT):

        # Create the water body at fixed place (water surface at z=0) xy center at (0,0)
        water = BRepPrimAPI_MakeBox(
            gp_Pnt(-BOX_SIZE_XY/2, -BOX_SIZE_XY/2, -BOX_DEPTH), # lower-left-bottom corner
            BOX_SIZE_XY, BOX_SIZE_XY, BOX_DEPTH                 # X, Y size, height
        ).Shape()

        # Heave: translate the already-pitched hull by z
        trsf = gp_Trsf()
        trsf.SetTranslation(gp_Vec(0, 0, z))
        hull_shifted = BRepBuilderAPI_Transform(hull_pitched, trsf).Shape()

        # Intersect the two (submerged part)
        inter = BRepAlgoAPI_Common(hull_shifted, water).Shape()
        # Calculate the submerged volume, buoyancy etc.
        rho = 1000  # kg/m³
        g = 9.81    # m/s²
        props = GProp_GProps()
        try:
            brepgprop.VolumeProperties(inter, props)
        except Exception as exc:
            print(f"[SKIP] theta={theta_deg} deg, z={z} mm: VolumeProperties failed ({exc})")
            continue
        V = props.Mass() / 1e9          # mm³ → m³  (since OCC works in mm)
        cob = props.CentreOfMass()      # center of buoyancy of the submerged part, STEP frame
        F_b = V * rho * g                # buoyant force [N]

        # Boat's actual COM: rotation about itself leaves it fixed, then heave translates it.
        com_pnt = COM_STEP_MM.Transformed(rot).Transformed(trsf)

        # CoB relative to COM, in world/STEP axes (meters)
        dx = (cob.X() - com_pnt.X()) / 1000
        dy = (cob.Y() - com_pnt.Y()) / 1000
        dz = (cob.Z() - com_pnt.Z()) / 1000

        # Un-rotate this offset vector by -rot_rad about X so it's expressed in the hull's
        # OWN (body-fixed) axes again, then map STEP-local axes -> NED body axes.
        dy_body = dy * np.cos(rot_rad) + dz * np.sin(rot_rad)
        dz_body = -dy * np.sin(rot_rad) + dz * np.cos(rot_rad)
        dx_body = dx
        CoB_x_B = -dy_body   # forward
        CoB_y_B = dx_body    # right
        CoB_z_B = -dz_body   # down

        # Heave (COM height relative to water, NED z-down: positive = COM underwater)
        heave_com_m = (DOWN_MM - z) / 1000
        pitch_rad = theta_rad

        print(f"theta={theta_deg:>6.1f} deg | z={z:>6.1f} mm | V={V:.6f} m³ | F_b={F_b:8.2f} N | heave_com={heave_com_m:.4f} m | CoB_B=({CoB_x_B:.4f}, {CoB_y_B:.4f}, {CoB_z_B:.4f}) m")

        results.append({
            "heave_com_m": heave_com_m,
            "pitch_rad": pitch_rad,
            "volume_m3": V,
            "F_b_N": F_b,
            "CoB_x_B_m": CoB_x_B,
            "CoB_y_B_m": CoB_y_B,
            "CoB_z_B_m": CoB_z_B,
        })

        if z in HEIGHTS_TO_SAVE:
            # Zapisz do pliku STEP dla podglądu
            writer = STEPControl_Writer()
            writer.Transfer(inter, STEPControl_AsIs)
            writer.Write(f"cut_{theta_deg}_{i}_z{z}.step")
            print(f"[OK] Zapisano przekrój do cut_{theta_deg}_{i}_z{z}.step")

        if z in HEIGHTS_TO_DISPLAY:
            # Create a small marker (bigger + black for contrast against the red hull)
            cob_marker = BRepPrimAPI_MakeSphere(gp_Pnt(cob.X(), cob.Y(), cob.Z()), 15).Shape()

            # Two line segments, both transformed by the same rot+trsf as hull_shifted so
            # they track the currently displayed hull: origin -> forward (-Y), then forward -> down (-Z).
            origin_pnt = gp_Pnt(0, 0, 0).Transformed(rot).Transformed(trsf)
            forward_tip = gp_Pnt(0, -FORWARD_MM, 0).Transformed(rot).Transformed(trsf)
            origin_marker = BRepPrimAPI_MakeSphere(origin_pnt, 10).Shape()
            com_marker = BRepPrimAPI_MakeSphere(com_pnt, 10).Shape()
            forward_edge = BRepBuilderAPI_MakeEdge(origin_pnt, forward_tip).Shape()
            down_edge = BRepBuilderAPI_MakeEdge(forward_tip, com_pnt).Shape()

            display.EraseAll()
            ais_water = display.DisplayShape(water, update=False, color='BLUE1')[0]
            ais_water.SetTransparency(0.1)
            ais_hull = display.DisplayShape(hull_shifted, update=False, color='RED')[0]
            ais_hull.SetTransparency(0.2)
            ais_inter = display.DisplayShape(inter, update=False, color='RED')
            # ais_inter.SetTransparency(0.5)
            display.DisplayShape(cob_marker, update=False, color='BLACK')
            display.DisplayShape(origin_marker, update=False, color='WHITE')
            display.DisplayShape(com_marker, update=False, color='GREEN')
            display.DisplayShape(forward_edge, update=False, color='MAGENTA1')
            display.DisplayShape(down_edge, update=False, color='CYAN1')
            display.display_triedron()  # X (red) / Y (green) / Z (blue) arrows at the origin
            display.FitAll()
            display.Repaint()
            input(f"Press Enter for next display...")

# # Fit view and start GUI once (outside loop)
# display.FitAll()
# start_display()

# Save results to CSV
with open("wtf.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=results[0].keys())
    writer.writeheader()
    writer.writerows(results)

print("[OK] Results saved to wtf.csv")
