from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.Core.gp import gp_Pnt
from OCC.Display.SimpleGui import init_display
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCC.Core.gp import gp_Trsf, gp_Vec
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop_VolumeProperties
import numpy as np

import OCC
print(OCC.VERSION)

import warnings
warnings.filterwarnings("ignore")

import csv
results = []

# Configuration
HULL_STEP_FILE = "FULL.step"
HEIGHTS_TO_CUT_AT   = [57, 58]  # mm
# HEIGHTS_TO_CUT_AT = np.linspace(0, 180, 18+1)
# HEIGHTS_TO_CUT_AT = np.concatenate([np.arange(0, 80, 0.5), np.arange(80, 180+1, 5)]).tolist()
HEIGHTS_TO_SAVE     = []  # mm
# HEIGHTS_TO_DISPLAY  = [30, 90, 150]  # mm
# HEIGHTS_TO_DISPLAY  = [HEIGHTS_TO_CUT_AT[0], HEIGHTS_TO_CUT_AT[-1]]  # mm
HEIGHTS_TO_DISPLAY = [57 ,58]

print(f"{HEIGHTS_TO_CUT_AT=}")

# Let the water body be a 3x3 meters wide and long, 1 meter deep centered at the (0, 0, -.5)
BOX_SIZE_XY = 3000     # 3 m wide/long
BOX_DEPTH   = 1000     # 1 m depth

# Read the hull model
reader = STEPControl_Reader()
status = reader.ReadFile(HULL_STEP_FILE)
if status != 1:
    raise RuntimeError("Nie udało się wczytać pliku STEP.")
reader.TransferRoots()
hull = reader.OneShape()

# cleanup
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Copy
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
# Copy to detach transient references
hull = BRepBuilderAPI_Copy(hull).Shape()
# Merge coplanar/collinear faces (reduces complexity)
unifier = ShapeUpgrade_UnifySameDomain(hull, True, True, True)
unifier.Build()
hull = unifier.Shape()
# Pre-mesh to speed up Boolean ops (tolerance ~1 mm)
BRepMesh_IncrementalMesh(hull, 1.0)

if HEIGHTS_TO_DISPLAY:
    # Prepare a display
    display, start_display, add_menu, add_function_to_menu = init_display()

# Iterate over the HEIGHTS_TO_CUT_AT
for i, z in enumerate(HEIGHTS_TO_CUT_AT):
   
    # Create the water body at fixed place (water surface at z=0) xy center at (0,0)
    water = BRepPrimAPI_MakeBox(
        gp_Pnt(-BOX_SIZE_XY/2, -BOX_SIZE_XY/2, -BOX_DEPTH), # lower-left-bottom corner
        BOX_SIZE_XY, BOX_SIZE_XY, BOX_DEPTH                 # X, Y size, height
    ).Shape()

    # Place the hull model in corect place 
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(0, 0, z))  # move hull by z
    hull_shifted = BRepBuilderAPI_Transform(hull, trsf).Shape()

    # Intersect the two (submerged part)
    inter = BRepAlgoAPI_Common(hull_shifted, water).Shape()
    # Calculate the submerged volume, buoyancy etc.
    rho = 1000  # kg/m³
    g = 9.81    # m/s²
    props = GProp_GProps()
    brepgprop_VolumeProperties(inter, props)
    V = props.Mass() / 1e9          # mm³ → m³  (since OCC works in mm)
    com = props.CentreOfMass()
    F_b = V * rho * g                # buoyant force [N]
    print(f"z={z:>6.1f} mm | V={V:.6f} m³ | F_b={F_b:8.2f} N | COMz={com.Z()/1000:.3f} m")

    results.append({
        "z_mm": z,
        "volume_m3": V,
        "F_b_N": F_b,
        "CoB_x_m": com.X() / 1000,
        "CoB_y_m": com.Y() / 1000,
        "CoB_z_m": com.Z() / 1000,
    })

    if z in HEIGHTS_TO_SAVE:
        # Zapisz do pliku STEP dla podglądu
        writer = STEPControl_Writer()
        writer.Transfer(inter, STEPControl_AsIs)
        writer.Write(f"cut_{i}_z{z}.step")
        print(f"[OK] Zapisano przekrój do cut_{i}_z{z}.step")

    if z in HEIGHTS_TO_DISPLAY:
        # Create a small marker (5 cm radius = 50 mm)
        cob_marker = BRepPrimAPI_MakeSphere(gp_Pnt(com.X(), com.Y(), com.Z()), 8).Shape()

        display.EraseAll()
        ais_water = display.DisplayShape(water, update=False, color='BLUE1')[0]
        ais_water.SetTransparency(0.1)
        ais_hull = display.DisplayShape(hull_shifted, update=False, color='RED')[0]
        ais_hull.SetTransparency(0.2)
        ais_inter = display.DisplayShape(inter, update=False, color='RED')
        # ais_inter.SetTransparency(0.5)
        display.DisplayShape(cob_marker, update=False, color='YELLOW')
        display.FitAll()
        display.Repaint()
        input(f"Press Enter for next display...")

# # Fit view and start GUI once (outside loop)
# display.FitAll()
# start_display()

# Save results to CSV
with open("buoyancy_results.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=results[0].keys())
    writer.writeheader()
    writer.writerows(results)

print("[OK] Results saved to buoyancy_results.csv")

