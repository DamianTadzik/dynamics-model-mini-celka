import subprocess
import re

XFOIL_PATH = r"XFOIL6.99/xfoil.exe"

TIMEOUT = 10
NCRIT = 9

cases = [
    # (12, 0.05)
    # thickness, chord [m]
    (24, 0.0350), 
    (28, 0.0372), 
    (31, 0.0390), 
    (34, 0.0416), 
    (38, 0.0449), 
    (43, 0.0500),
    
    # ALL OF THEM
    # (24, 0.0350),
    # # (24, 0.0350),
    # # (24, 0.0351),
    # # (24, 0.0352),
    # # (25, 0.0354),
    # # (25, 0.0357),
    # (26, 0.0360),
    # (27, 0.0365),
    # (28, 0.0372),
    # (29, 0.0380),
    # (31, 0.0390),
    # (32, 0.0402),
    # (34, 0.0416),
    # (36, 0.0432),
    # (38, 0.0449),
    # (40, 0.0465),
    # (41, 0.0478),
    # (42, 0.0488),
    # (42, 0.0494),
    # # (43, 0.0498),
    # # (43, 0.0498),
    # (43, 0.0500),
]

velocities = [2.0, 2.5, 3.0]

NU = 1.0e-6


def run_xfoil(thickness, Re):

    naca = f"00{thickness:02d}"

#     commands = f"""NACA {naca}
# PANE
# OPER
# VISC {Re:.0f}
# ITER 300
# VPAR
# N {NCRIT}
# VACC 0
# 
# INIT
# ALFA 0
# QUIT
# """
    commands = f"""NACA {naca}
PANE
OPER
VISC {Re:.0f}
ITER 300
ALFA 0
QUIT
"""
    process = subprocess.Popen(
        [XFOIL_PATH],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    try:
        stdout, _ = process.communicate(
            input=commands,
            timeout=TIMEOUT
        )

    except subprocess.TimeoutExpired:

        process.kill()
        stdout, _ = process.communicate()
    
        print(f"\n{'='*70}")
        print(f"NACA {naca}, Re={Re:.0f}: TIMEOUT")
        print("XFOIL OUTPUT:")
        print(stdout)
        print(f"{'='*70}\n")
        return None

    cds = re.findall(
        r"CD\s*=\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?)",
        stdout
    )

    cls = re.findall(
        r"CL\s*=\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?)",
        stdout
    )

    if not cds or not cls:
        print(
            f"NACA {naca}, Re={Re:.0f}: NO RESULT"
        )
        return None

    cd = float(cds[-1])
    cl = float(cls[-1])

    if abs(cl) > 0.01:
        print(
            f"NACA {naca}, Re={Re:.0f}: "
            f"SUSPICIOUS CL={cl:.5f}"
        )
        return None

    return cd, cl


results = []

for thickness, chord in cases:

    for velocity in velocities:

        Re = velocity * chord / NU

        result = run_xfoil(thickness, Re)

        if result is None:
            cd = None
            cl = None
        else:
            cd, cl = result

        results.append(
            (thickness, chord, velocity, Re, cd, cl)
        )

        print(
            f"NACA 00{thickness:02d} | "
            f"V={velocity:.1f} m/s | "
            f"Re={Re:.0f} | "
            f"CD={cd} | CL={cl}"
        )