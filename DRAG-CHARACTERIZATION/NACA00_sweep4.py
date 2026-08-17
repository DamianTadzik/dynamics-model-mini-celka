import subprocess
import re

XFOIL_PATH = r"XFOIL6.99/xfoil.exe"

TIMEOUT = 30
NU = 1.0e-6

# Jeden profil = jedna sesja XFOIL
cases = [
    # thickness [%], chord [m]
    # (24, 0.0350),
    # (28, 0.0372),
    # (31, 0.0390),
    # (34, 0.0416),
    # (38, 0.0449),
    # (43, 0.0500),
    
    # ALL OF THEM
    (24, 0.0350),
    # (24, 0.0350),
    # (24, 0.0351),
    # (24, 0.0352),
    # (25, 0.0354),
    # (25, 0.0357),
    (26, 0.0360),
    (27, 0.0365),
    (28, 0.0372),
    (29, 0.0380),
    (31, 0.0390),
    (32, 0.0402),
    (34, 0.0416),
    (36, 0.0432),
    (38, 0.0449),
    (40, 0.0465),
    (41, 0.0478),
    (42, 0.0488),
    (42, 0.0494),
    # (43, 0.0498),
    # (43, 0.0498),
    (43, 0.0500),
]

# Celowo od największego Re do najmniejszego
velocities = [4.0, 3.5, 3.0, 2.5, 2.0]


def extract_last_solution(block):
    """
    Extract final CL and CD from one completed ALFA 0 calculation.
    """

    matches = re.findall(
        r"a\s*=\s*[-+0-9.Ee]+\s+CL\s*=\s*([-+0-9.Ee]+)"
        r".*?"
        r"CD\s*=\s*([-+0-9.Ee]+)",
        block,
        flags=re.DOTALL
    )

    if not matches:
        return None

    cl, cd = matches[-1]

    return float(cd), float(cl)


def run_xfoil(thickness, chord):

    naca = f"00{thickness:02d}"

    reynolds = [
        velocity * chord / NU
        for velocity in velocities
    ]

    print("\n" + "=" * 70)
    print(f"NACA {naca}")
    print("=" * 70)

    # ---------------------------------------------------------
    # Build one XFOIL session for the entire profile
    # ---------------------------------------------------------

    commands = f"""NACA {naca}
PANE
OPER
VISC {reynolds[0]:.0f}
ITER 300
"""

    for i, Re in enumerate(reynolds):

        if i > 0:
            commands += f"RE {Re:.0f}\n"

        commands += """ALFA 0
CPMN
"""

    # Empty line leaves OPER menu, then QUIT exits XFOIL
    commands += """
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

        print(
            f"NACA {naca}: TIMEOUT after {TIMEOUT} s"
        )

        print("\nXFOIL OUTPUT:")
        print(stdout)

        return None

    # ---------------------------------------------------------
    # Each CPMN produces:
    #
    # Minimum Inviscid Cp = ...
    # Minimum Viscous Cp  = ...
    #
    # This reliably marks the end of one operating point.
    # ---------------------------------------------------------

    marker = "Minimum Inviscid Cp"

    parts = stdout.split(marker)

    # part[0] contains everything before first CPMN.
    # Each subsequent part starts after one completed ALFA.
    if len(parts) - 1 != len(velocities):

        print(
            f"NACA {naca}: expected {len(velocities)} "
            f"completed operating points, got {len(parts) - 1}"
        )

        print("\nXFOIL OUTPUT:")
        print(stdout)

        return None

    # The result corresponding to each CPMN is located BEFORE
    # the marker, therefore reconstruct the operating-point blocks.
    blocks = []

    start = 0

    for _ in velocities:

        marker_pos = stdout.find(marker, start)

        if marker_pos == -1:
            break

        block = stdout[start:marker_pos]

        blocks.append(block)

        start = marker_pos + len(marker)

    if len(blocks) != len(velocities):

        print(
            f"NACA {naca}: parsing failed, "
            f"expected {len(velocities)} blocks, got {len(blocks)}"
        )

        return None

    # ---------------------------------------------------------
    # Extract final solution from every operating point
    # ---------------------------------------------------------

    profile_results = []

    for velocity, Re, block in zip(
        velocities,
        reynolds,
        blocks
    ):

        solution = extract_last_solution(block)

        if solution is None:

            profile_results.append({
                "thickness": thickness,
                "chord": chord,
                "velocity": velocity,
                "Re": Re,
                "CD": None,
                "CL": None,
                "valid": False,
                "reason": "no final CL/CD found"
            })

            continue

        cd, cl = solution

        valid = True
        reasons = []

        # Symmetric NACA00xx at alpha=0 should give CL ~= 0
        if abs(cl) > 0.01:
            valid = False
            reasons.append(
                f"|CL|={abs(cl):.5f} > 0.01"
            )

        if cd <= 0.0 or cd > 0.2:
            valid = False
            reasons.append(
                f"suspicious CD={cd:.5f}"
            )

        profile_results.append({
            "thickness": thickness,
            "chord": chord,
            "velocity": velocity,
            "Re": Re,
            "CD": cd,
            "CL": cl,
            "valid": valid,
            "reason": ", ".join(reasons)
        })

    return profile_results


# =============================================================
# MAIN
# =============================================================

all_results = []

for thickness, chord in cases:

    results = run_xfoil(
        thickness,
        chord
    )

    if results is None:
        continue

    all_results.extend(results)

    for result in results:

        if result["CD"] is None:

            print(
                f"NACA 00{result['thickness']:02d} | "
                f"V={result['velocity']:.1f} m/s | "
                f"Re={result['Re']:.0f} | "
                f"NO RESULT | "
                f"{result['reason']}"
            )

            continue

        status = "OK"

        if not result["valid"]:
            status = f"INVALID ({result['reason']})"

        print(
            f"NACA 00{result['thickness']:02d} | "
            f"V={result['velocity']:.1f} m/s | "
            f"Re={result['Re']:.0f} | "
            f"CD={result['CD']:.5f} | "
            f"CL={result['CL']:.5f} | "
            f"{status}"
        )


# =============================================================
# FINAL SUMMARY
# =============================================================

print()
print("=" * 100)
print("FINAL RESULTS")
print("=" * 100)

print(
    f"{'NACA':<10}"
    f"{'V [m/s]':>10}"
    f"{'Re':>12}"
    f"{'CD':>12}"
    f"{'CL':>12}"
    f"{'STATUS':>15}"
)

for result in all_results:

    naca_name = f"00{result['thickness']:02d}"

    if result["CD"] is None:

        print(
            f"{naca_name:<10}"
            f"{result['velocity']:>10.1f}"
            f"{result['Re']:>12.0f}"
            f"{'---':>12}"
            f"{'---':>12}"
            f"{'FAILED':>15}"
        )

        continue

    status = "OK" if result["valid"] else "INVALID"

    print(
        f"{naca_name:<10}"
        f"{result['velocity']:>10.1f}"
        f"{result['Re']:>12.0f}"
        f"{result['CD']:>12.5f}"
        f"{result['CL']:>12.5f}"
        f"{status:>15}"
    )

# =============================================================
# SAVE RESULTS TO CSV
# =============================================================
import csv
CSV_FILE = "xfoil_results.csv"

with open(CSV_FILE, "w", newline="") as f:

    writer = csv.writer(f)

    writer.writerow([
        "NACA",
        "thickness_percent",
        "chord_m",
        "velocity_ms",
        "Re",
        "CD",
        "CL",
        "valid"
    ])

    for result in all_results:

        writer.writerow([
            f"00{result['thickness']:02d}",
            result["thickness"],
            result["chord"],
            result["velocity"],
            result["Re"],
            result["CD"],
            result["CL"],
            int(result["valid"])
        ])

print(f"\nResults saved to: {CSV_FILE}")
