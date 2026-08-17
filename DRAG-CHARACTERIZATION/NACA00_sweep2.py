import subprocess
import re

XFOIL_PATH = r"XFOIL6.99/xfoil.exe"

TIMEOUT = 30
NU = 1.0e-6

# Jeden profil = jedna sesja XFOIL
cases = [
    # thickness [%], chord [m]
    (24, 0.0350), 
    (28, 0.0372), 
    (31, 0.0390), 
    (34, 0.0416), 
    (38, 0.0449), 
    (43, 0.0500),
]

# Celowo od największego Re do najmniejszego
velocities = [4.0, 3.5, 3.0, 2.5, 2.0]


def run_xfoil(thickness, chord):

    naca = f"00{thickness:02d}"

    reynolds = [
        velocity * chord / NU
        for velocity in velocities
    ]

    print("\n" + "=" * 70)
    print(f"NACA {naca}")
    print("=" * 70)

    # Pierwszy Reynolds uruchamia viscous mode.
    commands = f"""NACA {naca}
PANE
OPER
VISC {reynolds[0]:.0f}
ITER 300
ALFA 0
"""

    # Kolejne Reynoldsy są liczone w TEJ SAMEJ sesji.
    for Re in reynolds[1:]:
        commands += f"""RE {Re:.0f}
ALFA 0
"""

    commands += "QUIT\n"

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
    # Extract CL and CD values from all calculated operating points
    # ---------------------------------------------------------

    cl_matches = re.findall(
        r"CL\s*=\s*"
        r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)"
        r"(?:[Ee][+-]?\d+)?)",
        stdout
    )

    cd_matches = re.findall(
        r"CD\s*=\s*"
        r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)"
        r"(?:[Ee][+-]?\d+)?)",
        stdout
    )

    cls = [float(x) for x in cl_matches]
    cds = [float(x) for x in cd_matches]

    expected = len(velocities)

    if len(cls) < expected or len(cds) < expected:

        print(
            f"NACA {naca}: expected {expected} results, "
            f"got CL={len(cls)}, CD={len(cds)}"
        )

        print("\nXFOIL OUTPUT:")
        print(stdout)

        return None

    # Bierzemy ostatnie N wyników, odpowiadające naszym ALFA 0.
    cls = cls[-expected:]
    cds = cds[-expected:]

    profile_results = []

    for velocity, Re, cd, cl in zip(
        velocities,
        reynolds,
        cds,
        cls
    ):

        valid = True
        reason = ""

        # Symetryczny NACA00xx przy alpha = 0 powinien mieć CL ~ 0.
        if abs(cl) > 0.01:
            valid = False
            reason = f"|CL|={abs(cl):.5f} > 0.01"

        # Dodatkowy sanity check.
        if cd <= 0.0 or cd > 0.2:
            valid = False
            reason = f"suspicious CD={cd:.5f}"

        profile_results.append({
            "thickness": thickness,
            "chord": chord,
            "velocity": velocity,
            "Re": Re,
            "CD": cd,
            "CL": cl,
            "valid": valid,
            "reason": reason
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

print("\n")
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

    status = "OK" if result["valid"] else "INVALID"

    print(
        f"00{result['thickness']:02d:<6}"
        f"{result['velocity']:>10.1f}"
        f"{result['Re']:>12.0f}"
        f"{result['CD']:>12.5f}"
        f"{result['CL']:>12.5f}"
        f"{status:>15}"
    )