"""
pmsm_dfc_example.py  -  PMSM Control Example with Mode Switching
                       Supports:
                         - Python open‑loop / DFC controller
                         - C open‑loop / DFC controller
                         - Python plant / FMU plant
                       Clean, minimal, and easy to modify.
"""

from __future__ import annotations

import sys
from pathlib import Path

# ================================================================
# Path setup (use the provided _path_utils)
# ================================================================
from _path_utils import (
    get_project_root,
    get_embedsim_import_path,
    get_current_parent,
    get_pmsm_path,
    get_pmsm_c_src_path,
    get_modelica_path,
)

_HERE = get_current_parent()
_ROOT = get_project_root()
_PMSM = get_pmsm_path()
_C_SRC = get_pmsm_c_src_path()

for _p in (get_embedsim_import_path(), str(_PMSM), str(_C_SRC)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# FMU path (used only if PLANT_MODE == "FMU")
FMU_PATH = get_modelica_path("PMSM_Plant_FMU.fmu")

# ================================================================
# Imports
# ================================================================
import numpy as np
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from embedsim import EmbedSim, ODESolver, VectorEnd
from embedsim.source_blocks import VectorStep
from embedsim.plot_helper import create_plotter

# Plant models
from pmsm_python_plant import PMSM_Python_Plant
from PMSM_Plant_FMUBlock import PMSM_Plant_FMUBlock

# Helpers
from embedsim_connections import CtrlPacker, LoadAdapter, MotorVectorDelay

# Python controller
from pmsm_dfc import PythonController

# C controller
from embedsim_control_block import EmbedSimControlBlock, SIM_CTRL_OPEN_LOOP, SIM_CTRL_DFC

# =============================================================================
# CONFIGURATION – CHANGE THESE TO SWITCH MODES
# =============================================================================
CONTROLLER_MODE = "PYTHON_DFC"      # Options: PYTHON_OPEN_LOOP, PYTHON_DFC,
                                    #          C_OPEN_LOOP, C_DFC
PLANT_MODE = "FMU"                  # Options: PYTHON, FMU

# Simulation parameters
T_SIM = 10.0
DT = 50e-6
V_DC = 12.0
TARGET_RPM = 850.0
STEP_TIME = 0.5

# Motor parameters (used only with Python plant)
R_S = 0.19
L_D = 0.125e-3
L_Q = 0.125e-3
LAMBDA_PM = 0.0014
J_ROTOR = 2.4e-6
B_FRIC = 1.0e-6
P_POLES = 4.0

# FMU‑specific tuning (used only with FMU plant)
FMU_SPINNING_PAST_INDEX = 500
FMU_STOPPED_PAST_INDEX = 100
FMU_SETTLING_TIME = 2.0

# =============================================================================
# Plant factory
# =============================================================================
def create_plant(mode: str):
    if mode == "PYTHON":
        motor = PMSM_Python_Plant(
            name="motor",
            R=R_S,
            L_d=L_D,
            L_q=L_Q,
            lambda_pm=LAMBDA_PM,
            J=J_ROTOR,
            B_fric=B_FRIC,
            p=P_POLES,
            v_dc=V_DC,
        )
        print("[Plant] Python PMSM model")
    elif mode == "FMU":
        motor = PMSM_Plant_FMUBlock(
            name="motor",
            fmu_path=FMU_PATH,
        )
        print(f"[Plant] FMU model: {FMU_PATH}")
    else:
        raise ValueError(f"Unknown PLANT_MODE: {mode}")
    return motor

# =============================================================================
# Controller factory
# =============================================================================
def create_controller(mode: str):
    if mode == "PYTHON_OPEN_LOOP":
        ctrl = PythonController(
            name="ctrl",
            dt_s=DT,
            vdc_nom=V_DC,
            controller_mode="OPEN_LOOP",
            pole_pairs=P_POLES,
            rs=R_S,
            ld=L_D,
            lq=L_Q,
            lambda_pm=LAMBDA_PM,
        )
        label = "Python_OpenLoop"
    elif mode == "PYTHON_DFC":
        ctrl = PythonController(
            name="ctrl",
            dt_s=DT,
            vdc_nom=V_DC,
            controller_mode="DFC",
            pole_pairs=P_POLES,
            rs=R_S,
            ld=L_D,
            lq=L_Q,
            lambda_pm=LAMBDA_PM,
            j=J_ROTOR,
            b=B_FRIC,
            kp_speed=0.0039,
            ki_speed=0.0002,
            kp_d=0.0001,
            kp_q=0.0195,
            ki_d=0.0005,
            ki_q=0.0002,
            integral_limit=25.0,
            max_current=100.0,
            max_iq_dot=1000.0,
            startup_mod_min=0.05,
            startup_mod_max=0.25,
            startup_increment=0.001,
            spinning_past_index=(FMU_SPINNING_PAST_INDEX if PLANT_MODE == "FMU" else 89500),
            stopped_past_index=(FMU_STOPPED_PAST_INDEX if PLANT_MODE == "FMU" else 2000),
            use_python=True,
            debug=False,
        )
        label = "Python_DFC"
    elif mode == "C_OPEN_LOOP":
        ctrl = EmbedSimControlBlock(
            name="ctrl",
            dt_s=DT,
            ctrl_alg=SIM_CTRL_OPEN_LOOP,
            vdc_nom=V_DC,
            use_c_backend=True,
        )
        label = "C_OpenLoop"
    elif mode == "C_DFC":
        ctrl = EmbedSimControlBlock(
            name="ctrl",
            dt_s=DT,
            ctrl_alg=SIM_CTRL_DFC,
            vdc_nom=V_DC,
            use_c_backend=True,
        )
        label = "C_DFC"
    else:
        raise ValueError(f"Unknown CONTROLLER_MODE: {mode}")
    return ctrl, label

# =============================================================================
# Build blocks and simulation
# =============================================================================
# Speed reference
speed_ref = VectorStep(
    "speed_ref",
    step_time=STEP_TIME,
    before_value=0.0,
    after_value=TARGET_RPM,
    dim=1,
)

# Plant
motor = create_plant(PLANT_MODE)

# Controller
ctrl, ctrl_label = create_controller(CONTROLLER_MODE)

# Utility blocks
valid_flag = 1
ctrl_packer = CtrlPacker("ctrl_packer", vdc=V_DC, valid_flag=valid_flag)
load_adapter = LoadAdapter("load_adapter", vdc=V_DC, tload=0.0)

# --- FIX: Use constant output size (both plant types output 8 signals) ---
MOTOR_OUT_SIZE = 8
motor_delay = MotorVectorDelay("motor_delay", vector_size=MOTOR_OUT_SIZE)

sink = VectorEnd("sink")

# Connections
speed_ref   >> ctrl_packer
motor_delay >> ctrl_packer
ctrl_packer >> ctrl
ctrl        >> load_adapter
load_adapter >> motor
motor       >> motor_delay
motor_delay >> sink

# Simulation
sim = EmbedSim(
    sinks=[sink],
    T=T_SIM,
    dt=DT,
    solver=ODESolver.EULER,
)

# Add signals to scope (use block names as keys)
sim.scope.add(speed_ref, indices=[0], label=speed_ref.name)   # key "speed_ref[0]"
sim.scope.add(motor, indices=[0], label=motor.name)           # key "motor[0]" (RPM)

# =============================================================================
# Run simulation
# =============================================================================
print("\n" + "=" * 60)
print(f" PMSM CONTROL SIMULATION")
print("=" * 60)
print(f" Controller : {CONTROLLER_MODE} ({ctrl_label})")
print(f" Plant      : {PLANT_MODE}")
print(f" Target RPM : {TARGET_RPM}")
print(f" Simulation : {T_SIM}s, dt = {DT*1e6:.0f} µs")
print("=" * 60 + "\n")

sim.run(progress_bar=True)

# =============================================================================
# Plot RPM response
# =============================================================================
ph = create_plotter(sim)
plot_title = f"Speed Control – {ctrl_label} ({PLANT_MODE} Plant)"
ph.easyplot(
    [f"{speed_ref.name}[0]", f"{motor.name}[0]"],
    title=plot_title,
    time_range=(0, T_SIM),
    figsize=(10, 4),
    save_path=None,   # set a path here if you want to save the figure
)

# =============================================================================
# Summary
# =============================================================================
sc = sim.scope
speed_data = sc.get_signal(motor.name, 0)
if speed_data is not None and len(speed_data) > 0:
    final_speed = speed_data[-1]
    steady_start = int(len(speed_data) * 0.9)
    steady_speed = np.mean(speed_data[steady_start:])
    steady_std = np.std(speed_data[steady_start:])
    print("\n" + "=" * 60)
    print(f" {ctrl_label} SUMMARY ({PLANT_MODE} Plant)")
    print("=" * 60)
    print(f" Final speed:        {final_speed:.1f} RPM")
    print(f" Steady‑state speed: {steady_speed:.1f} ± {steady_std:.1f} RPM")
    print(f" Error:              {steady_speed - TARGET_RPM:+.1f} RPM")
    print("=" * 60)

plt.show()