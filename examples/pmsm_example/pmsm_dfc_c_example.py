"""
pmsm_c_dfc_only.py  -  Minimal simulation using C DFC controller only.
                       Use this to isolate and test the C closed‑loop implementation.
"""

from __future__ import annotations

import sys
import math
from pathlib import Path

# Path setup
from _path_utils import get_project_root, get_embedsim_import_path, get_current_parent, get_pmsm_path, get_pmsm_c_src_path, get_modelica_path

_HERE = get_current_parent()
_ROOT = get_project_root()
_PMSM = get_pmsm_path()
_C_SRC = get_pmsm_c_src_path()

for _p in (get_embedsim_import_path(), str(_PMSM), str(_C_SRC)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from embedsim import EmbedSim, ODESolver, VectorEnd
from embedsim.source_blocks import VectorStep
from embedsim.plot_helper import create_plotter

# Plant
from pmsm_python_plant import PMSM_Python_Plant

# Helpers
from embedsim_connections import CtrlPacker, LoadAdapter, MotorVectorDelay, SignalPrinter

# C controller
from embedsim_control_block import EmbedSimControlBlock, SIM_CTRL_DFC

# =============================================================================
# Simulation Parameters
# =============================================================================

T_SIM = 10.0
DT = 50e-6
V_DC = 12.0
TARGET_RPM =  850.0
STEP_TIME = 0.01

# Motor parameters (Python plant)
R_S = 0.19
L_D = 0.125e-3
L_Q = 0.125e-3
LAMBDA_PM = 0.0014
J_ROTOR = 2.4e-6
B_FRIC = 1.0e-6
P_POLES = 4.0

# =============================================================================
# Build blocks
# =============================================================================

# Speed reference step
speed_ref = VectorStep(
    "speed_ref",
    step_time=STEP_TIME,
    before_value=0.0,
    after_value=TARGET_RPM,
    dim=1
)

# Motor plant
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
motor_out_size = 8

# Debug printer for motor state (prints every 100 steps)
motor_fields = ["rpm", "ia", "ib", "ic", "theta_m", "Tem", "id", "iq"]
printer = SignalPrinter(
    name="motor_debug",
    fields=motor_fields,
    print_prefix="Motor State: ",
    every_n=10000          # print once every 100 calls (5 ms at 50 µs)
)

# C DFC controller
ctrl = EmbedSimControlBlock(
    name="ctrl",
    dt_s=DT,
    ctrl_alg=SIM_CTRL_DFC,
    vdc_nom=V_DC,
    use_c_backend=True,
)

# Packer, adapter, delay, sink
valid_flag = 1
ctrl_packer = CtrlPacker("ctrl_packer", vdc=V_DC, valid_flag=valid_flag)
load_adapter = LoadAdapter("load_adapter", vdc=V_DC, tload=0.0)
motor_delay = MotorVectorDelay("motor_delay", vector_size=motor_out_size)
sink = VectorEnd("sink")

# =============================================================================
# Connections
# =============================================================================

speed_ref   >> ctrl_packer
motor_delay >> ctrl_packer
ctrl_packer >> ctrl >> load_adapter >> motor >> motor_delay >> printer >> sink
# (Optional) If you want to also log the raw motor output, you could add another sink,
# but printer already carries the delayed signal which is the same except for one step.

# =============================================================================
# Simulation
# =============================================================================

sim = EmbedSim(
    sinks=[sink],           # sink now receives the printer output (delayed motor state)
    T=T_SIM,
    dt=DT,
    solver=ODESolver.EULER
)

# Scope: use the block names as labels (they become the keys for plotting)
sim.scope.add(speed_ref, indices=[0], label=speed_ref.name)   # key "speed_ref[0]"
sim.scope.add(motor, indices=[0], label=motor.name)           # key "motor[0]"

print("\n" + "="*60)
print(" MINIMAL C DFC SIMULATION")
print("="*60)
print(f" Target: {TARGET_RPM} RPM")
print(f" Time: {T_SIM}s, dt={DT*1e6:.0f}µs")
print(f" Controller: C_DFC")
print(f" Plant: Python PMSM")
print("="*60 + "\n")

# Run
sim.run(progress_bar=True)

# =============================================================================
# Plot RPM
# =============================================================================

ph = create_plotter(sim)
# Now use the correct keys (block names)
ph.easyplot(
    [f"{speed_ref.name}[0]", f"{motor.name}[0]"],
    title="Speed Control - C DFC (Python Plant)",
    time_range=(0, T_SIM),
    figsize=(10, 4),
    save_path=None
)

# =============================================================================
# Quick summary
# =============================================================================

sc = sim.scope
speed_data = sc.get_signal(motor.name, 0)  # use block name
if speed_data is not None and len(speed_data) > 0:
    final_speed = speed_data[-1]
    steady_start = int(len(speed_data) * 0.9)
    steady_speed = np.mean(speed_data[steady_start:])
    steady_std = np.std(speed_data[steady_start:])
    print("\n" + "="*60)
    print(" C DFC SUMMARY")
    print("="*60)
    print(f" Final speed: {final_speed:.1f} RPM")
    print(f" Steady-state: {steady_speed:.1f} ± {steady_std:.1f} RPM")
    print(f" Error: {steady_speed - TARGET_RPM:+.1f} RPM")
    print("="*60)

plt.show()