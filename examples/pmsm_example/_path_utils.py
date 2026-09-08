"""
_path_utils.py
==============

Shared path utility for the pmsm_blocks library.

Every module in this package calls `setup_embedsim_path()` at import time
so the embedsim package is always locatable regardless of how the script
is launched or where the working directory is.

The project root is identified by the presence of a `.project_root_marker`
file in one of the parent directories.
"""

import sys
from pathlib import Path


def get_project_root() -> Path:
    current_path = Path(__file__).resolve()

    # Walk up the directory tree looking for the marker file
    for parent in current_path.parents:
        if (parent / ".project_root_marker").exists():
            return parent

    # Fallback: assume project root is two levels up
    return current_path.parent.parent


def get_embedsim_import_path() -> str:
    """
    Returns the path that should be added to sys.path
    so that 'import embedsim' works.
    """
    return str(get_project_root())


def get_modelica_path(name: str) -> str:
    """
    Returns full path to a Modelica file in examples/rlc_fmu/modelica.
    """
    root = get_project_root()
    return str(get_pmsm_path() / "modelica" / name)

def get_current_parent() -> Path:
    """
    Returns current parent directory of the current Python process.
    """
    current_path = Path(__file__).resolve()
    return current_path.parent



def get_pmsm_path() -> Path:
    """
    Returns pmsm file path for the current Python process.
    """
    return get_project_root() / "pmsm"

def get_pmsm_c_src_path() -> Path:
    """
    Returns pmsm c_src file path for the current Python process.
    """
    return get_pmsm_path() / "c_src"
