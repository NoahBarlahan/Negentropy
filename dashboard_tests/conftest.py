from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def flat_defaults():
    """Keep the existing flat-foot engineering regressions unchanged."""
    from config import defaults
    c=defaults()
    c.update(foot_model='Flat sole',L_foot_com_z=-3.,R_foot_com_z=-3.)
    return c
