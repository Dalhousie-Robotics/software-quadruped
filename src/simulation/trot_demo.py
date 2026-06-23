"""
trot_demo.py -- Open-loop trot gait on the Unitree A1 model.

The A1 uses position actuators (kp=100 built in), so we write target
joint angles directly to data.ctrl -- no separate PD controller needed.

Trot pattern: diagonal pairs move together.
  FL + RR share one phase.
  FR + RL share the opposite phase (offset by pi).

Usage:
    python src/simulation/trot_demo.py

Controls:
    Space       Pause / unpause
    Backspace   Reset
    Esc         Quit
"""

import os
import numpy as np
import mujoco
import mujoco.viewer

SCENE_PATH = os.path.join(os.path.dirname(__file__), "models", "unitree_a1", "scene.xml")

# ---------------------------------------------------------------------------
# Standing pose (matches the "home" keyframe in a1.xml)
# ---------------------------------------------------------------------------
STAND_HIP   =  0.0   # abduction/adduction -- 0 = straight
STAND_THIGH =  0.9   # hip flexion  (rad)
STAND_CALF  = -1.8   # knee flexion (rad, negative = bent)

# ---------------------------------------------------------------------------
# Gait parameters -- tune here
# ---------------------------------------------------------------------------
GAIT_FREQ   = 1.4    # Hz -- step frequency
THIGH_AMP   = 0.28   # rad -- how far each thigh swings per step
CALF_LIFT   = 0.50   # rad -- extra knee bend at peak of swing (foot height)
WARMUP_SECS = 1.5    # seconds to hold standing before walking starts

# ---------------------------------------------------------------------------
# Phase offsets for trot (diagonal pairs)
# ---------------------------------------------------------------------------
PHASE_OFFSET = {
    "FL": 0.0,
    "RR": 0.0,
    "FR": np.pi,
    "RL": np.pi,
}

# ---------------------------------------------------------------------------
# Actuator name lookup (matches a1.xml <actuator> block)
# ---------------------------------------------------------------------------
ACT_NAMES = {
    "FL": ("FL_hip", "FL_thigh", "FL_calf"),
    "FR": ("FR_hip", "FR_thigh", "FR_calf"),
    "RL": ("RL_hip", "RL_thigh", "RL_calf"),
    "RR": ("RR_hip", "RR_thigh", "RR_calf"),
}


def build_act_index(model) -> dict:
    """Return a dict: actuator_name -> ctrl index."""
    idx = {}
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name:
            idx[name] = i
    return idx


def set_stand(data, act_idx: dict) -> None:
    """Write the standing pose to all actuators."""
    for leg, (hip, thigh, calf) in ACT_NAMES.items():
        data.ctrl[act_idx[hip]]   = STAND_HIP
        data.ctrl[act_idx[thigh]] = STAND_THIGH
        data.ctrl[act_idx[calf]]  = STAND_CALF


def set_trot(data, act_idx: dict, gait_phase: float) -> None:
    """Write trot joint targets to all actuators."""
    for leg, (hip_name, thigh_name, calf_name) in ACT_NAMES.items():
        phase = (gait_phase + PHASE_OFFSET[leg]) % (2.0 * np.pi)

        # Thigh: sinusoidal swing.
        #   sin > 0 (swing, 0->pi):    thigh moves forward (increases, repositioning foot)
        #   sin < 0 (stance, pi->2pi): thigh moves backward (decreases, pushes body forward)
        thigh = STAND_THIGH + THIGH_AMP * np.sin(phase)

        # Calf: lift during swing only.
        swing_factor = max(0.0, np.sin(phase))   # 0 during stance, bell curve during swing
        calf = STAND_CALF - CALF_LIFT * swing_factor

        data.ctrl[act_idx[hip_name]]   = STAND_HIP
        data.ctrl[act_idx[thigh_name]] = thigh
        data.ctrl[act_idx[calf_name]]  = calf


def main() -> None:
    print("Loading Unitree A1 model...")
    model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    data  = mujoco.MjData(model)

    # Start from the home keyframe
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    act_idx = build_act_index(model)

    print(f"Trot gait: freq={GAIT_FREQ} Hz  thigh_amp={THIGH_AMP} rad  calf_lift={CALF_LIFT} rad")
    print(f"Standing for {WARMUP_SECS}s then walking starts.")
    print("Space=pause  Backspace=reset  Esc=quit")

    sim_time = 0.0
    dt       = model.opt.timestep

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

            if sim_time < WARMUP_SECS:
                set_stand(data, act_idx)
            else:
                walk_time  = sim_time - WARMUP_SECS
                gait_phase = (2.0 * np.pi * GAIT_FREQ * walk_time) % (2.0 * np.pi)
                set_trot(data, act_idx, gait_phase)

            mujoco.mj_step(model, data)
            sim_time += dt
            viewer.sync()


if __name__ == "__main__":
    main()
