#!/usr/bin/env python3
"""
can_frame_visualizer.py -- MIT Mini Cheetah CAN frame encoder and visualizer.

YOUR TASK: Complete the encode_frame() function and the plot_frame() function.

The float_to_uint helper and all parameter ranges are already provided.
Read the comments carefully -- they tell you exactly what each part should do.

Requirements:
    pip install matplotlib

Run with:
    python scripts/can_frame_visualizer.py
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# -----------------------------------------------------------------------
# AK40-10 parameter ranges (MIT Mini Cheetah protocol)
# These are the physical limits of the motor. Do not change them.
# -----------------------------------------------------------------------
P_MIN,  P_MAX  = -12.5, 12.5   # position, rad
V_MIN,  V_MAX  = -65.0, 65.0   # velocity, rad/s
KP_MIN, KP_MAX =   0.0, 500.0  # position gain, Nm/rad
KD_MIN, KD_MAX =   0.0,   5.0  # damping gain, Nm-s/rad
T_MIN,  T_MAX  = -18.0,  18.0  # torque, Nm


def float_to_uint(value, v_min, v_max, bits):
    """
    Map a float in [v_min, v_max] to an unsigned integer with `bits` bits.
    This is provided for you -- you will call it inside encode_frame().

    Example:
        float_to_uint(0.0, -12.5, 12.5, 16) returns 32767  (midpoint)
        float_to_uint(12.5, -12.5, 12.5, 16) returns 65535 (maximum)
    """
    value = max(v_min, min(v_max, value))
    span  = v_max - v_min
    return int((value - v_min) * ((1 << bits) - 1) / span)


def encode_frame(pos, vel, kp, kd, torque):
    """
    YOUR TASK: Encode the inputs into an 8-byte MIT CAN frame.

    Return a list of 8 integers, each between 0 and 255.

    Step 1: Convert each float to an integer using float_to_uint().
        pos   -> 16 bits  (use P_MIN,  P_MAX)
        vel   -> 12 bits  (use V_MIN,  V_MAX)
        kp    -> 12 bits  (use KP_MIN, KP_MAX)
        kd    -> 12 bits  (use KD_MIN, KD_MAX)
        torque -> 12 bits (use T_MIN,  T_MAX)

    Step 2: Pack those integers into 8 bytes like this:
        frame[0] = upper 8 bits of pos
        frame[1] = lower 8 bits of pos
        frame[2] = upper 8 bits of vel  (vel >> 4)
        frame[3] = lower 4 bits of vel shifted up, upper 4 bits of kp
        frame[4] = lower 8 bits of kp
        frame[5] = upper 8 bits of kd   (kd >> 4)
        frame[6] = lower 4 bits of kd shifted up, upper 4 bits of torque
        frame[7] = lower 8 bits of torque

    Hint: use >> to shift right, & 0xFF to keep only 8 bits,
          & 0xF to keep only 4 bits, << 4 to shift left by 4.
    """
    # TODO: call float_to_uint for each value
    p_int  = 0   # replace with float_to_uint(...)
    v_int  = 0   # replace with float_to_uint(...)
    kp_int = 0   # replace with float_to_uint(...)
    kd_int = 0   # replace with float_to_uint(...)
    t_int  = 0   # replace with float_to_uint(...)

    # TODO: pack the integers into the 8-byte frame
    frame = [0] * 8
    # frame[0] = ...
    # frame[1] = ...
    # frame[2] = ...
    # frame[3] = ...
    # frame[4] = ...
    # frame[5] = ...
    # frame[6] = ...
    # frame[7] = ...

    return frame


def plot_frame(pos, vel, kp, kd, torque):
    """
    YOUR TASK: Call encode_frame() and plot the result.

    The graph should have:
    - A bar chart with 8 bars, one per byte, showing the byte value (0-255)
    - Each bar labelled with what it represents (pos, vel, kp, kd, torque)
    - The full hex frame printed as text below the chart

    Hints:
    - Use matplotlib.pyplot (imported as plt)
    - plt.bar() makes a bar chart
    - plt.show() displays it
    - f"0x{value:02X}" formats a number as hex (e.g. 0x4F)
    - Use different colors for each field so it is easy to read
    """
    frame = encode_frame(pos, vel, kp, kd, torque)

    # Print to terminal so you can check your work
    print(f"\nEncoded frame: {' '.join(f'{b:02X}' for b in frame)}")
    print(f"As decimal:    {frame}")

    # TODO: build the bar chart
    # Suggested labels for each byte:
    labels = [
        "Byte 0\npos[15:8]",
        "Byte 1\npos[7:0]",
        "Byte 2\nvel[11:4]",
        "Byte 3\nvel[3:0]\nkp[11:8]",
        "Byte 4\nkp[7:0]",
        "Byte 5\nkd[11:4]",
        "Byte 6\nkd[3:0]\ntorq[11:8]",
        "Byte 7\ntorq[7:0]",
    ]

    # TODO: create the plot and call plt.show()


def main():
    print("MIT CAN Frame Encoder")
    print("Enter values to encode. Press Enter to use the default.\n")

    def get(prompt, default, lo, hi):
        while True:
            raw = input(f"{prompt} (default {default}, range {lo} to {hi}): ").strip()
            if raw == "":
                return default
            try:
                val = float(raw)
                if lo <= val <= hi:
                    return val
                print(f"  Must be between {lo} and {hi}.")
            except ValueError:
                print("  Enter a number.")

    pos    = get("Position (rad)",         0.0, P_MIN,  P_MAX)
    vel    = get("Velocity (rad/s)",        0.0, V_MIN,  V_MAX)
    kp     = get("kp position gain",        5.0, KP_MIN, KP_MAX)
    kd     = get("kd damping gain",         0.5, KD_MIN, KD_MAX)
    torque = get("Torque feedforward (Nm)", 0.0, T_MIN,  T_MAX)

    plot_frame(pos, vel, kp, kd, torque)


if __name__ == "__main__":
    main()
