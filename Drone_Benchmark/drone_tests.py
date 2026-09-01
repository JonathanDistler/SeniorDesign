#!/usr/bin/env python3

"""
PX4 / GAZEBO OBSTACLE COURSE
Velocity-Controlled Test Suite

============================================================
FLIGHT MECHANISM
============================================================

PX4 terminal:

Then this Python program:

    1. Connects to PX4 through MAVSDK
    2. Waits until the vehicle is airborne
    3. Sends an initial velocity setpoint
    4. Starts MAVSDK Offboard mode
    5. Runs the selected velocity test
    6. Sends zero velocity
    7. Stops Offboard

The Python program does NOT:

    - arm the vehicle
    - call MAVSDK action.arm()
    - call MAVSDK action.takeoff()
    - use position targets
    - use MLE
    - use Kalman filtering
    - use laser physics
    - use laser estimation
    - use position-based navigation

============================================================
COURSE COORDINATE SYSTEM
============================================================

The obstacle course is defined in the Gazebo world.

Logical course coordinates:

    forward = direction through the obstacle course
    lateral = left/right across the course
    down    = vertical downward

For this world, the conversion to PX4 NED is:

    PX4 North = course lateral
    PX4 East  = course forward
    PX4 Down  = course down

Therefore:

    FORWARD:
        course [ +1,  0,  0 ]
        PX4    [  0, +1,  0 ]

    RIGHT:
        course [  0, +1,  0 ]
        PX4    [ +1,  0,  0 ]

    LEFT:
        course [ 0, -1,  0 ]
        PX4    [-1,  0,  0 ]

The vehicle yaw is held at 90 degrees so that the nose
points approximately along the course-forward direction.

============================================================
RUN
============================================================

Test 1:

    python drone_tests.py 1

Test 2:

    python drone_tests.py 2

Test 3:

    python drone_tests.py 3

============================================================
"""

import asyncio
import csv
import math
import os
import sys
import time

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from mavsdk import System

from mavsdk.offboard import (
    OffboardError,
    VelocityNedYaw,
)


# ============================================================
# CONNECTION
# ============================================================

SYSTEM_ADDRESS = (
    "udpin://127.0.0.1:14540"
)


# ============================================================
# CONTROL
# ============================================================

CONTROL_RATE_HZ = 20.0

CONTROL_PERIOD = (
    1.0 /
    CONTROL_RATE_HZ
)


# ============================================================
# VELOCITY LIMITS
# ============================================================

MAX_HORIZONTAL_SPEED_MPS = 2.0

MAX_VERTICAL_SPEED_MPS = 0.75


# ============================================================
# EXPECTED TAKEOFF
# ============================================================

MIN_AIRBORNE_ALTITUDE_M = 1.0

AIRBORNE_TIMEOUT_S = 20.0


# ============================================================
# YAW
# ============================================================

# Course forward corresponds to PX4 East.
# 90 degrees means the nose points approximately East.

COURSE_YAW_DEG = 90.0


# ============================================================
# TEST 1 — CONE WEAVE
# ============================================================

TEST1_FORWARD_SPEED = 1.0

TEST1_SIDE_SPEED = 0.75


TEST1_SEGMENTS = [

    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        4.0,
    ),

    (
        "WEAVE RIGHT",
        TEST1_FORWARD_SPEED,
        TEST1_SIDE_SPEED,
        0.0,
        2.0,
    ),

    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        3.0,
    ),

    (
        "WEAVE LEFT",
        TEST1_FORWARD_SPEED,
        -TEST1_SIDE_SPEED,
        0.0,
        4.0,
    ),

    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        3.0,
    ),

    (
        "WEAVE RIGHT",
        TEST1_FORWARD_SPEED,
        TEST1_SIDE_SPEED,
        0.0,
        4.0,
    ),

    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        3.0,
    ),

    (
        "WEAVE LEFT",
        TEST1_FORWARD_SPEED,
        -TEST1_SIDE_SPEED,
        0.0,
        4.0,
    ),

    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        4.0,
    ),
]


# ============================================================
# TEST 2 — FLAG
# ============================================================

FLAG_DIRECTION = "left"

TEST2_FORWARD_SPEED = 1.0

TEST2_SIDE_SPEED = 0.90

TEST2_APPROACH_TIME_S = 7.0

TEST2_PASS_TIME_S = 6.0


# ============================================================
# TEST 3 — BOX
# ============================================================

TEST3_FORWARD_SPEED = 1.0

TEST3_ENTRY_TIME_S = 5.0

TEST3_THROUGH_TIME_S = 8.0


# ============================================================
# OUTPUT
# ============================================================

OUTPUT_ROOT = os.path.expanduser(
    "~/drone_obstacle_course/test_results"
)


# ============================================================
# VEHICLE STATE
# ============================================================

@dataclass
class VehicleState:

    connected: bool = False

    armed: bool = False

    in_air: bool = False

    north_m: float = 0.0

    east_m: float = 0.0

    down_m: float = 0.0

    north_velocity_mps: float = 0.0

    east_velocity_mps: float = 0.0

    down_velocity_mps: float = 0.0

    yaw_deg: float = 0.0

    relative_altitude_m: float = 0.0


STATE = VehicleState()


# ============================================================
# FLIGHT SAMPLE
# ============================================================

@dataclass
class FlightSample:

    time_s: float

    north_m: float

    east_m: float

    down_m: float

    velocity_north_mps: float

    velocity_east_mps: float

    velocity_down_mps: float

    command_forward_mps: float

    command_lateral_mps: float

    command_down_mps: float

    command_north_mps: float

    command_east_mps: float

    yaw_deg: float

    altitude_m: float

    test_segment: str


# ============================================================
# METRICS
# ============================================================

@dataclass
class Metrics:

    start_time: float = 0.0

    samples: List[FlightSample] = field(
        default_factory=list
    )

    total_distance_m: float = 0.0

    horizontal_distance_m: float = 0.0

    maximum_speed_mps: float = 0.0

    maximum_horizontal_speed_mps: float = 0.0

    maximum_altitude_m: float = 0.0

    minimum_altitude_m: float = float(
        "inf"
    )

    previous_position: Optional[
        Tuple[float, float, float]
    ] = None

    previous_time: Optional[float] = None


# ============================================================
# PRINT HEADER
# ============================================================

def print_header():

    print()

    print(
        "=" * 60
    )

    print(
        "PX4 / GAZEBO OBSTACLE COURSE"
    )

    print(
        "=" * 60
    )

    print(
        "CONTROL MODE: VELOCITY"
    )

    print(
        "COURSE FRAME: GAZEBO WORLD"
    )

    print(
        "NED POSITION CONTROL: DISABLED"
    )

    print(
        "LASER: DISABLED"
    )

    print(
        "MLE: DISABLED"
    )

    print(
        "KALMAN TRACKS: DISABLED"
    )

    print(
        "=" * 60
    )

    print()


# ============================================================
# PRINT VEHICLE STATUS
# ============================================================

def print_vehicle_status():

    speed = math.sqrt(
        STATE.north_velocity_mps ** 2
        +
        STATE.east_velocity_mps ** 2
        +
        STATE.down_velocity_mps ** 2
    )

    print(
        "[DRONE] "
        f"NED=["
        f"{STATE.north_m:+7.3f}, "
        f"{STATE.east_m:+7.3f}, "
        f"{STATE.down_m:+7.3f}] m   "
        f"VEL=["
        f"{STATE.north_velocity_mps:+6.2f}, "
        f"{STATE.east_velocity_mps:+6.2f}, "
        f"{STATE.down_velocity_mps:+6.2f}] m/s   "
        f"ALT={STATE.relative_altitude_m:5.2f} m   "
        f"SPEED={speed:4.2f} m/s   "
        f"YAW={STATE.yaw_deg:+7.2f} deg"
    )


# ============================================================
# CONNECTION TELEMETRY
# ============================================================

async def monitor_connection(
    drone: System
):

    try:

        async for state in (
            drone.core.connection_state()
        ):

            STATE.connected = (
                state.is_connected
            )

    except asyncio.CancelledError:

        pass


# ============================================================
# POSITION TELEMETRY
# ============================================================

async def monitor_position(
    drone: System
):

    try:

        async for position in (
            drone.telemetry.position()
        ):

            STATE.relative_altitude_m = (
                float(
                    position.relative_altitude_m
                )
            )

    except asyncio.CancelledError:

        pass

    except Exception as exc:

        print(
            f"[TELEMETRY] Position monitor "
            f"stopped: {exc}"
        )


# ============================================================
# NED TELEMETRY
# ============================================================

async def monitor_ned(
    drone: System
):

    try:

        async for pv in (
            drone.telemetry.position_velocity_ned()
        ):

            STATE.north_m = float(
                pv.position.north_m
            )

            STATE.east_m = float(
                pv.position.east_m
            )

            STATE.down_m = float(
                pv.position.down_m
            )

            STATE.north_velocity_mps = float(
                pv.velocity.north_m_s
            )

            STATE.east_velocity_mps = float(
                pv.velocity.east_m_s
            )

            STATE.down_velocity_mps = float(
                pv.velocity.down_m_s
            )

    except asyncio.CancelledError:

        pass

    except Exception as exc:

        print(
            f"[TELEMETRY] NED monitor "
            f"stopped: {exc}"
        )


# ============================================================
# HEADING TELEMETRY
# ============================================================

async def monitor_heading(
    drone: System
):

    try:

        async for heading in (
            drone.telemetry.heading()
        ):

            STATE.yaw_deg = float(
                heading.heading_deg
            )

    except asyncio.CancelledError:

        pass

    except Exception as exc:

        print(
            f"[TELEMETRY] Heading monitor "
            f"stopped: {exc}"
        )


# ============================================================
# ARMED TELEMETRY
# ============================================================

async def monitor_armed(
    drone: System
):

    try:

        async for armed in (
            drone.telemetry.armed()
        ):

            STATE.armed = bool(
                armed
            )

    except asyncio.CancelledError:

        pass

    except Exception as exc:

        print(
            f"[TELEMETRY] Armed monitor "
            f"stopped: {exc}"
        )


# ============================================================
# IN-AIR TELEMETRY
# ============================================================

async def monitor_in_air(
    drone: System
):

    try:

        async for in_air in (
            drone.telemetry.in_air()
        ):

            STATE.in_air = bool(
                in_air
            )

    except asyncio.CancelledError:

        pass

    except Exception as exc:

        print(
            f"[TELEMETRY] In-air monitor "
            f"stopped: {exc}"
        )


# ============================================================
# START TELEMETRY
# ============================================================

async def start_telemetry(
    drone: System
):

    print(
        "[DRONE] Starting telemetry monitors..."
    )

    tasks = [

        asyncio.create_task(
            monitor_connection(drone)
        ),

        asyncio.create_task(
            monitor_position(drone)
        ),

        asyncio.create_task(
            monitor_ned(drone)
        ),

        asyncio.create_task(
            monitor_heading(drone)
        ),

        asyncio.create_task(
            monitor_armed(drone)
        ),

        asyncio.create_task(
            monitor_in_air(drone)
        ),
    ]

    await asyncio.sleep(
        2.0
    )

    print(
        "[DRONE] Checking telemetry..."
    )

    print(
        "[DRONE] Position = "
        f"[{STATE.north_m:.2f}, "
        f"{STATE.east_m:.2f}, "
        f"{STATE.down_m:.2f}]"
    )

    print(
        "[DRONE] Velocity = "
        f"[{STATE.north_velocity_mps:.2f}, "
        f"{STATE.east_velocity_mps:.2f}, "
        f"{STATE.down_velocity_mps:.2f}]"
    )

    print(
        "[DRONE] Relative altitude = "
        f"{STATE.relative_altitude_m:.2f} m"
    )

    print(
        "[DRONE] In air = "
        f"{STATE.in_air}"
    )

    print(
        "[DRONE] Armed = "
        f"{STATE.armed}"
    )

    print(
        "[DRONE] Yaw = "
        f"{STATE.yaw_deg:.2f} deg"
    )

    return tasks


# ============================================================
# CONNECT
# ============================================================

async def connect_drone() -> System:

    drone = System()

    print(
        f"[DRONE] Connecting to "
        f"{SYSTEM_ADDRESS}"
    )

    await drone.connect(
        system_address=SYSTEM_ADDRESS
    )

    print(
        "[DRONE] Waiting for PX4 connection..."
    )

    async for state in (
        drone.core.connection_state()
    ):

        if state.is_connected:

            STATE.connected = True

            print(
                "[DRONE] Connected."
            )

            return drone


# ============================================================
# COURSE FRAME → PX4 NED
# ============================================================

def course_to_ned(
    forward_mps: float,
    lateral_mps: float,
    down_mps: float,
):
    """
    Convert obstacle-course coordinates into PX4 NED.

    COURSE:
        forward = through the course
        lateral = left/right
        down    = downward

    MAPPING:

        North = lateral
        East  = forward
        Down  = down
    """

    north_mps = lateral_mps

    east_mps = forward_mps

    return (
        north_mps,
        east_mps,
        down_mps
    )


# ============================================================
# SEND VELOCITY
# ============================================================

async def send_course_velocity(
    drone: System,
    forward_mps: float,
    lateral_mps: float,
    down_mps: float,
    yaw_deg: float = COURSE_YAW_DEG,
):
    """
    Send velocity in the logical obstacle-course frame.
    """

    (
        north_mps,
        east_mps,
        down_mps
    ) = course_to_ned(
        forward_mps,
        lateral_mps,
        down_mps
    )

    # --------------------------------------------------------
    # Horizontal speed limit
    # --------------------------------------------------------

    horizontal_speed = math.sqrt(
        north_mps ** 2
        +
        east_mps ** 2
    )

    if (
        horizontal_speed
        >
        MAX_HORIZONTAL_SPEED_MPS
    ):

        scale = (
            MAX_HORIZONTAL_SPEED_MPS
            /
            horizontal_speed
        )

        north_mps *= scale

        east_mps *= scale

    # --------------------------------------------------------
    # Vertical speed limit
    # --------------------------------------------------------

    down_mps = max(
        -MAX_VERTICAL_SPEED_MPS,
        min(
            MAX_VERTICAL_SPEED_MPS,
            down_mps
        )
    )

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            float(north_mps),
            float(east_mps),
            float(down_mps),
            float(yaw_deg)
        )
    )


# ============================================================
# UPDATE METRICS
# ============================================================

def update_metrics(
    metrics: Metrics,
    forward_mps: float,
    lateral_mps: float,
    down_mps: float,
    segment_name: str,
):

    now = time.perf_counter()

    if metrics.start_time <= 0.0:

        metrics.start_time = now

    position = (
        STATE.north_m,
        STATE.east_m,
        STATE.down_m
    )

    if (
        metrics.previous_position
        is not None
        and
        metrics.previous_time
        is not None
    ):

        dx = (
            position[0]
            -
            metrics.previous_position[0]
        )

        dy = (
            position[1]
            -
            metrics.previous_position[1]
        )

        dz = (
            position[2]
            -
            metrics.previous_position[2]
        )

        step_distance = math.sqrt(
            dx ** 2
            +
            dy ** 2
            +
            dz ** 2
        )

        step_horizontal = math.sqrt(
            dx ** 2
            +
            dy ** 2
        )

        metrics.total_distance_m += (
            step_distance
        )

        metrics.horizontal_distance_m += (
            step_horizontal
        )

    speed = math.sqrt(
        STATE.north_velocity_mps ** 2
        +
        STATE.east_velocity_mps ** 2
        +
        STATE.down_velocity_mps ** 2
    )

    horizontal_speed = math.sqrt(
        STATE.north_velocity_mps ** 2
        +
        STATE.east_velocity_mps ** 2
    )

    metrics.maximum_speed_mps = max(
        metrics.maximum_speed_mps,
        speed
    )

    metrics.maximum_horizontal_speed_mps = max(
        metrics.maximum_horizontal_speed_mps,
        horizontal_speed
    )

    metrics.maximum_altitude_m = max(
        metrics.maximum_altitude_m,
        STATE.relative_altitude_m
    )

    metrics.minimum_altitude_m = min(
        metrics.minimum_altitude_m,
        STATE.relative_altitude_m
    )

    (
        command_north_mps,
        command_east_mps,
        command_down_mps
    ) = course_to_ned(
        forward_mps,
        lateral_mps,
        down_mps
    )

    sample = FlightSample(

        time_s=(
            now -
            metrics.start_time
        ),

        north_m=STATE.north_m,

        east_m=STATE.east_m,

        down_m=STATE.down_m,

        velocity_north_mps=(
            STATE.north_velocity_mps
        ),

        velocity_east_mps=(
            STATE.east_velocity_mps
        ),

        velocity_down_mps=(
            STATE.down_velocity_mps
        ),

        command_forward_mps=(
            forward_mps
        ),

        command_lateral_mps=(
            lateral_mps
        ),

        command_down_mps=(
            down_mps
        ),

        command_north_mps=(
            command_north_mps
        ),

        command_east_mps=(
            command_east_mps
        ),

        yaw_deg=STATE.yaw_deg,

        altitude_m=(
            STATE.relative_altitude_m
        ),

        test_segment=segment_name
    )

    metrics.samples.append(
        sample
    )

    metrics.previous_position = (
        position
    )

    metrics.previous_time = now


# ============================================================
# WAIT UNTIL AIRBORNE
# ============================================================

async def wait_until_airborne(
    drone: System
):

    print()
    print(
        "[DRONE] Waiting for the vehicle "
        "to be airborne..."
    )

    start = time.monotonic()

    while True:

        # We accept either MAVSDK's in_air flag
        # or a meaningful positive altitude.

        if (
            STATE.in_air
            or
            STATE.relative_altitude_m
            >=
            MIN_AIRBORNE_ALTITUDE_M
        ):

            print(
                "[DRONE] Vehicle is airborne."
            )

            print(
                f"[DRONE] Altitude = "
                f"{STATE.relative_altitude_m:.2f} m"
            )

            return

        if (
            time.monotonic() -
            start
            >
            AIRBORNE_TIMEOUT_S
        ):

            raise RuntimeError(
                "Vehicle did not become airborne. "
                "Run 'commander arm -f' and "
                "'commander takeoff' in the PX4 terminal "
                "before starting this Python program."
            )

        await asyncio.sleep(
            0.1
        )


# ============================================================
# START OFFBOARD
# ============================================================

async def start_offboard(
    drone: System
):

    print()
    print(
        "[OFFBOARD] Sending initial "
        "zero-velocity setpoints..."
    )

    # Send an initial setpoint.
    await send_course_velocity(
        drone,
        0.0,
        0.0,
        0.0,
        COURSE_YAW_DEG
    )

    # Continue sending zero velocity long enough
    # for PX4 to accept the Offboard stream.

    for _ in range(20):

        await send_course_velocity(
            drone,
            0.0,
            0.0,
            0.0,
            COURSE_YAW_DEG
        )

        await asyncio.sleep(
            CONTROL_PERIOD
        )

    print(
        "[OFFBOARD] Starting..."
    )

    try:

        await drone.offboard.start()

    except OffboardError as exc:

        print()
        print(
            "[ERROR] Offboard start failed."
        )

        print(
            f"        {exc}"
        )

        print()
        print(
            "Vehicle status:"
        )

        print(
            f"    Armed: {STATE.armed}"
        )

        print(
            f"    In air: {STATE.in_air}"
        )

        print(
            f"    Altitude: "
            f"{STATE.relative_altitude_m:.2f} m"
        )

        raise

    print(
        "[OFFBOARD] Active."
    )


# ============================================================
# HOLD
# ============================================================

async def hold_zero_velocity(
    drone: System,
    metrics: Metrics,
    duration_s: float,
    name: str,
):

    print(
        f"[DRONE] Holding for "
        f"{duration_s:.1f} s..."
    )

    end_time = (
        time.monotonic()
        +
        duration_s
    )

    while (
        time.monotonic()
        <
        end_time
    ):

        await send_course_velocity(
            drone,
            0.0,
            0.0,
            0.0,
            COURSE_YAW_DEG
        )

        update_metrics(
            metrics,
            0.0,
            0.0,
            0.0,
            name
        )

        await asyncio.sleep(
            CONTROL_PERIOD
        )


# ============================================================
# RUN VELOCITY SEGMENT
# ============================================================

async def run_velocity_segment(
    drone: System,
    metrics: Metrics,
    name: str,
    forward_mps: float,
    lateral_mps: float,
    down_mps: float,
    duration_s: float,
):

    print()
    print(
        "-" * 60
    )

    print(
        f"[TEST] {name}"
    )

    print(
        "[TEST] Course velocity = "
        f"[forward={forward_mps:+.2f}, "
        f"lateral={lateral_mps:+.2f}, "
        f"down={down_mps:+.2f}] m/s"
    )

    (
        north_cmd,
        east_cmd,
        down_cmd
    ) = course_to_ned(
        forward_mps,
        lateral_mps,
        down_mps
    )

    print(
        "[TEST] PX4 NED command = "
        f"[N={north_cmd:+.2f}, "
        f"E={east_cmd:+.2f}, "
        f"D={down_cmd:+.2f}] m/s"
    )

    print(
        f"[TEST] Yaw = "
        f"{COURSE_YAW_DEG:+.1f} deg"
    )

    print(
        f"[TEST] Duration = "
        f"{duration_s:.1f} s"
    )

    print(
        "-" * 60
    )

    start = time.monotonic()

    next_print = start

    while True:

        now = time.monotonic()

        elapsed = (
            now -
            start
        )

        if elapsed >= duration_s:

            break

        await send_course_velocity(
            drone,
            forward_mps,
            lateral_mps,
            down_mps,
            COURSE_YAW_DEG
        )

        update_metrics(
            metrics,
            forward_mps,
            lateral_mps,
            down_mps,
            name
        )

        if now >= next_print:

            print_vehicle_status()

            next_print = (
                now + 1.0
            )

        await asyncio.sleep(
            CONTROL_PERIOD
        )


# ============================================================
# TEST 1 — CONE WEAVE
# ============================================================

async def run_test_1(
    drone: System,
    metrics: Metrics
):

    print()
    print(
        "=" * 60
    )

    print(
        "TEST 1: CONE WEAVE"
    )

    print(
        "=" * 60
    )

    print()
    print(
        "Course direction: FORWARD"
    )

    print(
        "Pattern: RIGHT -> LEFT -> RIGHT"
    )

    print(
        "Position targets: DISABLED"
    )

    print(
        "Velocity commands: ENABLED"
    )

    print()

    for segment in TEST1_SEGMENTS:

        (
            name,
            forward_mps,
            lateral_mps,
            down_mps,
            duration_s
        ) = segment

        await run_velocity_segment(
            drone,
            metrics,
            name,
            forward_mps,
            lateral_mps,
            down_mps,
            duration_s
        )

    await hold_zero_velocity(
        drone,
        metrics,
        2.0,
        "TEST 1 FINISH HOLD"
    )

    print()
    print(
        "=" * 60
    )

    print(
        "TEST 1 COMPLETE"
    )

    print(
        "=" * 60
    )


# ============================================================
# TEST 2 — FLAG
# ============================================================

async def run_test_2(
    drone: System,
    metrics: Metrics
):

    direction = (
        FLAG_DIRECTION
        .strip()
        .lower()
    )

    if direction not in (
        "left",
        "right"
    ):

        raise ValueError(
            "FLAG_DIRECTION must be "
            "'left' or 'right'."
        )

    if direction == "left":

        lateral_speed = (
            -TEST2_SIDE_SPEED
        )

    else:

        lateral_speed = (
            TEST2_SIDE_SPEED
        )

    print()
    print(
        "=" * 60
    )

    print(
        "TEST 2: FLAG"
    )

    print(
        "=" * 60
    )

    print(
        f"Direction: "
        f"{direction.upper()}"
    )

    print(
        "Velocity commands only."
    )

    print()

    await run_velocity_segment(
        drone,
        metrics,
        f"FLAG {direction.upper()} APPROACH",
        TEST2_FORWARD_SPEED,
        lateral_speed,
        0.0,
        TEST2_APPROACH_TIME_S
    )

    await run_velocity_segment(
        drone,
        metrics,
        f"FLAG PASS {direction.upper()}",
        TEST2_FORWARD_SPEED,
        lateral_speed,
        0.0,
        TEST2_PASS_TIME_S
    )

    await hold_zero_velocity(
        drone,
        metrics,
        2.0,
        "TEST 2 FINISH HOLD"
    )

    print()
    print(
        "=" * 60
    )

    print(
        "TEST 2 COMPLETE"
    )

    print(
        "=" * 60
    )


# ============================================================
# TEST 3 — BOX
# ============================================================

async def run_test_3(
    drone: System,
    metrics: Metrics
):

    print()
    print(
        "=" * 60
    )

    print(
        "TEST 3: BOX PASS-THROUGH"
    )

    print(
        "=" * 60
    )

    print()
    print(
        "Course direction: FORWARD"
    )

    print(
        "Lateral velocity: ZERO"
    )

    print(
        "Position targets: DISABLED"
    )

    print()

    await run_velocity_segment(
        drone,
        metrics,
        "BOX ENTRY",
        TEST3_FORWARD_SPEED,
        0.0,
        0.0,
        TEST3_ENTRY_TIME_S
    )

    await run_velocity_segment(
        drone,
        metrics,
        "BOX THROUGH OPENING",
        TEST3_FORWARD_SPEED,
        0.0,
        0.0,
        TEST3_THROUGH_TIME_S
    )

    await hold_zero_velocity(
        drone,
        metrics,
        2.0,
        "TEST 3 FINISH HOLD"
    )

    print()
    print(
        "=" * 60
    )

    print(
        "TEST 3 COMPLETE"
    )

    print(
        "=" * 60
    )


# ============================================================
# STOP OFFBOARD
# ============================================================

async def stop_offboard(
    drone: System,
    metrics: Metrics
):

    print()
    print(
        "[OFFBOARD] Sending zero velocity..."
    )

    try:

        await hold_zero_velocity(
            drone,
            metrics,
            1.0,
            "FINAL STOP"
        )

    except Exception as exc:

        print(
            "[WARN] Zero-velocity stop "
            f"failed: {exc}"
        )

    try:

        await drone.offboard.stop()

        print(
            "[OFFBOARD] Offboard stopped."
        )

    except Exception as exc:

        print(
            "[WARN] Offboard stop failed:"
        )

        print(
            f"       {exc}"
        )


# ============================================================
# LAND
# ============================================================

async def land_vehicle(
    drone: System
):

    print()
    print(
        "[DRONE] Landing..."
    )

    try:

        await drone.action.land()

        print(
            "[DRONE] Landing command sent."
        )

    except Exception as exc:

        print(
            "[WARN] Landing command failed:"
        )

        print(
            f"       {exc}"
        )

        return

    start = time.monotonic()

    while (
        time.monotonic() -
        start
        <
        20.0
    ):

        if (
            not STATE.in_air
            and
            STATE.relative_altitude_m
            <
            0.25
        ):

            break

        await asyncio.sleep(
            0.25
        )

    print(
        "[DRONE] Landing sequence complete."
    )


# ============================================================
# DISARM
# ============================================================

async def disarm_vehicle(
    drone: System
):

    if not STATE.armed:

        print(
            "[DRONE] Already disarmed."
        )

        return

    print(
        "[DRONE] Disarming..."
    )

    try:

        await drone.action.disarm()

        print(
            "[DRONE] Disarmed."
        )

    except Exception as exc:

        print(
            "[WARN] Disarm command failed:"
        )

        print(
            f"       {exc}"
        )


# ============================================================
# SAVE CSV
# ============================================================

def save_csv(
    metrics: Metrics,
    test_name: str
):

    output_directory = os.path.join(
        OUTPUT_ROOT,
        test_name
    )

    os.makedirs(
        output_directory,
        exist_ok=True
    )

    path = os.path.join(
        output_directory,
        "flight_data.csv"
    )

    with open(
        path,
        "w",
        newline=""
    ) as file:

        writer = csv.writer(
            file
        )

        writer.writerow([
            "time_s",

            "north_m",
            "east_m",
            "down_m",

            "velocity_north_mps",
            "velocity_east_mps",
            "velocity_down_mps",

            "command_forward_mps",
            "command_lateral_mps",
            "command_down_mps",

            "command_north_mps",
            "command_east_mps",

            "yaw_deg",

            "altitude_m",

            "test_segment",
        ])

        for sample in metrics.samples:

            writer.writerow([
                sample.time_s,

                sample.north_m,
                sample.east_m,
                sample.down_m,

                sample.velocity_north_mps,
                sample.velocity_east_mps,
                sample.velocity_down_mps,

                sample.command_forward_mps,
                sample.command_lateral_mps,
                sample.command_down_mps,

                sample.command_north_mps,
                sample.command_east_mps,

                sample.yaw_deg,

                sample.altitude_m,

                sample.test_segment,
            ])

    print()
    print(
        "[OUTPUT] Flight data saved to:"
    )

    print(
        f"         {path}"
    )


# ============================================================
# SAVE SUMMARY
# ============================================================

def save_summary(
    metrics: Metrics,
    test_name: str
):

    output_directory = os.path.join(
        OUTPUT_ROOT,
        test_name
    )

    os.makedirs(
        output_directory,
        exist_ok=True
    )

    path = os.path.join(
        output_directory,
        "summary.txt"
    )

    if metrics.samples:

        total_time = (
            metrics.samples[-1].time_s
        )

    else:

        total_time = 0.0

    if total_time > 0.0:

        average_speed = (
            metrics.total_distance_m
            /
            total_time
        )

    else:

        average_speed = 0.0

    with open(
        path,
        "w"
    ) as file:

        file.write(
            "PX4 / GAZEBO OBSTACLE COURSE\n"
        )

        file.write(
            "========================================\n\n"
        )

        file.write(
            f"Test: {test_name}\n"
        )

        file.write(
            "Control mode: Velocity\n"
        )

        file.write(
            "Position targets: Disabled\n"
        )

        file.write(
            "Laser: Disabled\n"
        )

        file.write(
            "MLE: Disabled\n"
        )

        file.write(
            "Kalman filtering: Disabled\n\n"
        )

        file.write(
            "COURSE / NED MAPPING\n"
        )

        file.write(
            "----------------------------------------\n"
        )

        file.write(
            "Course forward -> PX4 East\n"
        )

        file.write(
            "Course lateral -> PX4 North\n"
        )

        file.write(
            "Course down -> PX4 Down\n\n"
        )

        file.write(
            "FLIGHT METRICS\n"
        )

        file.write(
            "----------------------------------------\n"
        )

        file.write(
            f"Flight time: "
            f"{total_time:.3f} s\n"
        )

        file.write(
            f"3D distance: "
            f"{metrics.total_distance_m:.3f} m\n"
        )

        file.write(
            f"Horizontal distance: "
            f"{metrics.horizontal_distance_m:.3f} m\n"
        )

        file.write(
            f"Average speed: "
            f"{average_speed:.3f} m/s\n"
        )

        file.write(
            f"Maximum speed: "
            f"{metrics.maximum_speed_mps:.3f} m/s\n"
        )

        file.write(
            f"Maximum horizontal speed: "
            f"{metrics.maximum_horizontal_speed_mps:.3f} m/s\n"
        )

        file.write(
            f"Maximum altitude: "
            f"{metrics.maximum_altitude_m:.3f} m\n"
        )

        if (
            metrics.minimum_altitude_m
            !=
            float("inf")
        ):

            file.write(
                f"Minimum altitude: "
                f"{metrics.minimum_altitude_m:.3f} m\n"
            )

        file.write(
            f"Position samples: "
            f"{len(metrics.samples)}\n"
        )

    print(
        "[OUTPUT] Summary saved to:"
    )

    print(
        f"         {path}"
    )


# ============================================================
# MAIN TEST RUNNER
# ============================================================

async def run_test(
    test_number: int
):

    test_names = {

        1: "test1_cone_weave",

        2: "test2_flag",

        3: "test3_box",
    }

    if test_number not in test_names:

        raise ValueError(
            "Test must be 1, 2, or 3."
        )

    test_name = test_names[
        test_number
    ]

    print_header()

    print(
        f"Selected test: {test_number}"
    )

    print(
        f"Name: {test_name}"
    )

    print(
        "ONLY THIS TEST WILL RUN."
    )

    print()

    drone: Optional[System] = None

    telemetry_tasks = []

    metrics = Metrics()

    offboard_active = False

    try:

        # ====================================================
        # CONNECT
        # ====================================================

        drone = await connect_drone()

        # ====================================================
        # TELEMETRY
        # ====================================================

        telemetry_tasks = await start_telemetry(
            drone
        )

        # ====================================================
        # VERIFY THE VEHICLE IS ALREADY AIRBORNE
        # ====================================================

        print()

        print(
            "[DRONE] Python will NOT arm "
            "or take off the vehicle."
        )

        print(
            "[DRONE] PX4 should already have:"
        )

        print(
            "         commander arm -f"
        )

        print(
            "         commander takeoff"
        )

        await wait_until_airborne(
            drone
        )

        # ====================================================
        # START OFFBOARD
        # ====================================================

        await start_offboard(
            drone
        )

        offboard_active = True

        # ====================================================
        # START METRICS
        # ====================================================

        metrics.start_time = (
            time.perf_counter()
        )

        # ====================================================
        # INITIAL HOLD
        # ====================================================

        await hold_zero_velocity(
            drone,
            metrics,
            2.0,
            "INITIAL HOLD"
        )

        # ====================================================
        # RUN SELECTED TEST
        # ====================================================

        if test_number == 1:

            await run_test_1(
                drone,
                metrics
            )

        elif test_number == 2:

            await run_test_2(
                drone,
                metrics
            )

        elif test_number == 3:

            await run_test_3(
                drone,
                metrics
            )

        # ====================================================
        # STOP OFFBOARD
        # ====================================================

        await stop_offboard(
            drone,
            metrics
        )

        offboard_active = False

        # ====================================================
        # LAND
        # ====================================================

        await land_vehicle(
            drone
        )

        # ====================================================
        # DISARM
        # ====================================================

        await disarm_vehicle(
            drone
        )

        # ====================================================
        # SAVE RESULTS
        # ====================================================

        save_csv(
            metrics,
            test_name
        )

        save_summary(
            metrics,
            test_name
        )

        # ====================================================
        # COMPLETE
        # ====================================================

        print()

        print(
            "=" * 60
        )

        print(
            "FLIGHT COMPLETE"
        )

        print(
            "=" * 60
        )

        print()

    except KeyboardInterrupt:

        print()

        print(
            "=" * 60
        )

        print(
            "[STOP] Ctrl+C detected."
        )

        print(
            "=" * 60
        )

        if drone is not None:

            try:

                if offboard_active:

                    await stop_offboard(
                        drone,
                        metrics
                    )

            except Exception:

                pass

            try:

                await land_vehicle(
                    drone
                )

            except Exception:

                pass

            try:

                await disarm_vehicle(
                    drone
                )

            except Exception:

                pass

    except Exception as exc:

        print()

        print(
            "=" * 60
        )

        print(
            "[ERROR] FLIGHT FAILED"
        )

        print(
            "=" * 60
        )

        print()

        print(
            f"{type(exc).__name__}: {exc}"
        )

        print()

        if drone is not None:

            try:

                await send_course_velocity(
                    drone,
                    0.0,
                    0.0,
                    0.0,
                    COURSE_YAW_DEG
                )

            except Exception:

                pass

            if offboard_active:

                try:

                    await drone.offboard.stop()

                except Exception:

                    pass

            try:

                if STATE.armed:

                    await land_vehicle(
                        drone
                    )

            except Exception:

                pass

            try:

                await disarm_vehicle(
                    drone
                )

            except Exception:

                pass

    finally:

        for task in telemetry_tasks:

            task.cancel()

        if telemetry_tasks:

            await asyncio.gather(
                *telemetry_tasks,
                return_exceptions=True
            )


# ============================================================
# COMMAND LINE
# ============================================================

def main():

    if len(sys.argv) != 2:

        print()

        print(
            "Usage:"
        )

        print(
            "    python drone_tests.py 1"
        )

        print(
            "    python drone_tests.py 2"
        )

        print(
            "    python drone_tests.py 3"
        )

        print()

        sys.exit(1)

    try:

        test_number = int(
            sys.argv[1]
        )

    except ValueError:

        print(
            "[ERROR] Test number must be "
            "1, 2, or 3."
        )

        sys.exit(1)

    if test_number not in (
        1,
        2,
        3
    ):

        print(
            "[ERROR] Test number must be "
            "1, 2, or 3."
        )

        sys.exit(1)

    try:

        asyncio.run(
            run_test(
                test_number
            )
        )

    except KeyboardInterrupt:

        print()

        print(
            "[STOP] Ctrl+C."
        )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()