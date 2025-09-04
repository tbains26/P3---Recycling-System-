import sys, time, random, math
sys.path.append('../')
from Common.project_library import *

# =========================
# 1) INTERFACE CONFIG
# =========================
PROJECT_ID     = 'P3B'              # 'P0','P2A','P2B','P3A','P3B'
IP_ADDRESS     = '169.254.48.137'   # your PC's IP
USE_HARDWARE   = False              # False = simulation

# =========================
# 2) SERVO TABLE CONFIG
# =========================
SHORT_TOWER_DEG = 315   # identification tower
TALL_TOWER_DEG  = 90    # classification tower
DROP_TUBE_DEG   = 180   # clockwise from 0°

# =========================
# 3) QBOT CONFIG
# =========================
BOT_CAMERA_DEG = 0      # between -21.5 and 0

# =========================
# 4) BIN CONFIG (colors & offsets)
# line color == bin color
# =========================
BIN_OFFSETS = [0.17, 0.17, 0.17, 0.17]
BIN_COLORS  = [
    [1, 0, 0],  # Bin01 red   -> metal
    [0, 1, 0],  # Bin02 green -> paper
    [0, 0, 1],  # Bin03 blue  -> plastic
    [0, 0, 0],  # Bin04 black -> garbage
]

# ------------------ DO NOT MODIFY BELOW (setup) ------------------
if PROJECT_ID == 'P0':
    QLabs = configure_environment(PROJECT_ID, IP_ADDRESS, USE_HARDWARE).QLabs
    bot = qbot(0.1, IP_ADDRESS, QLabs, None, USE_HARDWARE)

elif PROJECT_ID in ['P2A', 'P2B']:
    QLabs = configure_environment(PROJECT_ID, IP_ADDRESS, USE_HARDWARE).QLabs
    arm = qarm(PROJECT_ID, IP_ADDRESS, QLabs, USE_HARDWARE)

elif PROJECT_ID == 'P3A':
    table_cfg = [SHORT_TOWER_DEG, TALL_TOWER_DEG, DROP_TUBE_DEG]
    config = [table_cfg, None, None]
    QLabs = configure_environment(PROJECT_ID, IP_ADDRESS, USE_HARDWARE, config).QLabs
    table = servo_table(IP_ADDRESS, QLabs, table_cfg, USE_HARDWARE)
    arm = qarm(PROJECT_ID, IP_ADDRESS, QLabs, USE_HARDWARE)

elif PROJECT_ID == 'P3B':
    table_cfg = [SHORT_TOWER_DEG, TALL_TOWER_DEG, DROP_TUBE_DEG]
    qbot_cfg  = [BOT_CAMERA_DEG]
    bin_cfg   = [BIN_OFFSETS, BIN_COLORS]
    config    = [table_cfg, qbot_cfg, bin_cfg]
    QLabs = configure_environment(PROJECT_ID, IP_ADDRESS, USE_HARDWARE, config).QLabs
    table = servo_table(IP_ADDRESS, QLabs, table_cfg, USE_HARDWARE)
    arm   = qarm(PROJECT_ID, IP_ADDRESS, QLabs, USE_HARDWARE)
    bins  = bins(bin_cfg)
    bot   = qbot(0.1, IP_ADDRESS, QLabs, bins, USE_HARDWARE)

# =================================================================
# STUDENT CODE (REFACTORED, SAME OBJECTIVE)
# =================================================================

def delay(s: float) -> None:
    """Small wrapper to keep sleeps consistent and readable."""
    time.sleep(s)

def dispense_random_container():
    """
    Dispense one of the 6 containers and return (mass, destination_bin_str).
    """
    idx = random.randint(1, 6)
    location, weight, bin_name = table.dispense_container(idx, True)
    return weight, bin_name

def place_on_hopper_slot(slot_index: int) -> None:
    """
    Place the currently gripped container onto the hopper at the slot
    determined by slot_index ∈ {0,1,2}. Uses bot position to compute arm target.
    """
    # slot-dependent x-offsets relative to bot
    # original code used +0.12, +0.04, -0.05 along arm X
    slot_offsets = [0.12, 0.04, -0.05]
    # slot-dependent Y baseline along arm Y (used two values: -2.1 and -2.11)
    # tiny adjustment preserved from original logic
    y_base = -2.11 if slot_index == 2 else -2.10

    # ensure bot is within arm reach; if not, nudge forward then restore line
    in_range = (bot.position()[0] + y_base) >= -0.643
    if not in_range:
        bot.forward_distance(0.05)

    # Compute arm target using current bot pose
    bx, by, _ = bot.position()
    arm.move_arm(-by + slot_offsets[slot_index], bx + y_base, 0.52)
    delay(1.0)
    arm.control_gripper(-10)     # release
    delay(2.0)
    arm.rotate_shoulder(-20)

    if not in_range:
        # restore to line (preserved original maneuver)
        delay(0.5)
        bot.rotate(180)
        bot.forward_distance(0.05)
        bot.rotate(-180)

def load_container_to_hopper(slot_index: int) -> None:
    """
    Full pick-and-place sequence for loading a single container
    onto the hopper at the given slot index.
    """
    arm.home()
    delay(1.0)
    arm.move_arm(0.65, 0.0, 0.27)  # reach to sorting station
    delay(1.0)
    arm.control_gripper(36)        # grip
    delay(1.0)
    arm.move_arm(0.20, 0.0, 0.40)
    arm.rotate_elbow(-30)
    arm.rotate_base(-90)
    delay(1.0)

    place_on_hopper_slot(slot_index)

    delay(1.0)                     # let container settle
    arm.rotate_elbow(-20)
    arm.home()

def move_to_bin(bin_name: str) -> None:
    """
    Follow the line and stop at the correct bin using color + ultrasonic cues.
    Then advance a small distance to center at the chute.
    """
    # Prepare
    bot.rotate(95)                         # face the track
    bot.activate_ultrasonic_sensor()
    bot.activate_color_sensor()
    bot.set_wheel_speed([0.04, 0.04])

    # Per-bin sensing thresholds (preserved)
    if bin_name == "Bin01":
        dist_thr, color = 0.08, [1, 0, 0]
        print("Going to Bin 01!")
    elif bin_name == "Bin02":
        dist_thr, color = 0.06, [0, 1, 0]
        print("Going to Bin 02!")
    elif bin_name == "Bin03":
        dist_thr, color = 0.05, [0, 0, 1]
        print("Going to Bin 03!")
    else:
        dist_thr, color = 0.05, [0, 0, 0]
        print("Going to Bin 04!")

    # Line-follow loop until both cues match
    while True:
        seen_color = bot.read_color_sensor()[0]
        distance   = bot.read_ultrasonic_sensor()
        if distance <= dist_thr and seen_color == color:
            print("I see the bin!", distance, seen_color)
            break

        left, right = bot.line_following_sensors()
        if left == 1 and right == 1:
            bot.set_wheel_speed([0.04, 0.04])
        elif left > right:
            bot.set_wheel_speed([0.04, 0.064])
        elif left < right:
            bot.set_wheel_speed([0.064, 0.04])
        else:
            bot.stop()  # safety stop if line lost
            break

        print("Colour Sensor:", seen_color)
        print("Ultrasonic Sensor:", distance)
        print("Line Sensors:", [left, right])

    print("Stopping...")
    bot.stop()
    bot.forward_distance(0.05)  # small approach to account for early detection

def unload_hopper() -> None:
    """
    Incrementally tilt the hopper to ensure all containers fall out safely,
    then return hopper to zero.
    """
    bot.activate_stepper_motor()
    for angle, wait_s in [(30, 1.5), (45, 1.5), (60, 1.5), (90, 1.0)]:
        bot.rotate_hopper(angle)
        delay(wait_s)
    bot.rotate_hopper(0)

def return_to_home() -> None:
    """
    Follow the line back to the home window (same bounds as original).
    """
    bot.set_wheel_speed([0.04, 0.04])
    pos = bot.position()

    while not (1.3 < pos[0] < 1.7 and 0 < pos[1] < 0.2):
        left, right = bot.line_following_sensors()
        pos = bot.position()
        print("Bot Position:", pos[0], pos[1])
        print("Line Sensors:", [left, right])

        if left == 1 and right == 1:
            bot.set_wheel_speed([0.05, 0.05])
        elif left > right:
            bot.set_wheel_speed([0.02, 0.08])
        elif left < right:
            bot.set_wheel_speed([0.08, 0.02])
        else:
            bot.set_wheel_speed([-0.05, -0.05])

    bot.stop()

def main():
    """
    Continuous cycle:
    - Rotate bot for easier loading
    - Dispense container and attempt to batch up to 3 like-destination items
      (total mass < 90) onto hopper
    - Deliver to correct bin, unload, return home
    - Repeat indefinitely
    """
    bot.rotate(-95)

    # Prime first container
    weight, dest_bin = dispense_random_container()

    containers_on_hopper = 0
    total_mass = 0
    previous_dest = ""

    while True:
        new_mass     = weight
        new_dest     = dest_bin
        can_batch    = (
            containers_on_hopper < 3 and
            total_mass + new_mass < 90 and
            (previous_dest == "" or new_dest == previous_dest)
        )

        if can_batch:
            total_mass += new_mass
            previous_dest = new_dest
            load_container_to_hopper(containers_on_hopper)
            containers_on_hopper += 1

            # Get the next container ready
            weight, dest_bin = dispense_random_container()
        else:
            # Deliver current batch
            move_to_bin(previous_dest)
            unload_hopper()
            return_to_home()
            bot.rotate(-95)

            # Reset batch tracking; keep the just-dispensed item as the seed
            containers_on_hopper = 0
            total_mass = 0
            previous_dest = new_dest

# Record and show initial home position (same as original)
print("Home position:", bot.position())

# Start loop
if __name__ == "__main__":
    main()
