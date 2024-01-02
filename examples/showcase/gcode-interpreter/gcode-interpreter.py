import os
import sys
import math
from machine import Pin

from pimoroni_yukon import Yukon

from pimoroni_yukon import SLOT2 as SLOT_X
from pimoroni_yukon import SLOT3 as SLOT_Y1
from pimoroni_yukon import SLOT1 as SLOT_Y2
from pimoroni_yukon import SLOT4 as SLOT_Z
from pimoroni_yukon import SLOT5 as SLOT_LIMITS_1
from pimoroni_yukon import SLOT6 as SLOT_LIMITS_2

from pimoroni_yukon.modules import DualMotorModule, QuadServoDirectModule
from pimoroni_yukon.devices.stepper import OkayStepper

from parser import GCodeParser

"""
A serial GCode interpreter on the Yukon mimicking GRBL.

Based off the Yukon Plotter showcase.
"""


STEPS_PER_REV = 200                 # The number of steps each stepper motor takes to perform a full revolution
BELT_PITCH = 3                      # The spacing between each notch of the belts used on the X and Y axes
PULLEY_TOOTH_COUNT = 20             # The number of teeth on the X and Y axes direct-driven pulleys
BELT_STEPS_PER_MM = STEPS_PER_REV / (BELT_PITCH * PULLEY_TOOTH_COUNT)

SCREW_MM_PER_REV = 8                # How many millimeters of travel a single revolution of the Z axis lead screw produces
SCREW_STEPS_PER_MM = STEPS_PER_REV / SCREW_MM_PER_REV

PLOT_ORIGIN_X_MM = 200              # The X position to start the plotting from
PLOT_ORIGIN_Y_MM = 100              # The X position to start the plotting from
PLOT_SIZE_MM = 300                  # The size to scale the largest dimension (width or height) of the gcode data to

BELT_SPEED_MM_PER_SEC = 50          # The speed to move the X and Y axes at when plotting
SCREW_SPEED_MM_PER_SEC = 20         # The speed to move the Z axis at when plotting

PEN_LOWER_HEIGHT_MM = 62            # The height to lower the Z axis by to make the pen touch the paper
PEN_LIFT_AMOUNT_MM = 10             # The height to raise the pen by when switching between shapes to draw
PEN_LIFT_HEIGHT_MM = PEN_LOWER_HEIGHT_MM - PEN_LIFT_AMOUNT_MM

X_RETREAT_STEPS = 20                # The number of steps the X axis will retreat from its limit switch by, when homing
Y_RETREAT_STEPS = 50                # The number of steps the Y axis will retreat from its limit switch by, when homing
Z_RETREAT_STEPS = 20                # The number of steps the Z axis will retreat from its limit switch by, when homing
HOMING_SPEED_STEPS_PER_SEC = 5      # The speed to move each axis towards their limit switches by, when homing
RETREAT_SPEED_STEPS_PER_SEC = 200   # The speed to move each axis away (retreat) from their limit switches by, when homing
MAX_HOMING_STEPS = 3000             # The number of steps allowed before homing is considered to have failed


GCODE_FILE = "data.gcode"

if os.listdir("/").count(GCODE_FILE) == 1:
    os.remove(GCODE_FILE)

file = open(GCODE_FILE, 'w')

# Create a new Yukon object
yukon = Yukon()

class GcodeEngine:
    def parse_line(self, line):
        if line.startswith("G"):
            file.write(line)
            print("ok")
        elif line.startswith("M"):
            file.write(line)
            print("ok")

            # assume spindle off is the last command
            if line.startswith("M05"):
                file.close()
                self.start()

        elif line.startswith("?"):
            # TODO> deliver correct status
            print("<idle|WPos:0,0,0|Bf:0,0|WCO:0,0,0>")
            print("ok")
        elif line.startswith("$I"):
            # TODO: deliver a correct version string
            print("[VER:1.1y.20231229:]")
            print("ok")
        elif line.startswith("$$"):
            # this is a dump from GRBL to make Universal Gcode sender happy
            # TODO: adapt the values to this implementation
            print("$0=10")
            print("$1=25")
            print("$2=0")
            print("$3=0")
            print("$4=0")
            print("$5=0")
            print("$6=0")
            print("$10=255")
            print("$11=0.010")
            print("$12=0.002")
            print("$13=0")
            print("$20=0")
            print("$21=0")
            print("$22=0")
            print("$23=0")
            print("$24=25.000")
            print("$25=500.000")
            print("$26=250")
            print("$27=1.000")
            print("$30=1000")
            print("$31=0")
            print("$32=0")
            print("$100=250.000")
            print("$101=250.000")
            print("$102=250.000")
            print("$110=500.000")
            print("$111=500.000")
            print("$112=500.000")
            print("$120=10.000")
            print("$121=10.000")
            print("$122=10.000")
            print("$130=200.000")
            print("$131=200.000")
            print("$132=200.000")
            print("ok")
        elif line.startswith("$G"):
            print("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0.0 S0]")
            print("ok")
        else:
            print(f"UNKNOWN: {line}")

    def start(self):
        self.x_module = DualMotorModule()        # Create a DualMotorModule object for the X axis
        self.y1_module = DualMotorModule()       # Create a DualMotorModule object for one side of the Y axis
        self.y2_module = DualMotorModule()       # Create a DualMotorModule object for the other side of the Y axis
        self.z_module = DualMotorModule()        # Create a DualMotorModule object for the Z axis
        self.l1_module = QuadServoDirectModule(init_servos=False)    # Create a QuadServoDirectModule object for reading two limit switches
        self.l2_module = QuadServoDirectModule(init_servos=False)    # Create a QuadServoDirectModule object for reading two more limit switches
        # self.x_stepper = None                    # Variable for storing the X axis OkayStepper object created later
        # self.y_stepper = None                    # Variable for storing the Y axis OkayStepper object created later
        # self.z_stepper = None                    # Variable for storing the Z axis OkayStepper object created later
        # self.has_homed = False                   # Record if the plotter has been homed, meaning it knows its position
        # self.first_home = HOME_ON_PROGRAM_RUN    # Is this the first time homing?

        # Create an instance of the GCodeParser, giving it the functions to call for movement and pen control
        self.parser = GCodeParser(absolute_callback=self.move_to_xy,
                     relative_callback=self.move_by_xy,
                     lower_pen_callback=self.lower_pen,
                     lift_pen_callback=self.lift_pen,
                     raise_pen_callback=self.raise_pen)

        # Wrap the code in a try block, to catch any exceptions (including KeyboardInterrupt)
        try:
            # Register the four DualMotorModule objects with their respective slots
            yukon.register_with_slot(self.x_module, SLOT_X)
            yukon.register_with_slot(self.y1_module, SLOT_Y1)
            yukon.register_with_slot(self.y2_module, SLOT_Y2)
            yukon.register_with_slot(self.z_module, SLOT_Z)
            yukon.register_with_slot(self.l1_module, SLOT_LIMITS_1)
            yukon.register_with_slot(self.l2_module, SLOT_LIMITS_2)

            # Verify that the DualMotorModules are attached to Yukon, and initialise them
            yukon.verify_and_initialise()

            # Create OkayStepper classes to drive the plotter's stepper motors, with their units scaled to be in millimeters
            self.x_stepper = OkayStepper(
                self.x_module.motor1, self.x_module.motor2,
                steps_per_unit=BELT_STEPS_PER_MM)
            self.y_stepper = OkayStepper(
                self.y1_module.motor1, self.y1_module.motor2,
                self.y2_module.motor2, self.y2_module.motor1,     # The Y axis has two sets of motors as it is driven from both sides
                steps_per_unit=BELT_STEPS_PER_MM)
            self.z_stepper = OkayStepper(
                self.z_module.motor1, self.z_module.motor2,
                steps_per_unit=SCREW_STEPS_PER_MM)

            # Set the hardware current limit of each DualMotorModule to its maximum as OkayStepper controls current with PWM instead
            self.x_module.set_current_limit(DualMotorModule.MAX_CURRENT_LIMIT)
            self.y1_module.set_current_limit(DualMotorModule.MAX_CURRENT_LIMIT)
            self.y2_module.set_current_limit(DualMotorModule.MAX_CURRENT_LIMIT)
            self.z_module.set_current_limit(DualMotorModule.MAX_CURRENT_LIMIT)

            # Access the pins of the QuadServoDirect modules and set them up to use with limit switches
            z_up_limit = self.l1_module.servo_pins[0]
            x_left_limit = self.l1_module.servo_pins[2]
            x_right_limit = self.l2_module.servo_pins[0]
            y_back_limit = self.l2_module.servo_pins[2]
            z_up_limit.init(Pin.IN, Pin.PULL_UP)
            x_left_limit.init(Pin.IN, Pin.PULL_UP)
            x_right_limit.init(Pin.IN, Pin.PULL_UP)
            y_back_limit.init(Pin.IN, Pin.PULL_UP)

            # Have the parser load in the GCode file
            # print(f"Loading '{GCODE_FILE}' ... ", end="")
            self.parser.load_file(GCODE_FILE)

            self.activate()                  # Turn on power and energize the steppers

            # Home each axis in turn
            self.home_axis("Z", self.z_stepper, Z_RETREAT_STEPS, z_up_limit)
            self.home_axis("X", self.x_stepper, X_RETREAT_STEPS, x_right_limit)
            self.home_axis("Y", self.y_stepper, Y_RETREAT_STEPS, y_back_limit)

            # Move to the origin of the plot
            self.move_to_xy(PLOT_ORIGIN_X_MM, PLOT_ORIGIN_Y_MM)

            self.parser.start_parsing(PLOT_ORIGIN_X_MM,
                                      PLOT_ORIGIN_Y_MM,
                                      PLOT_SIZE_MM)

            # Loop until the BOOT/USER button is pressed
            while not yukon.is_boot_pressed():
                # Attempt to parse the next gcode command.
                # If there are no more commands to parse, False is returned
                if not self.parser.parse_next():
                    # Move to the origin of the plot
                    self.move_to_xy(PLOT_ORIGIN_X_MM, PLOT_ORIGIN_Y_MM)
    
                    self.deactivate()                        # De-energize the steppers and turn off power
                    break
        finally:
            # Release each stepper motor (if not already done so)
            if self.x_stepper is not None:
                self.x_stepper.release()

            if self.y_stepper is not None:
                self.y_stepper.release()

            if self.z_stepper is not None:
                self.z_stepper.release()

            # Put the board back into a safe state, regardless of how the program may have ended
            yukon.reset()

    # Activate the main power, all modules, and all steppers
    def activate(self):
        yukon.enable_main_output()
        self.x_module.enable()
        self.y1_module.enable()
        self.y2_module.enable()
        self.z_module.enable()
        self.x_stepper.hold()
        self.y_stepper.hold()
        self.z_stepper.hold()

    # Deactivate all steppers, all modules and the main output
    def deactivate(self):
        self.x_stepper.release()
        self.y_stepper.release()
        self.z_stepper.release()
        self.x_module.disable()
        self.y1_module.disable()
        self.y2_module.disable()
        self.z_module.disable()
        yukon.disable_main_output()

    # Home a single plotter axis, given a stepper, retreat steps, and limit switch
    def home_axis(self, name, stepper, retreat_steps, limit_switch):
        # print(f"Homing {name} ... ", end="")

        # Perform two passes of the homing, first at a high speed, then at a lower speed
        iteration = 0
        while iteration < 2:
            iteration += 1
            # Is the limit switch pressed?
            if limit_switch.value() == 1:
                # Move away from the limit switch by an amount sufficient to release the limit switch
                stepper.move_by_steps(retreat_steps, (1.0 / HOMING_SPEED_STEPS_PER_SEC) * (iteration * iteration))
                stepper.wait_for_move()

            # Move towards the limit switch one step at a time, until it is pressed or too many steps occur
            steps = 0
            while limit_switch.value() != 1:
                stepper.move_by_steps(-1, (1.0 / RETREAT_SPEED_STEPS_PER_SEC) * (iteration * iteration))
                stepper.wait_for_move()
                steps += 1

                # Have too many steps passed?
                if steps > MAX_HOMING_STEPS:
                    yukon.disable_main_output()
                    raise RuntimeError(f"Could not home {name}")

        stepper.hold()              # Keep the stepper energized but at a lower power than when moving
        stepper.zero_position()     # Use the stepper's current position as its zero value
        # print("done")


    # Move the plotter to a given x and y position (in mm) at a set speed
    def move_to_xy(self, x, y, speed=BELT_SPEED_MM_PER_SEC):
        dx = self.x_stepper.unit_diff(x)
        dy = self.y_stepper.unit_diff(y)

        duration = math.sqrt(dx * dx + dy * dy) / speed
        if duration > 0:
            # print(f"Moving to X {x}, Y {y}, in T {duration}")
            self.x_stepper.move_to(x, duration)
            self.y_stepper.move_to(y, duration)

            self.x_stepper.wait_for_move()
            self.y_stepper.wait_for_move()


    # Move the plotter by a given x and y position (in mm) at a set speed
    def move_by_xy(self, dx, dy, speed=BELT_SPEED_MM_PER_SEC):
        duration = math.sqrt(dx * dx + dy * dy) / speed
        if duration > 0:
            # print(f"Moving by X {dx}, Y {dy}, in T {duration}")
            self.x_stepper.move_by(dx, duration)
            self.y_stepper.move_by(dy, duration)

            self.x_stepper.wait_for_move()
            self.y_stepper.wait_for_move()


    # Move the plotter's pen to its drawing height
    def lower_pen(self):
        dz = self.z_stepper.unit_diff(PEN_LOWER_HEIGHT_MM)
        duration = abs(dz) / SCREW_SPEED_MM_PER_SEC
        if duration > 0:
            print(f"Lowering Pen to {PEN_LOWER_HEIGHT_MM}, in T {duration}")
            self.z_stepper.move_to(PEN_LOWER_HEIGHT_MM, duration)
            self.z_stepper.wait_for_move()


    # Move the plotter's pen to just above its drawing height
    def lift_pen(self):
        dz = self.z_stepper.unit_diff(PEN_LIFT_HEIGHT_MM)
        duration = abs(dz) / SCREW_SPEED_MM_PER_SEC
        if duration > 0:
            print(f"Lifting Pen to {PEN_LIFT_HEIGHT_MM}, in T {duration}")
            self.z_stepper.move_to(PEN_LIFT_HEIGHT_MM, duration)
            self.z_stepper.wait_for_move()


    # Move the plotter's pen to its home height
    def raise_pen(self):
        dz = self.z_stepper.units()
        duration = abs(dz) / SCREW_SPEED_MM_PER_SEC
        if duration > 0:
            print(f"Raising Pen to 0, in T {duration}")
            self.z_stepper.move_to(0, duration)
            self.z_stepper.wait_for_move()

gcodeengine = GcodeEngine()

print("Grbl 1.1y ['$' for help] Yukon")

try:
    # Loop until the BOOT/USER button is pressed
    while not yukon.is_boot_pressed():
        line = sys.stdin.readline()
        parse_line = gcodeengine.parse_line(line)
finally:
    # Put the board back into a safe state, regardless of how the program may have ended
    yukon.reset()
