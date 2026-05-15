from enum import Enum
import sys
import time
import logging
from typing import Optional, Callable, Any

from connect import ensure_serial_connection
from read import run_command
from set import set_command
from odrive_enums import ODriveAxisState, ODriveError, ProcedureResult

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('motor_controller.log', encoding='utf-8')
    ]
)

logger = logging.getLogger()
logger.setLevel(logging.INFO)

def enable_logging():
    logger.setLevel(logging.INFO)

def disable_logging():
    logger.setLevel(logging.CRITICAL)

class MotorState(Enum):
    INITIALIZING = "INITIALIZING"
    CHECKING_VOLTAGE = "CHECKING_VOLTAGE"
    CHECKING_STATUS = "CHECKING_STATUS"
    CALIBRATING = "CALIBRATING"
    CALIBRATION_WAITING = "CALIBRATION_WAITING"
    TRANSITIONING_TO_CLOSED_LOOP = "TRANSITIONING_TO_CLOSED_LOOP"
    CLOSED_LOOP = "CLOSED_LOOP"
    VELOCITY_CONTROL = "VELOCITY_CONTROL"
    ERROR = "ERROR"

class MotorController:
    def __init__(self, ser):
        self.ser = ser
        self.current_state = MotorState.INITIALIZING
        self.axis_state: Optional[ODriveAxisState] = None
        self.voltage: Optional[float] = None
        self.calibration_flag = False
        self.procedure_result: Optional[ProcedureResult] = None
        self.errors = []
        self.retry_count = 0
        self.max_retries = 3
        self.last_state_change = time.time()
        self.state_timeout = 30

        self.current_torque = 0.0
        self.previous_torque = 0.0
        self.torque_threshold = 0.5
        self.current_velocity = 1.0
        self.torque_control_enabled = True
        self.last_direction_change = time.time()
        self.direction_change_cooldown = 0.5
        self.stable_torque = 0.0
        self.torque_stabilization_count = 0
        self.required_stable_readings = 5

        self._setup_state_transitions()
        self._setup_state_handlers()

    def _setup_state_transitions(self):
        self.valid_transitions = {
            MotorState.INITIALIZING: [MotorState.CHECKING_VOLTAGE, MotorState.ERROR],
            MotorState.CHECKING_VOLTAGE: [MotorState.CHECKING_STATUS, MotorState.ERROR],
            MotorState.CHECKING_STATUS: [MotorState.CALIBRATING, MotorState.TRANSITIONING_TO_CLOSED_LOOP, MotorState.ERROR],
            MotorState.CALIBRATING: [MotorState.CALIBRATION_WAITING, MotorState.ERROR],
            MotorState.CALIBRATION_WAITING: [MotorState.TRANSITIONING_TO_CLOSED_LOOP, MotorState.ERROR],
            MotorState.TRANSITIONING_TO_CLOSED_LOOP: [MotorState.CLOSED_LOOP, MotorState.ERROR],
            MotorState.CLOSED_LOOP: [MotorState.VELOCITY_CONTROL, MotorState.ERROR],
            MotorState.VELOCITY_CONTROL: [MotorState.ERROR],
            MotorState.ERROR: []
        }

    def _setup_state_handlers(self):
        self.state_handlers = {
            MotorState.INITIALIZING: self.handle_initializing_state,
            MotorState.CHECKING_VOLTAGE: self.handle_checking_voltage_state,
            MotorState.CHECKING_STATUS: self.handle_checking_status_state,
            MotorState.CALIBRATING: self.handle_calibrating_state,
            MotorState.CALIBRATION_WAITING: self.handle_calibration_waiting_state,
            MotorState.TRANSITIONING_TO_CLOSED_LOOP: self.handle_transitioning_to_closed_loop_state,
            MotorState.CLOSED_LOOP: self.handle_closed_loop_state,
            MotorState.VELOCITY_CONTROL: self.handle_velocity_control_state,
            MotorState.ERROR: self.handle_error_state
        }

    def check_voltage(self, value: Optional[str]) -> bool:
        try:
            self.voltage = float(value) if value is not None else None
            logging.info(f"DC bus voltage: {self.voltage}V")
            return self.voltage is not None and self.voltage >= 10
        except ValueError:
            logging.error(f"Invalid voltage value: {value}")
            return False

    def check_status(self, value: Optional[str]) -> bool:
        try:
            if value is None:
                logging.error("Could not get status value")
                self.axis_state = None
                return False
            value = int(float(value))
            logging.info(f"Current Motor State: {value}")
            for state in ODriveAxisState:
                if state.value == value:
                    self.axis_state = state
                    logging.info(f"Motor state: {state.name}")
                    return True
            logging.warning(f"Unknown state: {value}")
            self.axis_state = None
            return False
        except (ValueError, TypeError) as e:
            logging.error(f"Status reading error: {e}")
            self.axis_state = None
            return False

    def check_procedure_result(self, value: Optional[str]) -> Optional[ProcedureResult]:
        try:
            if value is None:
                return None
            error_code = int(float(value))
            for result in ProcedureResult:
                if result.value == error_code:
                    self.procedure_result = result
                    logging.info(f"Procedure result: {result.name}")
                    return result
            return None
        except (ValueError, TypeError) as e:
            logging.error(f"Error reading procedure result: {e}")
            return None

    def check_active_errors(self, error_code: Optional[str]) -> None:
        try:
            error_code = int(float(error_code)) if error_code is not None else 0
            self.errors = []
            for error in ODriveError:
                if error_code & error.value:
                    self.errors.append(error.name)
            logging.info(f"Active errors: {self.errors}")
        except Exception as e:
            logging.error(f"Error in error checking: {e}")
            self.errors = []

    def clear_error(self) -> None:
        logging.info("Clearing errors...")
        if self.ser and self.ser.is_open:
            self.ser.write("sc\n".encode())
            time.sleep(0.1)

    def check_timeout(self) -> bool:
        if time.time() - self.last_state_change > self.state_timeout:
            logging.error(f"State change timeout: {self.current_state}")
            return True
        return False

    def change_state(self, new_state: MotorState) -> None:
        if new_state != self.current_state:
            logging.info(f"State changing: {self.current_state.value} -> {new_state.value}")
            self.current_state = new_state
            self.last_state_change = time.time()

    def handle_error(self, error: Exception = None) -> None:
        if error:
            logging.error(f"Error occurred: {str(error)}")
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            logging.warning(f"Retrying {self.retry_count}/{self.max_retries}")
            self.clear_error()
            self.change_state(MotorState.INITIALIZING)
        else:
            logging.critical("Maximum retry count reached")
            raise RuntimeError("Maximum retry count exceeded")

    def check_torque(self, value: Optional[str]) -> float:
        try:
            if value is None:
                logging.warning("Received None torque value")
                return 0.0
            new_torque = float(value)
            self.previous_torque = self.current_torque
            self.current_torque = new_torque
            current_time = time.time()
            if current_time - self.last_direction_change < self.direction_change_cooldown:
                logging.debug(f"COOLDOWN - Current: {self.current_torque:.2f}, Previous: {self.previous_torque:.2f}")
                return self.current_torque
            torque_change = abs(self.current_torque - self.previous_torque)
            logging.debug(f"TORQUE - Current: {self.current_torque:.2f}, Previous: {self.previous_torque:.2f}, Change: {torque_change:.2f}")
            if torque_change > self.torque_threshold:
                self.handle_torque_change(torque_change)
            return self.current_torque
        except (ValueError, TypeError) as e:
            logging.error(f"Error in check_torque: {e}")
            return 0.0

    def handle_torque_change(self, torque_change: float) -> None:
        try:
            current_time = time.time()
            if current_time - self.last_direction_change < self.direction_change_cooldown:
                logging.debug(f"SKIP DIRECTION CHANGE - Too soon after last change")
                return
            logging.info(f"DIRECTION CHANGE - Torque change: {torque_change:.2f}")
            self.current_velocity = -self.current_velocity
            success = set_command(self.ser, "axis0.controller.input_vel", self.current_velocity)
            if success:
                logging.info(f"MOTOR DIRECTION CHANGED - New velocity: {self.current_velocity}")
                self.last_direction_change = current_time
            else:
                logging.error("Failed to change motor direction")
                self.change_state(MotorState.ERROR)
        except Exception as e:
            logging.error(f"Error in handle_torque_change: {e}")
            self.change_state(MotorState.ERROR)

    def run_state_machine(self) -> None:
        if not self.ser.is_open:
            logging.error("Serial connection is closed.")
            return
        try:
            while True:
                try:
                    if self.current_state in self.state_handlers:
                        self.state_handlers[self.current_state]()
                    else:
                        raise ValueError(f"No handler for state: {self.current_state}")
                    if self.check_timeout():
                        raise TimeoutError("State timeout occurred")
                except Exception as e:
                    self.handle_error(e)
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.handle_shutdown()

    def handle_initializing_state(self):
        run_command(self.ser, "vbus_voltage", lambda v: self.check_voltage(v))
        if self.voltage is not None and self.voltage >= 10:
            self.change_state(MotorState.CHECKING_STATUS)
        else:
            logging.warning("Voltage is low or could not be read")

    def handle_checking_voltage_state(self):
        self.clear_error()
        run_command(self.ser, "axis0.current_state", lambda v: self.check_status(v))
        if self.axis_state == ODriveAxisState.AXIS_STATE_IDLE:
            self.change_state(MotorState.TRANSITIONING_TO_CLOSED_LOOP)
        elif self.axis_state is None:
            logging.error("Could not read motor state")
            time.sleep(1)

    def handle_checking_status_state(self):
        self.clear_error()
        run_command(self.ser, "axis0.current_state", lambda v: self.check_status(v))
        if self.axis_state == ODriveAxisState.AXIS_STATE_IDLE:
            self.change_state(MotorState.TRANSITIONING_TO_CLOSED_LOOP)
        elif self.axis_state is None:
            logging.error("Could not read motor state")
            time.sleep(1)

    def handle_calibrating_state(self):
        success = set_command(self.ser, "axis0.requested_state", 3)
        if success:
            self.change_state(MotorState.CALIBRATION_WAITING)
        else:
            logging.error("Could not start calibration")
            time.sleep(1)

    def handle_calibration_waiting_state(self):
        run_command(self.ser, "axis0.procedure_result", lambda v: self.check_procedure_result(v))
        if self.procedure_result == ProcedureResult.BUSY:
            logging.info("Calibration in progress...")
            time.sleep(0.5)
        elif self.procedure_result == ProcedureResult.CANCELLED:
            logging.warning("Calibration cancelled!")
            self.clear_error()
            run_command(self.ser, "axis0.active_errors", self.check_active_errors)
            if "ODRIVE_ERROR_CALIBRATION_ERROR" in self.errors:
                logging.error("Calibration error detected")
                self.change_state(MotorState.ERROR)
            else:
                logging.info("Restarting calibration...")
                self.change_state(MotorState.CALIBRATING)
        elif self.procedure_result == ProcedureResult.SUCCESS:
            logging.info("Calibration successful!")
            self.calibration_flag = True
            self.change_state(MotorState.TRANSITIONING_TO_CLOSED_LOOP)
        else:
            logging.error(f"Calibration error: {self.procedure_result}")
            self.change_state(MotorState.ERROR)

    def handle_transitioning_to_closed_loop_state(self):
        success = set_command(self.ser, "axis0.requested_state", 8)
        if not success:
            run_command(self.ser, "axis0.procedure_result", lambda v: self.check_procedure_result(v))
            if self.procedure_result == ProcedureResult.NOT_CALIBRATED:
                self.change_state(MotorState.CALIBRATING)
            else:
                logging.error(f"Closed loop transition error: {self.procedure_result}")
                time.sleep(1)
        else:
            self.change_state(MotorState.CLOSED_LOOP)

    def handle_closed_loop_state(self):
        run_command(self.ser, "axis0.current_state", lambda v: self.check_status(v))
        if self.axis_state != ODriveAxisState.AXIS_STATE_CLOSED_LOOP_CONTROL:
            logging.error("Lost closed loop state")
            self.change_state(MotorState.CHECKING_STATUS)
        else:
            logging.info("Transitioning to velocity control mode...")
            success = True
            success &= set_command(self.ser, "axis0.controller.config.control_mode", 2)
            success &= set_command(self.ser, "axis0.controller.config.input_mode", 1)
            success &= set_command(self.ser, "axis0.controller.input_vel", 1)
            if success:
                self.change_state(MotorState.VELOCITY_CONTROL)
            else:
                logging.error("Failed to transition to velocity control")
                self.change_state(MotorState.ERROR)

    def handle_velocity_control_state(self):
        try:
            run_command(self.ser, "axis0.current_state", lambda v: self.check_status(v))
            if self.axis_state != ODriveAxisState.AXIS_STATE_CLOSED_LOOP_CONTROL:
                logging.error("Lost velocity control state")
                self.change_state(MotorState.ERROR)
                return
            success = run_command(self.ser, "axis0.motor.foc.Iq_measured", lambda v: self.check_torque(v))
            while not success:
                success = run_command(self.ser, "axis0.motor.foc.Iq_measured", lambda v: self.check_torque(v))
                logging.debug("Retrying to read torque value")
        except Exception as e:
            logging.error(f"Error in velocity control: {e}")
            self.change_state(MotorState.ERROR)

    def handle_error_state(self):
        self.handle_error()

    def handle_shutdown(self):
        logging.warning("Keyboard interrupt detected! Setting motor to IDLE mode...")
        set_command(self.ser, "axis0.requested_state", ODriveAxisState.AXIS_STATE_IDLE.value)
        self.change_state(MotorState.INITIALIZING)
        logging.info("State machine safely stopped.")

def main():
    disable_logging()
    enable_logging()
    try:
        ser = ensure_serial_connection()
        if ser:
            motor_controller = MotorController(ser)
            motor_controller.run_state_machine()
        else:
            logging.error("Could not initialize serial connection")
            sys.exit(1)
    except KeyboardInterrupt:
        logging.info("Program terminated by user")
    except Exception as e:
        logging.error(f"Program error: {e}")
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            logging.info("Serial port closed")

if __name__ == "__main__":
    main()
    