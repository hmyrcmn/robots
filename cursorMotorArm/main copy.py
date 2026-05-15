from enum import Enum
import sys
import time
import logging
from typing import Optional, Callable, Dict, List

from connect import connect_to_serial, ensure_serial_connection
from read import run_command
from set import set_command
from odrive_enums import ODriveAxisState, ODriveError, ProcedureResult

logger = logging.getLogger()
logger.setLevel(logging.INFO) 

def enable_logging():
    logger.setLevel(logging.DEBUG)  # Daha detaylı log için DEBUG seviyesine çekilebilir

def disable_logging():
    logger.setLevel(logging.ERROR)  # Sadece kritik hataları göster


# Logging configuration
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('motor_controller.log', encoding='utf-8')
    ]
)

class MotorState(Enum):
    """Enum class representing motor control states"""
    INITIALIZING = "INITIALIZING"
    CHECKING_VOLTAGE = "CHECKING_VOLTAGE"
    CHECKING_STATUS = "CHECKING_STATUS"
    CALIBRATING = "CALIBRATING"
    CALIBRATION_WAITING = "CALIBRATION_WAITING"
    TRANSITIONING_TO_CLOSED_LOOP = "TRANSITIONING_TO_CLOSED_LOOP"
    CLOSED_LOOP = "CLOSED_LOOP"
    VELOCITY_CONTROL = "VELOCITY_CONTROL"
    ERROR = "ERROR"

class StateEvent:
    def __init__(self):
        self.handlers: List[Callable] = []

    def add_handler(self, handler: Callable):
        self.handlers.append(handler)

    def trigger(self, *args, **kwargs):
        for handler in self.handlers:
            handler(*args, **kwargs)

class MotorController:
    """Main class for motor control and state management"""
    
    def __init__(self, ser):
        """
        Initialize motor controller
        
        Args:
            ser: Serial port connection object
        """
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
        self.state_timeout = 30  # seconds
        self.current_torque = 0.0
        self.previous_torque = 0.0
        self.torque_threshold = 0.5  # Ani değişim eşiği
        self.current_velocity = 1.0  # Başlangıç hızı
        self.torque_control_enabled = True
        self.last_direction_change = time.time()
        self.direction_change_cooldown = 0.5  # Yön değişimi sonrası bekleme süresi (saniye)
        self.stable_torque = 0.0  # Kararlı durum torku
        self.torque_stabilization_count = 0  # Kararlı durum sayacı
        self.required_stable_readings = 5  # Kararlı durum için gereken okuma sayısı
        self.valid_transitions = {
            MotorState.INITIALIZING: [MotorState.CHECKING_VOLTAGE, MotorState.CHECKING_STATUS, MotorState.ERROR],
            MotorState.CHECKING_VOLTAGE: [MotorState.CHECKING_STATUS, MotorState.ERROR],
            MotorState.CHECKING_STATUS: [MotorState.CALIBRATING, MotorState.TRANSITIONING_TO_CLOSED_LOOP, MotorState.ERROR],
            MotorState.CALIBRATING: [MotorState.CALIBRATION_WAITING, MotorState.ERROR],
            MotorState.CALIBRATION_WAITING: [MotorState.TRANSITIONING_TO_CLOSED_LOOP, MotorState.ERROR],
            MotorState.TRANSITIONING_TO_CLOSED_LOOP: [MotorState.CLOSED_LOOP, MotorState.ERROR],
            MotorState.CLOSED_LOOP: [MotorState.VELOCITY_CONTROL, MotorState.ERROR],
            MotorState.VELOCITY_CONTROL: [MotorState.ERROR],
            MotorState.ERROR: [MotorState.INITIALIZING]
        }
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
        self.state_history = []  # State geçiş geçmişi
        self.max_history_size = 10
        self.state_events = {
            "on_state_change": StateEvent(),
            "on_error": StateEvent(),
            "on_calibration_complete": StateEvent(),
        }
        self.stop_motor_on_error = True  # Hata durumunda motoru durdur

    def check_voltage(self, value: Optional[str]) -> bool:
        """
        Check voltage value
        
        Args:
            value: Read voltage value
            
        Returns:
            bool: True if voltage is appropriate
        """
        try:
            self.voltage = float(value) if value is not None else None
            logging.info(f"DC bus voltage: {self.voltage}V")
            return self.voltage is not None and self.voltage >= 10
        except ValueError:
            logging.error(f"Invalid voltage value: {value}")
            return False

    def check_status(self, value: Optional[str]) -> bool:
        """
        Check motor status
        
        Args:
            value: Read status value
            
        Returns:
            bool: True if status is valid
        """
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
        """
        Check procedure result
        
        Args:
            value: Read procedure result
            
        Returns:
            Optional[ProcedureResult]: Procedure result enum value
        """
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
        """
        Check and record active errors
        
        Args:
            error_code: Error code
        """
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
        """Clear errors"""
        logging.info("Clearing errors...")
        self.errors = []
        self.retry_count = 0  # Hata temizlendiğinde retry sayacını sıfırla

    def check_timeout(self) -> bool:
        """Durum değişim zaman aşımı kontrolü"""
        if time.time() - self.last_state_change > self.state_timeout:
            logging.error(f"Durum değişim zaman aşımı: {self.current_state}")
            self.handle_error(f"Zaman aşımı: {self.current_state}")
            return True
        return False

    def change_state(self, new_state: MotorState) -> None:
        """
        Perform and log state change with validation
        
        Args:
            new_state: New state to transition to
        """
        # Aynı duruma geçiş kontrolü
        if new_state == self.current_state:
            return
            
        # Geçiş validasyonu
        if new_state not in self.valid_transitions[self.current_state]:
            logging.error(f"Geçersiz durum geçişi: {self.current_state} -> {new_state}")
            if new_state != MotorState.ERROR:  # Sonsuz döngüyü önle
                self.change_state(MotorState.ERROR)
            return
        
        # Durum değişikliğini gerçekleştir
        old_state = self.current_state
        self.current_state = new_state
        self.last_state_change = time.time()
        
        # Durum geçiş logları ve event tetikleme
        logging.info(f"Durum değişiyor: {old_state.value} -> {new_state.value}")
        self.state_history.append((old_state, time.time()))
        if len(self.state_history) > self.max_history_size:
            self.state_history.pop(0)
        self.state_events["on_state_change"].trigger(old_state, new_state)

    def handle_error_state(self):
        """Hata durumu işleme"""
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            logging.warning(f"Hata durumundan kurtarma denemesi {self.retry_count}/{self.max_retries}")
            self.clear_error()
            time.sleep(1)
            
            # Motoru yeniden başlatma
            if self.stop_motor():
                time.sleep(0.5)  # Motorun durması için bekle
                self.change_state(MotorState.INITIALIZING)
            else:
                logging.error("Motor durdurulamadı")
                raise RuntimeError("Motor kontrol edilemiyor")
        else:
            logging.critical("Maksimum yeniden deneme sayısına ulaşıldı")
            self.stop_motor()  # Son bir kez daha durdurma denemesi
            raise RuntimeError("Maksimum yeniden deneme sayısı aşıldı")

    def check_torque(self, value: Optional[str]) -> float:
        """Tork değerini kontrol et ve işle"""
        try:
            if value is None:
                logging.warning("Received None torque value")
                return self.current_torque  # Önceki değeri koru
            
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
            logging.warning(f"Torque reading error: {e}")
            return self.current_torque  # Önceki değeri koru

    def handle_torque_change(self, torque_change: float) -> None:
        """Ani tork değişimlerini yönet ve motor yönünü değiştir"""
        try:
            current_time = time.time()
            
            # Yön değişimi için minimum süre kontrolü
            if current_time - self.last_direction_change < self.direction_change_cooldown:
                print(f"SKIP DIRECTION CHANGE - Too soon after last change")
                return
                
            print(f"DIRECTION CHANGE - Torque change: {torque_change:.2f}")
            # Motor yönünü değiştir
            self.current_velocity = -self.current_velocity
            success = set_command(self.ser, "axis0.controller.input_vel", self.current_velocity)
            
            if success:
                print(f"MOTOR DIRECTION CHANGED - New velocity: {self.current_velocity}")
                self.last_direction_change = current_time
            else:
                print("ERROR: Failed to change motor direction")
                self.change_state(MotorState.ERROR)
                
        except Exception as e:
            print(f"ERROR in handle_torque_change: {e}")
            self.change_state(MotorState.ERROR)

    def run_state_machine(self) -> None:
        """Ana durum makinesi döngüsü"""
        if not self.ser.is_open:
            logging.error("Serial bağlantı kapalı")
            return

        try:
            while True:
                try:
                    if self.current_state in self.state_handlers:
                        self.state_handlers[self.current_state]()
                    else:
                        raise ValueError(f"Durum işleyici bulunamadı: {self.current_state}")

                    if self.check_timeout():
                        continue  # Timeout handle_error'a yönlendirildi

                except Exception as e:
                    self.handle_error(e)
                    if self.current_state == MotorState.ERROR:
                        time.sleep(1)  # Hata durumunda kısa bekleme

                time.sleep(0.1)  # Ana döngü gecikmesi

        except KeyboardInterrupt:
            logging.warning("Program kullanıcı tarafından durduruldu")
            self.stop_motor()
        except Exception as e:
            logging.critical(f"Kritik hata: {e}")
            self.stop_motor()
            raise

    def handle_initializing_state(self):
        """Handle initializing state operations"""
        run_command(self.ser, "vbus_voltage", lambda v: self.check_voltage(v))
        if self.voltage is not None and self.voltage >= 10:
            self.change_state(MotorState.CHECKING_STATUS)
        else:
            logging.warning("Voltage is low or could not be read")

    def handle_checking_voltage_state(self):
        """Handle checking voltage state operations"""
        self.clear_error()
        run_command(self.ser, "axis0.current_state", lambda v: self.check_status(v))

        if self.axis_state == ODriveAxisState.AXIS_STATE_IDLE:
            self.change_state(MotorState.TRANSITIONING_TO_CLOSED_LOOP)
        elif self.axis_state is None:
            logging.error("Could not read motor state")
            time.sleep(1)

    def handle_checking_status_state(self):
        """Handle checking status state operations"""
        self.clear_error()
        run_command(self.ser, "axis0.current_state", lambda v: self.check_status(v))

        if self.axis_state == ODriveAxisState.AXIS_STATE_IDLE:
            self.change_state(MotorState.TRANSITIONING_TO_CLOSED_LOOP)
        elif self.axis_state is None:
            logging.error("Could not read motor state")
            time.sleep(1)

    def handle_calibrating_state(self):
        """Handle calibrating state operations"""
        success = set_command(self.ser, "axis0.requested_state", 3)
        if success:
            self.change_state(MotorState.CALIBRATION_WAITING)
        else:
            logging.error("Could not start calibration")
            time.sleep(1)

    def handle_calibration_waiting_state(self):
        """Handle calibration waiting state operations"""
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
            self.state_events["on_calibration_complete"].trigger()
        else:
            logging.error(f"Calibration error: {self.procedure_result}")
            self.change_state(MotorState.ERROR)

    def handle_transitioning_to_closed_loop_state(self):
        """Closed loop durumuna geçiş yönetimi"""
        try:
            # Önce IDLE durumuna geç
            success = set_command(self.ser, "axis0.requested_state", ODriveAxisState.AXIS_STATE_IDLE.value)
            if not success:
                raise Exception("Failed to transition to IDLE state")
            
            time.sleep(0.5)  # Durum değişimi için bekle
            
            # Sonra CLOSED_LOOP durumuna geç
            success = set_command(self.ser, "axis0.requested_state", ODriveAxisState.AXIS_STATE_CLOSED_LOOP_CONTROL.value)
            if not success:
                # Procedure result'ı kontrol et
                run_command(self.ser, "axis0.procedure_result", lambda v: self.check_procedure_result(v))
                if self.procedure_result == ProcedureResult.CANCELLED:
                    logging.warning("Transition cancelled, retrying...")
                    time.sleep(1)
                    return
                raise Exception(f"Failed to transition to closed loop: {self.procedure_result}")
            
            self.change_state(MotorState.CLOSED_LOOP)
            
        except Exception as e:
            logging.error(f"Closed loop transition error: {e}")
            self.handle_error(str(e))

    def handle_closed_loop_state(self):
        """Handle closed loop state operations"""
        run_command(self.ser, "axis0.current_state", lambda v: self.check_status(v))
        if self.axis_state != ODriveAxisState.AXIS_STATE_CLOSED_LOOP_CONTROL:
            logging.error("Lost closed loop state")
            self.change_state(MotorState.CHECKING_STATUS)
        else:
            # If we are in closed loop, transition to velocity control
            logging.info("Transitioning to velocity control mode...")
            success = True
            # Set control mode to velocity control
            success &= set_command(self.ser, "axis0.controller.config.control_mode", 2)
            # Set input mode to velocity
            success &= set_command(self.ser, "axis0.controller.config.input_mode", 1)
            # Set initial velocity (1 turn per second)
            success &= set_command(self.ser, "axis0.controller.input_vel", 1)
            
            if success:
                self.change_state(MotorState.VELOCITY_CONTROL)
            else:
                logging.error("Failed to transition to velocity control")
                self.change_state(MotorState.ERROR)

    def handle_velocity_control_state(self):
        """Hız kontrolü durumu işleme"""
        try:
            # Motor durumunu kontrol et
            run_command(self.ser, "axis0.current_state", lambda v: self.check_status(v))
            if self.axis_state != ODriveAxisState.AXIS_STATE_CLOSED_LOOP_CONTROL:
                logging.error("Lost closed loop control")
                self.change_state(MotorState.CHECKING_STATUS)
                return

            # Tork değerini oku ve kontrol et
            success = run_command(self.ser, "axis0.motor.foc.Iq_measured", lambda v: self.check_torque(v))
            if not success:
                logging.warning("Torque read failed, retrying...")
                time.sleep(0.1)  # Kısa bekleme ekle
                return  # Durumu değiştirmeden devam et
                
        except Exception as e:
            logging.error(f"Velocity control error: {e}")
            self.handle_error(str(e))

    def stop_motor(self):
        """Motoru güvenli bir şekilde durdur"""
        try:
            logging.info("Motor durduruluyor...")
            # Önce hızı sıfırla
            set_command(self.ser, "axis0.controller.input_vel", 0)
            time.sleep(0.1)  # Kısa bekleme
            # Sonra IDLE moduna geç
            set_command(self.ser, "axis0.requested_state", ODriveAxisState.AXIS_STATE_IDLE.value)
            logging.info("Motor başarıyla durduruldu")
            return True
        except Exception as e:
            logging.error(f"Motor durdurma hatası: {e}")
            return False

    def handle_error(self, error=None):
        """Geliştirilmiş hata yönetimi"""
        if error:
            logging.error(f"Motor hatası: {error}")
        
        if self.stop_motor_on_error:
            self.stop_motor()
        
        self.errors.append(str(error) if error else "Bilinmeyen hata")
        self.change_state(MotorState.ERROR)

def main():
    """Main program loop"""
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