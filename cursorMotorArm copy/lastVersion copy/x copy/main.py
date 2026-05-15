from enum import Enum
import sys
import time
import logging
from typing import Optional, Dict, Callable, Any

from connect import  ensure_serial_connection
from read import run_command
from set import set_command
from odrive_enums import ODriveAxisState, ODriveError, ProcedureResult

# Loglama yapılandırması
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
    """Loglamayı INFO seviyesinde etkinleştirir"""
    logger.setLevel(logging.INFO)  

def disable_logging():
    """Sadece WARNING ve üzeri logları gösterir"""
    logger.setLevel(logging.WARNING)

class MotorState(Enum):
    """Motor kontrol durumlarını temsil eden enum sınıf"""
    INITIALIZING = "INITIALIZING"
    CHECKING_VOLTAGE = "CHECKING_VOLTAGE"
    CHECKING_STATUS = "CHECKING_STATUS"
    CALIBRATING = "CALIBRATING"
    CALIBRATION_WAITING = "CALIBRATION_WAITING"
    TRANSITIONING_TO_CLOSED_LOOP = "TRANSITIONING_TO_CLOSED_LOOP"
    CLOSED_LOOP = "CLOSED_LOOP"
    VELOCITY_CONTROL = "VELOCITY_CONTROL"
    AWAITING_VOLTAGE_RECOVERY = "AWAITING_VOLTAGE_RECOVERY"  # Yeni durum
    ERROR = "ERROR"

class MotorController:
    """Motor kontrolü ve durum yönetimi için ana sınıf"""
    
    def __init__(self, ser):
        """
        Motor kontrolcüsünü başlatır
        
        Args:
            ser: Seri port bağlantı nesnesi
        """
        self.ser = ser
        self.current_state = MotorState.INITIALIZING
        self.axis_state: Optional[ODriveAxisState] = None
        self.voltage: Optional[float] = None
        self.calibration_flag = False
        self.procedure_result: Optional[ProcedureResult] = None
        self.errors = []
        self.retry_count = 0  # Genel hata deneme sayacı
        self.max_retries = 5
        
        self.initial_voltage_check_retries = 0 # Başlangıç voltaj kontrolü için deneme sayacı
        self.max_initial_voltage_check_retries = 3 # Başlangıç voltaj kontrolü için maksimum deneme

        self.voltage_recovery_retries = 0 # Voltaj kurtarma deneme sayacı
        self.max_voltage_recovery_retries = 5 # Voltaj kurtarma için maksimum deneme
        
        self.last_state_change = time.time()
        self.state_timeout = 10  # saniye
        
        # Tork kontrolü parametreleri
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
        
        # Durum geçişleri ve işleyicileri
        self._setup_state_transitions()
        self._setup_state_handlers()

    def _setup_state_transitions(self):
        """Geçerli durum geçişlerini tanımlar"""
        self.valid_transitions = {
            MotorState.INITIALIZING: [MotorState.CHECKING_VOLTAGE, MotorState.ERROR],
            MotorState.CHECKING_VOLTAGE: [MotorState.CHECKING_STATUS, MotorState.ERROR],
            MotorState.CHECKING_STATUS: [MotorState.CALIBRATING, MotorState.TRANSITIONING_TO_CLOSED_LOOP, MotorState.ERROR],
            MotorState.CALIBRATING: [MotorState.CALIBRATION_WAITING, MotorState.ERROR],
            MotorState.CALIBRATION_WAITING: [MotorState.TRANSITIONING_TO_CLOSED_LOOP, MotorState.ERROR],
            MotorState.TRANSITIONING_TO_CLOSED_LOOP: [MotorState.CLOSED_LOOP, MotorState.ERROR],
            MotorState.CLOSED_LOOP: [MotorState.VELOCITY_CONTROL, MotorState.AWAITING_VOLTAGE_RECOVERY, MotorState.ERROR],
            MotorState.VELOCITY_CONTROL: [MotorState.AWAITING_VOLTAGE_RECOVERY, MotorState.ERROR],
            MotorState.AWAITING_VOLTAGE_RECOVERY: [MotorState.INITIALIZING, MotorState.ERROR],
            MotorState.ERROR: []
        }

    def _setup_state_handlers(self):
        """Durum işleyicilerini tanımlar"""
        self.state_handlers = {
            MotorState.INITIALIZING: self.handle_initializing_state,
            MotorState.CHECKING_VOLTAGE: self.handle_checking_voltage_state,
            MotorState.CHECKING_STATUS: self.handle_checking_status_state,
            MotorState.CALIBRATING: self.handle_calibrating_state,
            MotorState.CALIBRATION_WAITING: self.handle_calibration_waiting_state,
            MotorState.TRANSITIONING_TO_CLOSED_LOOP: self.handle_transitioning_to_closed_loop_state,
            MotorState.CLOSED_LOOP: self.handle_closed_loop_state,
            MotorState.VELOCITY_CONTROL: self.handle_velocity_control_state,
            MotorState.AWAITING_VOLTAGE_RECOVERY: self.handle_awaiting_voltage_recovery_state,
            MotorState.ERROR: self.handle_error_state
        }

    def handle_checking_voltage_state(self) -> None:
        """
        CHECKING_VOLTAGE durumunu yönetir.
        Bu metodun içeriğini, voltaj kontrolü ve sonraki duruma geçiş mantığıyla doldurmanız gerekmektedir.
        """
        logging.info(f"State: {self.current_state.value} - Handling checking voltage.")
        
        # Örnek voltaj okuma ve kontrol mantığı:
        # voltage_str = run_command(self.ser, "r axis0.vbus_voltage")
        # if self.check_voltage(voltage_str):
        #     logging.info(f"Voltage OK: {self.voltage}V")
        #     self.initial_voltage_check_retries = 0 # Başarılı kontrolde sayacı sıfırla
        #     # Voltajda bir hata olup olmadığını kontrol et (örneğin ODriveError'dan)
        #     # Eğer ODriveError.DC_BUS_UNDER_VOLTAGE gibi bir hata varsa AWAITING_VOLTAGE_RECOVERY'e geç
        #     active_errors_str = run_command(self.ser, "r axis0.active_errors")
        #     self.check_active_errors(active_errors_str) # Hataları güncelle
        #     if ODriveError.DC_BUS_UNDER_VOLTAGE.name in self.errors or \
        #        ODriveError.DC_BUS_OVER_VOLTAGE.name in self.errors:
        #         logging.warning("Voltage related error detected on ODrive. Transitioning to AWAITING_VOLTAGE_RECOVERY.")
        #         self.transition_to(MotorState.AWAITING_VOLTAGE_RECOVERY)
        #     else:
        #         self.transition_to(MotorState.CHECKING_STATUS)
        # else:
        #     logging.warning(f"Voltage check failed. Value: {voltage_str}")
        #     self.initial_voltage_check_retries += 1
        #     if self.initial_voltage_check_retries >= self.max_initial_voltage_check_retries:
        #         logging.error("Max retries for initial voltage check reached. Transitioning to ERROR.")
        #         self.transition_to(MotorState.ERROR)
        #     else:
        #         logging.info(f"Retrying voltage check ({self.initial_voltage_check_retries}/{self.max_initial_voltage_check_retries}). Waiting 1 second.")
        #         time.sleep(1) # Tekrar denemeden önce kısa bir bekleme
        #         # State değişmediği için bir sonraki döngüde bu handler tekrar çağrılacak
        pass # TODO: Bu metodun gerçek implementasyonunu ekleyin.

    def check_voltage(self, value: Optional[str]) -> bool:
        """
        Voltaj değerini kontrol eder
        
        Args:
            value: Okunan voltaj değeri
            
        Returns:
            bool: Voltaj uygunsa True
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
        Motor durumunu kontrol eder
        
        Args:
            value: Okunan durum değeri
            
        Returns:
            bool: Durum geçerliyse True
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
        İşlem sonucunu kontrol eder
        
        Args:
            value: Okunan işlem sonucu
            
        Returns:
            Optional[ProcedureResult]: İşlem sonucu enum değeri
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
        Aktif hataları kontrol eder ve kaydeder
        
        Args:
            error_code: Hata kodu
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
        """Hataları temizler"""
        logging.info("Clearing errors...")
        if self.ser and self.ser.is_open:
            self.ser.write("sc\n".encode())
            time.sleep(0.1)

    def check_timeout(self) -> bool:
        """
        Durum değişimi zaman aşımını kontrol eder
        
        Returns:
            bool: Zaman aşımı oluştuysa True
        """
        if time.time() - self.last_state_change > self.state_timeout:
            logging.error(f"State change timeout: {self.current_state}")
            return True
        return False

    def change_state(self, new_state: MotorState) -> None:
        """
        Durum değişimini gerçekleştirir ve loglar
        
        Args:
            new_state: Yeni durum
        """
        if new_state != self.current_state:
            logging.info(f"State changing: {self.current_state.value} -> {new_state.value}")
            self.current_state = new_state
            self.last_state_change = time.time()

    def handle_error(self, error: Exception = None) -> None:
        """Hata durumunu yönetir"""
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
        """
        Tork değerini kontrol eder ve değişimleri yönetir
        
        Args:
            value: Okunan tork değeri
            
        Returns:
            float: Güncel tork değeri
        """
        try:
            if value is None:
                logging.warning("Received None torque value")
                return 0.0
            
            new_torque = float(value)
            
            self.previous_torque = self.current_torque
            self.current_torque = new_torque
            
            current_time = time.time()
            # Yön değişimi sonrası bekleme süresi kontrolü
            if current_time - self.last_direction_change < self.direction_change_cooldown:
                logging.debug(f"COOLDOWN - Current: {self.current_torque:.2f}, Previous: {self.previous_torque:.2f}")
                return self.current_torque
            
            # Tork değişimini hesapla
            torque_change = abs(self.current_torque - self.previous_torque)
            logging.debug(f"TORQUE - Current: {self.current_torque:.2f}, Previous: {self.previous_torque:.2f}, Change: {torque_change:.2f}")
            
            if torque_change > self.torque_threshold:
                self.handle_torque_change(torque_change)
            
            return self.current_torque
            
        except (ValueError, TypeError) as e:
            logging.error(f"Error in check_torque: {e}")
            return 0.0

    def handle_torque_change(self, torque_change: float) -> None:
        """
        Ani tork değişimlerini yönetir ve motor yönünü değiştirir
        
        Args:
            torque_change: Tork değişim miktarı
        """
        try:
            current_time = time.time()
            
            # Yön değişimi için minimum süre kontrolü
            if current_time - self.last_direction_change < self.direction_change_cooldown:
                logging.debug(f"SKIP DIRECTION CHANGE - Too soon after last change")
                return
                
            logging.info(f"DIRECTION CHANGE - Torque change: {torque_change:.2f}")
            # Motor yönünü değiştir
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
        """Ana durum makinesi döngüsü"""
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

                time.sleep(0.1)  # Ana döngü gecikmesi

        except KeyboardInterrupt:
            self.handle_shutdown()

    def handle_initializing_state(self):
        """Initialization state: Wait for sufficient voltage before proceeding."""
        run_command(self.ser, "vbus_voltage", lambda v: self.check_voltage(v))
        
        if self.voltage is None:
            logging.error("Voltage could not be read!")
            self.initial_voltage_check_retries += 1
            if self.initial_voltage_check_retries > self.max_initial_voltage_check_retries:
                logging.error("Maximum retries for initial voltage check reached. Moving to ERROR state.")
                self.change_state(MotorState.ERROR)
            else:
                logging.warning(f"Waiting for voltage to recover... (Attempt {self.initial_voltage_check_retries}/{self.max_initial_voltage_check_retries})")
                time.sleep(2)
            return

        if self.voltage < 10:
            logging.error(f"Voltage too low: {self.voltage}V.")
            self.initial_voltage_check_retries += 1
            if self.initial_voltage_check_retries > self.max_initial_voltage_check_retries:
                logging.error("Maximum retries for initial low voltage reached. Moving to ERROR state.")
                self.change_state(MotorState.ERROR)
            else:
                logging.warning(f"Waiting for voltage to rise... (Attempt {self.initial_voltage_check_retries}/{self.max_initial_voltage_check_retries})")
                time.sleep(2)
            return

        logging.info("Voltage is sufficient. Proceeding to status check.")
        self.initial_voltage_check_retries = 0 # Reset counter on success
        self.change_state(MotorState.CHECKING_STATUS)

    def handle_closed_loop_state(self):
        """Closed loop state: Monitor voltage and stop if it drops."""
        run_command(self.ser, "vbus_voltage", lambda v: self.check_voltage(v))
        if self.voltage is not None and self.voltage < 10:
            logging.warning(f"WARNING dc bus uner voltage hata uyarısı: {self.voltage}V. Stopping motor for safety.")
            set_command(self.ser, "axis0.requested_state", ODriveAxisState.AXIS_STATE_IDLE.value)
            self.change_state(MotorState.AWAITING_VOLTAGE_RECOVERY)
            return
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
        """Velocity control state: Monitor voltage and stop if it drops."""
        run_command(self.ser, "vbus_voltage", lambda v: self.check_voltage(v))
        if self.voltage is not None and self.voltage < 10:
            logging.warning(f"WARNING dc bus uner voltage hata uyarısı: {self.voltage}V. Stopping motor for safety.")
            set_command(self.ser, "axis0.requested_state", ODriveAxisState.AXIS_STATE_IDLE.value)
            self.change_state(MotorState.AWAITING_VOLTAGE_RECOVERY)
            return
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
        """Hata durumu işlemleri"""
        self.handle_error() # Bu genel hata işleyiciyi çağırır

    def handle_awaiting_voltage_recovery_state(self):
        """Awaiting voltage recovery state: Monitor voltage and try to recover or go to error."""
        logging.info("Awaiting voltage recovery...")
        run_command(self.ser, "vbus_voltage", lambda v: self.check_voltage(v))

        if self.voltage is not None and self.voltage >= 10:
            logging.info(f"Voltage recovered: {self.voltage}V. Returning to initializing state.")
            self.voltage_recovery_retries = 0 # Reset counter
            self.change_state(MotorState.INITIALIZING) # Ya da CHECKING_STATUS gibi daha uygun bir duruma
            return
        
        self.voltage_recovery_retries += 1
        if self.voltage_recovery_retries > self.max_voltage_recovery_retries:
            logging.error(f"Maximum retries for voltage recovery reached. Last voltage: {self.voltage}V. Moving to ERROR state.")
            self.change_state(MotorState.ERROR)
        else:
            if self.voltage is None:
                logging.warning(f"Could not read voltage. Waiting... (Attempt {self.voltage_recovery_retries}/{self.max_voltage_recovery_retries})")
            else:
                logging.warning(f"Voltage still low: {self.voltage}V. Waiting... (Attempt {self.voltage_recovery_retries}/{self.max_voltage_recovery_retries})")
            time.sleep(3) # Bekleme süresi

    def handle_shutdown(self):
        """Kapatma işlemleri"""
        logging.warning("Keyboard interrupt detected! Setting motor to IDLE mode...")
        set_command(self.ser, "axis0.requested_state", ODriveAxisState.AXIS_STATE_IDLE.value)
        self.change_state(MotorState.INITIALIZING)
        logging.info("State machine safely stopped.")

def main():
    """Ana program döngüsü"""
    disable_logging()    
    # enable_logging()  
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