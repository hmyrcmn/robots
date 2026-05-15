from enum import Enum, IntFlag, auto
import sys
import time
import logging
from typing import Optional, Dict, Callable, Any, List, Set, Type
from abc import ABC, abstractmethod
from dataclasses import dataclass
import yaml

from connect import connect_to_serial, ensure_serial_connection
from read import run_command
from set import set_command
from odrive_enums import ODriveAxisState, ProcedureResult

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
    """Loglamayı sadece kritik hatalar için etkinleştirir"""
    logger.setLevel(logging.CRITICAL)

class MotorState(Enum):
    """Motor kontrol durumlarını temsil eden enum sınıfı"""
    INITIALIZING = ("INITIALIZING", ["CHECKING_VOLTAGE", "ERROR"])
    CHECKING_VOLTAGE = ("CHECKING_VOLTAGE", ["CHECKING_STATUS", "ERROR"])
    CHECKING_STATUS = ("CHECKING_STATUS", ["CALIBRATING", "TRANSITIONING_TO_CLOSED_LOOP", "ERROR"])
    CALIBRATING = ("CALIBRATING", ["CALIBRATION_WAITING", "ERROR"])
    CALIBRATION_WAITING = ("CALIBRATION_WAITING", ["TRANSITIONING_TO_CLOSED_LOOP", "ERROR"])
    TRANSITIONING_TO_CLOSED_LOOP = ("TRANSITIONING_TO_CLOSED_LOOP", ["CLOSED_LOOP", "ERROR"])
    CLOSED_LOOP = ("CLOSED_LOOP", ["VELOCITY_CONTROL", "ERROR"])
    VELOCITY_CONTROL = ("VELOCITY_CONTROL", ["ERROR"])
    ERROR = ("ERROR", [])

    def __init__(self, name: str, valid_transitions: List[str]):
        self.state_name = name
        self.valid_transitions = valid_transitions

class ErrorState(Enum):
    """ODrive hata durumlarını temsil eden enum sınıfı"""
    NO_ERROR = "NO_ERROR"
    SYSTEM_ERROR = "SYSTEM_ERROR"  # Sistem seviyesi hatalar
    VOLTAGE_ERROR = "VOLTAGE_ERROR"  # Voltaj ile ilgili hatalar
    CURRENT_ERROR = "CURRENT_ERROR"  # Akım ile ilgili hatalar
    TEMPERATURE_ERROR = "TEMPERATURE_ERROR"  # Sıcaklık ile ilgili hatalar
    MOTOR_ERROR = "MOTOR_ERROR"  # Motor ile ilgili hatalar
    ENCODER_ERROR = "ENCODER_ERROR"  # Encoder ile ilgili hatalar
    CONTROLLER_ERROR = "CONTROLLER_ERROR"  # Kontrolcü ile ilgili hatalar
    TIMING_ERROR = "TIMING_ERROR"  # Zamanlama ile ilgili hatalar
    COMMUNICATION_ERROR = "COMMUNICATION_ERROR"  # İletişim ile ilgili hatalar
    CALIBRATION_ERROR = "CALIBRATION_ERROR"  # Kalibrasyon ile ilgili hatalar
    SAFETY_ERROR = "SAFETY_ERROR"  # Güvenlik ile ilgili hatalar

class ErrorType(Enum):
    VOLTAGE_ERROR = "voltage_error"
    CURRENT_ERROR = "current_error"
    TEMPERATURE_ERROR = "temperature_error"

class RecoveryStrategy(ABC):
    @abstractmethod
    def recover(self, context: 'MotorContext') -> bool:
        pass

class VoltageErrorRecovery(RecoveryStrategy):
    def recover(self, context: 'MotorContext') -> bool:
        logging.info("Voltaj hatası gideriliyor...")
        # Voltajı tekrar kontrol et
        voltage = context.check_voltage()
        if voltage >= context.config.min_voltage:
            return True
        return False

class ErrorHandler:
    def __init__(self):
        self._recovery_strategies = {
            ErrorType.VOLTAGE_ERROR: VoltageErrorRecovery(),
            ErrorType.CURRENT_ERROR: CurrentErrorRecovery(),
            ErrorType.TEMPERATURE_ERROR: TemperatureErrorRecovery()
        }
    
    def handle_error(self, error_type: ErrorType, context: 'MotorContext') -> bool:
        """Hatayı yönet ve recovery stratejisini uygula"""
        if error_type in self._recovery_strategies:
            return self._recovery_strategies[error_type].recover(context)
        return False

class StateHandler:
    def handle(self, controller: 'MotorController') -> None:
        pass

class State(ABC):
    @abstractmethod
    def enter(self, context: 'MotorContext') -> None:
        pass

    @abstractmethod
    def exit(self, context: 'MotorContext') -> None:
        pass

class InitializingState(State):
    def enter(self, context: 'MotorContext') -> None:
        context.reset_errors()
        
    def handle(self, context: 'MotorContext') -> None:
        voltage = context.check_voltage()
        if voltage >= context.config.min_voltage:
            context.transition_to(CheckingVoltageState())

class MotorConfig:
    def __init__(self, min_voltage: float, max_current: float, temperature_limit: float, retry_limit: int, timeout_seconds: float):
        self.min_voltage = min_voltage
        self.max_current = max_current
        self.temperature_limit = temperature_limit
        self.retry_limit = retry_limit
        self.timeout_seconds = timeout_seconds

    @classmethod
    def from_yaml(cls, file_path: str) -> 'MotorConfig':
        """YAML dosyasından konfigürasyon yükle"""
        with open(file_path, 'r') as f:
            config_data = yaml.safe_load(f)
        return cls(**config_data)

class SerialConnection(ABC):
    @abstractmethod
    def write(self, data: bytes) -> int:
        pass
    
    @abstractmethod
    def read(self, size: int) -> bytes:
        pass

class MotorLogger:
    def __init__(self, log_level: int = logging.INFO):
        self._logger = logging.getLogger("MotorController")
        self._setup_logger(log_level)
    
    def _setup_logger(self, log_level: int):
        # Dosyaya ve konsola log
        file_handler = logging.FileHandler('motor.log')
        console_handler = logging.StreamHandler()
        
        # Format
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        
        self._logger.addHandler(file_handler)
        self._logger.addHandler(console_handler)
        self._logger.setLevel(log_level)

class MotorDependencies:
    def __init__(self, serial: SerialConnection, config: MotorConfig, error_handler: ErrorHandler, logger: MotorLogger):
        self.serial = serial
        self.config = config
        self.error_handler = error_handler
        self.logger = logger

class MotorContext:
    def __init__(self, dependencies: MotorDependencies):
        self._deps = dependencies
        self._state: State = InitializingState()
        self._previous_states: List[State] = []
    
    def transition_to(self, new_state: Type[State]) -> None:
        self._previous_states.append(self._state)
        self._state.exit(self)
        self._state = new_state()
        self._state.enter(self)
    
    def revert_to_previous_state(self) -> None:
        if self._previous_states:
            previous = self._previous_states.pop()
            self.transition_to(type(previous))

class MotorController:
    """Motor kontrolü ve durum yönetimi için ana sınıf"""
    
    def __init__(self,
                 config: MotorConfig,
                 serial: SerialConnection,
                 error_handler: ErrorHandler,
                 logger: MotorLogger):
        self._config = config
        self._serial = serial
        self._error_handler = error_handler
        self._logger = logger
        self.current_state = MotorState.INITIALIZING
        self.axis_state: Optional[ODriveAxisState] = None
        self.voltage: Optional[float] = None
        self.calibration_flag = False
        self.procedure_result: Optional[ProcedureResult] = None
        self.errors = []
        self.retry_count = 0
        self.max_retries = 3
        self.last_state_change = time.time()
        self.state_timeout = 30  # saniye
        
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

        self.active_error_states: Set[ErrorState] = set()
        self.error_handlers = {
            ErrorState.VOLTAGE_ERROR: self._handle_voltage_error,
            ErrorState.CURRENT_ERROR: self._handle_current_error,
            ErrorState.TEMPERATURE_ERROR: self._handle_temperature_error,
            ErrorState.MOTOR_ERROR: self._handle_motor_error,
            ErrorState.CALIBRATION_ERROR: self._handle_calibration_error,
            ErrorState.SAFETY_ERROR: self._handle_safety_error,
        }

        self._context = MotorContext(MotorDependencies(serial, config, error_handler, logger))

    def _setup_state_transitions(self):
        """Geçerli durum geçişlerini tanımlar"""
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
            MotorState.ERROR: self.handle_error_state
        }

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
        """Aktif hataları kontrol eder ve kaydeder"""
        try:
            error_code = int(float(error_code)) if error_code is not None else 0
            self.errors = []
            self.active_error_states.clear()
            
            for error in ODriveError:
                if error_code & error.value:
                    self.errors.append(error.name)
                    error_state = self._map_odrive_error_to_error_state(error)
                    self.active_error_states.add(error_state)
                    
            if self.active_error_states:
                self._handle_error_states()
            
            logging.info(f"Active errors: {self.errors}")
            logging.info(f"Active error states: {[state.name for state in self.active_error_states]}")
            
        except Exception as e:
            logging.error(f"Error in error checking: {e}")
            self.errors = []
            self.active_error_states = {ErrorState.SYSTEM_ERROR}

    def _handle_error_states(self) -> None:
        """Aktif hata durumlarını yönetir"""
        for error_state in self.active_error_states:
            if error_state in self.error_handlers:
                self.error_handlers[error_state]()

    def _handle_voltage_error(self) -> None:
        """Voltaj hatalarını yönetir"""
        logging.error("Voltage error detected - checking voltage levels")
        run_command(self._serial, "vbus_voltage", lambda v: self.check_voltage(v))
        time.sleep(1)  # Voltajın dengelenmesi için bekle

    def _handle_current_error(self) -> None:
        """Akım hatalarını yönetir"""
        logging.error("Current error detected - reducing motor current")
        set_command(self._serial, "axis0.motor.config.current_lim", 10)  # Akım limitini düşür
        self.change_state(MotorState.CHECKING_STATUS)

    def _handle_temperature_error(self) -> None:
        """Sıcaklık hatalarını yönetir"""
        logging.error("Temperature error detected - stopping motor")
        set_command(self._serial, "axis0.requested_state", ODriveAxisState.AXIS_STATE_IDLE.value)
        self.change_state(MotorState.ERROR)

    def _handle_motor_error(self) -> None:
        """Motor hatalarını yönetir"""
        logging.error("Motor error detected - attempting recovery")
        self.clear_error()
        self.change_state(MotorState.CHECKING_STATUS)

    def _handle_calibration_error(self) -> None:
        """Kalibrasyon hatalarını yönetir"""
        logging.error("Calibration error detected - restarting calibration")
        if self.retry_count < self.max_retries:
            self.change_state(MotorState.CALIBRATING)
        else:
            self.change_state(MotorState.ERROR)

    def _handle_safety_error(self) -> None:
        """Güvenlik hatalarını yönetir"""
        logging.error("Safety error detected - emergency stop")
        set_command(self._serial, "axis0.requested_state", ODriveAxisState.AXIS_STATE_IDLE.value)
        self.change_state(MotorState.ERROR)

    def clear_error(self) -> None:
        """Hataları temizler"""
        logging.info("Clearing errors...")
        if self._serial and self._serial.is_open:
            self._serial.write("sc\n".encode())
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
            success = set_command(self._serial, "axis0.controller.input_vel", self.current_velocity)
            
            if success:
                logging.info(f"MOTOR DIRECTION CHANGED - New velocity: {self.current_velocity}")
                self.last_direction_change = current_time
            else:
                logging.error("Failed to change motor direction")
                self.change_state(MotorState.ERROR)
                
        except Exception as e:
            logging.error(f"Error in handle_torque_change: {e}")
            self.change_state(MotorState.ERROR)

    def run(self):
        """Ana döngü"""
        while True:
            try:
                self._context.handle()
            except Exception as e:
                self._error_handler.handle(e, self._context)

    def handle_initializing_state(self):
        """Başlatma durumu işlemleri"""
        run_command(self._serial, "vbus_voltage", lambda v: self.check_voltage(v))
        if self.voltage is not None and self.voltage >= 10:
            self.change_state(MotorState.CHECKING_STATUS)
        else:
            logging.warning("Voltage is low or could not be read")

    def handle_checking_voltage_state(self):
        """Voltaj kontrol durumu işlemleri"""
        self.clear_error()
        run_command(self._serial, "axis0.current_state", lambda v: self.check_status(v))

        if self.axis_state == ODriveAxisState.AXIS_STATE_IDLE:
            self.change_state(MotorState.TRANSITIONING_TO_CLOSED_LOOP)
        elif self.axis_state is None:
            logging.error("Could not read motor state")
            time.sleep(1)

    def handle_checking_status_state(self):
        """Durum kontrol işlemleri"""
        self.clear_error()
        run_command(self._serial, "axis0.current_state", lambda v: self.check_status(v))

        if self.axis_state == ODriveAxisState.AXIS_STATE_IDLE:
            self.change_state(MotorState.TRANSITIONING_TO_CLOSED_LOOP)
        elif self.axis_state is None:
            logging.error("Could not read motor state")
            time.sleep(1)

    def handle_calibrating_state(self):
        """Kalibrasyon durumu işlemleri"""
        success = set_command(self._serial, "axis0.requested_state", 3)
        if success:
            self.change_state(MotorState.CALIBRATION_WAITING)
        else:
            logging.error("Could not start calibration")
            time.sleep(1)

    def handle_calibration_waiting_state(self):
        """Kalibrasyon bekleme durumu işlemleri"""
        run_command(self._serial, "axis0.procedure_result", lambda v: self.check_procedure_result(v))

        if self.procedure_result == ProcedureResult.BUSY:
            logging.info("Calibration in progress...")
            time.sleep(0.5)
        elif self.procedure_result == ProcedureResult.CANCELLED:
            logging.warning("Calibration cancelled!")
            self.clear_error()
            run_command(self._serial, "axis0.active_errors", self.check_active_errors)
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
        """Kapalı döngüye geçiş durumu işlemleri"""
        success = set_command(self._serial, "axis0.requested_state", 8)
        if not success:
            run_command(self._serial, "axis0.procedure_result", lambda v: self.check_procedure_result(v))
            if self.procedure_result == ProcedureResult.NOT_CALIBRATED:
                self.change_state(MotorState.CALIBRATING)
            else:
                logging.error(f"Closed loop transition error: {self.procedure_result}")
                time.sleep(1)
        else:
            self.change_state(MotorState.CLOSED_LOOP)

    def handle_closed_loop_state(self):
        """Kapalı döngü durumu işlemleri"""
        run_command(self._serial, "axis0.current_state", lambda v: self.check_status(v))
        if self.axis_state != ODriveAxisState.AXIS_STATE_CLOSED_LOOP_CONTROL:
            logging.error("Lost closed loop state")
            self.change_state(MotorState.CHECKING_STATUS)
        else:
            # Kapalı döngüdeyiz, hız kontrolüne geçiş yap
            logging.info("Transitioning to velocity control mode...")
            success = True
            # Kontrol modunu hız kontrolüne ayarla
            success &= set_command(self._serial, "axis0.controller.config.control_mode", 2)
            # Giriş modunu hız olarak ayarla
            success &= set_command(self._serial, "axis0.controller.config.input_mode", 1)
            # Başlangıç hızını ayarla (saniyede 1 tur)
            success &= set_command(self._serial, "axis0.controller.input_vel", 1)
            
            if success:
                self.change_state(MotorState.VELOCITY_CONTROL)
            else:
                logging.error("Failed to transition to velocity control")
                self.change_state(MotorState.ERROR)

    def handle_velocity_control_state(self):
        """Hız kontrolü durumu işlemleri"""
        try:
            # Motor durumunu kontrol et
            run_command(self._serial, "axis0.current_state", lambda v: self.check_status(v))
            if self.axis_state != ODriveAxisState.AXIS_STATE_CLOSED_LOOP_CONTROL:
                logging.error("Lost velocity control state")
                self.change_state(MotorState.ERROR)
                return

            # Sürekli tork değerini oku ve kontrol et
            success = run_command(self._serial, "axis0.motor.foc.Iq_measured", lambda v: self.check_torque(v))
            while not success:
                success = run_command(self._serial, "axis0.motor.foc.Iq_measured", lambda v: self.check_torque(v))
                logging.debug("Retrying to read torque value")
                
        except Exception as e:
            logging.error(f"Error in velocity control: {e}")
            self.change_state(MotorState.ERROR)

    def handle_error_state(self):
        """Hata durumu işlemleri"""
        self.handle_error()

    def handle_shutdown(self):
        """Kapatma işlemleri"""
        logging.warning("Keyboard interrupt detected! Setting motor to IDLE mode...")
        set_command(self._serial, "axis0.requested_state", ODriveAxisState.AXIS_STATE_IDLE.value)
        self.change_state(MotorState.INITIALIZING)
        logging.info("State machine safely stopped.")

    def _map_odrive_error_to_error_state(self, odrive_error: ODriveError) -> ErrorState:
        """ODrive hata kodlarını ErrorState'e dönüştürür"""
        error_mapping = {
            ODriveError.NONE: ErrorState.NO_ERROR,
            ODriveError.INITIALIZING: ErrorState.SYSTEM_ERROR,
            ODriveError.SYSTEM_LEVEL: ErrorState.SYSTEM_ERROR,
            ODriveError.TIMING_ERROR: ErrorState.TIMING_ERROR,
            ODriveError.MISSING_ESTIMATE: ErrorState.ENCODER_ERROR,
            ODriveError.BAD_CONFIG: ErrorState.CONTROLLER_ERROR,
            ODriveError.DRV_FAULT: ErrorState.MOTOR_ERROR,
            ODriveError.MISSING_INPUT: ErrorState.CONTROLLER_ERROR,
            ODriveError.DC_BUS_OVER_VOLTAGE: ErrorState.VOLTAGE_ERROR,
            ODriveError.DC_BUS_UNDER_VOLTAGE: ErrorState.VOLTAGE_ERROR,
            ODriveError.DC_BUS_OVER_CURRENT: ErrorState.CURRENT_ERROR,
            ODriveError.DC_BUS_OVER_REGEN_CURRENT: ErrorState.CURRENT_ERROR,
            ODriveError.CURRENT_LIMIT_VIOLATION: ErrorState.CURRENT_ERROR,
            ODriveError.MOTOR_OVER_TEMP: ErrorState.TEMPERATURE_ERROR,
            ODriveError.INVERTER_OVER_TEMP: ErrorState.TEMPERATURE_ERROR,
            ODriveError.VELOCITY_LIMIT_VIOLATION: ErrorState.SAFETY_ERROR,
            ODriveError.POSITION_LIMIT_VIOLATION: ErrorState.SAFETY_ERROR,
            ODriveError.WATCHDOG_TIMER_EXPIRED: ErrorState.TIMING_ERROR,
            ODriveError.ESTOP_REQUESTED: ErrorState.SAFETY_ERROR,
            ODriveError.SPINOUT_DETECTED: ErrorState.MOTOR_ERROR,
            ODriveError.BRAKE_RESISTOR_DISARMED: ErrorState.SAFETY_ERROR,
            ODriveError.THERMISTOR_DISCONNECTED: ErrorState.TEMPERATURE_ERROR,
            ODriveError.CALIBRATION_ERROR: ErrorState.CALIBRATION_ERROR,
        }
        return error_mapping.get(odrive_error, ErrorState.SYSTEM_ERROR)

def main():
    """Ana program döngüsü"""
    disable_logging()      
    try:
        ser = ensure_serial_connection()
        if ser:
            config = MotorConfig.from_yaml('motor_config.yaml')
            error_handler = ErrorHandler()
            logger = MotorLogger()
            motor_controller = MotorController(
                config=config,
                serial=ser,
                error_handler=error_handler,
                logger=logger
            )
            motor_controller.run()
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