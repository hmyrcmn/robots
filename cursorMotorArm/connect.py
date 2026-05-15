import serial
import logging
import time
from typing import Optional
import serial.tools.list_ports

def connect_to_serial(port=None, baudrate=115200, timeout=0.5):
    """
    Connect to serial port with improved error handling
    """
    try:
        # Otomatik port bulma
        if port is None:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                if "USB" in p.description:
                    port = p.device
                    break
        
        if port is None:
            raise Exception("No suitable USB port found")
            
        # Serial bağlantıyı oluştur
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=0.5,
            inter_byte_timeout=0.1
        )
        
        # Bağlantıyı test et
        if ser.is_open:
            ser.write(b"\r\n")  # Yeni satır gönder
            time.sleep(0.1)
            ser.reset_input_buffer()  # Buffer'ı temizle
            logging.info(f"Successfully connected to {port}")
            return ser
            
    except Exception as e:
        logging.error(f"Serial connection error: {str(e)}")
        return None

def try_available_ports(baudrate=115200, timeout=1) -> Optional[serial.Serial]:
    """
    Kullanılabilir tüm portları deneyerek bağlanmayı dener
    
    Args:
        baudrate: Baud rate değeri
        timeout: Timeout süresi
        
    Returns:
        Optional[serial.Serial]: Bağlantı başarılı ise Serial nesnesi, değilse None
    """
    available_ports = [port.device for port in serial.tools.list_ports.comports()]
    logging.info(f"Kullanılabilir portlar: {available_ports}")
    
    for port in available_ports:
        try:
            ser = serial.Serial(port, baudrate, timeout=timeout)
            logging.info(f"Bağlantı başarılı: {port}")
            return ser
        except:
            continue
    
    return None

def ensure_serial_connection(port='COM3', baudrate=115200, timeout=1, max_attempts=None) -> Optional[serial.Serial]:
    """
    Serial bağlantısını sürekli deneyerek sağlar
    
    Args:
        port: Seri port adı
        baudrate: Baud rate değeri
        timeout: Timeout süresi
        max_attempts: Maksimum deneme sayısı (None ise sonsuz)
        
    Returns:
        Optional[serial.Serial]: Bağlantı başarılı ise Serial nesnesi, değilse None
    """
    attempt = 0
    while True:
        try:
            logging.info(f"Serial port bağlantısı deneniyor... Deneme: {attempt + 1}")
            
            # Önce belirtilen porta bağlanmayı dene
            ser = connect_to_serial(port, baudrate, timeout)
            if ser:
                return ser
                
            # Belirtilen port başarısız olursa tüm portları dene
            ser = try_available_ports(baudrate, timeout)
            if ser:
                return ser
                
            if max_attempts and attempt >= max_attempts:
                logging.error(f"Maksimum deneme sayısına ulaşıldı ({max_attempts})")
                return None
                
            attempt += 1
            logging.info("5 saniye sonra tekrar denenecek...")
            time.sleep(5)
            
        except Exception as e:
            logging.error(f"Bağlantı hatası: {e}")
            if max_attempts and attempt >= max_attempts:
                return None
            attempt += 1
            time.sleep(5)