import time
import logging

def read_command(ser, command, callback):
    try:
        full_command = f"r {command}\n"
        ser.write(full_command.encode())
        response = read_response(ser, command)
        callback(response)
    except Exception as e:
        logging.error(f"Komut okuma hatası: {e}")
        callback(None)

def read_response(ser, command, timeout=1.0):
    start_time = time.time()
    response = b""
    
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            byte = ser.read()
            if byte == b'\n':
                break
            response += byte
        time.sleep(0.01)
    
    if not response:
        logging.warning(f"Timeout: {command} için yanıt alınamadı")
        return None
        
    try:
        value = float(response.decode().strip())
        logging.info(f"{command}: {value}")
        return value
    except ValueError:
        logging.error(f"Geçersiz yanıt {command}: {response.decode().strip()}")
        return None

def run_command(ser, command: str, callback=None) -> bool:
    """
    Geliştirilmiş komut çalıştırma
    """
    max_retries = 3
    retry_count = 0
    
    while retry_count < max_retries:
        try:
            if not ser.is_open:
                logging.error("Serial port is not open")
                return False
                
            # Buffer'ları temizle
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Komutu gönder
            command_str = f"r {command}\n"
            logging.debug(f"Sending command: {command_str.strip()}")
            ser.write(command_str.encode())
            ser.flush()
            
            # Yanıt için bekle
            time.sleep(0.1)
            
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                logging.debug(f"Raw response: {response}")
                
                if "invalid" in response.lower():
                    retry_count += 1
                    logging.warning(f"Invalid command, retry {retry_count}/{max_retries}")
                    time.sleep(0.1)
                    continue
                    
                if callback:
                    return callback(response)
                return True
                
            retry_count += 1
            logging.warning(f"No response, retry {retry_count}/{max_retries}")
            time.sleep(0.1)
            
        except Exception as e:
            logging.error(f"Command error: {str(e)}")
            retry_count += 1
            time.sleep(0.1)
            
    return False