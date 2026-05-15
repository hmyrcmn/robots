import time
import logging

def execute_with_retry(operation, max_retries=3, delay=0.5, *args, **kwargs):
    for attempt in range(max_retries):
        try:
            return operation(*args, **kwargs)
        except Exception as e:
            logging.error(f"Attempt {attempt+1}/{max_retries} failed: {e}")
            time.sleep(delay)
    logging.error("Maximum retry count exceeded")
    return None

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

def run_command(ser, command, callback):
    def operation():
        logging.info(f"Komut çalıştırılıyor: {command}")
        full_command = f"r {command}\n"
        ser.write(full_command.encode())
        response = read_response(ser, command)
        callback(response)
    execute_with_retry(operation)
    
    
    