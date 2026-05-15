import logging
import time
from read import read_response

def execute_with_retry(operation, max_retries=3, delay=0.5, *args, **kwargs):
    for attempt in range(max_retries):
        try:
            return operation(*args, **kwargs)
        except Exception as e:
            logging.error(f"Attempt {attempt+1}/{max_retries} failed: {e}")
            time.sleep(delay)
    logging.error("Maximum retry count exceeded")
    return False

def set_command_callback(ser, send_command, value):
    try:
        command = f"r {send_command}\n"
        ser.write(command.encode())
        response = read_response(ser, send_command)
        
        if response is None:
            return False
            
        if abs(float(response) - value) < 0.001:  # Floating point karşılaştırması için
            logging.info(f"{send_command} başarıyla ayarlandı: {response}")
            return True
        else:
            logging.error(f"{send_command} ayarlanamadı. Beklenen: {value}, Alınan: {response}")
            return False
            
    except Exception as e:
        logging.error(f"Callback hatası: {e}")
        return False

def set_command(ser, send_command, value):
    def operation():
        command = f"w {send_command} {value}\n"
        ser.write(command.encode())
        return set_command_callback(ser, send_command, value)
    return execute_with_retry(operation)