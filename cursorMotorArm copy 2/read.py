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

def run_command(ser, command, callback):
    logging.info(f"Komut çalıştırılıyor: {command}")
    read_command(ser, command, callback)