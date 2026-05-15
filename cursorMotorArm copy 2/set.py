import logging
from read import read_response

def set_command(ser, send_command, value):
    try:
        command = f"w {send_command} {value}\n"
        ser.write(command.encode())
        return set_command_callback(ser, send_command, value)
    except Exception as e:
        logging.error(f"Komut gönderme hatası: {e}")
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