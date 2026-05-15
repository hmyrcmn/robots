import csv
import time

class DataCollector:
    def __init__(self, file_name="training_data.csv"):
        """
        Veri toplama sınıfını başlatır.
        Args:
            file_name: Verilerin kaydedileceği dosya adı
        """
        self.file_name = file_name
        self.data_file = open(self.file_name, mode="w", newline="")
        self.data_writer = csv.writer(self.data_file)
        # Başlık satırını yaz
        self.data_writer.writerow(["timestamp", "torque", "previous_torque", "torque_change", "velocity", "voltage", "errors", "label"])

    def log_data(self, torque, previous_torque, torque_change, velocity, voltage, errors, label):
        """
        Veriyi CSV dosyasına kaydeder.
        Args:
            torque: Güncel tork değeri
            previous_torque: Önceki tork değeri
            torque_change: Tork değişimi
            velocity: Motor hızı
            voltage: Voltaj değeri
            errors: Hata listesi
            label: Veri etiketi (örneğin, normal=0, arıza=1)
        """
        timestamp = time.time()
        self.data_writer.writerow([timestamp, torque, previous_torque, torque_change, velocity, voltage, errors, label])

    def close(self):
        """Veri dosyasını kapatır."""
        self.data_file.close()