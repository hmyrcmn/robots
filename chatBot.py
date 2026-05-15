import pandas as pd

# CSV dosyasını yükleme
def load_qa_data(file_path):
    try:
        data = pd.read_csv(file_path)
        return data
    except Exception as e:
        print("CSV dosyası yüklenirken hata oluştu:", e)
        return None

# Soruya göre cevap bulma
def get_answer(question, qa_data):
    # Soruyu küçük harfe çevirerek karşılaştırma
    question = question.lower()
    for index, row in qa_data.iterrows():
        if row['question'].lower() == question:
            return row['answer']
    return "Üzgünüm, bu soruya cevap bulamadım."

# Ana döngü
def main():
    file_path = "kulchatbot500.csv"
    qa_data = load_qa_data(file_path)

    if qa_data is not None:
        print("Chatbot'a hoş geldiniz! Çıkış yapmak için 'çıkış' yazın.")
        while True:
            user_input = input("Soru: ").strip()
            if user_input.lower() in ["çıkış", "exit"]:
                print("Chatbot kapatılıyor...")
                break
            answer = get_answer(user_input, qa_data)
            print("Cevap:", answer)

if __name__ == "__main__":
    main()