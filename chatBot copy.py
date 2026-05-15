chatBot.pyfrom transformers import AutoModelForCausalLM, AutoTokenizer
import torch

# Modeli ve tokenizer'ı yükle
model_name = "deepseek-ai/DeepSeek-V3"
tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
model = AutoModelForCausalLM.from_pretrained(model_name, trust_remote_code=True)

# Chatbot fonksiyonu
def chat_with_model(prompt):
    # Giriş metnini tokenlara dönüştür
    inputs = tokenizer(prompt, return_tensors="pt")

    # Modelden cevap al
    with torch.no_grad():
        outputs = model.generate(**inputs, max_length=100, num_return_sequences=1)

    # Tokenleri tekrar metne dönüştür
    response = tokenizer.decode(outputs[0], skip_special_tokens=True)
    return response

# Ana döngü
def main():
    print("Chatbot'a hoş geldiniz! Çıkış yapmak için 'çıkış' yazın.")
    while True:
        user_input = input("Soru: ").strip()
        if user_input.lower() in ["çıkış", "exit"]:
            print("Chatbot kapatılıyor...")
            break
        response = chat_with_model(user_input)
        print("Cevap:", response)

if __name__ == "__main__":
    main()from transformers import AutoModelForCausalLM, AutoTokenizer
import torch

# Modeli ve tokenizer'ı yükle
model_name = "deepseek-ai/DeepSeek-V3"
tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
model = AutoModelForCausalLM.from_pretrained(model_name, trust_remote_code=True)

# Chatbot fonksiyonu
def chat_with_model(prompt):
    # Giriş metnini tokenlara dönüştür
    inputs = tokenizer(prompt, return_tensors="pt")

    # Modelden cevap al
    with torch.no_grad():
        outputs = model.generate(**inputs, max_length=100, num_return_sequences=1)

    # Tokenleri tekrar metne dönüştür
    response = tokenizer.decode(outputs[0], skip_special_tokens=True)
    return response

# Ana döngü
def main():
    print("Chatbot'a hoş geldiniz! Çıkış yapmak için 'çıkış' yazın.")
    while True:
        user_input = input("Soru: ").strip()
        if user_input.lower() in ["çıkış", "exit"]:
            print("Chatbot kapatılıyor...")
            break
        response = chat_with_model(user_input)
        print("Cevap:", response)

if __name__ == "__main__":
    main()