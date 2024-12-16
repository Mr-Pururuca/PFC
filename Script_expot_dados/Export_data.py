import json
import csv

import csv
from datetime import datetime
import paho.mqtt.client as mqtt

# Configurações do Broker
BROKER = "localhost"  # Substitua pelo endereço do seu broker
PORT = 1883  # Porta padrão MQTT
USER = "Client"
PASSW = "PFC123"
TOPIC = "PFC/Medidas/Theta_1"  # Substitua pelo tópico que você está usando
CSV_FILE = "D:\\Documentos\\UFMG\\PFC\\Simulação\\PDC_dados.csv"

# Callback para conexão com o broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Conectado ao broker MQTT!")
        client.subscribe(TOPIC)
    else:
        print(f"Erro ao conectar. Código de retorno: {rc}")

# Callback para mensagens recebidas
def on_message(client, userdata, msg):
    try:
        # Decodifica a mensagem e imprime
        payload = msg.payload.decode('utf-8')
        timestamp = datetime.now().strftime('%M:%S.%f')[:-3]  # Gera a timestamp atual
        print(f"[{timestamp}] Tópico: {msg.topic}, Dados: {payload}")

        # Escreve no arquivo CSV
        with open(CSV_FILE, mode='a', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, payload])
    except Exception as e:
        print(f"Erro ao processar mensagem: {e}")

# Configuração do cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Adiciona credenciais de usuário e senha
client.username_pw_set(USER, PASSW)

# Conecta ao broker
client.connect(BROKER, PORT, 60)

# Adiciona o cabeçalho ao CSV (se o arquivo estiver vazio)
try:
    with open(CSV_FILE, mode='x', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Theta1"])
except FileExistsError:
    pass  # Se o arquivo já existe, não adiciona o cabeçalho novamente

# Inicia o loop
try:
    print("Iniciando loop MQTT. Pressione Ctrl+C para sair.")
    client.loop_forever()
except KeyboardInterrupt:
    print("Desconectando...")
    client.disconnect()