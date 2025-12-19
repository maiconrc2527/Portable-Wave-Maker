#!/usr/bin/env python3
import time
import math
import threading
import spidev       # Para comunicação SPI com MCP3008
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt

# ===============================
# Configuração dos Pinos
# ===============================
# Utilize o esquema BCM; ajuste conforme necessário
STEP_PIN   = 12   # Pino STEP do driver do motor
DIR_PIN    = 5    # Pino de direção
EN_PIN     = 8    # Pino de enable (caso necessário)
CHAVE_PIN  = 10   # Pino para emergência (botão com pull-up)
# Não há pino analógico; iremos ler o potenciômetro via ADC (canal 0 do MCP3008)
POT_ADC_CHANNEL = 0

# ===============================
# Parâmetros do Sinal e Motor
# ===============================
# Parâmetros de teste
vel = 700           # Velocidade em "steps/s" (valor de referência, ajuste conforme sua aplicação)
vell = 1000         # Velocidade máxima (limite)
T = 0.8             # Período do sinal (em s)
Amp_mm = 30         # Amplitude total em mm (nota: no original isto é metade da amplitude)
steps_per_mm = 6.25 # Fator de conversão (exemplo do cálculo do Arduino)
A = Amp_mm * steps_per_mm   # Amplitude em passos
w = (2 * math.pi) / T       # Frequência angular

# Intervalo de atualização do sinal (em segundos)
update_interval = 0.006  # Equivalente a 6000 microsegundos

# Parâmetro de calibração (porcentagem)
calibValue = 50

# ===============================
# Configuração de Hardware
# ===============================
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(EN_PIN, GPIO.OUT)
GPIO.setup(CHAVE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Inicializa o pino enable (se necessário, ativo em LOW ou HIGH conforme seu driver)
GPIO.output(EN_PIN, GPIO.LOW)  

# Configuração do SPI para o ADC MCP3008
spi = spidev.SpiDev()
spi.open(0, 0)  # Abrir SPI no bus 0, dispositivo 0
spi.max_speed_hz = 1350000

def read_adc(channel):
    """
    Lê o canal especificado do MCP3008 (0-7).
    Retorna um valor entre 0 e 1023.
    """
    if channel < 0 or channel > 7:
        return -1
    # Envia 3 bytes: [start, configuração, dummy]
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def map_value(x, in_min, in_max, out_min, out_max):
    """Função similar à map do Arduino."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# ===============================
# Função de Controle do Motor de Passo
# ===============================
def stepp(steps_per_sec, npassos):
    """
    Gera pulsos para mover o motor o número de passos (npassos).
    A direção é determinada pelo sinal de npassos.
    A velocidade (steps_per_sec) define o atraso entre os passos.
    """
    if npassos == 0:
        return

    # Define a direção: HIGH para passos positivos, LOW para negativos
    if npassos > 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)
        
    n_steps = abs(int(round(npassos)))
    delay = 1.0 / abs(steps_per_sec) if steps_per_sec != 0 else 0.01

    for _ in range(n_steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay / 2)  # pulso HIGH
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay / 2)  # pulso LOW

# ===============================
# Funções Auxiliares para Calibração e Emergência
# ===============================
def read_potentiometer():
    """
    Lê o valor do potenciômetro do ADC e retorna uma porcentagem mapeada.
    No código Arduino, o valor é mapeado de 0-1023 para 0-100.
    """
    pot_value = read_adc(POT_ADC_CHANNEL)
    pot_percent = map_value(pot_value, 0, 1023, 0, 100)
    return pot_percent

def calibracao():
    """
    Função simples de calibração: move o motor até que a leitura do potenciômetro se estabilize no valor de calibração.
    """
    pot_map = read_potentiometer()
    print(f"Calibração iniciada: Leitura = {pot_map:.1f}%")
    while pot_map > (calibValue + 1):
        stepp(100, -1)
        pot_map = read_potentiometer()
        print(f"Calibrando: {pot_map:.1f}%")
    while pot_map < (calibValue - 1):
        stepp(100, 1)
        pot_map = read_potentiometer()
        print(f"Calibrando: {pot_map:.1f}%")
    print("Calibrado!")

def check_emergency():
    """
    Verifica se o botão de emergência foi pressionado.
    Se sim, desativa os pinos de STEP e DIR.
    """
    if GPIO.input(CHAVE_PIN) == GPIO.LOW:
        # Em caso de emergência, coloca os pinos em modo de segurança
        GPIO.setup(STEP_PIN, GPIO.IN)
        GPIO.setup(DIR_PIN, GPIO.IN)
        print("EMERGÊNCIA ACIONADA!")
        return True
    return False

# ===============================
# Thread para Atualizar e Armazenar Sinais para Plotagem
# ===============================
class SignalLogger(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.timestamps = []
        self.signals = []
        self.running = True

    def run(self):
        start_time = time.time()
        while self.running:
            current_time = time.time() - start_time
            # Aqui, registramos o sinal (valor desejado para o motor) conforme calculado na função de controle
            # Para isso, a thread principal deverá atualizar uma variável global 'current_wave'
            self.timestamps.append(current_time)
            self.signals.append(current_wave)
            time.sleep(update_interval)

    def stop(self):
        self.running = False

# Variável global que armazena o valor atual do sinal senoidal (em passos)
current_wave = 0.0

# ===============================
# Loop Principal
# ===============================
def main():
    global current_wave
    try:
        # Caso queira calibrar antes do início:
        # calibracao()

        # Inicia o logger para armazenar os sinais
        logger = SignalLogger()
        logger.start()

        t = 0.0         # instância de tempo para o sinal
        old_wave = 0.0  # armazena o valor anterior do sinal

        last_update = time.time()

        print("Início do controle do motor...")
        while True:
            # Verifica condição de emergência
            if check_emergency():
                break

            now = time.time()
            if (now - last_update) >= update_interval:
                t += update_interval
                # Calcula o sinal senoidal (valor desejado em passos)
                wave = A * math.sin(w * t)
                current_wave = wave  # atualiza o sinal para o logger

                # Calcula a variação de passos (diferença entre o novo e antigo sinal)
                delta_steps = wave - old_wave

                # Se a velocidade do motor estiver dentro dos limites (0 <= vel <= vell)
                if 0 <= vel <= vell:
                    # Executa os passos necessários
                    stepp(vel, delta_steps)
                old_wave = wave

                last_update = now

    except KeyboardInterrupt:
        print("Interrupção pelo usuário.")
    finally:
        # Para o logger e plota o gráfico
        logger.stop()
        logger.join()

        # Finaliza o SPI e limpa os pinos GPIO
        spi.close()
        GPIO.cleanup()

        # Plot do sinal
        plt.figure(figsize=(10, 4))
        plt.plot(logger.timestamps, logger.signals, label='Sinal Gerado (passos)')
        plt.title('Sinal Senoidal Enviado ao Motor de Passo')
        plt.xlabel('Tempo (s)')
        plt.ylabel('Valor do Sinal (passos)')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    main()
