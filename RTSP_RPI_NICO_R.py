import cv2
import threading
import time
import serial
import queue
import numpy as np
from paddleocr import PaddleOCR
from datetime import datetime
from collections import Counter
from pynput import keyboard
import serial.tools.list_ports
# -import sys
import socket
import atexit

# Variable global de control
exit_event = threading.Event()

_window_name = "CAPTURA"
# __UART_JETSON__ = '/dev/ttyTHS1'
#__UART_JETSON__ = "/dev/serial/by-id/" + \
#    "usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0"#"usb-FTDI_FT232R_USB_UART_00000000-if00-port0"
__UART_JETSON__ = '/dev/ttyAMA0'

__SALIDA__ = 0
__ENTRADA__ = 1
__UMBRA_TAM__ = 1000
__SCALA__ = 0.15
_tiempo_ventana = 0
IP_camera1 = "192.168.18.225"
IP_camera2 = "192.168.18.226"
IP_camera3 = "192.168.18.227"
IP_camera4 = "192.168.18.228"
IP_camera_l = "192.168.18.180"

rtsp_url1 = (
    'rtspsrc location=rtsp://admin:admin2025@192.168.18.225:554/cam/realmonitor?channel=1&subtype=0?buffer_size=0 latency=0 ! '
    'rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate ! '
    'video/x-raw, format=BGRx, framerate=5/1 ! appsink'
)


class EstadoDispositivos:
    def __init__(self):
        # Diccionario con el estado inicial
        self.estado = {
            "E1": 0,
            "E2": 0,
            "C1": 0,
            "C2": 0
        }
        self.lock = threading.Lock()

    def set_estado(self, dispositivo, valor):
        with self.lock:
            if dispositivo in self.estado:
                self.estado[dispositivo] = valor
                print(f"[{dispositivo}] actualizado a: {valor}")

    def get_estado(self):
        with self.lock:
            # Retornar copia para no exponer el original
            return self.estado.copy()

    def get_estado_str(self):
        with self.lock:
            return "#" + ",".join(f"{k}:{v}" for k, v in self.estado.items())

    def mostrar_estado(self):
        with self.lock:
            estado_str = ",".join([f"{k}:{v}" for k, v in self.estado.items()])
            print(f"[Estado actual] {estado_str}")



class scannerUSB:
    def __init__(self, cola, port='/dev/ttyS0', baudrate=9600):
        self.window_name = 'Detector_Fatiga'
        self.cola = cola
        self.capturing = False
        self.data = ""
        self.data_validada = ""
        self.listener_running = True

        # Inicializa el listener de teclado
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Hilo para la comunicación serial
        self.serial_thread = threading.Thread(
            target=self.serial_communication, daemon=True)
        self.serial_thread.start()

    # *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # * LECTURA DE TECLADO

    def on_press(self, key):
        try:
            if key.char:
                self.data += key.char
                self.capturing = True
        except AttributeError:
            if key == keyboard.Key.enter and self.capturing:
                self.data_validada = self.data
                self.data = ""
                self.capturing = False

    def on_release(self, key):
        if key == keyboard.Key.esc:
            return False

    def close_program(self):
        self.listener_running = False
        time.sleep(0.8)
        # self.ser.close()

    # *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # * COMUNICACIÓN SERIAL

    def serial_communication(self):
        while True:
            try:
                if len(self.data_validada) > 0 and not self.capturing:
                    envio_QBAR = f"#Q:{self.data_validada},D:1"  # .encode()
                    self.data_validada = ""
                    self.cola.put(envio_QBAR)
                    print(envio_QBAR)
                    # self.ser.write(envio_QBAR)
                    # self.ser.flush()
                if not self.listener_running:
                    raise Exception("Cerrando el programa")
            except Exception as e:
                print(e)
                self.close_program()
                break
            time.sleep(0.1)


class SerialScanner_RT:
    def __init__(self, cola, port, estado, nombre, baudrate=9600):

        self.cola = cola
        self.port = port
        self.estado = estado
        self.nombre = nombre
        self.ser = serial.Serial(self.port, baudrate, timeout=0)
        self.running = True
        self.ser.read_all()

        self.serial_thread = threading.Thread(
            target=self.serial_scanner, daemon=True)
        self.serial_thread.start()
        self.estado.set_estado(self.nombre, 1)

    def serial_scanner(self):
        terminadores = [b'\r\n', b'\t']
        buffer = b''
        try:
            while self.running:
                if self.ser.in_waiting > 0:
                    buffer += self.ser.read(self.ser.in_waiting)
                    # buffer = bytes([b for b in buffer if 32 <= b <= 126])#[x]
                    # Permitir caracteres imprimibles (32-126) y CR(13), LF(10), TAB(9)
                    buffer = bytes([b for b in buffer if 32 <=
                                   b <= 126 or b in (9, 10, 13)])
                    for t in terminadores:
                        if t in buffer:
                            mensaje, _, buffer = buffer.partition(t)
                            try:
                                # Filtrar caracteres inválidos antes de decodificar
                                # mensaje = bytes([b for b in mensaje if 32 <= b <= 126])
                                mensaje = bytes(
                                    [b for b in mensaje if 32 <= b <= 126 or b in (9, 10, 13)])
                                mensaje_str = mensaje.decode(
                                    errors='ignore').strip()
                                if t == b"\r\n":
                                    print(
                                        f"Mensaje recibido <CR><LN>: {mensaje_str}")
                                    self.cola.put(f"#Q:{mensaje_str},D:0")
                                else:
                                    print(
                                        f"Mensaje recibido <TAB>: {mensaje_str}")
                                    self.cola.put(f"#Q:{mensaje_str},D:1")
                            except Exception as e:
                                print(e, "ERROR en decode SerialScanner")
                            finally:
                                print("b >", buffer)
                else:
                    time.sleep(0.001)
        except Exception as e:  # (serial.SerialException, OSError) as e:
            self.estado.set_estado(self.nombre, 0)
            self.cola.put(self.estado.get_estado_str())
            print(f"[ERROR] Puerto {self.port} desconectado: {e}")

            self.running = False
            exit_event.set()  # Señalamos que se debe salir
        finally:
            try:
                self.stop()
            except:
                pass

    def stop(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        #self.serial_thread.join()


class SerialSender:
    def __init__(self, cola, port='/dev/ttyS0', baudrate=115200, intervalo_ms=200):
        self.cola = cola
        self.intervalo = intervalo_ms / 1000.0  # Convertir a segundos
        self.running = True

        # Inicializar puerto serial
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        # Iniciar hilo
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                if not self.cola.empty():
                    dato = self.cola.get()
                    print(f"[SerialSender] Enviando: {dato}")
                    self.ser.write(f"{dato}\r\n".encode())
                    self.ser.flush()
            except Exception as e:
                print(f"[SerialSender] Error: {e}")
            time.sleep(self.intervalo)

    def stop(self):
        self.running = False
        self.thread.join()
        self.ser.close()


class ServidorTCP:
    def __init__(self, cola, estados, host='0.0.0.0', port=65432):
        self.host = host
        self.port = port
        self.estados = estados
        self.server_socket = None
        self.client_conn = None
        self.running = False
        self.thread = None
        self.cola = cola

    def iniciar(self):
        self.running = True
        self.thread = threading.Thread(
            target=self._ejecutar_servidor, daemon=True)
        self.thread.start()

    def _ejecutar_servidor(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((self.host, self.port))
                s.listen()
                self.server_socket = s
                print("Esperando conexión...")
                s.settimeout(1.0)  # Para poder salir con self.running = False

                while self.running:
                    try:
                        data_lleg = ""
                        conn, addr = s.accept()
                        print("Conectado por", addr)
                        with conn:
                            # self.client_conn = conn
                            while self.running:
                                data = conn.recv(1024)
                                """if not data:
                                    break"""
                                if data:
                                    data_lleg = data.decode().strip()
                                    print("Recibido:", data_lleg)
                                    if "#C1:" in data_lleg:
                                        if "0" in data_lleg:
                                            self.estados.set_estado("C1", 0)
                                            self.cola.put(
                                                self.estados.get_estado_str())
                                        elif "1" in data_lleg:
                                            self.estados.set_estado("C1", 1)
                                    elif "#C2:" in data_lleg:
                                        if "0" in data_lleg:
                                            self.estados.set_estado("C2", 0)
                                            self.cola.put(
                                                self.estados.get_estado_str())
                                        elif "1" in data_lleg:
                                            self.estados.set_estado("C2", 1)
                                    else:
                                        self.cola.put(f"{data_lleg}")

                    except socket.timeout:
                        continue  # Permite revisar self.running
        except Exception as e:
            print("Error en el socket:", e)
        finally:
            print("Servidor detenido.")

    def detener(self):
        self.running = False
        time.sleep(0.8)
        if self.client_conn:
            try:
                self.client_conn.shutdown(socket.SHUT_RDWR)
                self.client_conn.close()
            except:
                pass
            self.client_conn = None
        print("Servidor cerrado.")


class Baliza:
    def __init__(self, cola,estados, intervalo=300):
        self.intervalo = intervalo  # segundos
        self.cola = cola
        self.estados = estados
        self.running = False
        self.thread = None

    def iniciar(self):
        self.running = True
        self.thread = threading.Thread(target=self._ejecutar, daemon=True)
        self.thread.start()

    def detener(self):
        self.running = False
        if self.thread:
            self.thread.join()
        print("Tarea detenida.")

    def _ejecutar(self):
        while self.running:
            print("[Tarea] Ejecutando tarea periódica...")
            # Aquí va tu lógica:
            self.mi_tarea()

            for _ in range(self.intervalo):
                if not self.running:
                    break
                time.sleep(1)

    def mi_tarea(self):
        # 👉 Aquí pones lo que quieras ejecutar cada 5 minutos
        self.cola.put(self.estados.get_estado_str())


print("INICIANDO...........")

estados = EstadoDispositivos()

cola_datos = queue.Queue()

serial_app = SerialSender(cola=cola_datos, port=__UART_JETSON__)
app_scanner = scannerUSB(cola_datos)
servidor = ServidorTCP(cola_datos, estados)
servidor.iniciar()



puertos = serial.tools.list_ports.comports()
puertos = ["/dev/serial/by-id/"+"usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0","/dev/serial/by-id/"+"usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"]

conexiones = []
i = 1

for puerto in puertos:
    print(f"Abriendo puerto {puerto}")
    try:
        scan_ser = SerialScanner_RT(cola_datos,puerto,estados,f"E{i}",baudrate=9600)
        print(f"Puerto {puerto} abierto correctamente a 9600 baudios.")
        conexiones.append(scan_ser)
        i+=1
    except serial.SerialException as e:
        print(f"No se pudo abrir {puerto}: {e}")

time.sleep(0.200)


baliza = Baliza(cola_datos,estados,intervalo=300)  # 5 minutos
baliza.iniciar()

cv2.namedWindow(_window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(_window_name, cv2.WND_PROP_FULLSCREEN, 1)


# 2. Bucle de ventana OpenCV (en el hilo principal)
def opencv_window():
    window = 'Serial_Sender'        # mismo nombre que usa la clase
    
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    #cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, 1)
    #cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

    # Frame totalmente verde (BGR: 0,255,0) de 480x640
    green_frame = np.full((120, 120, 3), (255, 255, 0), dtype=np.uint8)
    
    while True:
        cv2.imshow(window, green_frame)
        
        # Si el usuario pulsa ESC en la ventana, salimos
        key = cv2.waitKey(60) & 0xFF
        if key == 27:# ESC
            break
        # Si alguno de los hilos pide cerrar, salimos también
        if not app_scanner.listener_running or not serial_app.running or not servidor.running or not all(c.running for c in conexiones):
            for c in conexiones:
                try:
                    c.stop()
                except:
                    pass
            print("ERROR GENERAL")
            break
        if exit_event.is_set():
            print("ERROR -- cerrando SCANNERS")
            break
    cv2.destroyAllWindows()

# 3. Limpiar correctamente al terminar (CTRL-C, etc.)
def cleanup():
    baliza.detener()
    servidor.detener()
    serial_app.stop()
    
    #servidor.detener()
    cv2.destroyAllWindows()

atexit.register(cleanup)


opencv_window()




