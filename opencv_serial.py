import cv2
import numpy as np
import threading
import queue
import atexit
import time
from pynput import keyboard
import serial
import serial.tools.list_ports
import socket
# ------------------------------------------------------------------
# 1. Cola compartida y lanzamiento de tus clases
cola = queue.Queue()
exit_event = threading.Event()


__UART_JETSON__ = '/dev/ttyAMA0'#'/dev/ttyTHS1'


class DetectorScannerUSB:
    def __init__(self, cola):
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
        time.sleep(1)
        #self.ser.close()

    # *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # * COMUNICACIÓN SERIAL

    def serial_communication(self):
        while True:
            try:
                if len(self.data_validada) > 0 and not self.capturing:
                    envio_QBAR = f"#Q:{self.data_validada},D:1"  # .encode()
                    self.data_validada = ""
                    self.cola.put(envio_QBAR)
                    # self.ser.write(envio_QBAR)
                    # self.ser.flush()
                if not self.listener_running:
                    raise Exception("Cerrando el programa")
            except Exception as e:
                print(e)
                self.close_program()
                break
            time.sleep(0.1)


class SerialSender:
    def __init__(self, cola, port='/dev/ttyAMA0', baudrate=115200, intervalo_ms=200):
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

class SerialScanner:
    def __init__(self, cola, port, baudrate=9600):
        self.window_name = 'Detector_Fatiga'
        self.cola = cola
        self.port = port
        self.ser = serial.Serial(self.port, baudrate, timeout=0)  # timeout=0 para lectura no bloqueante
        self.running = True
        
        # Hilo para la comunicación serial
        self.serial_thread = threading.Thread(
            target=self.serial_scanner, daemon=True)
        self.serial_thread.start()
        
    def serial_scanner(self):
        terminadores = [b'\r\n', b'\t']
        buffer = b''
        while self.running == True:
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)
                for t in terminadores:
                    if t in buffer:
                        
                        mensaje, _, buffer = buffer.partition(t)
                        try:    
                            mensaje_str = mensaje.decode(errors='ignore').strip()
                            if t == b"\r\n":
                                
                                print(f"Mensaje recibido <CR><LN>: {mensaje_str}")
                                self.cola.put(f"#Q:{mensaje_str},D:1")
                            else:
                                print(f"Mensaje recibido <TAB>: {mensaje_str}")
                                self.cola.put(f"#Q:{mensaje_str},D:0")
                        except Exception as e:
                            print(e,"ERROR en decode SerialScanner")
                time.sleep(0.001)
            else:
                # Evita ocupar el 100% de CPU
                time.sleep(0.01)
        
    def stop(self):
        self.running = False
        time.sleep(0.05)
        self.ser.close()
        self.serial_thread.join()

class SerialScanner_RT:
    def __init__(self, cola, port, baudrate=9600):
        self.cola = cola
        self.port = port
        self.ser = serial.Serial(self.port, baudrate, timeout=0)
        self.running = True
        self.ser.read_all()
        self.serial_thread = threading.Thread(
            target=self.serial_scanner, daemon=True)
        self.serial_thread.start()
        

    def serial_scanner(self):
        terminadores = [b'\r\n', b'\t']
        buffer = b''
        try:
            while self.running:
                if self.ser.in_waiting > 0:
                    buffer += self.ser.read(self.ser.in_waiting)
                    #buffer = bytes([b for b in buffer if 32 <= b <= 126])#[x]
                    # Permitir caracteres imprimibles (32-126) y CR(13), LF(10), TAB(9)
                    buffer = bytes([b for b in buffer if 32 <= b <= 126 or b in (9, 10, 13)])
                    for t in terminadores:
                        if t in buffer:
                            mensaje, _, buffer = buffer.partition(t)
                            try:
                                # Filtrar caracteres inválidos antes de decodificar
                                #mensaje = bytes([b for b in mensaje if 32 <= b <= 126])
                                mensaje = bytes([b for b in mensaje if 32 <= b <= 126 or b in (9, 10, 13)])
                                mensaje_str = mensaje.decode(errors='ignore').strip()
                                if t == b"\r\n":
                                    print(f"Mensaje recibido <CR><LN>: {mensaje_str}")
                                    self.cola.put(f"#Q:{mensaje_str},D:1")
                                else:
                                    print(f"Mensaje recibido <TAB>: {mensaje_str}")
                                    self.cola.put(f"#Q:{mensaje_str},D:0")
                            except Exception as e:
                                print(e, "ERROR en decode SerialScanner")
                            finally:
                                print(buffer)
                else:
                    time.sleep(0.001)
        except Exception as e:#(serial.SerialException, OSError) as e:
            print(f"[ERROR] Puerto {self.port} desconectado: {e}")
            
            self.running = False
            exit_event.set()  # Señalamos que se debe salir
        finally:
            try:
                self.ser.close()
            except:
                pass

    def stop(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        self.serial_thread.join()

class ServidorTCP:
    def __init__(self, cola,host='localhost', port=65432):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_conn = None
        self.running = False
        self.thread = None
        self.cola = cola
    def iniciar(self):
        self.running = True
        self.thread = threading.Thread(target=self._ejecutar_servidor, daemon=True)
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
                        conn, addr = s.accept()
                        print("Conectado por", addr)
                        with conn:
                            self.client_conn = conn
                            while self.running:
                                time.sleep(0.001)
                                data = conn.recv(1024)
                                if not data:
                                    break
                                print("Recibido:", data.decode())
                                self.cola.put(f"{data.decode()}")
                                
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

detector = DetectorScannerUSB(cola)    # ya lanza sus propios hilos
sender   = SerialSender(cola,port=__UART_JETSON__)
#servidor = ServidorTCP(cola)
puertos = serial.tools.list_ports.comports()

puertos_filtrados = [p.device for p in puertos if 'ttyUSB' in p.device]
conexiones = []
for puerto in puertos_filtrados:
    print(f"Abriendo puerto {puerto}")
    try:
        scan_ser = SerialScanner_RT(cola=cola,port=puerto,baudrate=9600)
        print(f"Puerto {puerto} abierto correctamente a 9600 baudios.")
        conexiones.append(scan_ser)
        
    except serial.SerialException as e:
        print(f"No se pudo abrir {puerto}: {e}")


# ------------------------------------------------------------------
# 2. Bucle de ventana OpenCV (en el hilo principal)
def opencv_window():
    window = 'Serial_Sender'        # mismo nombre que usa la clase
    
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, 1)
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
        if not detector.listener_running or not sender.running:
            break
        if exit_event.is_set():
            print("ERROR -- cerrando SCANNERS")
            
            break
    cv2.destroyAllWindows()

# ------------------------------------------------------------------
# 3. Limpiar correctamente al terminar (CTRL‑C, etc.)
def cleanup():
    detector.close_program()
    sender.stop()
    #servidor.detener()
    cv2.destroyAllWindows()

atexit.register(cleanup)

# ------------------------------------------------------------------
# 4. Arrancar la ventana (bloqueante)
if __name__ == '__main__':
    try:
        opencv_window()
    except KeyboardInterrupt:
        pass
