import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.filedialog import asksaveasfilename, askopenfilename
import json
import sympy as sp
import numpy as np
import math
import serial
import serial.tools.list_ports
import time
import threading
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# =====================================================================
# KOMUNIKACJA SERIAL Z ESP-32
# =====================================================================

class RobotSerial:
    """
    Klasa odpowiedzialna za komunikację szeregową z modułem ESP-32.
    Obejmuje otwieranie portu, nasłuchiwanie danych w tle oraz wysyłanie komend sterujących.
    Zaprojektowana tak, aby reszta aplikacji mogła korzystać z aktualnych danych bez blokowania GUI.
    """

    def __init__(self, port=None, baudrate=115200):
        self.port_name = port
        self.baudrate = baudrate

        self.ser = None             # Obiekt Serial (utworzony dopiero w connect()).
        self.running = False        # Flaga kontrolująca pracę wątku odczytu.
        self.reader_thread = None   # Wątek odpowiedzialny za odbiór danych w tle.

        # Aktualne wartości odczytane z kontrolera [X, Y, Z, E, A] w stopniach.
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Lock zabezpiecza dane przy odczycie/zapisie z różnych wątków (mutex w c++)
        self.lock = threading.Lock()

        # Miejsce na funkcję, która zaktualizuje liczby i wykresy w oknie aplikacji.
        # Dzięki temu robot może poinformować GUI, że ma nowe dane do wyświetlenia.
        self.gui_callback = None

    def find_esp32_port(self):
        """
        Przeszukaj wszystkie dostępne porty COM, starając się wykryć ESP32.
        """
        # W naszym przypadku to "CP210x USB to UART Bridge (COM9)". 
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "CP210" in p.description: 
                return p.device

        return None # None — jeśli urządzenie nie zostało znalezione

    def connect(self):
        """
        Nawiązuje połączenie z urządzeniem ESP-32 przez port szeregowy.
        Zamyka poprzednie połączenie, wykrywa odpowiedni port, otwiera go,
        inicjalizuje mikrokontroler i uruchamia wątek odczytu.
        Zwraca: (bool, str) - informacja o powodzeniu operacji.
        """

        # Jeśli istnieje wcześniejsze połączenie
        # Zamknij, aby nie blokować portu lub wątku w tle.
        if self.ser and self.ser.is_open:
            self.close()

        # Wybór portu do użycia. Dopiero po udanej inicjalizacji trafia do self.port_name.
        target_port = self.port_name or self.find_esp32_port()
        if target_port is None:
            return False, "Nie znaleziono urządzenia ESP-32"

        try:
            # Otwarcie portu. Timeout zapobiega zawieszeniu odczytu.
            self.ser = serial.Serial(target_port, self.baudrate, timeout=1)

            # ESP32 wykonuje reset po zestawieniu połączenia (sygnał DTR).
            # Krótki odstęp pozwala mu się ponownie uruchomić.
            time.sleep(2)

            # Czyścimy bufor, aby uniknąć pracy na śmieciowych danych po resecie.
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # Start wątku odpowiedzialnego za odbiór ramek z kontrolera
            self.running = True
            self.reader_thread = threading.Thread(
                target=self._continuous_read,
                daemon=True # Zabija wątek przy zamknięciu aplikacji
            )
            self.reader_thread.start()

            self.port_name = target_port # Port uznajemy za działający.
            return True, f"Połączono z {self.port_name}"

        except Exception as e:
            # Sprzątanie w razie błędu – metoda close() zabezpiecza wszystkie zasoby
            self.close()
            return False, f"Błąd połączenia: {e}"

    def _continuous_read(self):
        """
        Wątek odbierający dane z portu szeregowego.
        Odczyt działa blokująco, dzięki czemu w spoczynku nie obciąża procesora.
        Metoda parsuje ramki z danymi kątów i aktualizuje stan robota.
        """

        AXIS_MAP = {"X": 0, "Y": 1, "Z": 2, "E": 3, "A": 4}

        while self.running and self.ser and self.ser.is_open:
            try:
                # Blokujący odczyt z UART — czeka na pełną linię lub timeout.
                raw_line = self.ser.readline()
                if not raw_line: continue

                # Dekodowanie odebranych danych.
                line = raw_line.decode("utf-8", errors="ignore").strip()
                if not line: continue

                # Obsługa ramek danych (<X...,Y...,Z...,E...,A...>)
                if line.startswith("<") and line.endswith(">"):
                    updates = []
                    items = line[1:-1].split(',')

                    # Parsowanie
                    for item in items:
                        if len(item) >= 2 and item[0] in AXIS_MAP:
                            try:
                                updates.append((AXIS_MAP[item[0]], float(item[1:])))
                            except ValueError: pass

                    # Aktualizacja współdzielonego stanu z blokadą
                    if updates: # Czy mamy nowe dane do zapisania?
                        with self.lock: # Blokada na czas zapisu i kopiowania
                            for idx, val in updates: # Zapisywanie nowych wartości
                                self.current_angles[idx] = val
                            # Zrobiona kopia do wysłania do GUI
                            data_to_send = self.current_angles.copy() if self.gui_callback else None

                        # Callback poza blokadą, żeby nie wieszać wątku, jeżeli GUI byłoby wolne
                        if self.gui_callback and data_to_send:
                            self.gui_callback(data_to_send)
                            

                else:
                    if self.gui_callback: # Jeżeli nie jest ramką, to wypisz log.
                        self.gui_callback(None, log_message=line)

            except (serial.SerialException, OSError):
                # Najczęstszy scenariusz to odłączenie kabla w trakcie pracy.
                self.running = False
                try:
                    self.ser.close()
                except Exception: pass

                if self.gui_callback:
                    self.gui_callback(None, log_message="#CONNECTION_LOST#")
                break

            except Exception as e:
                # Inne błędy odczytu nie powinny zatrzymywać całej aplikacji.
                if self.running and self.gui_callback:
                    self.gui_callback(None, log_message=f"Błąd odczytu: {e}")
                break

    def get_current_angles(self):
        """
        Zwraca aktualne kąty jako listę.
        Tworzona jest kopia, aby zapobiec przypadkowej modyfikacji danych przez inne moduły.
        """
        with self.lock:
            return self.current_angles.copy()

    def send_target_angles(self, th1, th2, th3, th4):
        """
        Wysyła wartości docelowe osi do ESP32.
        Oczekuje wartości w radianach — konwertuje je na stopnie.
        Format wysyłanej komendy: X...,Y...,Z...,E...
        """
        if not self.ser or not self.ser.is_open:
            return False, "Port nie jest otwarty"

        # Konwersja radianów na stopnie i budowa ramki
        radians = [th1, th2, th3, th4]
        degrees = [math.degrees(v) for v in radians]
        cmd = (
            f"X{degrees[0]:.2f},"
            f"Y{degrees[1]:.2f},"
            f"Z{degrees[2]:.2f},"
            f"E{degrees[3]:.2f}\n"
        )

        try: # Wysłanie ramki i wyświetlenie potwierdzenia
            self.ser.write(cmd.encode("utf-8"))
            return True, f"Wysłano: {cmd.strip()}"
        except Exception as e:
            return False, f"Błąd wysyłania: {e}"

    def close(self):
        """
        Zatrzymuje wątek odczytu i zamyka port.
        """
        self.running = False

        # Czekamy na zakończenie wątku, żeby uniknąć wycieków wątków.
        if self.reader_thread:
            self.reader_thread.join(timeout=1.0)

        # Bezpieczne zamknięcie portu.
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except: pass

# =====================================================================
# KINEMATYKA (Model matematyczny robota 5-DOF)
# =====================================================================

# Parametry geometryczne (Denavit-Hartenberg / wymiary ogniw w mm)
l1_val = 18.4
l2_val = 149.0
l3_val = 120.3
l4_val = 87.8
l5_val = 23.0
lambda1_val = 110.8
lambda5_val = 10.0

phi_offset = math.atan2(lambda5_val, l4_val + l5_val)

# Zmienne symboliczne (używane przez SymPy do analizy, rzadziej w runtime)
th1, th2, th3, th4, alpha5 = sp.symbols('θ1 θ2 θ3 θ4 α5')
pi = sp.pi

# --- Macierze transformacji jednorodnych ---
def RotZ(theta):
    c, s = sp.cos(theta), sp.sin(theta)
    return sp.Matrix([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def RotX(alpha):
    c, s = sp.cos(alpha), sp.sin(alpha)
    return sp.Matrix([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def TransZ(d):
    return sp.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])

def TransX(a):
    return sp.Matrix([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def forward_kinematics(th1_val, th2_val, th3_val, th4_val, alpha5_val=0.0):
    """
    Oblicza kinematykę prostą (FK) przy użyciu macierzy symbolicznych SymPy.
    UWAGA: Funkcja kosztowna obliczeniowo. Do wizualizacji realtime użyj get_joint_positions.
    """
    alpha1_val = np.pi / 2
    alpha4_val = -np.pi / 2
    
    A1 = RotZ(th1_val) * TransZ(lambda1_val) * TransX(l1_val) * RotX(alpha1_val)
    A2 = RotZ(th2_val) * TransX(l2_val)
    A3 = RotZ(-th3_val) * TransX(l3_val)
    A4 = RotZ(-th4_val) * TransX(l4_val) * RotX(alpha4_val)
    A5 = TransZ(lambda5_val) * TransX(l5_val) * RotX(alpha5_val)
    
    A_total = A1 * A2 * A3 * A4 * A5
    A_num = np.array(A_total.evalf().tolist()).astype(float)

    print("[DEBUG FK] Asum =\n", A_num)
    return A_num

def inverse_kinematics(R, Z, th1_base, phi_deg=0.0, elbow_up=True, reverse_base=False):
    """
    Rozwiązuje zagadnienie kinematyki odwrotnej (IK) metodą geometryczną.
    Redukuje problem 3D do płaszczyzny 2D zdefiniowanej przez kąt bazy th1.
    """
    th1 = th1_base
    
    # Efektywne długości ramion
    L1 = l2_val
    L2 = l3_val
    L3 = math.sqrt((l4_val + l5_val)**2 + lambda5_val**2)
    
    phi_rad = math.radians(phi_deg)
    phi_corr = phi_rad + phi_offset
    
    # Współrzędna radialna w układzie cylindrycznym
    R_ik = -R if reverse_base else R
    
    # Wyznaczenie pozycji nadgarstka (Wrist Point)
    R_wrist = R_ik - l1_val - L3 * math.cos(phi_corr)
    Z_wrist = Z - lambda1_val - L3 * math.sin(phi_corr)
    
    D = math.hypot(R_wrist, Z_wrist)
    
    # Sprawdzenie zasięgu (nierówność trójkąta)
    if D > (L1 + L2) or D < abs(L1 - L2):
        return None
    
    # Twierdzenie cosinusów
    cos_th3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_th3 = np.clip(cos_th3, -1.0, 1.0)
    
    th3_ik = math.acos(cos_th3) if elbow_up else -math.acos(cos_th3)
    
    alpha = math.atan2(Z_wrist, R_wrist)
    beta = math.atan2(L2 * math.sin(th3_ik), L1 + L2 * math.cos(th3_ik))
    
    th2 = alpha - beta
    
    # Kąty wynikowe
    th4_ik = phi_rad - th2 - th3_ik
    th3 = -th3_ik
    th4 = -th4_ik
    
    # Normalizacja do [-pi, pi]
    th1 = math.atan2(math.sin(th1), math.cos(th1))
    th2 = math.atan2(math.sin(th2), math.cos(th2))
    th3 = math.atan2(math.sin(th3), math.cos(th3))
    th4 = math.atan2(math.sin(th4), math.cos(th4))
    
    return (th1, th2, th3, th4)

JOINT_CONST = pi/2
JOINT_LIMITS = {
    'th1': (-JOINT_CONST, JOINT_CONST),       # Baza
    'th2': (0            , JOINT_CONST * 1.5),# Bark
    'th3': (-JOINT_CONST, JOINT_CONST),       # Łokieć
    'th4': (-JOINT_CONST, JOINT_CONST)        # Nadgarstek
}

def check_constraints(th1, th2, th3, th4):
    """Weryfikuje limity kątowe oraz bezkolizyjność z płaszczyzną stołu (Z=0)."""
    # 1. Limity złączy
    if not (JOINT_LIMITS['th1'][0] <= th1 <= JOINT_LIMITS['th1'][1]): return False
    if not (JOINT_LIMITS['th2'][0] <= th2 <= JOINT_LIMITS['th2'][1]): return False
    if not (JOINT_LIMITS['th3'][0] <= th3 <= JOINT_LIMITS['th3'][1]): return False
    if not (JOINT_LIMITS['th4'][0] <= th4 <= JOINT_LIMITS['th4'][1]): return False

    # 2. Kolizja z podłożem (Z < 0)
    z_elbow = lambda1_val + l2_val * math.sin(th2)
    if z_elbow < 0: return False

    z_wrist = z_elbow + l3_val * math.sin(th2 - th3)
    if z_wrist < 0: return False

    return True

def calculate_joint_distance(q_current, q_target):
    """Oblicza ważoną odległość w przestrzeni konfiguracyjnej (C-Space)."""
    if q_target is None: return float('inf')
    
    total_distance = 0
    weights = [1.0, 1.0, 1.0, 1.0]
    
    for i in range(len(q_current)):
        diff = q_target[i] - q_current[i]
        # Normalizacja różnicy kątowej
        normalized_diff = (diff + math.pi) % (2 * math.pi) - math.pi
        total_distance += abs(normalized_diff) * weights[i]
    
    return total_distance

def calculate_configuration_cost(angles, current_angles):
    """
    Funkcja kosztu dla optymalizatora IK.
    Składniki: JRA (odległość od limitów), Distance Cost (ruch), Singularity Cost.
    """
    th1, th2, th3, th4 = angles
    
    # 1. Koszt limitów
    joint_limit_cost = 0
    joints = [th1, th2, th3, th4]
    limits = [JOINT_LIMITS['th1'], JOINT_LIMITS['th2'], JOINT_LIMITS['th3'], JOINT_LIMITS['th4']]
    
    for val, (min_l, max_l) in zip(joints, limits):
        normalized = (val - min_l) / (max_l - min_l)
        joint_limit_cost += (normalized - 0.5)**2
    
    # 2. Koszt ruchu
    motion_cost = calculate_joint_distance(current_angles, angles)
    
    # 3. Koszt singularności (th3 bliskie 0)
    singularity_cost = 1.0 / (abs(th3) + 0.1)
    
    return (2.0 * joint_limit_cost + 1.0 * motion_cost + 3.0 * singularity_cost)

def solve_ik_for_cartesian(x_target, y_target, z_target, phi_deg, current_angles):
    """
    Wrapper IK z optymalizacją. Przeszukuje przestrzeń orientacji (Phi), jeśli nie jest zadana,
    oraz sprawdza różne konfiguracje geometryczne (elbow up/down, base flip).
    """
    R_target = math.sqrt(x_target**2 + y_target**2)
    th1_base = math.atan2(y_target, x_target) if R_target > 0.01 else 0.0
    
    phi_range = [phi_deg] if phi_deg is not None else range(-180, 180, 5)
    
    all_solutions = []
    strategies = [
        (True, False, "Baza Normalna, Łokieć GÓRA"),
        (False, False, "Baza Normalna, Łokieć DÓŁ"),
        (True, True, "Baza Odwrócona, Łokieć GÓRA"),
        (False, True, "Baza Odwrócona, Łokieć DÓŁ")
    ]

    for phi in phi_range:
        for elbow_up, reverse_base, name in strategies:
            # Korekta bazy dla konfiguracji odwróconej
            th1_in = ((th1_base + math.pi) + math.pi) % (2*math.pi) - math.pi if reverse_base else th1_base

            sol = inverse_kinematics(R_target, z_target, th1_in, phi, elbow_up, reverse_base)
            
            if sol and check_constraints(*sol):
                mech_cost = calculate_configuration_cost(sol, current_angles)
                orientation_penalty = abs(phi) * 10 if phi_deg is None else 0.0
                all_solutions.append((mech_cost + orientation_penalty, sol, f"{name}, φ={phi}°"))

    if not all_solutions:
        return None, "BRAK ROZWIĄZANIA (Ograniczenia lub Zasięg)"
    
    # Zwróć rozwiązanie o najmniejszym koszcie
    all_solutions.sort(key=lambda x: x[0])
    return all_solutions[0][1], all_solutions[0][2]

def get_joint_positions(th1_val, th2_val, th3_val, th4_val, alpha5_val=0.0):
    """
    Szybka kinematyka prosta oparta na NumPy dla celów wizualizacji (zwraca pozycje węzłów).
    Wykorzystuje te same parametry DH co wersja SymPy.
    """
    alpha1_val, alpha4_val = np.pi / 2, -np.pi / 2

    # Funkcje pomocnicze inline (dla wydajności)
    def np_RotZ(t): c, s = np.cos(t), np.sin(t); return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    def np_RotX(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
    def np_TransZ(d): return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])
    def np_TransX(a): return np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # Łańcuch transformacji
    T1 = np_RotZ(th1_val) @ np_TransZ(lambda1_val) @ np_TransX(l1_val) @ np_RotX(alpha1_val)
    T2 = np_RotZ(th2_val) @ np_TransX(l2_val)
    T3 = np_RotZ(-th3_val) @ np_TransX(l3_val)
    T4 = np_RotZ(-th4_val) @ np_TransX(l4_val) @ np_RotX(alpha4_val)
    T5 = np_TransZ(lambda5_val) @ np_TransX(l5_val) @ np_RotX(alpha5_val)

    # Obliczenie pozycji węzłów względem bazy
    origin = np.array([0, 0, 0, 1])
    points = [
        np.array([0, 0, 0]),          # P1: Baza
        (T1 @ origin)[:3],            # P2: Bark
        (T1 @ T2 @ origin)[:3],       # P3: Łokieć
        (T1 @ T2 @ T3 @ origin)[:3],  # P4: Nadgarstek
        (T1 @ T2 @ T3 @ T4 @ T5 @ origin)[:3] # P5: Końcówka (TCP)
    ]

    return np.array(points)

# =====================================================================
# GUI APPLICATION
# =====================================================================

POSITION_TOLERANCE = 5.0 # mm

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry("1400x800")

        self.last_gui_update_time = 0.0
        self.last_tcp_pos = np.array([0.0, 0.0, 0.0])
        
        self.robot = RobotSerial()
        self.robot.gui_callback = self.update_position_display
        
        # Wizualizacja
        self.plot_artists = []
        self.max_reach = l2_val + l3_val + l4_val + l5_val
        self.control_mode = tk.StringVar(value='position')
        
        # Stan sterowania
        self.angle_send_timer = None
        self.angle_send_active = False
        self._collision_warning_shown = False
        self._last_collision_state = False
        
        # Sekwencer
        self.is_connected = False
        self.sequence_data = []  
        self.sequence_list_frame = None
        self.canvas_window_seq = None
        self.current_sequence_index = 0
        self.sequence_playing = False
        self.sequence_timer = None
        self.play_button = None 
        self.target_xyz = None 

        self.setup_ui()
        self.setup_3d_plot()
        self.update_3d_visualization()
        self.root.after(100, self.connect_robot)

    def setup_ui(self):
        # Stałe kolorów
        COLORS = {
            'BG': "#2f3347", 'FRAME': "#262a3e", 'BORDER': "#565a6c",
            'TEXT': "#c4cad0", 'ACCENT': "#101122", 'BTN': "#2f3347",
            'STOP': "#cb0000", 'PLAY': "#008000"
        }

        # Konfiguracja stylu
        style = ttk.Style()
        style.theme_use('clam')
        style.configure(".", background=COLORS['BG'], foreground=COLORS['TEXT'], fieldbackground=COLORS['FRAME'],
                        darkcolor=COLORS['FRAME'], lightcolor=COLORS['FRAME'], bordercolor=COLORS['BORDER'])
        style.configure("TLabelframe", background=COLORS['FRAME'], bordercolor=COLORS['BORDER'], relief="solid")
        style.configure("TLabelframe.Label", background=COLORS['FRAME'], foreground=COLORS['TEXT']) 
        style.configure("TLabel", background=COLORS['FRAME'], foreground=COLORS['TEXT'])
        style.configure("TEntry", fieldbackground=COLORS['FRAME'], foreground=COLORS['TEXT'], insertcolor=COLORS['TEXT'], bordercolor=COLORS['BORDER'])
        
        # Style przycisków
        btn_configs = [
            ("TButton", COLORS['BTN'], COLORS['ACCENT']),
            ("Play.TButton", COLORS['PLAY'], '#004d00'),
            ("Stop.TButton", COLORS['STOP'], '#8c0000')
        ]
        for name, bg, active in btn_configs:
            style.configure(name, background=bg, foreground="white" if name != "TButton" else COLORS['TEXT'], bordercolor=COLORS['BORDER'])
            style.map(name, background=[('active', active)])

        # Główne kontenery (Grid Layout)
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nswe")
        
        # Lewa kolumna (Scrollbar)
        scroll_canvas = tk.Canvas(main_frame, borderwidth=0, background=COLORS['BG'], highlightthickness=0)
        scroll_canvas.grid(row=0, column=0, sticky="nswe", padx=(0, 5))
        vscrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=scroll_canvas.yview)
        vscrollbar.grid(row=0, column=0, sticky="nse") 
        scroll_canvas.configure(yscrollcommand=vscrollbar.set)
        
        self.content_frame = tk.Frame(scroll_canvas, bg=COLORS['BG']) 
        self.content_frame.grid_columnconfigure(0, weight=1) 
        self.canvas_window = scroll_canvas.create_window((0, 0), window=self.content_frame, anchor="nw", width=1) 
        
        # Obsługa skalowania Canvas
        self.content_frame.bind("<Configure>", lambda e: scroll_canvas.configure(scrollregion=scroll_canvas.bbox("all")))
        scroll_canvas.bind("<Configure>", lambda e: scroll_canvas.itemconfig(self.canvas_window, width=e.width - vscrollbar.winfo_width()))

        # Prawa kolumna - 3D
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky="nswe", padx=(5, 0))
        
        # === ELEMENTY LEWEJ KOLUMNY ===
        
        # 1. Status
        status_frame = ttk.LabelFrame(self.content_frame, text="Status połączenia", padding="5")
        status_frame.grid(row=0, column=0, sticky="ew", pady=5, padx=5)
        self.status_label = ttk.Label(status_frame, text="Rozłączony", foreground="red")
        self.status_label.grid(row=0, column=0, sticky="w")
        ttk.Button(status_frame, text="Połącz", command=self.connect_robot).grid(row=0, column=1, padx=5)
        
        # 2. Tryb sterowania
        mode_frame = ttk.LabelFrame(self.content_frame, text="Tryb sterowania", padding="10")
        mode_frame.grid(row=1, column=0, sticky="ew", pady=5, padx=5)
        for i, (txt, val) in enumerate([("Sterowanie pozycją (XYZ)", 'position'), ("Sterowanie kątami (ciągłe)", 'angles')]):
            ttk.Radiobutton(mode_frame, text=txt, variable=self.control_mode, value=val, 
                            command=self.switch_control_mode).grid(row=0, column=i, sticky="w", pady=2)
        
        # 3. Aktualna pozycja
        pos_frame = ttk.LabelFrame(self.content_frame, text="Aktualna pozycja", padding="10")
        pos_frame.grid(row=2, column=0, sticky="ew", pady=5, padx=5)
        
        self.pos_labels = {}
        for i, axis in enumerate(['X', 'Y', 'Z', 'E', 'A']):
            ttk.Label(pos_frame, text=f"{axis}:").grid(row=0, column=i, sticky="w", pady=2)
            lbl = ttk.Label(pos_frame, text="0.00°", font=('Arial', 10))
            lbl.grid(row=0, column=i, sticky="w", padx=10)
            self.pos_labels[axis] = lbl
            
        ttk.Separator(pos_frame, orient='horizontal').grid(row=1, column=0, columnspan=5, sticky='ew', pady=5)
        
        self.tcp_labels = {}
        for i, axis in enumerate(['TCP_X', 'TCP_Y', 'TCP_Z']):
            col = i * 2
            ttk.Label(pos_frame, text=f"{axis.replace('TCP_', '')} [mm]:").grid(row=2, column=col, sticky="w", pady=2)
            lbl = ttk.Label(pos_frame, text="0.00", font=('Arial', 10, 'bold'), foreground="#377df0")
            lbl.grid(row=2, column=col+1, sticky="w", padx=5)
            self.tcp_labels[axis] = lbl

        # 4. Kąty docelowe (IK)
        self.angles_frame = ttk.LabelFrame(self.content_frame, text="Kąty docelowe (wynik IK)", padding="10")
        self.angles_frame.grid(row=3, column=0, sticky="ew", pady=5, padx=5)
        self.angle_labels = {}
        labels_map = ['θ1 (X)', 'θ2 (Y)', 'θ3 (Z)', 'θ4 (E)']
        for i, axis in enumerate(labels_map):
            ttk.Label(self.angles_frame, text=f"{axis}:").grid(row=0, column=i*2, sticky="e", padx=5, pady=2)
            lbl = ttk.Label(self.angles_frame, text="0.00°", font=('Arial', 10))
            lbl.grid(row=0, column=i*2+1, sticky="w", padx=5, pady=2)
            self.angle_labels[axis] = lbl
        
        # 5. Sterowanie kątami (Suwaki)
        self.angle_control_frame = ttk.LabelFrame(self.content_frame, text="Sterowanie kątami [°]", padding="10")
        self.angle_control_frame.grid(row=4, column=0, sticky="ew", pady=5, padx=5)
        
        self.angle_sliders = {}
        self.angle_value_labels = {}
        joint_defs = [('th1', 'θ1 (X - Baza)'), ('th2', 'θ2 (Y - Ramię L2)'), 
                      ('th3', 'θ3 (Z - Ramię L3)'), ('th4', 'θ4 (E - Nadgarstek)')]
        
        for i, (key, label) in enumerate(joint_defs):
            ttk.Label(self.angle_control_frame, text=label).grid(row=i, column=0, sticky="w", pady=5)
            min_rad, max_rad = JOINT_LIMITS[key]
            slider = tk.Scale(self.angle_control_frame, from_=math.degrees(min_rad), to=math.degrees(max_rad),    
                              orient=tk.HORIZONTAL, resolution=0.1, length=250,
                              bg=COLORS['FRAME'], fg=COLORS['TEXT'], troughcolor=COLORS['BG'], highlightthickness=0, 
                              command=lambda val, k=key: self.on_angle_slider_change(k, val))
            slider.set(0.0)
            slider.grid(row=i, column=1, padx=5, pady=5)
            self.angle_sliders[key] = slider
            
            val_lbl = ttk.Label(self.angle_control_frame, text="0.0°", font=('Arial', 10, 'bold'))
            val_lbl.grid(row=i, column=2, padx=5)
            self.angle_value_labels[key] = val_lbl
        
        self.live_control_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(self.angle_control_frame, text="WŁĄCZ LIVE (Transmisja ciągła)",
                        variable=self.live_control_var, command=self.toggle_live_control).grid(row=len(joint_defs), column=0, columnspan=3, pady=10, sticky="w")

        ttk.Button(self.angle_control_frame, text="DODAJ AKTUALNE KĄTY (do sekwencji)", 
                   command=self.add_current_angles_to_sequence).grid(row=len(joint_defs)+1, column=0, columnspan=3, pady=5, sticky="ew")
        
        # 6. Pozycja docelowa (Input XYZ)
        self.target_frame = ttk.LabelFrame(self.content_frame, text="Pozycja docelowa [mm]", padding="10")
        self.target_frame.grid(row=5, column=0, sticky="ew", pady=5, padx=5)
        
        # Generowanie pól w pętli (optymalizacja redundancji)
        input_defs = [("X:", "-87.5", "x_entry"), ("Y:", "0", "y_entry"), 
                      ("Z:", "372.5", "z_entry"), ("Orientacja φ [°]:", "0", "phi_entry")]
        for i, (lbl_txt, def_val, attr_name) in enumerate(input_defs):
            ttk.Label(self.target_frame, text=lbl_txt).grid(row=i, column=0, sticky="w", pady=5)
            entry = ttk.Entry(self.target_frame, width=15)
            entry.grid(row=i, column=1, padx=5)
            entry.insert(0, def_val)
            setattr(self, attr_name, entry)
        
        self.auto_phi_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.target_frame, text="Orientacja automatyczna",
                        variable=self.auto_phi_var, command=self.toggle_phi_entry).grid(row=4, column=0, columnspan=2, pady=5, sticky="w")
        
        btn_row = ttk.Frame(self.target_frame)
        btn_row.grid(row=5, column=0, columnspan=2, sticky="ew", pady=5)
        btn_row.columnconfigure((0, 1), weight=1, uniform="g")
        ttk.Button(btn_row, text="WYŚLIJ POZYCJĘ", command=self.send_position).grid(row=0, column=0, padx=(0, 2), sticky="ew")
        ttk.Button(btn_row, text="DODAJ PUNKT", command=self.add_point_to_sequence).grid(row=0, column=1, padx=(2, 0), sticky="ew")
        
        self.toggle_phi_entry()
        
        # 7. Log
        log_frame = ttk.LabelFrame(self.content_frame, text="Log komunikacji", padding="10")
        log_frame.grid(row=7, column=0, sticky="ew", pady=5, padx=5)
        self.log_text = tk.Text(log_frame, height=10, width=50, bg=COLORS['FRAME'], fg=COLORS['TEXT'], 
                                insertbackground=COLORS['TEXT'], selectbackground=COLORS['ACCENT'],
                                highlightbackground=COLORS['BORDER'], highlightthickness=1)
        self.log_text.grid(row=0, column=0, sticky="nswe")
        sb = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.log_text['yscrollcommand'] = sb.set
        log_frame.grid_columnconfigure(0, weight=1)

        # 8. Sekwencer
        self._create_sequence_frame(self.content_frame, 8) 

        # === WIZUALIZACJA 3D (PRAWA STRONA) ===
        viz_frame = ttk.LabelFrame(right_frame, text="Wizualizacja 3D", padding="10")
        viz_frame.grid(row=0, column=0, sticky="nswe")
        
        self.fig = Figure(figsize=(7, 6), dpi=100, facecolor=COLORS['BG']) 
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Konfiguracja kolorów wykresu
        self.ax.set_facecolor(COLORS['BG'])
        for axis in [self.ax.xaxis, self.ax.yaxis, self.ax.zaxis]:
            axis.label.set_color(COLORS['TEXT'])
            axis.set_pane_color((0.14, 0.16, 0.24, 1.0))
            axis.set_tick_params(colors=COLORS['TEXT'])
            
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        ttk.Button(viz_frame, text="Resetuj widok", command=self.reset_3d_view).pack(side=tk.TOP, anchor="nw", pady=5, padx=5)
        self.canvas.get_tk_widget().config(bg=COLORS['BG'])
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Konfiguracja wag grid
        self.root.columnconfigure(0, weight=1); self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=3); main_frame.columnconfigure(1, weight=7) 
        main_frame.rowconfigure(0, weight=1); right_frame.rowconfigure(0, weight=1)
        
        self.switch_control_mode()

    def setup_3d_plot(self):
        """Inicjalizacja statycznych elementów wykresu."""
        L_COL = "#c4cad0"
        self.ax.set_xlabel('X [mm]', color=L_COL)
        self.ax.set_ylabel('Y [mm]', color=L_COL)
        self.ax.set_zlabel('Z [mm]', color=L_COL)
        self.ax.set_title('Pozycja robota', color=L_COL)
        
        self.ax.set_xlim([-self.max_reach, self.max_reach])
        self.ax.set_ylim([-self.max_reach, self.max_reach])
        self.ax.set_zlim([0, self.max_reach])
        self.ax.view_init(elev=20, azim=45)
        
        # Siatka
        r = np.arange(-300, 301, 100)
        X, Y = np.meshgrid(r, r)
        self.ax.plot_wireframe(X, Y, np.zeros_like(X), alpha=0.1, color='gray', linewidth=0.5)
        
        # Układ współrzędnych
        al = 100
        self.ax.quiver(0, 0, 0, al, 0, 0, color='red', arrow_length_ratio=0.1, lw=2, label='X')
        self.ax.quiver(0, 0, 0, 0, al, 0, color='green', arrow_length_ratio=0.1, lw=2, label='Y')
        self.ax.quiver(0, 0, 0, 0, 0, al, color='blue', arrow_length_ratio=0.1, lw=2, label='Z')
        self.ax.legend(loc='upper right')

    def reset_3d_view(self):
        self.ax.set_xlim([-self.max_reach, self.max_reach])
        self.ax.set_ylim([-self.max_reach, self.max_reach])
        self.ax.set_zlim([0, self.max_reach])
        self.ax.view_init(elev=20, azim=45)
        self.canvas.draw()

    def update_3d_visualization(self):
        """Odświeża pozycję ramienia na wykresie 3D."""
        try:
            # Usunięcie starych elementów
            for artist in self.plot_artists:
                if isinstance(artist, list):
                    for i in artist: i.remove()
                else:
                    artist.remove()
            self.plot_artists.clear()
            
            # Pobranie pozycji
            angles = self.robot.get_current_angles()
            rads = [math.radians(a) for a in angles[:4]]
            positions = get_joint_positions(*rads) # Zwraca tablicę NumPy
            self.last_tcp_pos = positions[-1]
            
            # Rysowanie
            xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]
            
            lines = self.ax.plot(xs, ys, zs, 'o-', linewidth=2, markersize=6, color='#377df0', markerfacecolor='lightblue')
            tcp = self.ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], c="#cb0000", s=50, edgecolors='darkred', linewidths=2)
            
            self.plot_artists.extend([lines, tcp])
            
            # Etykiety węzłów
            labels = ['Base', 'J1', 'J2', 'J3', 'J4', 'TCP']
            for i, pos in enumerate(positions):
                txt = self.ax.text(pos[0], pos[1], pos[2], f'  {labels[i]}', fontsize=8, color="#c4cad0")
                self.plot_artists.append(txt)
            
            self.canvas.draw()
            
        except Exception as e:
            if not hasattr(self, '_viz_err'): 
                self.log(f"Błąd wizualizacji: {e}")
                self._viz_err = True
        
        self.root.after(50, self.update_3d_visualization)
        
    def log(self, message):
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)

    def update_position_display(self, angles=None, log_message=None):
        """Callback z wątku serial. Zoptymalizowany pod kątem redukcji wywołań event loop."""
        
        def _update_ui():
            if log_message:
                if log_message == "#CONNECTION_LOST#":
                    self.status_label.config(text="Rozłączony (Błąd I/O)", foreground="red")
                    self.log("BŁĄD KRYTYCZNY: Utracono połączenie")
                else:
                    self.log(log_message)
            
            if angles:
                # Aktualizacja etykiet kątów
                for i, axis in enumerate(['X', 'Y', 'Z', 'E', 'A']):
                    self.pos_labels[axis].config(text=f"{angles[i]:.2f}°")
                
                # Aktualizacja etykiet TCP
                tcp = self.last_tcp_pos
                for i, axis in enumerate(['TCP_X', 'TCP_Y', 'TCP_Z']):
                    self.tcp_labels[axis].config(text=f"{tcp[i]:.2f}")

        # Limitowanie częstotliwości odświeżania GUI (throttle)
        now = time.time() * 1000.0
        if log_message or (now - self.last_gui_update_time >= 200.0):
            self.last_gui_update_time = now
            # Pojedyncze wywołanie after zamiast serii
            self.root.after(0, _update_ui)

    def toggle_phi_entry(self):
        state = 'disabled' if self.auto_phi_var.get() else 'normal'
        self.phi_entry.config(state=state)

    def connect_robot(self):
        success, msg = self.robot.connect()
        self.status_label.config(text="Połączono" if success else "Rozłączony", 
                                 foreground="green" if success else "red")
        if not success: messagebox.showerror("Błąd", msg)
        self.log(msg)
        self.is_connected = success
    
    def send_position(self):
        try:
            x = float(self.x_entry.get()); y = float(self.y_entry.get()); z = float(self.z_entry.get())
            phi = None if self.auto_phi_var.get() else float(self.phi_entry.get())
            
            self.log(f"IK dla: X={x}, Y={y}, Z={z}")
            cur_rads = [math.radians(a) for a in self.robot.get_current_angles()[:4]]
            
            sol, name = solve_ik_for_cartesian(x, y, z, phi, tuple(cur_rads))
            
            if sol is None:
                messagebox.showerror("Błąd IK", "Pozycja poza zasięgiem.")
                return
            
            # Weryfikacja kolizji końcowej
            pos = get_joint_positions(*sol)
            if pos[3][2] < -0.01 or pos[4][2] < -0.01:
                messagebox.showerror("Kolizja", "Rozwiązanie koliduje z podłożem!")
                return
            
            # Aktualizacja wyświetlacza kątów docelowych
            for i, (val, lbl) in enumerate(zip(sol, ['θ1 (X)', 'θ2 (Y)', 'θ3 (Z)', 'θ4 (E)'])):
                self.angle_labels[lbl].config(text=f"{math.degrees(val):.2f}°")
            
            ok, msg = self.robot.send_target_angles(*sol)
            if ok: self.log(f"Wysłano konfigurację: {name}")
            else: messagebox.showerror("Błąd komunikacji", msg)
                
        except ValueError:
            messagebox.showerror("Błąd", "Niepoprawne dane wejściowe")
        except Exception as e:
            self.log(f"Błąd: {e}")
    
    def on_angle_slider_change(self, key, val):
        angle = float(val)
        self.angle_value_labels[key].config(text=f"{angle:.1f}°")
        
        # Szybkie sprawdzenie kolizji dla suwaków
        rads = [math.radians(self.angle_sliders[k].get()) for k in ['th1','th2','th3','th4']]
        pos = get_joint_positions(*rads)
        min_z = np.min(pos[:, 2])
        
        is_collision = min_z < 0
        color = 'red' if is_collision else 'green'
        self.angle_value_labels[key].config(foreground=color)
        
        if is_collision and not self._last_collision_state:
            self.log(f"OSTRZEŻENIE: Kolizja Z={min_z:.1f}mm")
        self._last_collision_state = is_collision
    
    def switch_control_mode(self):
        mode = self.control_mode.get()
        self.stop_continuous_send()
        self.live_control_var.set(False)
        
        if mode == 'position':
            self.target_frame.grid()
            self.angle_control_frame.grid_remove()
            self.angles_frame.grid()
            self.log("Tryb: Pozycja (XYZ)")
        else:
            cur = self.robot.get_current_angles()
            for i, k in enumerate(['th1','th2','th3','th4']):
                self.angle_sliders[k].set(cur[i])
            
            self.target_frame.grid_remove()
            self.angle_control_frame.grid()
            self.angles_frame.grid_remove()
            self.log("Tryb: Kąty (Joints)")
    
    def start_continuous_send(self):
        self.angle_send_active = True
        self.send_angles_continuously()
    
    def stop_continuous_send(self):
        self.angle_send_active = False
        if self.angle_send_timer:
            self.root.after_cancel(self.angle_send_timer)
            self.angle_send_timer = None
    
    def send_angles_continuously(self):
        if not self.angle_send_active: return
        
        rads = [math.radians(self.angle_sliders[k].get()) for k in ['th1','th2','th3','th4']]
        pos = get_joint_positions(*rads)
        
        if np.min(pos[:, 2]) < 0:
            if not self._collision_warning_shown:
                self.log("KRYTYCZNE: Ruch zablokowany (Kolizja)")
                self._collision_warning_shown = True
        else:
            if self._collision_warning_shown:
                self.log("Strefa bezpieczna")
                self._collision_warning_shown = False
            
            t = time.time()
            if not hasattr(self, '_last_snd') or t - self._last_snd >= 0.2:
                self.robot.send_target_angles(*rads)
                self._last_snd = t
                
        self.angle_send_timer = self.root.after(50, self.send_angles_continuously)
    
    def on_closing(self):
        self.is_connected = False
        self.stop_continuous_send()
        self.stop_sequence("APP CLOSE")
        self.robot.close()
        self.root.destroy()
        
    # --- PROGRAMOWANIE SEKWENCYJNE ---
    
    def toggle_live_control(self):
        if self.live_control_var.get():
            if self.sequence_playing:
                self.live_control_var.set(False)
                return
            self.start_continuous_send()
        else:
            self.stop_continuous_send()

    def _create_sequence_frame(self, master, row):
        frame = ttk.LabelFrame(master, text="Rejestrator/Odtwarzacz sekwencji", padding="10")
        frame.grid(row=row, column=0, sticky="nsew", pady=5, padx=5)
        frame.columnconfigure(0, weight=1)
        
        ctrl = tk.Frame(frame, bg="#262a3e")
        ctrl.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 5))
        ctrl.columnconfigure((0, 4), weight=1)
        
        ttk.Button(ctrl, text="WCZYTAJ", command=self.load_sequence_from_json).grid(row=0, column=1, padx=5)
        ttk.Button(ctrl, text="ZAPISZ", command=self.save_sequence_to_json).grid(row=0, column=2, padx=5)
        self.play_button = ttk.Button(ctrl, text="START SEKWENCJI", command=self.play_sequence, style='Play.TButton')
        self.play_button.grid(row=0, column=3, padx=5)
        
        ttk.Separator(frame, orient='horizontal').grid(row=1, column=0, columnspan=2, sticky='ew', pady=5)
        
        cnv = tk.Canvas(frame, borderwidth=0, background="#262a3e", highlightthickness=0)
        vsb = ttk.Scrollbar(frame, orient="vertical", command=cnv.yview)
        
        self.sequence_list_frame = ttk.Frame(cnv, padding="5", style='TLabelframe')
        self.sequence_list_frame.columnconfigure(0, weight=1)
        
        self.canvas_window_seq = cnv.create_window((0, 0), window=self.sequence_list_frame, anchor="nw", width=1)
        cnv.configure(yscrollcommand=vsb.set)
        
        cnv.grid(row=2, column=0, sticky="nswe")
        vsb.grid(row=2, column=1, sticky="ns")
        frame.rowconfigure(2, weight=1)
        
        self.sequence_list_frame.bind("<Configure>", lambda e: cnv.configure(scrollregion=cnv.bbox("all")))
        cnv.bind("<Configure>", lambda e: cnv.itemconfig(self.canvas_window_seq, width=e.width - vsb.winfo_width()))
        
        self._update_sequence_display()

    def load_sequence_from_json(self):
        f = askopenfilename(filetypes=[("JSON", "*.json")])
        if f:
            try:
                with open(f, 'r', encoding='utf-8') as fp:
                    self.sequence_data = json.load(fp)
                self._update_sequence_display()
                self.log(f"Wczytano {len(self.sequence_data)} pkt")
            except Exception as e:
                messagebox.showerror("Błąd", str(e))

    def play_sequence(self):
        if not self.is_connected:
            messagebox.showwarning("Info", "Brak połączenia")
            return
        
        if self.live_control_var.get():
            self.live_control_var.set(False)
            self.stop_continuous_send()
            
        if self.sequence_playing:
            self.stop_sequence()
        elif self.sequence_data:
            self.sequence_playing = True
            self.current_sequence_index = 0
            self.play_button.config(text="STOP SEKWENCJI", style='Stop.TButton')
            self._execute_sequence_step()
            
    def stop_sequence(self, msg="Stop"):
        self.sequence_playing = False
        if self.sequence_timer:
            self.root.after_cancel(self.sequence_timer)
            self.sequence_timer = None
        if self.play_button:
            self.play_button.config(text="START SEKWENCJI", style='Play.TButton')
        self.log(msg)
        self._update_sequence_display()

    def _execute_sequence_step(self):
        if not self.sequence_playing or self.current_sequence_index >= len(self.sequence_data):
            self.stop_sequence("Koniec sekwencji")
            return
        
        pt = self.sequence_data[self.current_sequence_index]
        self.log(f"Krok {self.current_sequence_index+1}: {pt['X'], pt['Y'], pt['Z']}")
        self.target_xyz = np.array([pt['X'], pt['Y'], pt['Z']])
        self._update_sequence_display(self.current_sequence_index)
        
        try:
            if pt.get('Joints'):
                rads = [math.radians(d) for d in pt['Joints']]
                self.robot.send_target_angles(*rads)
            else:
                cur = [math.radians(a) for a in self.robot.get_current_angles()[:4]]
                phi = pt['Fi'] if not pt['Auto_Fi'] else None
                sol, _ = solve_ik_for_cartesian(pt['X'], pt['Y'], pt['Z'], phi, tuple(cur))
                if sol: self.robot.send_target_angles(*sol)
                else: raise Exception("Brak IK")
                
            self._wait_for_position()
        except Exception as e:
            self.stop_sequence(f"Błąd kroku: {e}")

    def _wait_for_position(self):
        if not self.sequence_playing: return
        
        dist = np.linalg.norm(self.last_tcp_pos - self.target_xyz)
        if dist <= POSITION_TOLERANCE:
            self.current_sequence_index += 1
            self.sequence_timer = self.root.after(100, self._execute_sequence_step)
        else:
            self.sequence_timer = self.root.after(100, self._wait_for_position)

    def add_point_to_sequence(self):
        try:
            x, y, z = float(self.x_entry.get()), float(self.y_entry.get()), float(self.z_entry.get())
            auto = self.auto_phi_var.get()
            phi = None if auto else float(self.phi_entry.get())
            
            cur = [math.radians(a) for a in self.robot.get_current_angles()[:4]]
            if solve_ik_for_cartesian(x, y, z, phi, tuple(cur))[0]:
                self.sequence_data.append({"X":x, "Y":y, "Z":z, "Fi":phi if phi else 0, "Auto_Fi":auto})
                self._update_sequence_display()
            else:
                messagebox.showerror("Błąd", "Pozycja nieosiągalna")
        except ValueError: pass

    def add_current_angles_to_sequence(self):
        rads = [math.radians(self.angle_sliders[k].get()) for k in ['th1','th2','th3','th4']]
        pos = get_joint_positions(*rads)[-1]
        fi_rad = math.atan2(math.sin(sum(rads[1:])), math.cos(sum(rads[1:])))
        
        self.sequence_data.append({
            "X": round(pos[0], 2), "Y": round(pos[1], 2), "Z": round(pos[2], 2),
            "Fi": round(math.degrees(fi_rad), 2), "Auto_Fi": False,
            "Joints": [math.degrees(r) for r in rads]
        })
        self._update_sequence_display()

    def remove_point(self, idx):
        del self.sequence_data[idx]
        self._update_sequence_display()

    def _update_sequence_display(self, highlight_idx=None):
        for w in self.sequence_list_frame.winfo_children(): w.destroy()
        
        if not self.sequence_data:
            ttk.Label(self.sequence_list_frame, text="Brak punktów").grid(row=0, column=0, padx=5)
            return
            
        for i, pt in enumerate(self.sequence_data):
            is_hl = (i == highlight_idx and self.sequence_playing)
            fg = "#FFD700" if is_hl else "#c4cad0"
            style = 'TButton' if is_hl else 'TLabelframe'
            
            f = ttk.Frame(self.sequence_list_frame, padding="5", relief=tk.SOLID, borderwidth=1, style=style)
            f.grid(row=i, column=0, sticky="ew", pady=2); f.columnconfigure(1, weight=1)
            
            ttk.Label(f, text=f"PKT {i+1}", font=('Arial', 10, 'bold'), foreground=fg).grid(row=0, column=0, sticky="w")
            txt = f"X:{pt['X']:.1f} Y:{pt['Y']:.1f} Z:{pt['Z']:.1f}"
            ttk.Label(f, text=txt, foreground=fg).grid(row=1, column=0, sticky="w")
            ttk.Button(f, text="✕", width=2, command=lambda x=i: self.remove_point(x)).grid(row=0, column=1, rowspan=2, sticky="e")

    def save_sequence_to_json(self):
        f = asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if f:
            try:
                with open(f, 'w') as fp: json.dump(self.sequence_data, fp, indent=4)
            except Exception as e: messagebox.showerror("Błąd", str(e))

def main():
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()