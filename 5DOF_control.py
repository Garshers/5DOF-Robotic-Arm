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
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.figure import Figure

# =====================================================================
# KOMUNIKACJA SERIAL Z ESP-32
# =====================================================================

class RobotSerial:
    def __init__(self, port=None, baudrate=115200):
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0]  # [X, Y, Z, E, A]
        self.running = False
        self.read_thread = None
        self.lock = threading.Lock()
        self.gui_callback = None
        
    def set_gui_callback(self, callback):
        """Ustaw callback do aktualizacji GUI"""
        self.gui_callback = callback
        
    def find_esp32_port(self):
        """Automatyczne wykrywanie portu ESP-32"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.description or 'CP210' in port.description or 'CH340' in port.description:
                return port.device
        return None
    
    def connect(self):
        """Połączenie z ESP-32 (z obsługą ponownego łączenia)"""
        if self.ser and self.ser.is_open: # Jeśli port jest otwarty, zamknij go, aby zwolnić zasób
            self.close()

        if self.port is None:
            self.port = self.find_esp32_port()
            if self.port is None:
                return False, "Nie znaleziono ESP-32"
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.running = True
            self.read_thread = threading.Thread(target=self._continuous_read, daemon=True)
            self.read_thread.start()
            
            return True, f"Połączono z {self.port}"
        except Exception as e: # Zwalniamy zasów jeżeli wystpił błąd
            if self.ser and self.ser.is_open:
                self.ser.close()
            return False, f"Błąd połączenia: {e}"
    
    def _continuous_read(self):
        """Ciągły odczyt danych w osobnym wątku"""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line.startswith('<') and line.endswith('>'):
                        data = line[1:-1].split(',')
                        
                        with self.lock:
                            for item in data:
                                if len(item) < 2:
                                    continue
                                axis = item[0]
                                try:
                                    angle = float(item[1:])
                                    
                                    if axis == 'X': self.current_angles[0] = angle
                                    elif axis == 'Y': self.current_angles[1] = angle
                                    elif axis == 'Z': self.current_angles[2] = angle
                                    elif axis == 'E': self.current_angles[3] = angle
                                    elif axis == 'A': self.current_angles[4] = angle
                                except ValueError:
                                    pass
                        
                        # Wywołaj callback GUI
                        if self.gui_callback:
                            self.gui_callback(self.current_angles.copy())
                    
                    elif line and not line.startswith('<'):
                        if self.gui_callback:
                            self.gui_callback(None, log_message=line)
                
                time.sleep(0.01)

            except (serial.SerialException, OSError) as e: # Krytyczny błąd sprzętowy (np. wyjęcie kabla)
                self.running = False
                if self.ser:
                    try:
                        self.ser.close()
                    except:
                        pass
                
                if self.gui_callback:
                    self.gui_callback(None, log_message="#CONNECTION_LOST#")
                break

            except Exception as e:
                if self.running:
                    if self.gui_callback:
                        self.gui_callback(None, log_message=f"Błąd odczytu: {e}")
                break
    
    def get_current_angles(self):
        """Zwraca aktualne kąty"""
        with self.lock:
            return self.current_angles.copy()
    
    def send_target_angles(self, th1, th2, th3, th4):
        """Wysłanie kątów docelowych do ESP-32"""
        if self.ser is None or not self.ser.is_open:
            return False, "Brak połączenia"
        
        x_deg = math.degrees(th1)
        y_deg = math.degrees(th2)
        z_deg = math.degrees(th3)
        e_deg = math.degrees(th4)
        
        command = f"X{x_deg:.2f},Y{y_deg:.2f},Z{z_deg:.2f},E{e_deg:.2f}\n"
        
        try:
            self.ser.write(command.encode('utf-8'))
            return True, f"Wysłano: {command.strip()}"
        except Exception as e:
            return False, f"Błąd wysyłania: {e}"
    
    def close(self):
        """Zamknięcie połączenia"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

# =====================================================================
# KINEMATYKA
# =====================================================================

l1_val = 18.4
l2_val = 149.0
l3_val = 120.3
l4_val = 87.8
l5_val = 23.0
lambda1_val = 110.8
lambda5_val = 10.0

phi_offset = math.atan2(lambda5_val, l4_val + l5_val)

th1, th2, th3, th4, alpha5 = sp.symbols('θ1 θ2 θ3 θ4 α5')
pi = sp.pi

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
    Oblicza kinematykę odwrotną dla robota 5-DOF (Solver 2D).
    POPRAWIONO: Prawidłowa obsługa ujemnych wartości R_wrist (brak abs).
    """
    
    th1 = th1_base
    
    L1 = l2_val
    L2 = l3_val
    L3 = math.sqrt((l4_val + l5_val)**2 + lambda5_val**2)
    
    phi_rad = math.radians(phi_deg)
    phi_corr = phi_rad + phi_offset
    
    # R_ik to współrzędna cylindryczna (odległość od osi Z)
    R_ik = -R if reverse_base else R
    
    # Obliczamy pozycję nadgarstka w układzie lokalnym ramienia
    # Tutaj wynik może być ujemny, jeśli cel jest bliżej osi Z niż offset l1_val
    R_wrist = R_ik - l1_val - L3 * math.cos(phi_corr)
    Z_wrist = Z - lambda1_val - L3 * math.sin(phi_corr)
    
    # Dystans euklidesowy zawsze jest dodatni (math.hypot to sqrt(x^2 + y^2))
    D = math.hypot(R_wrist, Z_wrist)
    
    # Sprawdzenie zasięgu (nierówność trójkąta)
    if D > (L1 + L2) or D < abs(L1 - L2):
        return None
    
    # Twierdzenie cosinusów dla kąta w łokciu
    cos_th3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_th3 = np.clip(cos_th3, -1.0, 1.0)
    
    th3_ik = math.acos(cos_th3) if elbow_up else -math.acos(cos_th3)
    
    # ZMIANA: math.atan2(Y, X) z zachowaniem znaku X (R_wrist).
    # Pozwala to na poprawne wyznaczenie kąta alpha w II i III ćwiartce.
    alpha = math.atan2(Z_wrist, R_wrist)
    
    beta = math.atan2(L2 * math.sin(th3_ik), L1 + L2 * math.cos(th3_ik))
    
    # Kąt barku
    th2 = alpha - beta
    
    # Obliczenie pozostałych kątów
    th4_ik = phi_rad - th2 - th3_ik
    th3 = -th3_ik
    th4 = -th4_ik
    
    # Normalizacja kątów do przedziału [-pi, pi]
    th1 = math.atan2(math.sin(th1), math.cos(th1))
    th2 = math.atan2(math.sin(th2), math.cos(th2))
    th3 = math.atan2(math.sin(th3), math.cos(th3))
    th4 = math.atan2(math.sin(th4), math.cos(th4))
    
    return (th1, th2, th3, th4)

JOINT_CONST = pi/2
JOINT_LIMITS = {
    'th1': (-JOINT_CONST, JOINT_CONST),       # Baza
    'th2': (0           , JOINT_CONST * 1.5), # Bark
    'th3': (-JOINT_CONST, JOINT_CONST),       # Łokieć
    'th4': (-JOINT_CONST, JOINT_CONST)        # Nadgarstek
}

def check_constraints(th1, th2, th3, th4):
    """
    Weryfikuje ograniczenia kątowe oraz geometryczne (kolizja z podłożem).
    Zwraca True, jeśli konfiguracja jest dopuszczalna.
    """
    # 1. Weryfikacja zakresów kątowych
    if not (JOINT_LIMITS['th1'][0] <= th1 <= JOINT_LIMITS['th1'][1]): return False
    if not (JOINT_LIMITS['th2'][0] <= th2 <= JOINT_LIMITS['th2'][1]): return False
    if not (JOINT_LIMITS['th3'][0] <= th3 <= JOINT_LIMITS['th3'][1]): return False
    if not (JOINT_LIMITS['th4'][0] <= th4 <= JOINT_LIMITS['th4'][1]): return False

    # 2. Weryfikacja kolizji z podłożem (Płaszczyzna Z=0)
    # Obliczenia oparte na łańcuchu kinematycznym w płaszczyźnie pionowej.
    # Z_shoulder = lambda1_val (stała > 0)
    
    # Wysokość łokcia (Joint 3)
    # Zależna od th2. Przy th2=0 ramię poziomo, przy th2=90 pionowo w górę.
    z_elbow = lambda1_val + l2_val * math.sin(th2)
    
    if z_elbow < 0:
        return False

    # Wysokość nadgarstka (Joint 4)
    # Zależna od th2 i th3. Uwaga: w macierzy A3 jest RotZ(-th3), więc kąt absolutny to th2 - th3.
    z_wrist = z_elbow + l3_val * math.sin(th2 - th3)
    
    if z_wrist < 0:
        return False

    return True

def calculate_joint_distance(q_current, q_target):
    if q_target is None:
        return float('inf')
    
    total_distance = 0
    weights = [1.0, 1.0, 1.0, 1.0]
    
    for i in range(len(q_current)):
        diff = q_target[i] - q_current[i]
        normalized_diff = (diff + math.pi) % (2 * math.pi) - math.pi
        total_distance += abs(normalized_diff) * weights[i]
    
    return total_distance

def calculate_configuration_cost(angles, current_angles):
    """
    Oblicza koszt konfiguracji uwzględniający:
    1. Odległość od limitów złączy (conditioning)
    2. Odległość w przestrzeni konfiguracyjnej
    3. Unikanie singularności (wyprostowany łokieć)
    
    Używane w przemyśle jako: Joint Range Availability (JRA) + Distance Cost
    """
    th1, th2, th3, th4 = angles
    
    # 1. Koszt odległości od limitów (im bliżej środka zakresu, tym lepiej)
    joint_limit_cost = 0
    joints = [th1, th2, th3, th4]
    limits = [JOINT_LIMITS['th1'], JOINT_LIMITS['th2'], JOINT_LIMITS['th3'], JOINT_LIMITS['th4']]
    
    for joint_val, (min_lim, max_lim) in zip(joints, limits):
        # Normalizacja do przedziału [0, 1], gdzie 0.5 = środek zakresu
        normalized = (joint_val - min_lim) / (max_lim - min_lim)
        # Koszt rośnie im dalej od środka (0.5)
        joint_limit_cost += (normalized - 0.5)**2
    
    # 2. Koszt ruchu (odległość w przestrzeni konfiguracyjnej)
    motion_cost = calculate_joint_distance(current_angles, angles)
    
    # 3. Koszt singularności (unikaj wyprostowanego łokcia, th3 ≈ 0)
    # W robotyce: Manipulability Index = det(J*J^T), ale uproszczamy do abs(th3)
    singularity_cost = 1.0 / (abs(th3) + 0.1)  # Duży koszt gdy th3 → 0
    
    # Wagi (dostrojone empirycznie dla robotyki przemysłowej)
    w_limit = 2.0      # Priorytet: unikaj limitów
    w_motion = 1.0     # Średni priorytet: minimalizuj ruch
    w_singular = 3.0   # Wysoki priorytet: unikaj singularności
    
    total_cost = (w_limit * joint_limit_cost + 
                  w_motion * motion_cost + 
                  w_singular * singularity_cost)
    
    return total_cost

def solve_ik_for_cartesian(x_target, y_target, z_target, phi_deg, current_angles):
    """
    Rozwiązanie IK dla współrzędnych kartezjańskich (X, Y, Z) z automatyczną
    optymalizacją orientacji nadgarstka (phi).
    
    Algorytm przemysłowy:
    - Testuje różne orientacje phi w zakresie [-180°, +180°]
    - Minimalizuje koszt konfiguracji (JRA + singularity avoidance)
    - Wybiera najlepsze rozwiązanie spełniające ograniczenia
    """
    R_target = math.sqrt(x_target**2 + y_target**2)
    Z_target = z_target
    
    th1_base = math.atan2(y_target, x_target) if R_target > 0.01 else 0.0
    
    # Jeśli użytkownik podał konkretne phi, użyj go
    if phi_deg is not None:
        phi_range = [phi_deg]
    else:
        # Automatyczna optymalizacja: próbkuj orientacje co 5°
        phi_range = range(-180, 180, 5)
    
    all_solutions = []
    
    # Definicja strategii poszukiwania rozwiązań
    strategies = [
        (True, False, "Baza Normalna, Łokieć GÓRA"),
        (False, False, "Baza Normalna, Łokieć DÓŁ"),
        (True, True, "Baza Odwrócona, Łokieć GÓRA"),
        (False, True, "Baza Odwrócona, Łokieć DÓŁ")
    ]

    for phi in phi_range:
        for elbow_up, reverse_base, name in strategies:
            # Dla odwróconej bazy przeliczamy th1
            if reverse_base:
                th1_input = ((th1_base + math.pi) + math.pi) % (2 * math.pi) - math.pi
            else:
                th1_input = th1_base

            sol = inverse_kinematics(R_target, Z_target, th1_input, phi, elbow_up, reverse_base)
            
            if sol:
                if check_constraints(*sol):
                    mech_cost = calculate_configuration_cost(sol, current_angles)

                    # Dodanie kosztu preferencji orientacji (odchylenie od 180st)
                    orientation_penalty = 0.0
                    if phi_deg is None:
                        orientation_penalty = abs(phi) * 10
                    
                    total_cost = mech_cost + orientation_penalty
                    all_solutions.append((total_cost, sol, f"{name}, φ={phi}°"))

    if not all_solutions:
        return None, "BRAK ROZWIĄZANIA (Ograniczenia lub Zasięg)"
    
    # Sortuj według kosztu i wybierz najlepsze
    all_solutions.sort(key=lambda x: x[0])
    best_cost, best_solution, best_name = all_solutions[0]
    
    return best_solution, best_name

def get_joint_positions(th1_val, th2_val, th3_val, th4_val, alpha5_val=0.0):
    """
    Oblicza pozycje (X, Y, Z) wszystkich węzłów robota dla zadanych kątów.
    Wykorzystuje macierze transformacji do dokładnego odwzorowania kinematyki.
    """
    # Definicje lokalne dla numerycznej wydajności (zamiast sp.Matrix.evalf w pętli)
    # Jeśli zależy Ci na idealnej spójności z forward_kinematics, używamy tych samych macierzy
    alpha1_val = np.pi / 2
    alpha4_val = -np.pi / 2

    # Funkcje pomocnicze dla numpy (szybsze niż sympy)
    def np_RotZ(t): c, s = np.cos(t), np.sin(t); return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    def np_RotX(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
    def np_TransZ(d): return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])
    def np_TransX(a): return np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # Macierze transformacji
    T1 = np_RotZ(th1_val) @ np_TransZ(lambda1_val) @ np_TransX(l1_val) @ np_RotX(alpha1_val)
    T2 = np_RotZ(th2_val) @ np_TransX(l2_val)
    T3 = np_RotZ(-th3_val) @ np_TransX(l3_val)
    T4 = np_RotZ(-th4_val) @ np_TransX(l4_val) @ np_RotX(alpha4_val)
    T5 = np_TransZ(lambda5_val) @ np_TransX(l5_val) @ np_RotX(alpha5_val)

    # Pozycje poszczególnych węzłów (tłumaczenie wektora [0,0,0,1])
    origin = np.array([0, 0, 0, 1])
    
    # P1: Baza (0,0,0)
    p1 = np.array([0, 0, 0])
    
    # P2: Bark (po T1)
    p2 = (T1 @ origin)[:3]
    
    # P3: Łokieć (po T1 * T2)
    p3 = (T1 @ T2 @ origin)[:3]
    
    # P4: Nadgarstek środek (po T1 * T2 * T3)
    p4 = (T1 @ T2 @ T3 @ origin)[:3]
    
    # P5: Końcówka (po T1 * T2 * T3 * T4 * T5)
    p5 = (T1 @ T2 @ T3 @ T4 @ T5 @ origin)[:3]

    return np.array([p1, p2, p3, p4, p5])

# =====================================================================
# GUI APPLICATION
# =====================================================================

POSITION_TOLERANCE = 5.0 # Tolerancja osiągnięcia pozycji [mm]

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry("1400x800")

        self.last_gui_update_time = 0.0
        # Ostatnia pozycja TCP odczytana z robota (używana do weryfikacji)
        self.last_tcp_pos = np.array([0.0, 0.0, 0.0])
        
        self.robot = RobotSerial()
        self.robot.set_gui_callback(self.update_position_display)
        
        # Elementy wizualizacji 3D
        self.plot_artists = []
        self.max_reach = l2_val + l3_val + l4_val + l5_val
        
        self.control_mode = tk.StringVar(value='position')
        
        # Flagi i timery dla sterowania kątami
        self.angle_send_timer = None
        self.angle_send_active = False
        self._collision_warning_shown = False
        self._last_collision_state = False
        
        # === Dane i stan sekwencji ===
        self.is_connected = False
        self.sequence_data = []  
        self.sequence_list_frame = None
        self.canvas_window_seq = None
        
        # Stan odtwarzania sekwencji
        self.current_sequence_index = 0
        self.sequence_playing = False
        self.sequence_timer = None
        self.play_button = None 
        # Docelowa pozycja XYZ dla weryfikacji sprzętowej
        self.target_xyz = None 
        # =============================

        self.setup_ui()
        self.setup_3d_plot()
        self.update_3d_visualization()
        self.root.after(100, self.connect_robot)

    def setup_ui(self):
        # Definicja palety kolorów
        COLOR_BG = "#2f3347"
        COLOR_FRAME_BG = "#262a3e"
        COLOR_BORDER = "#565a6c"
        COLOR_TEXT = "#c4cad0"
        COLOR_ACCENT = "#101122"
        COLOR_BUTTON = "#2f3347"
        COLOR_STOP = "#cb0000" 
        COLOR_PLAY = "#008000" 

        # Konfiguracja stylu TTK
        style = ttk.Style()
        style.theme_use('clam')
        style.configure(".", background=COLOR_BG, foreground=COLOR_TEXT, fieldbackground=COLOR_FRAME_BG,
                        darkcolor=COLOR_FRAME_BG, lightcolor=COLOR_FRAME_BG, bordercolor=COLOR_BORDER)
        style.configure("TLabelframe", background=COLOR_FRAME_BG, bordercolor=COLOR_BORDER, lightcolor=COLOR_BORDER, 
                        darkcolor=COLOR_BORDER, borderwidth=1, relief="solid")
        style.configure("TLabelframe.Label", background=COLOR_FRAME_BG, foreground=COLOR_TEXT) 
        style.configure("TLabel", background=COLOR_FRAME_BG, foreground=COLOR_TEXT)
        style.configure("TEntry", fieldbackground=COLOR_FRAME_BG, foreground=COLOR_TEXT, insertcolor=COLOR_TEXT, bordercolor=COLOR_BORDER)
        style.configure("TButton", background=COLOR_BUTTON, foreground=COLOR_TEXT, bordercolor=COLOR_BORDER)
        style.map("TButton", background=[('active', COLOR_ACCENT)])
        style.configure("Play.TButton", background=COLOR_PLAY, foreground="white", bordercolor=COLOR_BORDER)
        style.map("Play.TButton", background=[('active', '#004d00')])
        style.configure("Stop.TButton", background=COLOR_STOP, foreground="white", bordercolor=COLOR_BORDER)
        style.map("Stop.TButton", background=[('active', '#8c0000')])
        style.configure("TRadiobutton", background=COLOR_FRAME_BG, foreground=COLOR_TEXT, indicatorbackground=COLOR_TEXT)
        style.map("TRadiobutton", background=[('active', COLOR_FRAME_BG)])
        style.configure("TCheckbutton", background=COLOR_FRAME_BG, foreground=COLOR_TEXT, indicatorbackground=COLOR_TEXT)
        style.map("TCheckbutton", background=[('active', COLOR_FRAME_BG)])
        
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # === LEWA KOLUMNA: MECHANIZM PRZEWIJANIA ===
        scroll_canvas = tk.Canvas(main_frame, borderwidth=0, background=COLOR_BG, highlightthickness=0)
        scroll_canvas.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        vscrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=scroll_canvas.yview)
        vscrollbar.grid(row=0, column=0, sticky=tk.E + tk.N + tk.S) 
        scroll_canvas.configure(yscrollcommand=vscrollbar.set)
        
        self.content_frame = tk.Frame(scroll_canvas, bg=COLOR_BG) 
        self.content_frame.grid_columnconfigure(0, weight=1) 
        self.canvas_window = scroll_canvas.create_window((0, 0), window=self.content_frame, anchor="nw", width=1) 

        self.content_frame.bind(
            "<Configure>",
            lambda e: scroll_canvas.configure(scrollregion=scroll_canvas.bbox("all"))
        )
        def resize_content(event):
            width = event.width - vscrollbar.winfo_width()
            scroll_canvas.itemconfig(self.canvas_window, width=width)
            
        scroll_canvas.bind("<Configure>", resize_content)

        # Prawa kolumna - wizualizacja 3D
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        
        # === WIDŻETY LEWEJ KOLUMNY (wewnątrz self.content_frame) ===
        
        # Status połączenia (row 0)
        status_frame = ttk.LabelFrame(self.content_frame, text="Status połączenia", padding="5")
        status_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5, padx=5)
        self.status_label = ttk.Label(status_frame, text="Rozłączony", foreground="red")
        self.status_label.grid(row=0, column=0, sticky=tk.W)
        ttk.Button(status_frame, text="Połącz", command=self.connect_robot).grid(row=0, column=1, padx=5)
        
        # Wybór trybu sterowania (row 1)
        mode_frame = ttk.LabelFrame(self.content_frame, text="Tryb sterowania", padding="10")
        mode_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5, padx=5)
        ttk.Radiobutton(mode_frame, text="Sterowanie pozycją (XYZ)", variable=self.control_mode, value='position',
                        command=self.switch_control_mode).grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Radiobutton(mode_frame, text="Sterowanie kątami (ciągłe)", variable=self.control_mode, value='angles',
                        command=self.switch_control_mode).grid(row=0, column=1, sticky=tk.W, pady=2)
        
        # Aktualna pozycja (row 2)
        position_frame = ttk.LabelFrame(self.content_frame, text="Aktualna pozycja", padding="10")
        position_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=5, padx=5)
        self.pos_labels = {}
        for i, axis in enumerate(['X', 'Y', 'Z', 'E', 'A']):
            ttk.Label(position_frame, text=axis + ":").grid(row=0, column=i, sticky=tk.W, pady=2)
            self.pos_labels[axis] = ttk.Label(position_frame, text="0.00°", font=('Arial', 10))
            self.pos_labels[axis].grid(row=0, column=i, sticky=tk.W, padx=10)
        ttk.Separator(position_frame, orient='horizontal').grid(row=1, column=0, columnspan=5, sticky='ew', pady=5)
        self.tcp_labels = {}
        tcp_axes = ['TCP_X', 'TCP_Y', 'TCP_Z']
        for i, axis in enumerate(tcp_axes):
            col_idx = i * 2 
            ttk.Label(position_frame, text=axis.replace("TCP_", "") + " [mm]:").grid(row=2, column=col_idx, sticky=tk.W, pady=2)
            self.tcp_labels[axis] = ttk.Label(position_frame, text="0.00", font=('Arial', 10, 'bold'), foreground="#377df0")
            self.tcp_labels[axis].grid(row=2, column=col_idx+1, sticky=tk.W, padx=5)

        # Kąty docelowe (wynik IK) (row 3)
        self.angles_frame = ttk.LabelFrame(self.content_frame, text="Kąty docelowe (wynik IK)", padding="10")
        self.angles_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5, padx=5)
        self.angle_labels = {}
        for i, axis in enumerate(['θ1 (X)', 'θ2 (Y)', 'θ3 (Z)', 'θ4 (E)']):
            ttk.Label(self.angles_frame, text=axis + ":").grid(row=0, column=i*2, sticky=tk.E, padx=5, pady=2)
            self.angle_labels[axis] = ttk.Label(self.angles_frame, text="0.00°", font=('Arial', 10))
            self.angle_labels[axis].grid(row=0, column=i*2+1, sticky=tk.W, padx=5, pady=2)
        
        # Sterowanie kątami (suwaki) (row 4)
        self.angle_control_frame = ttk.LabelFrame(self.content_frame, text="Sterowanie kątami [°]", padding="10")
        self.angle_control_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=5, padx=5)
        self.angle_sliders = {}; self.angle_value_labels = {}
        joint_names = [('th1', 'θ1 (X - Baza)'), ('th2', 'θ2 (Y - Ramię L2)'), 
                       ('th3', 'θ3 (Z - Ramię L3)'), ('th4', 'θ4 (E - Nadgarstek)')]
        for i, (key, label) in enumerate(joint_names):
            ttk.Label(self.angle_control_frame, text=label).grid(row=i, column=0, sticky=tk.W, pady=5)
            limit_min_rad, limit_max_rad = JOINT_LIMITS[key]
            slider = tk.Scale(self.angle_control_frame, from_=math.degrees(limit_min_rad), to=math.degrees(limit_max_rad),    
                              orient=tk.HORIZONTAL, resolution=0.1, length=250,
                              bg=COLOR_FRAME_BG, fg=COLOR_TEXT, troughcolor=COLOR_BG, highlightthickness=0, 
                              command=lambda val, k=key: self.on_angle_slider_change(k, val))
            slider.set(0.0)
            slider.grid(row=i, column=1, padx=5, pady=5)
            self.angle_sliders[key] = slider
            value_label = ttk.Label(self.angle_control_frame, text="0.0°", font=('Arial', 10, 'bold'))
            value_label.grid(row=i, column=2, padx=5)
            self.angle_value_labels[key] = value_label
        
        self.live_control_var = tk.BooleanVar(value=False)
        self.live_control_check = ttk.Checkbutton(self.angle_control_frame, 
                                                  text="WŁĄCZ LIVE (Transmisja ciągła)",
                                                  variable=self.live_control_var,
                                                  command=self.toggle_live_control)
        self.live_control_check.grid(row=len(joint_names), column=0, columnspan=3, pady=10, sticky=tk.W)

        self.add_angle_point_button = ttk.Button(self.angle_control_frame, text="DODAJ AKTUALNE KĄTY (do sekwencji)", 
                                                 command=self.add_current_angles_to_sequence, style='TButton')
        self.add_angle_point_button.grid(row=len(joint_names)+1, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))
        
        # Pozycja docelowa (row 5)
        self.target_frame = ttk.LabelFrame(self.content_frame, text="Pozycja docelowa [mm]", padding="10")
        self.target_frame.grid(row=5, column=0, sticky=(tk.W, tk.E), pady=5, padx=5)
        ttk.Label(self.target_frame, text="X:").grid(row=0, column=0, sticky=tk.W, pady=5); self.x_entry = ttk.Entry(self.target_frame, width=15); self.x_entry.grid(row=0, column=1, padx=5); self.x_entry.insert(0, "-87.5")
        ttk.Label(self.target_frame, text="Y:").grid(row=1, column=0, sticky=tk.W, pady=5); self.y_entry = ttk.Entry(self.target_frame, width=15); self.y_entry.grid(row=1, column=1, padx=5); self.y_entry.insert(0, "0")
        ttk.Label(self.target_frame, text="Z:").grid(row=2, column=0, sticky=tk.W, pady=5); self.z_entry = ttk.Entry(self.target_frame, width=15); self.z_entry.grid(row=2, column=1, padx=5); self.z_entry.insert(0, "372.5")
        ttk.Label(self.target_frame, text="Orientacja φ [°]:").grid(row=3, column=0, sticky=tk.W, pady=5); self.phi_entry = ttk.Entry(self.target_frame, width=15); self.phi_entry.grid(row=3, column=1, padx=5); self.phi_entry.insert(0, "0")
        self.auto_phi_var = tk.BooleanVar(value=True)
        self.auto_phi_check = ttk.Checkbutton(self.target_frame, text="Orientacja automatyczna",
                                              variable=self.auto_phi_var, command=self.toggle_phi_entry)
        self.auto_phi_check.grid(row=4, column=0, columnspan=2, pady=5, sticky=tk.W)
        # Kontener na przyciski w jednym wierszu (row 5)
        buttons_row = ttk.Frame(self.target_frame)
        buttons_row.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Konfiguracja kolumn: weight=1 (rozciąganie) oraz uniform="row_group" (wymuszenie równej szerokości)
        buttons_row.columnconfigure(0, weight=1, uniform="row_group") 
        buttons_row.columnconfigure(1, weight=1, uniform="row_group")

        ttk.Button(buttons_row, text="WYŚLIJ POZYCJĘ", command=self.send_position, style='TButton').grid(row=0, column=0, padx=(0, 2), sticky=(tk.W, tk.E))
        ttk.Button(buttons_row, text="DODAJ PUNKT", command=self.add_point_to_sequence, style='TButton').grid(row=0, column=1, padx=(2, 0), sticky=(tk.W, tk.E))

        ttk.Button(buttons_row, text="WYŚLIJ POZYCJĘ", command=self.send_position, style='TButton').grid(row=0, column=0, padx=(0, 2), sticky=(tk.W, tk.E))
        ttk.Button(buttons_row, text="DODAJ PUNKT (do sekwencji)", command=self.add_point_to_sequence, style='TButton').grid(row=0, column=1, padx=(2, 0), sticky=(tk.W, tk.E))
        self.toggle_phi_entry()
        
        # Log (row 8)
        log_frame = ttk.LabelFrame(self.content_frame, text="Log komunikacji", padding="10")
        log_frame.grid(row=7, column=0, sticky=(tk.W, tk.E), pady=5, padx=5)
        self.log_text = tk.Text(log_frame, height=10, width=50, bg=COLOR_FRAME_BG, fg=COLOR_TEXT, 
                                insertbackground=COLOR_TEXT, selectbackground=COLOR_ACCENT,
                                highlightbackground=COLOR_BORDER, highlightthickness=1)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.log_text['yscrollcommand'] = scrollbar.set
        log_frame.grid_columnconfigure(0, weight=1); log_frame.grid_rowconfigure(0, weight=1)

        # Ramka zapisu sekwencji (row 7)
        self._create_sequence_frame(self.content_frame, 8) 

        # === WIZUALIZACJA 3D ===
        viz_frame = ttk.LabelFrame(right_frame, text="Wizualizacja 3D", padding="10")
        viz_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.fig = Figure(figsize=(7, 6), dpi=100, facecolor=COLOR_BG) 
        self.ax = self.fig.add_subplot(111, projection='3d')
        LABEL_COLOR = COLOR_TEXT
        self.ax.set_facecolor(COLOR_BG); self.ax.tick_params(axis='x', colors=LABEL_COLOR); self.ax.tick_params(axis='y', colors=LABEL_COLOR); self.ax.tick_params(axis='z', colors=LABEL_COLOR)
        self.ax.xaxis.label.set_color(LABEL_COLOR); self.ax.yaxis.label.set_color(LABEL_COLOR); self.ax.zaxis.label.set_color(LABEL_COLOR)
        self.ax.xaxis.set_pane_color((0.14, 0.16, 0.24, 1.0)); self.ax.yaxis.set_pane_color((0.14, 0.16, 0.24, 1.0)); self.ax.zaxis.set_pane_color((0.14, 0.16, 0.24, 1.0))
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        reset_button = ttk.Button(viz_frame, text="Resetuj widok", command=self.reset_3d_view); reset_button.pack(side=tk.TOP, anchor=tk.NW, pady=5, padx=5)
        self.canvas.get_tk_widget().config(bg=COLOR_BG); self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Konfiguracja grid weights
        self.root.columnconfigure(0, weight=1); self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=3); main_frame.columnconfigure(1, weight=7) 
        main_frame.rowconfigure(0, weight=1); right_frame.rowconfigure(0, weight=1)
        
        self.switch_control_mode()

    def setup_3d_plot(self):
        """Konfiguruje statyczne elementy wykresu 3D (osie, limity, siatka, podstawa)."""
        LABEL_COLOR = "#c4cad0"
        self.ax.set_xlabel('X [mm]', color=LABEL_COLOR)
        self.ax.set_ylabel('Y [mm]', color=LABEL_COLOR)
        self.ax.set_zlabel('Z [mm]', color=LABEL_COLOR)
        self.ax.set_title('Pozycja robota', color=LABEL_COLOR)
        
        self.ax.set_xlim([-self.max_reach, self.max_reach])
        self.ax.set_ylim([-self.max_reach, self.max_reach])
        self.ax.set_zlim([0, self.max_reach])
        
        self.ax.view_init(elev=20, azim=45)
        
        # Dodanie siatki płaszczyzny XY
        grid_size = 100
        grid_range = np.arange(-300, 301, grid_size)
        X_grid, Y_grid = np.meshgrid(grid_range, grid_range)
        Z_grid = np.zeros_like(X_grid)
        self.ax.plot_wireframe(X_grid, Y_grid, Z_grid, alpha=0.1, color='gray', linewidth=0.5)
        
        # Osie układu współrzędnych
        axis_length = 100
        self.ax.quiver(0, 0, 0, axis_length, 0, 0, color='red', arrow_length_ratio=0.1, linewidth=2, label='X')
        self.ax.quiver(0, 0, 0, 0, axis_length, 0, color='green', arrow_length_ratio=0.1, linewidth=2, label='Y')
        # Poprawiona linia: usunięto nadmiarowy argument pozycyjny '0'
        self.ax.quiver(0, 0, 0, 0, 0, axis_length, color='blue', arrow_length_ratio=0.1, linewidth=2, label='Z')
        
        self.ax.legend(loc='upper right')

    def reset_3d_view(self):
        """Resetuje widok 3D (limity osi i kamerę) do ustawień domyślnych."""
        self.ax.set_xlim([-self.max_reach, self.max_reach])
        self.ax.set_ylim([-self.max_reach, self.max_reach])
        self.ax.set_zlim([0, self.max_reach])
        self.ax.view_init(elev=20, azim=45)
        self.canvas.draw()

    def update_3d_visualization(self):
        """Aktualizacja wizualizacji 3D robota na podstawie aktualnych kątów."""
        LABEL_COLOR = "#c4cad0"
        try:
            for artist in self.plot_artists:
                if isinstance(artist, list):
                    for item in artist:
                        item.remove()
                else:
                    artist.remove()
            
            self.plot_artists = []
            
            current_pos = self.robot.get_current_angles()
            th1 = math.radians(current_pos[0]); th2 = math.radians(current_pos[1])
            th3 = math.radians(current_pos[2]); th4 = math.radians(current_pos[3])
            
            positions = get_joint_positions(th1, th2, th3, th4)
            self.last_tcp_pos = positions[-1]
            
            x_coords = [pos[0] for pos in positions]; y_coords = [pos[1] for pos in positions]; z_coords = [pos[2] for pos in positions]
            
            robot_lines = self.ax.plot(x_coords, y_coords, z_coords, 'o-', linewidth=2, 
                                        markersize=6, color='#377df0', label='Robot', markerfacecolor='lightblue')
            self.plot_artists.append(robot_lines)
            robot_tcp = self.ax.scatter([x_coords[-1]], [y_coords[-1]], [z_coords[-1]], 
                                         c="#cb0000", s=50, marker='o', label='Końcówka', edgecolors='darkred', linewidths=2)
            self.plot_artists.append(robot_tcp)
            labels = ['Base', 'J1', 'J2', 'J3', 'J4', 'TCP']
            for i, pos in enumerate(positions):
                text_label = self.ax.text(pos[0], pos[1], pos[2], f'  {labels[i]}', fontsize=8, color=LABEL_COLOR)
                self.plot_artists.append(text_label)
            
            self.canvas.draw()
            
        except Exception as e:
            if not hasattr(self, '_viz_error_logged'):
                self.log(f"Błąd wizualizacji 3D: {e}")
                self._viz_error_logged = True
        
        self.root.after(50, self.update_3d_visualization)
        
    def log(self, message):
        """Dodaj wiadomość do logu"""
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)

    def update_position_display(self, angles=None, log_message=None):
        """Callback z wątku serial - aktualizacja GUI"""
        if log_message:
            if log_message == "#CONNECTION_LOST#":
                self.status_label.config(text="Rozłączony (Błąd I/O)", foreground="red")
                self.log("BŁĄD KRYTYCZNY: Utracono połączenie z urządzeniem")
                return

            self.root.after(0, lambda: self.log(log_message))
        
        now = time.time() * 1000.0

        if now - self.last_gui_update_time >= 200.0:
            self.last_gui_update_time = now

            if angles:
                x, y, z, e, a = angles
                self.root.after(0, lambda: self.pos_labels['X'].config(text=f"{x:.2f}°"))
                self.root.after(0, lambda: self.pos_labels['Y'].config(text=f"{y:.2f}°"))
                self.root.after(0, lambda: self.pos_labels['Z'].config(text=f"{z:.2f}°"))
                self.root.after(0, lambda: self.pos_labels['E'].config(text=f"{e:.2f}°"))
                self.root.after(0, lambda: self.pos_labels['A'].config(text=f"{a:.2f}°"))
                
                tcp_pos = self.last_tcp_pos
                
                self.root.after(0, lambda: self.tcp_labels['TCP_X'].config(text=f"{tcp_pos[0]:.2f}"))
                self.root.after(0, lambda: self.tcp_labels['TCP_Y'].config(text=f"{tcp_pos[1]:.2f}"))
                self.root.after(0, lambda: self.tcp_labels['TCP_Z'].config(text=f"{tcp_pos[2]:.2f}"))
    
    def toggle_phi_entry(self):
        """Włącz/wyłącz pole orientacji w zależności od checkboxa automatycznego doboru Fi."""
        if self.auto_phi_var.get():
            self.phi_entry.config(state='disabled')
        else:
            self.phi_entry.config(state='normal')

    def connect_robot(self):
        """Inicjuje połączenie z robotem."""
        success, message = self.robot.connect()
        if success:
            self.is_connected = True
            self.status_label.config(text="Połączono", foreground="green")
            self.log(message)
        else:
            self.is_connected = False
            self.status_label.config(text="Rozłączony", foreground="red")
            messagebox.showerror("Błąd połączenia", message)
            self.log(message)
    
    def send_position(self):
        """Oblicza IK, weryfikuje kolizje geometryczne i wysyła kąty do robota."""
        try:
            x_target = float(self.x_entry.get()); y_target = float(self.y_entry.get()); z_target = float(self.z_entry.get())
            phi_deg = None if self.auto_phi_var.get() else float(self.phi_entry.get())
            
            self.log(f"Obliczanie IK dla pozycji: X={x_target}, Y={y_target}, Z={z_target}")
            current_pos = self.robot.get_current_angles()
            current_angles = tuple(math.radians(angle) for angle in current_pos[:4])
            solution, config_name = solve_ik_for_cartesian(x_target, y_target, z_target, phi_deg, current_angles)
            
            if solution is None:
                messagebox.showerror("Błąd IK", "Nie znaleziono osiągalnego rozwiązania IK. Pozycja poza zasięgiem robota.")
                self.log("BŁĄD: Pozycja poza zasięgiem robota")
                return
            
            th1, th2, th3, th4 = solution
            positions = get_joint_positions(th1, th2, th3, th4)
            z_wrist = positions[3][2]; z_tip = positions[4][2]
            
            if z_wrist < -0.01 or z_tip < -0.01:
                self.log(f"BLOKADA: Wykryto kolizję! Wrist Z={z_wrist:.2f}, Tip Z={z_tip:.2f}")
                messagebox.showerror("Naruszenie strefy bezpiecznej", 
                    f"Nie można wykonać ruchu! Rozwiązanie IK prowadzi do kolizji z podłożem:\n- Nadgarstek Z: {z_wrist:.2f} mm\n- Końcówka Z: {z_tip:.2f} mm\nOperacja przerwana.")
                return
            
            self.log(f"Wybrano konfigurację: {config_name}")
            
            self.angle_labels['θ1 (X)'].config(text=f"{math.degrees(th1):.2f}°"); self.angle_labels['θ2 (Y)'].config(text=f"{math.degrees(th2):.2f}°")
            self.angle_labels['θ3 (Z)'].config(text=f"{math.degrees(th3):.2f}°"); self.angle_labels['θ4 (E)'].config(text=f"{math.degrees(th4):.2f}°")
            
            success, message = self.robot.send_target_angles(th1, th2, th3, th4)
            if success:
                self.log(message); self.log("Pozycja wysłana pomyślnie")
            else:
                self.log(message); messagebox.showerror("Błąd komunikacji", message)
                
        except ValueError as e:
            messagebox.showerror("Błąd", "Wprowadź poprawne wartości liczbowe"); self.log(f"BŁĄD: Niepoprawne dane wejściowe - {e}")
        except Exception as e:
            messagebox.showerror("Błąd", f"Wystąpił błąd: {e}"); self.log(f"BŁĄD: {e}")
    
    def on_angle_slider_change(self, joint_key, value):
        """Callback dla zmiany wartości suwaka - aktualizuje etykietę i sprawdza kolizję."""
        angle = float(value)
        self.angle_value_labels[joint_key].config(text=f"{angle:.1f}°")
        
        th1 = math.radians(self.angle_sliders['th1'].get()); th2 = math.radians(self.angle_sliders['th2'].get())
        th3 = math.radians(self.angle_sliders['th3'].get()); th4 = math.radians(self.angle_sliders['th4'].get())
        positions = get_joint_positions(th1, th2, th3, th4)
        
        collision_detected = False; min_z = float('inf')
        for i, pos in enumerate(positions):
            if pos[2] < 0:
                collision_detected = True
                min_z = min(min_z, pos[2])
        
        if collision_detected:
            self.angle_value_labels[joint_key].config(foreground='red')
            if not hasattr(self, '_last_collision_state') or not self._last_collision_state:
                self.log(f"OSTRZEŻENIE: Staw znajduje się {abs(min_z):.1f}mm POD stołem!")
            self._last_collision_state = True
        else:
            self.angle_value_labels[joint_key].config(foreground='green')
            if hasattr(self, '_last_collision_state') and self._last_collision_state:
                self.log("INFO: Kolizja usunięta - pozycja bezpieczna")
            self._last_collision_state = False
    
    def switch_control_mode(self):
        """Przełącza między trybem sterowania pozycją a kątami."""
        mode = self.control_mode.get()
        
        # Zawsze zatrzymujemy transmisję przy zmianie trybu dla bezpieczeństwa
        self.stop_continuous_send()
        self.live_control_var.set(False) 
        
        if mode == 'position':
            self.target_frame.grid(); self.angle_control_frame.grid_remove()
            self.angles_frame.grid()
            self.log("INFO: Przełączono na tryb sterowania pozycją (XYZ)")
            
        elif mode == 'angles':
            current_pos = self.robot.get_current_angles()
            self.angle_sliders['th1'].set(current_pos[0]); self.angle_sliders['th2'].set(current_pos[1])
            self.angle_sliders['th3'].set(current_pos[2]); self.angle_sliders['th4'].set(current_pos[3])
            
            self.angle_value_labels['th1'].config(text=f"{current_pos[0]:.1f}°")
            self.angle_value_labels['th2'].config(text=f"{current_pos[1]:.1f}°")
            self.angle_value_labels['th3'].config(text=f"{current_pos[2]:.1f}°")
            self.angle_value_labels['th4'].config(text=f"{current_pos[3]:.1f}°")
            
            self.target_frame.grid_remove(); self.angle_control_frame.grid()
            self.angles_frame.grid_remove()
            
            self.log("INFO: Przełączono na tryb sterowania kątami. Zaznacz 'LIVE' aby sterować.")
    
    def start_continuous_send(self):
        """Rozpoczyna ciągłe wysyłanie kątów do robota."""
        self.angle_send_active = True
        self.send_angles_continuously()
    
    def stop_continuous_send(self):
        """Zatrzymuje ciągłe wysyłanie kątów."""
        self.angle_send_active = False
        if self.angle_send_timer:
            self.root.after_cancel(self.angle_send_timer)
            self.angle_send_timer = None
    
    def send_angles_continuously(self):
        """Wysyła aktualne wartości kątów z suwaków do robota w sposób ciągły, blokując kolizje."""
        if not self.angle_send_active:
            return
        
        try:
            th1_deg = self.angle_sliders['th1'].get(); th2_deg = self.angle_sliders['th2'].get()
            th3_deg = self.angle_sliders['th3'].get(); th4_deg = self.angle_sliders['th4'].get()
            
            th1 = math.radians(th1_deg); th2 = math.radians(th2_deg); th3 = math.radians(th3_deg); th4 = math.radians(th4_deg)
            
            positions = get_joint_positions(th1, th2, th3, th4)
            min_z = np.min(positions[:, 2])
            
            if min_z < 0:
                if not hasattr(self, '_collision_warning_shown') or not self._collision_warning_shown:
                    self.log(f"KRYTYCZNE: Wykryto kolizję (Z = {min_z:.2f} mm). Ruch zablokowany.")
                    self._collision_warning_shown = True
                self.angle_send_timer = self.root.after(50, self.send_angles_continuously)
                return

            if hasattr(self, '_collision_warning_shown') and self._collision_warning_shown:
                self.log("INFO: Strefa bezpieczna. Wznowiono transmisję.")
                self._collision_warning_shown = False
            
            current_time = time.time()
            if not hasattr(self, '_last_send_time'):
                self._last_send_time = 0

            if current_time - self._last_send_time >= 0.2:
                success, message = self.robot.send_target_angles(th1, th2, th3, th4)
                if success:
                    self._last_send_time = current_time
            
        except Exception as e:
            self.log(f"Błąd pętli sterowania: {e}")
        
        self.angle_send_timer = self.root.after(50, self.send_angles_continuously)
    
    def on_closing(self):
        """Metoda wywoływana przy zamykaniu aplikacji - zapewnia bezpieczne rozłączenie."""
        self.is_connected = False
        self.stop_continuous_send()
        self.stop_sequence("Aplikacja zamknięta.")
        self.robot.close()
        self.root.destroy()
        
    # ====================================================================================
    #           METODY OBSŁUGI SEKWENCJI RUCHÓW
    # ====================================================================================

    def toggle_live_control(self):
        """Obsługuje manualne włączanie/wyłączanie ciągłej transmisji."""
        if self.live_control_var.get():
            if self.sequence_playing:
                self.log("OSTRZEŻENIE: Nie można włączyć trybu LIVE podczas odtwarzania sekwencji!")
                self.live_control_var.set(False)
                return
            self.start_continuous_send()
            self.log("INFO: Transmisja LIVE włączona.")
        else:
            self.stop_continuous_send()
            self.log("INFO: Transmisja LIVE wyłączona.")

    def _create_sequence_frame(self, master, row):
        """Tworzy ramkę do zapisu, wczytywania i odtwarzania sekwencji."""
        COLOR_FRAME_BG = "#262a3e"
        
        sequence_outer_frame = ttk.LabelFrame(master, text="Rejestrator/Odtwarzacz sekwencji ruchów", padding="10")
        sequence_outer_frame.grid(row=row, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5, padx=5)
        sequence_outer_frame.grid_columnconfigure(0, weight=1)
        
        # Ramka na przyciski LOAD/SAVE/PLAY (Wszystkie w control_frame)
        control_frame = tk.Frame(sequence_outer_frame, bg=COLOR_FRAME_BG)
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # --- KONFIGURACJA ŚRODKOWANIA ---
        # Kolumny 0 i 4 to "sprężyny" (weight=1), które dopychają 3 przyciski do środka
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(4, weight=1)
        
        # Przycisk 1: WCZYTAJ (kolumna 1)
        ttk.Button(control_frame, text="WCZYTAJ", command=self.load_sequence_from_json, 
                   style='TButton').grid(row=0, column=1, padx=5)
        
        # Przycisk 2: ZAPISZ (kolumna 2) - Teraz wewnątrz control_frame!
        ttk.Button(control_frame, text="ZAPISZ", command=self.save_sequence_to_json, 
                   style='TButton').grid(row=0, column=2, padx=5)
        
        # Przycisk 3: START (kolumna 3)
        self.play_button = ttk.Button(control_frame, text="START SEKWENCJI", command=self.play_sequence, 
                                      style='Play.TButton')
        self.play_button.grid(row=0, column=3, padx=5)
        
        # Separator pod przyciskami (row 1 w ramce zewnętrznej)
        ttk.Separator(sequence_outer_frame, orient='horizontal').grid(row=1, column=0, columnspan=2, sticky='ew', pady=5)
        
        # Canvas i Scrollbar dla listy punktów
        canvas = tk.Canvas(sequence_outer_frame, borderwidth=0, background=COLOR_FRAME_BG, highlightthickness=0)
        vscrollbar = ttk.Scrollbar(sequence_outer_frame, orient="vertical", command=canvas.yview)
        
        self.sequence_list_frame = ttk.Frame(canvas, padding="5", style='TLabelframe')
        self.sequence_list_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        self.sequence_list_frame.grid_columnconfigure(0, weight=1)

        self.canvas_window_seq = canvas.create_window((0, 0), window=self.sequence_list_frame, anchor="nw", width=1)
        canvas.configure(yscrollcommand=vscrollbar.set)

        canvas.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S)) 
        vscrollbar.grid(row=2, column=1, sticky=(tk.N, tk.S))
        sequence_outer_frame.grid_rowconfigure(2, weight=1)
        
        def resize_content_seq(event):
             width = event.width - vscrollbar.winfo_width()
             canvas.itemconfig(self.canvas_window_seq, width=width)
        canvas.bind("<Configure>", resize_content_seq)

        self._update_sequence_display()

    def load_sequence_from_json(self):
        """Wczytuje sekwencję ruchów z pliku JSON."""
        file_path = askopenfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
            title="Wczytaj sekwencję ruchów"
        )
        
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
                if not isinstance(data, list) or not all(isinstance(item, dict) and 'X' in item for item in data):
                    raise ValueError("Plik JSON ma nieprawidłowy format sekwencji.")

                self.sequence_data = data
                self._update_sequence_display()
                self.log(f"INFO: Wczytano {len(self.sequence_data)} punktów z pliku: {file_path}")
                self.stop_sequence()
                
            except Exception as e:
                self.log(f"BŁĄD WCZYTYWANIA: Wystąpił błąd: {e}")
                messagebox.showerror("Błąd wczytywania", f"Nie udało się wczytać pliku sekwencji:\n{e}")

    def play_sequence(self):
        """Przełącza stan odtwarzania sekwencji (START/STOP)."""
        if not self.is_connected:
            messagebox.showwarning("Brak połączenia", "Robot musi być połączony, aby rozpocząć sekwencję.")
            return

        # 1. Obsługa konfliktu z trybem manualnym (LIVE)
        if self.live_control_var.get():
            self.live_control_var.set(False)
            self.stop_continuous_send()
            self.log("INFO: Tryb LIVE został automatycznie wyłączony przez sekwenser.")

        if not self.sequence_data:
            messagebox.showwarning("Brak sekwencji", "Wczytaj lub zarejestruj punkty sekwencji przed rozpoczęciem.")
            return

        # 2. Logika przełączania START/STOP
        if self.sequence_playing:
            self.stop_sequence()
        else:
            self.sequence_playing = True
            self.current_sequence_index = 0
            self.log("INFO: Rozpoczęto odtwarzanie sekwencji.")
            self.play_button.config(text="STOP SEKWENCJI", style='Stop.TButton')
            self._execute_sequence_step()

    def stop_sequence(self, message="Przerwano odtwarzanie sekwencji."):
        """Zatrzymuje odtwarzanie sekwencji i resetuje stan."""
        # Reset flagi odtwarzania
        self.sequence_playing = False
        self.current_sequence_index = 0
        self.target_xyz = None
        
        # Anulowanie timera sekwencji, jeśli istnieje
        if self.sequence_timer:
            self.root.after_cancel(self.sequence_timer)
            self.sequence_timer = None
        
        # Przywrócenie wyglądu przycisku Play
        if self.play_button:
             self.play_button.config(text="START SEKWENCJI", style='Play.TButton')
             
        self.log(f"INFO: {message}")
        
        # Odświeżenie listy (usunięcie podświetlenia)
        self._update_sequence_display()
                
    def _execute_sequence_step(self):
        """Wysyła cel do robota i uruchamia pętlę weryfikacji pozycji."""
        if not self.sequence_playing:
            return

        if self.current_sequence_index >= len(self.sequence_data):
            self.stop_sequence("Sekwencja zakończona pomyślnie.")
            return

        # 1. Pobranie punktu
        point = self.sequence_data[self.current_sequence_index]
        x, y, z = point['X'], point['Y'], point['Z']
        
        self.log(f"KROK {self.current_sequence_index + 1}/{len(self.sequence_data)}: Cel XYZ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Ustawienie docelowej pozycji dla pętli weryfikacyjnej
        self.target_xyz = np.array([x, y, z])
        self._update_sequence_display(highlight_index=self.current_sequence_index)
        
        try:
            # 2. Decyzja: Użycie zapisanych kątów (Teach Mode) LUB obliczenie IK (Cartesian Mode)
            if 'Joints' in point and point['Joints'] is not None:
                # Ścieżka szybka: Mamy gotowe kąty z nauczania
                th1_deg, th2_deg, th3_deg, th4_deg = point['Joints']
                th1, th2, th3, th4 = map(math.radians, [th1_deg, th2_deg, th3_deg, th4_deg])

            else:
                # Ścieżka standardowa: Obliczenie IK
                phi_deg = point['Fi'] if not point['Auto_Fi'] else None
                current_pos = self.robot.get_current_angles()
                current_angles = tuple(math.radians(angle) for angle in current_pos[:4])
                
                solution, _ = solve_ik_for_cartesian(x, y, z, phi_deg, current_angles)
                
                if solution is None:
                    raise Exception("Brak rozwiązania IK dla tego punktu.")
                
                th1, th2, th3, th4 = solution
            
            # 3. Wysyłka do robota
            success, message = self.robot.send_target_angles(th1, th2, th3, th4)
            
            if not success:
                 raise Exception(f"Wysyłka komendy ruchu nieudana: {message}")
                 
            self.log(f"KOMENDA: Kąty wysłane. Oczekiwanie na osiągnięcie pozycji...")

            # 4. Uruchomienie pętli weryfikacji pozycji
            self._wait_for_position()

        except Exception as e:
            self.log(f"BŁĄD KRYTYCZNY: Wykonanie kroku {self.current_sequence_index + 1} nie powiodło się: {e}")
            self.stop_sequence("Błąd krytyczny podczas wykonywania kroku. Sekwencja przerwana.")

    def _wait_for_position(self):
        """Pętla sprawdzająca osiągnięcie docelowej pozycji (target_xyz) na podstawie odczytu TCP z robota."""
        if not self.sequence_playing or self.target_xyz is None:
            return

        # Obliczenie odległości euklidesowej
        current_pos = self.last_tcp_pos 
        distance = np.linalg.norm(current_pos - self.target_xyz)
        
        print(f"\rWeryfikacja: Błąd={distance:.2f}mm", end="") 

        if distance <= POSITION_TOLERANCE:
            self.log(f"INFO: Krok {self.current_sequence_index + 1} osiągnięty. Błąd końcowy: {distance:.2f}mm.")
            
            # Przejście do następnego kroku
            self.current_sequence_index += 1
            self.target_xyz = None 
            
            # Zaplanuj natychmiastowe wykonanie następnego kroku
            self.sequence_timer = self.root.after(100, self._execute_sequence_step) 
        else:
            # Ponów sprawdzanie po 100 ms
            self.sequence_timer = self.root.after(100, self._wait_for_position)

    def add_point_to_sequence(self):
        """Dodaje zweryfikowaną IK pozycję z trybu XYZ do sekwencji."""
        try:
            x_target = float(self.x_entry.get()); y_target = float(self.y_entry.get()); z_target = float(self.z_entry.get())
            # Pobranie stanu checkboxa do zmiennej lokalnej
            is_auto_phi = self.auto_phi_var.get()
            phi_deg = None if is_auto_phi else float(self.phi_entry.get())
            
            self.log(f"INFO: Walidacja IK dla punktu: XYZ({x_target:.2f}, {y_target:.2f}, {z_target:.2f})")
            current_pos = self.robot.get_current_angles()
            current_angles = tuple(math.radians(angle) for angle in current_pos[:4])
            solution, _ = solve_ik_for_cartesian(x_target, y_target, z_target, phi_deg, current_angles)
            
            if solution is None:
                messagebox.showerror("Błąd Walidacji", "Pozycja XYZ jest poza zasięgiem (Brak rozwiązania IK).")
                self.log("BŁĄD: Dodawanie punktu odrzucone - Pozycja poza zasięgiem.")
                return

            new_point = {
                "X": round(x_target, 2), "Y": round(y_target, 2), "Z": round(z_target, 2),
                "Fi": round(phi_deg, 2) if phi_deg is not None else None,
                "Auto_Fi": is_auto_phi  # POPRAWKA: Użycie poprawnej zmiennej logicznej
            }
            
            self.sequence_data.append(new_point)
            self._update_sequence_display()
            self.log(f"INFO: Dodano punkt nr {len(self.sequence_data)} (zweryfikowany IK).")
            
        except ValueError:
            messagebox.showerror("Błąd", "Wprowadź poprawne wartości liczbowe dla X, Y, Z i Fi.")
        except Exception as e:
            self.log(f"BŁĄD: Wystąpił błąd podczas dodawania punktu: {e}")

    def add_current_angles_to_sequence(self):
        """Dodaje aktualną pozycję obliczoną z kątów (FK) do sekwencji, zapisując również kąty źródłowe."""
        try:
            # Pobranie wartości z suwaków
            th1_deg = self.angle_sliders['th1'].get(); th2_deg = self.angle_sliders['th2'].get()
            th3_deg = self.angle_sliders['th3'].get(); th4_deg = self.angle_sliders['th4'].get()
            
            th1 = math.radians(th1_deg); th2 = math.radians(th2_deg)
            th3 = math.radians(th3_deg); th4 = math.radians(th4_deg)

            positions = get_joint_positions(th1, th2, th3, th4)
            tcp_pos = positions[-1]
            
            # Normalizacja orientacji Fi
            raw_sum = th2 + th3 + th4
            fi_rad = math.atan2(math.sin(raw_sum), math.cos(raw_sum))
            fi_deg = math.degrees(fi_rad)

            new_point = {
                "X": round(tcp_pos[0], 2), "Y": round(tcp_pos[1], 2), "Z": round(tcp_pos[2], 2),
                "Fi": round(fi_deg, 2), "Auto_Fi": False,
                "Joints": [th1_deg, th2_deg, th3_deg, th4_deg]
            }
            
            self.sequence_data.append(new_point)
            self._update_sequence_display()
            self.log(f"INFO: Dodano punkt (Teach Mode): XYZ({new_point['X']:.2f}, ...), Joints zapisane.")

        except Exception as e:
            self.log(f"BŁĄD: Nie udało się dodać punktu z trybu kątów: {e}")

    def remove_point_from_sequence(self, index):
        """Usuwa punkt o danym indeksie z sekwencji."""
        if 0 <= index < len(self.sequence_data):
            del self.sequence_data[index]
            self._update_sequence_display()
            self.log(f"INFO: Usunięto punkt nr {index + 1} z sekwencji.")

    def _update_sequence_display(self, highlight_index=None):
        """Odświeża wizualizację listy punktów w przewijalnej ramce, z opcją podświetlenia."""
        for widget in self.sequence_list_frame.winfo_children():
            widget.destroy()

        COLOR_TEXT = "#c4cad0"; COLOR_HIGHLIGHT = "#FFD700" 
        
        if not self.sequence_data:
            ttk.Label(self.sequence_list_frame, text="Brak zapisanych punktów", foreground='gray').grid(row=0, column=0, padx=5, pady=5)
            return
            
        for i, point in enumerate(self.sequence_data):
            bg_style = 'TButton' if i == highlight_index and self.sequence_playing else 'TLabelframe'
            fg_color = COLOR_HIGHLIGHT if i == highlight_index and self.sequence_playing else COLOR_TEXT
            
            point_frame = ttk.Frame(self.sequence_list_frame, padding="5", relief=tk.SOLID, borderwidth=1, style=bg_style)
            point_frame.grid(row=i, column=0, sticky=(tk.W, tk.E), pady=2, padx=2); point_frame.columnconfigure(1, weight=1)
            
            ttk.Label(point_frame, text=f"PUNKT {i+1}", font=('Arial', 10, 'bold'), foreground=fg_color).grid(row=0, column=0, sticky=tk.W, padx=5)

            phi_display = f"Auto" if point['Auto_Fi'] else f"{point['Fi']:.2f}°"
            data_text = f"X: {point['X']:.2f}, Y: {point['Y']:.2f}, Z: {point['Z']:.2f}, φ: {phi_display}"
            
            ttk.Label(point_frame, text=data_text, foreground=fg_color).grid(row=1, column=0, sticky=tk.W, padx=5)
            
            ttk.Button(point_frame, text="✕", width=2, 
                       command=lambda idx=i: self.remove_point_from_sequence(idx)).grid(row=0, column=1, rowspan=2, sticky=tk.E, padx=5, pady=5)

        self.sequence_list_frame.update_idletasks()
        self.sequence_list_frame.master.master.update_idletasks()
        
    def save_sequence_to_json(self):
        """Zapisuje sekwencję do pliku JSON."""
        if not self.sequence_data:
            messagebox.showwarning("Brak danych", "Sekwencja jest pusta. Dodaj punkty, aby zapisać.")
            return

        filename = asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
            title="Zapisz sekwencję ruchów jako plik JSON"
        )

        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(self.sequence_data, f, indent=4)
                
                self.log(f"INFO: Zapisano {len(self.sequence_data)} punktów do: {filename}")
                messagebox.showinfo("Zapisano", f"Sekwencja została pomyślnie zapisana do:\n{filename}")
                
            except Exception as e:
                self.log(f"BŁĄD ZAPISU: Nie udało się zapisać pliku JSON: {e}")
                messagebox.showerror("Błąd zapisu", f"Nie udało się zapisać pliku JSON:\n{e}")

# =====================================================================
# MAIN
# =====================================================================

def main():
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()