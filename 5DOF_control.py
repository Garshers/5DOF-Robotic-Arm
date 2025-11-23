import tkinter as tk
from tkinter import ttk, messagebox
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
        self.current_angles = [0.0, 0.0, 0.0, 0.0]  # [X, Y, Z, E]
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
        """Połączenie z ESP-32"""
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
        except Exception as e:
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
                                except ValueError:
                                    pass
                        
                        # Wywołaj callback GUI
                        if self.gui_callback:
                            self.gui_callback(self.current_angles.copy())
                    
                    elif line and not line.startswith('<'):
                        if self.gui_callback:
                            self.gui_callback(None, log_message=line)
                
                time.sleep(0.01)
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
    
    return A_num

def inverse_kinematics(R, Z, th1_base, phi_deg=0.0, elbow_up=True, reverse_base=False):
    """
    Oblicza kinematykę odwrotną dla robota 5-DOF (Solver 2D).
    
    Argumenty:
    R, Z: Współrzędne celu (mm) w płaszczyźnie R-Z (cylindryczne!)
    th1_base: Obliczony kąt bazy (radian)
    phi_deg: Kąt orientacji końcówki (stopnie)
    elbow_up: Konfiguracja łokcia (True = łokieć w górze, False = łokieć w dole)
    reverse_base: Konfiguracja podstawy (True = baza odwrócona o 180 stopni)
    
    Zwraca:
    Tuple (th1, th2, th3, th4) w radianach lub None, jeśli nieosiągalne.
    """
    
    th1 = th1_base
    
    L1 = l2_val
    L2 = l3_val
    L3 = math.sqrt((l4_val + l5_val)**2 + lambda5_val**2)
    
    phi_rad = math.radians(phi_deg)
    phi_corr = phi_rad + phi_offset
    
    # KRYTYCZNE: R to współrzędna cylindryczna (odległość od osi Z)
    # FK dodaje l1_val do X, więc musimy to uwzględnić
    R_ik = -R if reverse_base else R
    R_wrist = R_ik - l1_val - L3 * math.cos(phi_corr)
    Z_wrist = Z - lambda1_val - L3 * math.sin(phi_corr)
    
    R_abs = abs(R_wrist)
    D = math.sqrt(R_abs**2 + Z_wrist**2)
    
    if D > (L1 + L2) or D < abs(L1 - L2):
        return None
    
    cos_th3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_th3 = np.clip(cos_th3, -1.0, 1.0)
    
    th3_ik = math.acos(cos_th3) if elbow_up else -math.acos(cos_th3)
    
    alpha = math.atan2(Z_wrist, R_abs)
    beta = math.atan2(L2 * math.sin(th3_ik), L1 + L2 * math.cos(th3_ik))
    th2 = alpha - beta
    
    if R_wrist < 0 and reverse_base:
        th2 = math.pi - th2
        th3_ik = -th3_ik
    
    th4_ik = phi_rad - th2 - th3_ik
    th3 = -th3_ik
    th4 = -th4_ik
    
    th1 = math.atan2(math.sin(th1), math.cos(th1))
    th2 = math.atan2(math.sin(th2), math.cos(th2))
    th3 = math.atan2(math.sin(th3), math.cos(th3))
    th4 = math.atan2(math.sin(th4), math.cos(th4))
    
    return (th1, th2, th3, th4)

JOINT_CONST = pi/2*1.05
JOINT_LIMITS = {
    'th1': (-JOINT_CONST, JOINT_CONST),       # Baza
    'th2': (-JOINT_CONST, JOINT_CONST),       # Bark
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
    - Testuje różne orientacje phi w zakresie [-90°, +90°]
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

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry("1400x700")
        
        self.robot = RobotSerial()
        self.robot.set_gui_callback(self.update_position_display)
        
        # Przechowuje dynamiczne elementy wykresu (linie, punkty)
        self.plot_artists = []
        # Przechowuje maksymalny zasięg do resetowania widoku
        self.max_reach = l2_val + l3_val + l4_val + l5_val
        
        self.setup_ui()
        
        # Konfiguracja 3D jest teraz wywołana RAZ po setup_ui
        self.setup_3d_plot()
        
        # Uruchom aktualizację wizualizacji 3D
        self.update_3d_visualization()
        
        # Auto-connect przy starcie
        self.root.after(100, self.connect_robot)
        
    def setup_ui(self):
        # Główny frame z dwiema kolumnami
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Lewa kolumna - kontrolki
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        # Prawa kolumna - wizualizacja 3D
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        
        # === LEWA KOLUMNA ===
        
        # Status połączenia
        status_frame = ttk.LabelFrame(left_frame, text="Status połączenia", padding="10")
        status_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Rozłączony", foreground="red")
        self.status_label.grid(row=0, column=0, padx=5)
        
        ttk.Button(status_frame, text="Połącz", command=self.connect_robot).grid(row=0, column=1, padx=5)
        
        # Aktualna pozycja
        position_frame = ttk.LabelFrame(left_frame, text="Aktualna pozycja", padding="10")
        position_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)
        
        self.pos_labels = {}
        for i, axis in enumerate(['X (baza)', 'Y (ramię L2)', 'Z (ramię L3)', 'E (nadgarstek)']):
            ttk.Label(position_frame, text=axis + ":").grid(row=i, column=0, sticky=tk.W, pady=2)
            self.pos_labels[axis] = ttk.Label(position_frame, text="0.00°", font=('Arial', 12, 'bold'))
            self.pos_labels[axis].grid(row=i, column=1, sticky=tk.W, padx=10)
        
        # Pozycja docelowa
        target_frame = ttk.LabelFrame(left_frame, text="Pozycja docelowa [mm]", padding="10")
        target_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(target_frame, text="X:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.x_entry = ttk.Entry(target_frame, width=15)
        self.x_entry.grid(row=0, column=1, padx=5)
        self.x_entry.insert(0, "-87.5")
        
        ttk.Label(target_frame, text="Y:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.y_entry = ttk.Entry(target_frame, width=15)
        self.y_entry.grid(row=1, column=1, padx=5)
        self.y_entry.insert(0, "0")
        
        ttk.Label(target_frame, text="Z:").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.z_entry = ttk.Entry(target_frame, width=15)
        self.z_entry.grid(row=2, column=1, padx=5)
        self.z_entry.insert(0, "372.5")

        ttk.Label(target_frame, text="Orientacja φ [°]:").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.phi_entry = ttk.Entry(target_frame, width=15)
        self.phi_entry.grid(row=3, column=1, padx=5)
        self.phi_entry.insert(0, "0")
        
        # Checkbox automatycznej orientacji
        self.auto_phi_var = tk.BooleanVar(value=True)
        self.auto_phi_check = ttk.Checkbutton(
            target_frame, 
            text="Orientacja automatyczna (optymalizacja przemysłowa)",
            variable=self.auto_phi_var,
            command=self.toggle_phi_entry
        )
        self.auto_phi_check.grid(row=4, column=0, columnspan=2, pady=5, sticky=tk.W)
        
        ttk.Button(target_frame, text="WYŚLIJ POZYCJĘ", command=self.send_position, 
                   style='Accent.TButton').grid(row=5, column=0, columnspan=2, pady=15, sticky=(tk.W, tk.E))
        
        # Początkowy stan (automatyczna orientacja włączona)
        self.toggle_phi_entry()
        
        # Kąty docelowe (wynik IK)
        angles_frame = ttk.LabelFrame(left_frame, text="Kąty docelowe (wynik IK)", padding="10")
        angles_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)
        
        self.angle_labels = {}
        for i, axis in enumerate(['θ1 (X)', 'θ2 (Y)', 'θ3 (Z)', 'θ4 (E)']):
            ttk.Label(angles_frame, text=axis + ":").grid(row=i//2, column=(i%2)*2, sticky=tk.W, padx=5, pady=2)
            self.angle_labels[axis] = ttk.Label(angles_frame, text="0.00°", font=('Arial', 10))
            self.angle_labels[axis].grid(row=i//2, column=(i%2)*2+1, sticky=tk.W, padx=5, pady=2)
        
        # Log
        log_frame = ttk.LabelFrame(left_frame, text="Log komunikacji", padding="10")
        log_frame.grid(row=4, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = tk.Text(log_frame, height=12, width=50)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.log_text['yscrollcommand'] = scrollbar.set
        
        # === PRAWA KOLUMNA - WIZUALIZACJA 3D ===
        
        # 1. Definicja ramki
        viz_frame = ttk.LabelFrame(right_frame, text="Wizualizacja 3D", padding="10")
        viz_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 2. Utworzenie wykresu matplotlib
        self.fig = Figure(figsize=(7, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 3. Utworzenie canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        
        # 4. Przycisk resetu widoku
        reset_button = ttk.Button(viz_frame, text="Resetuj widok", command=self.reset_3d_view)
        reset_button.pack(side=tk.TOP, anchor=tk.NW, pady=5, padx=5)
        
        # 5. Spakowanie canvas
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Konfiguracja grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=0)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
        left_frame.rowconfigure(4, weight=1)
        right_frame.rowconfigure(0, weight=1)
    
    def setup_3d_plot(self):
        """
        Konfiguruje statyczne elementy wykresu 3D (osie, limity, siatka, podstawa).
        Wywoływana tylko raz przy inicjalizacji.
        """
        self.ax.set_xlabel('X [mm]')
        self.ax.set_ylabel('Y [mm]')
        self.ax.set_zlabel('Z [mm]')
        self.ax.set_title('Pozycja robota')
        
        # Ustaw zakresy osi
        self.ax.set_xlim([-self.max_reach, self.max_reach])
        self.ax.set_ylim([-self.max_reach, self.max_reach])
        self.ax.set_zlim([0, self.max_reach])
        
        # Kąt widzenia
        self.ax.view_init(elev=20, azim=45)
        
        # Rysuj podstawę (cylinder)
        theta = np.linspace(0, 2*np.pi, 20)
        z_base = np.linspace(0, lambda1_val, 2)
        Theta, Z_base = np.meshgrid(theta, z_base)
        X_base = 30 * np.cos(Theta)
        Y_base = 30 * np.sin(Theta)
        self.ax.plot_surface(X_base, Y_base, Z_base, alpha=0.3, color='gray')
        
        # Dodaj siatkę płaszczyzny XY (na wysokości 0)
        grid_size = 100
        grid_range = np.arange(-300, 301, grid_size)
        X_grid, Y_grid = np.meshgrid(grid_range, grid_range)
        Z_grid = np.zeros_like(X_grid)
        self.ax.plot_wireframe(X_grid, Y_grid, Z_grid, alpha=0.1, color='gray', linewidth=0.5)
        
        # Osie układu współrzędnych
        axis_length = 100
        self.ax.quiver(0, 0, 0, axis_length, 0, 0, color='red', arrow_length_ratio=0.1, linewidth=2, label='X')
        self.ax.quiver(0, 0, 0, 0, axis_length, 0, color='green', arrow_length_ratio=0.1, linewidth=2, label='Y')
        self.ax.quiver(0, 0, 0, 0, 0, axis_length, color='blue', arrow_length_ratio=0.1, linewidth=2, label='Z')
        
        self.ax.legend(loc='upper right')

    def reset_3d_view(self):
        """
        Resetuje widok 3D (limity osi i kamerę) do ustawień domyślnych.
        """
        self.ax.set_xlim([-self.max_reach, self.max_reach])
        self.ax.set_ylim([-self.max_reach, self.max_reach])
        self.ax.set_zlim([0, self.max_reach])
        self.ax.view_init(elev=20, azim=45)
        self.canvas.draw()

    def update_3d_visualization(self):
        """
        Aktualizacja wizualizacji 3D robota.
        Ta funkcja usuwa tylko stare linie robota i rysuje nowe.
        """
        try:
            # --- 1. Usuń poprzednie elementy robota ---
            for artist in self.plot_artists:
                # Sprawdź, czy artist to lista (jak z ax.plot)
                if isinstance(artist, list):
                    for item in artist:
                        item.remove()
                else:
                    artist.remove() # Usuń pojedynczy element (np. z scatter, text)
            
            self.plot_artists = [] # Wyczyść listę
            
            # --- 2. Pobierz nowe dane ---
            current_pos = self.robot.get_current_angles()
            th1 = math.radians(current_pos[0])
            th2 = math.radians(current_pos[1])
            th3 = math.radians(current_pos[2])
            th4 = math.radians(current_pos[3])
            
            positions = get_joint_positions(th1, th2, th3, th4)
            
            x_coords = [pos[0] for pos in positions]
            y_coords = [pos[1] for pos in positions]
            z_coords = [pos[2] for pos in positions]
            
            # --- 3. Rysuj nowe elementy robota ---
            
            # Linie łączące przeguby
            robot_lines = self.ax.plot(x_coords, y_coords, z_coords, 'o-', linewidth=4, 
                                       markersize=10, color='blue', label='Robot', markerfacecolor='lightblue')
            self.plot_artists.append(robot_lines) # Dodaj do listy
            
            # Podświetl końcówkę
            robot_tcp = self.ax.scatter([x_coords[-1]], [y_coords[-1]], [z_coords[-1]], 
                                        c='red', s=150, marker='o', label='Końcówka', edgecolors='darkred', linewidths=2)
            self.plot_artists.append(robot_tcp) # Dodaj do listy
            
            # Oznaczenia przegubów
            labels = ['Base', 'J1', 'J2', 'J3', 'J4', 'TCP']
            for i, pos in enumerate(positions):
                text_label = self.ax.text(pos[0], pos[1], pos[2], f'  {labels[i]}', fontsize=8)
                self.plot_artists.append(text_label) # Dodaj do listy
            
            # --- 4. Odśwież canvas ---
            self.canvas.draw()
            
        except Exception as e:
            if not hasattr(self, '_viz_error_logged'):
                self.log(f"Błąd wizualizacji 3D: {e}")
                self._viz_error_logged = True
        
        # Zaplanuj kolejną aktualizację
        self.root.after(50, self.update_3d_visualization)
        
    def log(self, message):
        """Dodaj wiadomość do logu"""
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)
        
    def update_position_display(self, angles=None, log_message=None):
        """Callback z wątku serial - aktualizacja GUI"""
        if log_message:
            self.root.after(0, lambda: self.log(log_message))
        
        if angles:
            x, y, z, e = angles
            self.root.after(0, lambda: self.pos_labels['X (baza)'].config(text=f"{x:.2f}°"))
            self.root.after(0, lambda: self.pos_labels['Y (ramię L2)'].config(text=f"{y:.2f}°"))
            self.root.after(0, lambda: self.pos_labels['Z (ramię L3)'].config(text=f"{z:.2f}°"))
            self.root.after(0, lambda: self.pos_labels['E (nadgarstek)'].config(text=f"{e:.2f}°"))
    
    def toggle_phi_entry(self):
        """Włącz/wyłącz pole orientacji w zależności od checkboxa"""
        if self.auto_phi_var.get():
            self.phi_entry.config(state='disabled')
        else:
            self.phi_entry.config(state='normal')

    def connect_robot(self):
        """Połącz z robotem"""
        success, message = self.robot.connect()
        if success:
            self.status_label.config(text="Połączono", foreground="green")
            self.log(message)
        else:
            self.status_label.config(text="Rozłączony", foreground="red")
            messagebox.showerror("Błąd połączenia", message)
            self.log(message)
    
    def send_position(self):
        """Oblicz IK i wyślij do robota"""
        try:
            # Pobierz wartości
            x_target = float(self.x_entry.get())
            y_target = float(self.y_entry.get())
            z_target = float(self.z_entry.get())
            if self.auto_phi_var.get():
                phi_deg = None
            else:
                phi_deg = float(self.phi_entry.get())
            
            self.log(f"Obliczanie IK dla pozycji: X={x_target}, Y={y_target}, Z={z_target}, phi={phi_deg}")
            
            # DEBUG: Oblicz R i Z
            R_target = math.sqrt(x_target**2 + y_target**2)
            self.log(f"DEBUG: R_target={R_target:.2f}, Z_target={z_target:.2f}")
            
            # Pobierz aktualną pozycję
            current_pos = self.robot.get_current_angles()
            current_angles = tuple(math.radians(angle) for angle in current_pos)
            
            # Rozwiąż IK
            solution, config_name = solve_ik_for_cartesian(x_target, y_target, z_target, phi_deg, current_angles)
            
            if solution is None:
                messagebox.showerror("Błąd IK", "Nie znaleziono osiągalnego rozwiązania IK.\nPozycja poza zasięgiem robota.")
                self.log("BŁĄD: Pozycja poza zasięgiem robota")
                return
            
            th1, th2, th3, th4 = solution
            
            self.log(f"Wybrano konfigurację: {config_name}")
            self.log(f"Kąty: th1={math.degrees(th1):.2f}, th2={math.degrees(th2):.2f}, th3={math.degrees(th3):.2f}, th4={math.degrees(th4):.2f}")
            
            # Aktualizuj wyświetlane kąty
            self.angle_labels['θ1 (X)'].config(text=f"{math.degrees(th1):.2f}°")
            self.angle_labels['θ2 (Y)'].config(text=f"{math.degrees(th2):.2f}°")
            self.angle_labels['θ3 (Z)'].config(text=f"{math.degrees(th3):.2f}°")
            self.angle_labels['θ4 (E)'].config(text=f"{math.degrees(th4):.2f}°")
            
            # Weryfikacja FK
            A_calculated = forward_kinematics(th1, th2, th3, th4, 0.0)
            x_calc, y_calc, z_calc = A_calculated[0, 3], A_calculated[1, 3], A_calculated[2, 3]
            error_x = abs(x_target - x_calc)
            error_y = abs(y_target - y_calc)
            error_z = abs(z_target - z_calc)
            error_total = math.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            self.log(f"Weryfikacja FK:")
            self.log(f"  Cel:    X={x_target:.2f}, Y={y_target:.2f}, Z={z_target:.2f}")
            self.log(f"  Wynik:  X={x_calc:.2f}, Y={y_calc:.2f}, Z={z_calc:.2f}")
            self.log(f"  Błąd:   dX={error_x:.2f}, dY={error_y:.2f}, dZ={error_z:.2f}, total={error_total:.4f}mm")
            
            if error_total > 10.0:
                self.log(f"UWAGA: Duży błąd weryfikacji FK ({error_total:.2f}mm)!")
                response = messagebox.askyesno("Duży błąd FK", 
                    f"Błąd weryfikacji wynosi {error_total:.2f}mm.\nCzy mimo to wysłać komendy do robota?")
                if not response:
                    return
            elif error_total < 0.1:
                self.log("Weryfikacja POZYTYWNA (błąd < 0.1mm)")
            
            # Wyślij do robota
            success, message = self.robot.send_target_angles(th1, th2, th3, th4)
            if success:
                self.log(message)
                messagebox.showinfo("Sukces", "Komendy wysłane do robota")
            else:
                self.log(message)
                messagebox.showerror("Błąd", message)
                
        except ValueError as e:
            messagebox.showerror("Błąd", "Wprowadź poprawne wartości liczbowe")
            self.log(f"BŁĄD: Niepoprawne dane wejściowe - {e}")
        except Exception as e:
            messagebox.showerror("Błąd", f"Wystąpił błąd: {e}")
            self.log(f"BŁĄD: {e}")
            import traceback
            self.log(traceback.format_exc())
    
    def on_closing(self):
        """Zamknięcie aplikacji"""
        self.robot.close()
        self.root.destroy()

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