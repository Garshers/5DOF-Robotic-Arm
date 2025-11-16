import tkinter as tk
from tkinter import ttk, messagebox
import sympy as sp
import numpy as np
import math
import serial
import serial.tools.list_ports
import time
import threading

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

def solve_ik_for_cartesian(x_target, y_target, z_target, phi_deg, current_angles):
    """
    Rozwiązanie IK dla współrzędnych kartezjańskich (X, Y, Z).
    Zwraca najlepsze rozwiązanie (th1, th2, th3, th4) oraz nazwę konfiguracji.
    
    UWAGA: (X, Y, Z) to współrzędne kartezjańskie końcówki robota w układzie globalnym.
    """
    # Oblicz R (odległość radialna od osi Z) i Z (wysokość)
    # To są współrzędne CYLINDRYCZNE
    R_target = math.sqrt(x_target**2 + y_target**2)
    Z_target = z_target
    
    # Kąt podstawy (obrót wokół osi Z) - zależy tylko od kierunku X,Y
    th1_base = math.atan2(y_target, x_target) if R_target > 0.01 else 0.0
    
    # Generuj wszystkie możliwe konfiguracje
    solutions = []
    
    # 1. Normalna baza, łokieć góra
    sol = inverse_kinematics(R_target, Z_target, th1_base, phi_deg, elbow_up=True, reverse_base=False)
    if sol:
        solutions.append((sol, "Baza Normalna, Łokieć GÓRA"))
    
    # 2. Normalna baza, łokieć dół
    sol = inverse_kinematics(R_target, Z_target, th1_base, phi_deg, elbow_up=False, reverse_base=False)
    if sol:
        solutions.append((sol, "Baza Normalna, Łokieć DÓŁ"))
    
    # 3. Odwrócona baza, łokieć góra
    th1_reversed_raw = th1_base
    th1_reversed = ((th1_reversed_raw + math.pi) + math.pi) % (2 * math.pi) - math.pi
    sol = inverse_kinematics(R_target, Z_target, th1_reversed, phi_deg, elbow_up=True, reverse_base=True)
    if sol:
        solutions.append((sol, "Baza Odwrócona, Łokieć GÓRA"))
    
    # 4. Odwrócona baza, łokieć dół
    sol = inverse_kinematics(R_target, Z_target, th1_reversed, phi_deg, elbow_up=False, reverse_base=True)
    if sol:
        solutions.append((sol, "Baza Odwrócona, Łokieć DÓŁ"))
    
    if not solutions:
        return None, "BRAK ROZWIĄZANIA"
    
    # Wybierz rozwiązanie najbliższe aktualnej pozycji
    solution_costs = []
    for sol, name in solutions:
        cost = calculate_joint_distance(current_angles, sol)
        solution_costs.append((cost, sol, name))
    
    solution_costs.sort(key=lambda x: x[0])
    best_cost, best_solution, best_name = solution_costs[0]
    
    return best_solution, best_name

# =====================================================================
# GUI APPLICATION
# =====================================================================

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry("900x700")
        
        self.robot = RobotSerial()
        self.robot.set_gui_callback(self.update_position_display)
        
        self.setup_ui()
        
        # Auto-connect przy starcie
        self.root.after(500, self.connect_robot)
        
    def setup_ui(self):
        # Główny frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Status połączenia
        status_frame = ttk.LabelFrame(main_frame, text="Status połączenia", padding="10")
        status_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Rozłączony", foreground="red")
        self.status_label.grid(row=0, column=0, padx=5)
        
        ttk.Button(status_frame, text="Połącz", command=self.connect_robot).grid(row=0, column=1, padx=5)
        
        # Aktualna pozycja
        position_frame = ttk.LabelFrame(main_frame, text="Aktualna pozycja", padding="10")
        position_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.pos_labels = {}
        for i, axis in enumerate(['X (baza)', 'Y (ramię L2)', 'Z (ramię L3)', 'E (nadgarstek)']):
            ttk.Label(position_frame, text=axis + ":").grid(row=i, column=0, sticky=tk.W, pady=2)
            self.pos_labels[axis] = ttk.Label(position_frame, text="0.00°", font=('Arial', 12, 'bold'))
            self.pos_labels[axis].grid(row=i, column=1, sticky=tk.W, padx=10)
        
        # Pozycja docelowa
        target_frame = ttk.LabelFrame(main_frame, text="Pozycja docelowa [mm]", padding="10")
        target_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5, padx=5)
        
        ttk.Label(target_frame, text="X:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.x_entry = ttk.Entry(target_frame, width=15)
        self.x_entry.grid(row=0, column=1, padx=5)
        self.x_entry.insert(0, "0")
        
        ttk.Label(target_frame, text="Y:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.y_entry = ttk.Entry(target_frame, width=15)
        self.y_entry.grid(row=1, column=1, padx=5)
        self.y_entry.insert(0, "0")
        
        ttk.Label(target_frame, text="Z:").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.z_entry = ttk.Entry(target_frame, width=15)
        self.z_entry.grid(row=2, column=1, padx=5)
        self.z_entry.insert(0, "200")
        
        ttk.Label(target_frame, text="Orientacja φ [°]:").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.phi_entry = ttk.Entry(target_frame, width=15)
        self.phi_entry.grid(row=3, column=1, padx=5)
        self.phi_entry.insert(0, "0")
        
        ttk.Button(target_frame, text="WYŚLIJ POZYCJĘ", command=self.send_position, 
                   style='Accent.TButton').grid(row=4, column=0, columnspan=2, pady=15, sticky=(tk.W, tk.E))
        
        # Kąty docelowe (wynik IK)
        angles_frame = ttk.LabelFrame(main_frame, text="Kąty docelowe (wynik IK)", padding="10")
        angles_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.angle_labels = {}
        for i, axis in enumerate(['θ1 (X)', 'θ2 (Y)', 'θ3 (Z)', 'θ4 (E)']):
            ttk.Label(angles_frame, text=axis + ":").grid(row=0, column=i*2, sticky=tk.W, padx=5)
            self.angle_labels[axis] = ttk.Label(angles_frame, text="0.00°", font=('Arial', 10))
            self.angle_labels[axis].grid(row=0, column=i*2+1, sticky=tk.W, padx=5)
        
        # Log
        log_frame = ttk.LabelFrame(main_frame, text="Log komunikacji", padding="10")
        log_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = tk.Text(log_frame, height=15, width=80)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.log_text['yscrollcommand'] = scrollbar.set
        
        # Konfiguracja grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(3, weight=1)
        
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