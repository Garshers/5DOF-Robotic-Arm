import sympy as sp
import pandas as pd
import numpy as np
import os
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
        
    def find_esp32_port(self):
        """Automatyczne wykrywanie portu ESP-32"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.description or 'CP210' in port.description or 'CH340' in port.description:
                print(f"✓ Znaleziono ESP-32 na porcie: {port.device}")
                return port.device
        return None
    
    def connect(self):
        """Połączenie z ESP-32"""
        if self.port is None:
            self.port = self.find_esp32_port()
            if self.port is None:
                print("✗ Nie znaleziono ESP-32!")
                return False
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Czekaj na reset ESP-32
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"✓ Połączono z ESP-32 ({self.port}, {self.baudrate} baud)")
            
            # Uruchom wątek nasłuchujący
            self.running = True
            self.read_thread = threading.Thread(target=self._continuous_read, daemon=True)
            self.read_thread.start()
            
            return True
        except Exception as e:
            print(f"✗ Błąd połączenia: {e}")
            return False
    
    def _continuous_read(self):
        """Ciągły odczyt danych w osobnym wątku"""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Format z ESP-32: <E12.34,Z45.67,Y78.90,X0.00>
                    if line.startswith('<') and line.endswith('>'):
                        data = line[1:-1].split(',')
                        
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
                        
                        # Wyświetl aktualną pozycję
                        print(f"\r Pozycja: X={self.current_angles[0]:6.2f}° Y={self.current_angles[1]:6.2f}° Z={self.current_angles[2]:6.2f}° E={self.current_angles[3]:6.2f}°", end='', flush=True)
                    
                    # Wyświetl inne komunikaty z ESP32
                    elif line and not line.startswith('<'):
                        print(f"\n[ESP32] {line}")
                
                time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"\n✗ Błąd odczytu: {e}")
                break
    
    def get_current_angles(self):
        """Zwraca aktualne kąty"""
        return self.current_angles.copy()
    
    def send_target_angles(self, th1, th2, th3, th4):
        """
        Wysłanie kątów docelowych do ESP-32
        th1 - X (baza)
        th2 - Y (ramię L2)
        th3 - Z (ramię L3)
        th4 - E (nadgarstek)
        """
        if self.ser is None or not self.ser.is_open:
            return False
        
        # Konwersja radianów na stopnie
        x_deg = math.degrees(th1)
        y_deg = math.degrees(th2)
        z_deg = math.degrees(th3)
        e_deg = math.degrees(th4)
        
        # Format: X12.34,Y45.67,Z78.90,E12.34
        command = f"X{x_deg:.2f},Y{y_deg:.2f},Z{z_deg:.2f},E{e_deg:.2f}\n"
        
        try:
            self.ser.write(command.encode('utf-8'))
            print(f"\n→ Wysłano: {command.strip()}")
            return True
        except Exception as e:
            print(f"\n✗ Błąd wysyłania: {e}")
            return False
    
    def close(self):
        """Zamknięcie połączenia"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("\n✓ Rozłączono z ESP-32")

# =====================================================================
# DEFINICJA KINEMATYKI I PARAMETRÓW D-H
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

# =====================================================================
# KINEMATYKA PROSTA
# =====================================================================

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

# =====================================================================
# KINEMATYKA ODWROTNA
# =====================================================================

def inverse_kinematics(R, Z, th1_base, phi_deg=0.0, elbow_up=True, reverse_base=False):
    th1 = th1_base
    
    L1 = l2_val
    L2 = l3_val
    L3 = math.sqrt((l4_val + l5_val)**2 + lambda5_val**2)
    
    phi_rad = math.radians(phi_deg)
    phi_corr = phi_rad + phi_offset
    
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

# =====================================================================
# FUNKCJA KOSZTOWA
# =====================================================================

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

# =====================================================================
# GŁÓWNA FUNKCJA
# =====================================================================

def main():
    print("\n" + "="*60)
    print("  STEROWANIE ROBOTEM - Tryb ciągły")
    print("  Mapowanie osi:")
    print("    X - Baza (θ1)")
    print("    Y - Ramię L2 (θ2)")
    print("    Z - Ramię L3 (θ3)")
    print("    E - Nadgarstek (θ4)")
    print("="*60)
    
    # Inicjalizacja połączenia
    robot = RobotSerial()
    
    if not robot.connect():
        print("\n✗ Nie można połączyć z ESP-32. Sprawdź połączenie.")
        return
    
    print("\n[INFO] Uruchomiono ciągły odczyt enkoderów...")
    print("[INFO] Wpisz 'q' aby zakończyć, 'move' aby wysłać pozycję\n")
    
    time.sleep(2)  # Daj czas na pierwsze odczyty
    
    try:
        while True:
            command = input("\nKomenda (move/q): ").strip().lower()
            
            if command == 'q':
                print("\n[INFO] Zamykanie połączenia...")
                break
            
            elif command == 'move':
                # Pobierz aktualną pozycję
                current_pos = robot.get_current_angles()
                current_angles = tuple(math.radians(angle) for angle in current_pos)
                
                print(f"\n Aktualna pozycja:")
                print(f"   X (baza)       = {current_pos[0]:7.2f}°")
                print(f"   Y (ramię L2)   = {current_pos[1]:7.2f}°")
                print(f"   Z (ramię L3)   = {current_pos[2]:7.2f}°")
                print(f"   E (nadgarstek) = {current_pos[3]:7.2f}°")
                
                # Wprowadzenie współrzędnych docelowych
                try:
                    print("\n" + "-"*60)
                    print("Podaj współrzędne DOCELOWE (Target):")
                    x_target = float(input("  X [mm]: "))
                    y_target = float(input("  Y [mm]: "))
                    z_target = float(input("  Z [mm]: "))
                    
                    use_orientation = input("\nCzy określić orientację końcówki? (t/n): ").lower()
                    phi_deg = float(input("  Orientacja φ [stopnie]: ")) if use_orientation == 't' else 0.0
                    
                except ValueError:
                    print("\n✗ Błąd: Wprowadź poprawne wartości liczbowe!")
                    continue
                
                # Obliczenia kinematyki odwrotnej
                R_target = math.sqrt(x_target**2 + y_target**2)
                Z_target = z_target
                
                th1_normal = 0.0 if R_target < 0.01 else math.atan2(y_target, x_target)
                th1_reversed_raw = 0.0 if R_target < 0.01 else math.atan2(y_target, x_target)
                th1_reversed = ((th1_reversed_raw + math.pi) + math.pi) % (2 * math.pi) - math.pi
                
                sol_normal_up = inverse_kinematics(R_target, Z_target, th1_normal, phi_deg, elbow_up=True, reverse_base=False)
                sol_normal_down = inverse_kinematics(R_target, Z_target, th1_normal, phi_deg, elbow_up=False, reverse_base=False)
                sol_reversed_up = inverse_kinematics(R_target, Z_target, th1_reversed, phi_deg, elbow_up=True, reverse_base=True)
                sol_reversed_down = inverse_kinematics(R_target, Z_target, th1_reversed, phi_deg, elbow_up=False, reverse_base=True)
                
                solutions = [
                    (sol_normal_up, "Baza Normalna, Łokieć GÓRA"),
                    (sol_normal_down, "Baza Normalna, Łokieć DÓŁ"),
                    (sol_reversed_up, "Baza Odwrócona, Łokieć GÓRA"),
                    (sol_reversed_down, "Baza Odwrócona, Łokieć DÓŁ")
                ]
                
                solution_costs = []
                for sol, name in solutions:
                    cost = calculate_joint_distance(current_angles, sol)
                    if cost != float('inf'):
                        solution_costs.append((cost, sol, name))
                
                if not solution_costs:
                    print("\n✗ Nie udało się znaleźć żadnego osiągalnego rozwiązania IK.")
                    continue
                
                solution_costs.sort(key=lambda x: x[0])
                best_cost, best_solution, best_name = solution_costs[0]
                
                print(f"\n-> Wybrano konfigurację: {best_name} (koszt: {best_cost:.4f})")
                
                th1, th2, th3, th4 = best_solution
                
                print(f"\n Kąty docelowe:")
                print(f"   X (baza, θ1)       = {math.degrees(th1):7.2f}°")
                print(f"   Y (ramię L2, θ2)   = {math.degrees(th2):7.2f}°")
                print(f"   Z (ramię L3, θ3)   = {math.degrees(th3):7.2f}°")
                print(f"   E (nadgarstek, θ4) = {math.degrees(th4):7.2f}°")
                
                # Weryfikacja
                A_calculated = forward_kinematics(th1, th2, th3, th4, 0.0)
                x_calc, y_calc, z_calc = A_calculated[0, 3], A_calculated[1, 3], A_calculated[2, 3]
                
                error_total = math.sqrt((x_target - x_calc)**2 + (y_target - y_calc)**2 + (z_target - z_calc)**2)
                
                print(f"\n✓ Weryfikacja FK:")
                print(f"   Cel:    X={x_target:7.2f}, Y={y_target:7.2f}, Z={z_target:7.2f}")
                print(f"   Wynik:  X={x_calc:7.2f}, Y={y_calc:7.2f}, Z={z_calc:7.2f}")
                print(f"   Błąd:   {error_total:.4f} mm")
                
                # Wysłanie do ESP-32
                confirm = input("\nWysłać kąty do robota? (t/n): ").lower()
                if confirm == 't':
                    if robot.send_target_angles(th1, th2, th3, th4):
                        print("✓ Komendy wysłane do ESP-32!")
                    else:
                        print("✗ Błąd wysyłania komend")
            
            else:
                print("Nieznana komenda. Użyj 'move' lub 'q'")
    
    except KeyboardInterrupt:
        print("\n\n[INFO] Przerwano przez użytkownika (Ctrl+C)")
    
    finally:
        robot.close()

if __name__ == "__main__":
    main()