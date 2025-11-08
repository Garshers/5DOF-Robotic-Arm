import sympy as sp
import pandas as pd
import numpy as np
import os

def create_dh_excel_file():
    """Tworzy plik Excel z macierzami transformacji DH."""
    
    filename = "macierze_składowe_Ai.xlsx"

    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
    except NameError:
        script_dir = os.getcwd()
    
    full_path = os.path.join(script_dir, filename)

    # Definicja symboli
    th1, th2, th3, th4, alpha5 = sp.symbols('θ1 θ2 θ3 θ4 α5')
    pi = sp.pi

    # Parametry DH
    l1_val = 18.4
    l2_val = 149.0
    l3_val = 120.3
    l4_val = 87.9
    l5_val = 120.0
    lambda1_val = 78.8
    lambda5_val = 100.0
    alpha1_val = pi / 2 
    alpha4_val = -pi / 2 

    # Macierze transformacji elementarnych
    def RotZ(theta):
        c, s = sp.cos(theta), sp.sin(theta)
        return sp.Matrix([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ])

    def RotX(alpha):
        c, s = sp.cos(alpha), sp.sin(alpha)
        return sp.Matrix([
            [1, 0,  0, 0],
            [0, c, -s, 0],
            [0, s,  c, 0],
            [0, 0,  0, 1]
        ])

    def TransZ(d):
        return sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]
        ])

    def TransX(a):
        return sp.Matrix([
            [1, 0, 0, a],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    # Obliczanie macierzy składowych A_i
    A1 = RotZ(th1) * TransZ(lambda1_val) * TransX(l1_val) * RotX(alpha1_val)
    A2 = RotZ(th2) * TransX(l2_val)
    A3 = RotZ(-th3) * TransX(l3_val)
    A4 = RotZ(-th4) * TransX(l4_val) * RotX(alpha4_val)
    A5 = TransZ(lambda5_val) * TransX(l5_val) * RotX(alpha5)

    # Macierz wynikowa A = A1 * A2 * A3 * A4 * A5
    A = sp.simplify(A1 * A2 * A3 * A4 * A5)

    matrices = {
        'A1': A1,
        'A2': A2,
        'A3': A3,
        'A4': A4,
        'A5': A5,
        'A': A
    }

    # Zapis do pliku Excel
    try:
        with pd.ExcelWriter(full_path, engine='openpyxl') as writer:
            for sheet_name, matrix in matrices.items():
                matrix_list = matrix.tolist()
                df = pd.DataFrame(matrix_list).map(str)
                df.to_excel(writer, sheet_name=sheet_name, header=False, index=False)
        
        print(f"✓ Plik '{filename}' został zapisany pomyślnie.")
        return True
    except Exception as e:
        print(f"✗ Błąd zapisu pliku: {e}")
        return False

if __name__ == "__main__":
    # Macierz docelowa H
    H_target = np.array([
        [0.0, -1.0,  0.0, 150.0],
        [1.0,  0.0,  0.0, 100.0],
        [0.0,  0.0,  1.0, 250.0],
        [0.0,  0.0,  0.0,   1.0]
    ])
    
    print("Macierz docelowa H:")
    print(H_target)
    print()

    # Obliczanie kąta fi1 (th_1)
    px = H_target[0, 3]
    py = H_target[1, 3]
    th_1_rad = np.arctan2(py, px)
    th_1_deg = np.rad2deg(th_1_rad)
    
    print(f"Obliczony kąt fi1 (th_1):")
    print(f"  {th_1_deg:.2f}° ({th_1_rad:.4f} rad)")
    print()
    
    # Generowanie pliku Excel
    create_dh_excel_file()