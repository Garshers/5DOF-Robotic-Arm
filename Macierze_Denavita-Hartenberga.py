import sympy as sp
import pandas as pd
import numpy as np
import os

def create_dh_excel_file(filename="macierze_składowe_Ai.xlsx"):
    # Określ pełną ścieżkę do pliku
    script_dir = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(script_dir, filename)

    # --- Definicja symboli (zmiennych) ---
    theta1, theta2, theta3, theta4, alpha5 = sp.symbols('theta_1 theta_2 theta_3 theta_4 alpha_5')
    pi = sp.pi

    # --- Definicja stałych (na podstawie tabeli D-H) ---
    l1_val = 18.4
    l2_val = 149.0
    l3_val = 120.3
    l4_val = 87.9
    l5_val = 120.0
    lambda1_val = 78.8
    lambda5_val = 10.0
    
    # alpha_i (Skręt członu)
    alpha1_val = pi / 2 
    alpha4_val = -pi / 2 

    # --- Definicje pomocniczych macierzy transformacji 4x4 ---
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

    def TransZ(d): # Dla Trans(0, 0, d)
        return sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]
        ])

    def TransX(a): # Dla Trans(a, 0, 0)
        return sp.Matrix([
            [1, 0, 0, a],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    # --- Obliczanie macierzy A_i (zgodnie z obrazem) ---
    print("\nObliczam macierze składowe...")
    
    # A1 = Rot(z, theta1) Trans(0,0, lambda1) Trans(l1, 0,0) Rot(x, pi/2)
    A1 = RotZ(theta1) * TransZ(lambda1_val) * TransX(l1_val) * RotX(alpha1_val)
    
    # A2 = Rot(z, theta2) Trans(l2, 0,0)
    A2 = RotZ(theta2) * TransX(l2_val)
    
    # A3 = Rot(z, -theta3) Trans(l3, 0,0)
    A3 = RotZ(-theta3) * TransX(l3_val)

    # A4 = Rot(z, -theta4) Trans(l4, 0,0) Rot(x, -pi/2)
    A4 = RotZ(-theta4) * TransX(l4_val) * RotX(alpha4_val)
    
    # A5 = Trans(0,0, lambda5) Trans(l5, 0,0) Rot(x, alpha5)
    A5 = TransZ(lambda5_val) * TransX(l5_val) * RotX(alpha5)


    print("Obliczam macierz wynikową A = A1 * A2 * A3 * A4 * A5...")
    A = A1 * A2 * A3 * A4 * A5
    print("Macierz A obliczona!")
    
    matrices = {
        'A1': A1,
        'A2': A2,
        'A3': A3,
        'A4': A4,
        'A5': A5,
        'A': A
    }

    # --- 5. Zapis do pliku Excel ---
    try:
        with pd.ExcelWriter(full_path, engine='openpyxl') as writer:
            for sheet_name, matrix in matrices.items():
                
                # Konwersja macierzy SymPy na listę
                matrix_list = matrix.tolist()

                # Konwertujemy elementy na stringi, aby Excel poprawnie wyświetlił formuły
                df = pd.DataFrame(matrix_list).map(str)
                
                # Zapis do arkusza
                df.to_excel(writer, sheet_name=sheet_name, header=False, index=False)
                
                print(f" -> Zapisano macierz {sheet_name}")

        # Sprawdzenie czy plik istnieje
        if os.path.exists(full_path):
            file_size = os.path.getsize(full_path)
            print(f"\n✓ Sukces! Plik '{filename}' został wygenerowany.")
            print(f"  Lokalizacja: {full_path}")
            print(f"  Rozmiar: {file_size} bajtów")
        else:
            print(f"\n✗ BŁĄD: Plik nie został utworzony!")
            
    except Exception as e:
        print(f"\n✗ BŁĄD podczas zapisu: {e}")

if __name__ == "__main__":
    create_dh_excel_file()