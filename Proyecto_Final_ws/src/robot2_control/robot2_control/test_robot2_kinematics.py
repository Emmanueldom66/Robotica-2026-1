#!/usr/bin/env python3
"""
Script de prueba para validar la cinemática del Robot 2
"""

from sympy import *
import numpy as np

print("="*80)
print("PRUEBA DE CINEMÁTICA - ROBOT 2")
print("="*80)

# ============================================================================
# DEFINICIÓN DE FUNCIONES AUXILIARES
# ============================================================================

def trans_homo_xz(x=0, z=0, gamma=0, alpha=0):
    """Transformación homogénea para plano XZ"""
    R_z = Matrix([ [cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0],[0, 0, 1]])
    R_x = Matrix([ [1, 0, 0], [0, cos(gamma), -sin(gamma)],[0, sin(gamma), cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_z = Matrix([[0],[0],[z]])

    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_z

def trans_homo(x=0, y=0, z=0, gamma=0, beta=0, alpha=0):
    R_z = Matrix([ [cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0],[0, 0, 1]])
    R_y = Matrix([ [cos(beta), 0, sin(beta)], [0, 1, 0],[-sin(beta), 0, cos(beta)]])
    R_x = Matrix([ [1, 0, 0], [0, cos(gamma), -sin(gamma)],[0, sin(gamma), cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_y = Matrix([[0],[y],[0]])
    p_z = Matrix([[0],[0],[z]])
    
    
    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_y = Matrix.vstack(Matrix.hstack(R_y, p_y), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_y * T_z

# ============================================================================
# CONFIGURACIÓN DEL ROBOT
# ============================================================================

print("\n1. PARÁMETROS DEL ROBOT")
print("-" * 80)

theta_0_1, theta_1_2, theta_2_3 = symbols("theta_0_1, theta_1_2, theta_2_3")
l0 = 0.05  # Altura base
l1 = 0.05  # Eslabón 1
l2 = 0.20  # Eslabón 2
l3 = 0.35  # Eslabón 3

print(f"Altura de base (l0): {l0} m")
print(f"Longitud eslabón 1 (l1): {l1} m")
print(f"Longitud eslabón 2 (l2): {l2} m")
print(f"Longitud eslabón 3 (l3): {l3} m")

# ============================================================================
# CINEMÁTICA DIRECTA
# ============================================================================

print("\n2. CINEMÁTICA DIRECTA")
print("-" * 80)

# Transformaciones
T_0_1 = trans_homo_xz(0, l0, 0, theta_0_1)
T_1_2 = trans_homo(0, l1, 0, pi/2, 0, theta_1_2)
T_2_3 = trans_homo_xz(l2, 0, 0, theta_2_3)
T_3_p = trans_homo_xz(l3, 0, 0, 0)

print("\nTransformaciones del sistema i+1 al i\n")
print("\nTransformación del sistema 1 al 0")
pprint(T_0_1)
print("\nTransformación del sistema 2 al 1")
pprint(T_1_2)
print("\nTransformación del sistema 3 al 2")
pprint(T_2_3)
print("\nTransformación del sistema p al 3")
pprint(T_3_p)
print("\nCalculando transformaciones homogéneas...")
T_0_p = simplify(T_0_1 * T_1_2 * T_2_3 * T_3_p)

# Posición del efector final
x_0_p = T_0_p[0, 3]
y_0_p = T_0_p[1, 3]
z_0_p = T_0_p[2, 3]

xi_0_p = Matrix([x_0_p, y_0_p, z_0_p])

print("\nVector de posición del efector final:")
print(f"x = {x_0_p}")
print(f"y = {y_0_p}")
print(f"z = {z_0_p}")

# ============================================================================
# JACOBIANO
# ============================================================================

print("\n3. JACOBIANO")
print("-" * 80)

J = Matrix.hstack(
    diff(xi_0_p, theta_0_1),
    diff(xi_0_p, theta_1_2),
    diff(xi_0_p, theta_2_3)
)

print("Jacobiano calculado:")
pprint(J)

# Determinante
det_J = simplify(J.det())
print("\nDeterminante del Jacobiano:")
pprint(det_J)

J_inv = J.inv()
print("\nInversa del Jacobiano:")
pprint(J_inv)