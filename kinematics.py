#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kinematics.py
=============
RETO DEL BRAZO 2 (RB-2) — Ítem 1: Cinemática directa verificada.

Este módulo implementa la Cinemática Directa (Forward Kinematics, FK) del
JetCobot (Yahboom, brazo mecánico idéntico al myCobot 280 de Elephant
Robotics, solo cambia la SBC: Jetson Nano en vez de M5/Raspberry Pi) usando
la convención CLÁSICA de Denavit-Hartenberg (Craig, 2005 / Spong et al.):

    T_i^{i-1} = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)

Este módulo NO depende de ROS 2: es una librería pura de álgebra lineal
(numpy) para que pueda ser importada tanto por el script de verificación
standalone (Ítem 1) como, más adelante, por el nodo `arm_broker` (Ítem 2)
para el "portero" de admisión (goal_callback) y por el auditor cartesiano
(Ítem 4). Mantenerla desacoplada de rclpy es una decisión de diseño:
permite testear la FK con `pytest` puro, sin levantar un nodo ROS.

Fuente de la tabla DH: documentación oficial de Elephant Robotics
(GitBook "myCobot Series 6-Axis Collaborative Robotic Arm" — sección
"1.1 DH Parameters of Robotic Arm", modelo myCobot 280). El JetCobot
comparte la mecánica (mismas longitudes de eslabón) con el myCobot 280;
solo se debe re-verificar el "offset" de cada motor si el ensamblaje
mecánico del kit de tu equipo difiere (por eso el Ítem 1 exige
verificación física con 3 poses conocidas).
"""

from __future__ import annotations  # Permite anotaciones de tipo "hacia adelante" (Python 3.8+)

import math                         # Funciones trigonométricas y pi
from dataclasses import dataclass   # Para estructurar cada fila de la tabla DH de forma legible
from typing import List, Sequence, Tuple

import numpy as np                  # Álgebra lineal: matrices homogéneas 4x4


# ---------------------------------------------------------------------------
# 1. ESTRUCTURA DE UN PARÁMETRO DH
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DHParam:
    """
    Representa una fila de la tabla de Denavit-Hartenberg clásica para
    UNA articulación del manipulador.

    Atributos:
        d       : desplazamiento a lo largo de z_{i-1} (offset de eslabón), en mm.
                  Es CONSTANTE para una articulación de revolución (nuestro caso:
                  las 6 articulaciones del JetCobot son rotacionales/revolute).
        a       : longitud del eslabón a lo largo de x_i, en mm. Constante.
        alpha   : torsión del eslabón (ángulo entre z_{i-1} y z_i), en radianes.
                  Constante.
        offset  : desfase angular constante que se SUMA al valor articular
                  medido/comandado q_i para obtener el theta_i que realmente
                  entra en la matriz DH. Este offset nace de cómo el
                  fabricante define la posición "cero" mecánica de cada motor
                  frente al eje x_{i-1} teórico del modelo DH. Sin este
                  offset, q=0 en el robot real NO correspondería a la pose
                  "brazo extendido" del modelo matemático.
    """
    d: float       # mm
    a: float       # mm
    alpha: float   # rad
    offset: float  # rad


# ---------------------------------------------------------------------------
# 2. TABLA DH DEL JETCOBOT (idéntica a myCobot 280, ver docstring del módulo)
# ---------------------------------------------------------------------------
# Orden de columnas en la fuente oficial: [theta(variable), d, a, alpha, offset]
# Aquí NO almacenamos "theta" porque theta_i = q_i (variable articular) es lo
# que recibe fk(q) en tiempo de ejecución; solo guardamos las 4 constantes
# (d, a, alpha, offset) de cada articulación.
JETCOBOT_DH: List[DHParam] = [
    DHParam(d=134.75, a=0.0,    alpha=math.pi / 2,  offset=0.0),           # J1 (base -> hombro). Manual físico: 134.75 mm (GitBook: 131.22)
    DHParam(d=0.0,    a=-110.0, alpha=0.0,           offset=-math.pi / 2), # J2 (hombro -> codo, eslabón "húmero"). Manual físico: 110 mm (GitBook: 110.4)
    DHParam(d=0.0,    a=-96.0,  alpha=0.0,           offset=0.0),          # J3 (codo, eslabón "antebrazo")
    DHParam(d=63.4,   a=0.0,    alpha=math.pi / 2,   offset=-math.pi / 2), # J4 (muñeca 1, pitch)
    DHParam(d=75.05,  a=0.0,    alpha=-math.pi / 2,  offset=math.pi / 2),  # J5 (muñeca 2, roll)
    DHParam(d=45.6,   a=0.0,    alpha=0.0,           offset=0.0),          # J6 (muñeca 3 / brida-efector final)
]

# Límites articulares oficiales del fabricante, en GRADOS (ver ficha técnica
# myCobot 280). Se usan en el Ítem 2 como parte del "portero" de admisión
# (rechazo por límites articulares) pero se definen aquí porque son una
# propiedad física del robot, igual que la tabla DH.
JOINT_LIMITS_DEG: List[Tuple[float, float]] = [
    (-160.0, 160.0),  # J1 (manual físico: J1 -160° ~ +160°)
    (-165.0, 165.0),  # J2
    (-165.0, 165.0),  # J3
    (-165.0, 165.0),  # J4
    (-165.0, 165.0),  # J5
    (-175.0, 175.0),  # J6
]

NUM_JOINTS = len(JETCOBOT_DH)  # = 6, usado para validar longitud de q en todas las funciones


# ---------------------------------------------------------------------------
# 3. MATRIZ DE TRANSFORMACIÓN HOMOGÉNEA DH ELEMENTAL
# ---------------------------------------------------------------------------
def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    Construye la matriz de transformación homogénea 4x4 T_i^{i-1} para UNA
    articulación, siguiendo la convención DH CLÁSICA:

        T = Rot_z(theta) @ Trans_z(d) @ Trans_x(a) @ Rot_x(alpha)

    Desarrollando el producto de las 4 transformaciones elementales se
    obtiene la forma cerrada (la que implementamos aquí por eficiencia,
    evitando 4 multiplicaciones de matrices por articulación):

        | cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)   a*cos(theta) |
        | sin(theta)   cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)   a*sin(theta) |
        |     0             sin(alpha)              cos(alpha)              d      |
        |     0                 0                       0                   1      |

    Args:
        theta: ángulo de rotación alrededor de z_{i-1} (rad). Para una
               articulación de revolución, theta = q_i + offset_i.
        d:     desplazamiento a lo largo de z_{i-1} (mm).
        a:     longitud del eslabón a lo largo de x_i (mm).
        alpha: torsión del eslabón (rad).

    Returns:
        np.ndarray de forma (4, 4): matriz homogénea T_i^{i-1}.
    """
    ct, st = math.cos(theta), math.sin(theta)  # Cachear cos/sin evita recomputarlos 2 veces cada uno
    ca, sa = math.cos(alpha), math.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ], dtype=np.float64)


# ---------------------------------------------------------------------------
# 4. VALIDACIÓN DE ENTRADA
# ---------------------------------------------------------------------------
def _validar_q(q: Sequence[float]) -> None:
    """
    Verifica que el vector articular q tenga exactamente 6 componentes
    numéricas. Lanza ValueError con un mensaje explícito si no es así.
    Esta función se llama al inicio de fk() y fk_all_joints() para fallar
    RÁPIDO y con un motivo claro (coherente con la filosofía de "rechazo
    inmediato con motivo explícito" que pide el Ítem 2 del broker).
    """
    if len(q) != NUM_JOINTS:
        raise ValueError(
            f"Vector articular inválido: se esperaban {NUM_JOINTS} valores, "
            f"se recibieron {len(q)}."
        )
    for i, qi in enumerate(q):
        if not isinstance(qi, (int, float)) or math.isnan(qi) or math.isinf(qi):
            raise ValueError(f"El valor articular q[{i}]={qi!r} no es un número finito válido.")


# ---------------------------------------------------------------------------
# 5. CINEMÁTICA DIRECTA — FUNCIÓN PRINCIPAL DEL ÍTEM 1
# ---------------------------------------------------------------------------
def fk(q: Sequence[float]) -> np.ndarray:
    """
    Cinemática Directa: dado el vector articular q = [q1..q6] EN RADIANES,
    retorna la matriz de transformación homogénea T_6^0 (pose del efector
    final expresada en el sistema de referencia de la base).

    T_6^0 = T_1^0 * T_2^1 * T_3^2 * T_4^3 * T_5^4 * T_6^5

    Cada T_i^{i-1} se construye con dh_transform(theta_i, d_i, a_i, alpha_i),
    donde theta_i = q_i + offset_i (el offset viene de JETCOBOT_DH y
    representa el desfase mecánico de cada motor respecto al modelo DH).

    Args:
        q: secuencia de 6 ángulos articulares en RADIANES, en el orden
           [q1, q2, q3, q4, q5, q6] tal como los reporta/acepta el driver
           (pymycobot get_radians() / send_radians()).

    Returns:
        np.ndarray (4, 4): matriz homogénea T_6^0. La posición del efector
        final en mm es T[:3, 3]; la orientación es la submatriz de
        rotación T[:3, :3].

    Raises:
        ValueError: si q no tiene 6 componentes numéricas finitas.
    """
    _validar_q(q)  # Falla rápido con mensaje claro si el vector es inválido

    T = np.eye(4, dtype=np.float64)  # T_0^0 = identidad: partimos del sistema de la base

    # Multiplicamos en cadena las 6 transformaciones DH elementales.
    # El orden IMPORTA: T_6^0 = T_1^0 @ T_2^1 @ ... @ T_6^5 (producto de
    # matrices NO conmuta). Recorremos las articulaciones en orden 1..6.
    for qi, dh in zip(q, JETCOBOT_DH):
        theta_i = qi + dh.offset               # theta real = variable articular + offset mecánico
        T_i = dh_transform(theta_i, dh.d, dh.a, dh.alpha)
        T = T @ T_i                            # Acumulamos la transformación (post-multiplicación)

    return T


def fk_all_joints(q: Sequence[float]) -> List[np.ndarray]:
    """
    Variante de fk() que retorna la lista de TODAS las transformaciones
    acumuladas T_1^0, T_2^0, ..., T_6^0 (una por cada articulación), no
    solo la del efector final. Es útil para:
      - Dibujar el "esqueleto" del brazo en una visualización.
      - Calcular el Jacobiano geométrico más adelante (no requerido en
        el Ítem 1, pero se deja preparado porque no cuesta nada extra).

    Returns:
        Lista de 6 matrices np.ndarray (4,4): [T_1^0, T_2^0, ..., T_6^0].
    """
    _validar_q(q)
    transformaciones: List[np.ndarray] = []
    T = np.eye(4, dtype=np.float64)
    for qi, dh in zip(q, JETCOBOT_DH):
        theta_i = qi + dh.offset
        T = T @ dh_transform(theta_i, dh.d, dh.a, dh.alpha)
        transformaciones.append(T.copy())  # .copy() evita que todas las entradas apunten al mismo objeto mutable
    return transformaciones


# ---------------------------------------------------------------------------
# 6. UTILIDADES DE POSE (posición + orientación en formato "humano")
# ---------------------------------------------------------------------------
def pose_from_matrix(T: np.ndarray) -> Tuple[float, float, float, float, float, float]:
    """
    Extrae de una matriz homogénea 4x4 la pose en formato (x, y, z, roll,
    pitch, yaw), con posición en mm y ángulos de Euler en GRADOS, usando
    la convención ZYX intrínseca (yaw-pitch-roll), que es la que reporta
    pymycobot en get_coords()/send_coords(). Esto permite comparar
    directamente fk(q) contra las lecturas del robot (clave para el
    Ítem 4: auditoría cartesiana).

    Args:
        T: matriz homogénea 4x4 (típicamente la retornada por fk()).

    Returns:
        Tupla (x, y, z, roll, pitch, yaw): posición en mm, ángulos en grados.
    """
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    R = T[:3, :3]  # Submatriz de rotación 3x3

    # Extracción de ángulos de Euler ZYX (yaw-pitch-roll) desde R.
    # Se protege el caso "gimbal lock" (pitch = ±90°) con un umbral de
    # tolerancia numérica, evitando división por un número casi-cero.
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock: roll y yaw quedan acoplados; fijamos yaw=0 por convención.
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return (
        float(x), float(y), float(z),
        math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
    )


def fk_pose(q: Sequence[float]) -> Tuple[float, float, float, float, float, float]:
    """
    Atajo: calcula fk(q) y de una vez extrae la pose (x, y, z, roll, pitch,
    yaw) en mm/grados. Es la función que más se usará desde scripts de
    verificación y desde el nodo arm_broker, porque evita manipular
    matrices 4x4 directamente en el código de aplicación.
    """
    return pose_from_matrix(fk(q))


# ---------------------------------------------------------------------------
# 7. VALIDACIÓN DE LÍMITES ARTICULARES (usada por el "portero" del Ítem 2)
# ---------------------------------------------------------------------------
def dentro_de_limites_articulares(q_deg: Sequence[float]) -> Tuple[bool, str]:
    """
    Verifica si CADA articulación de q_deg (en GRADOS) está dentro de su
    rango físico permitido (JOINT_LIMITS_DEG). Se define aquí, junto a la
    FK, porque es información física del robot que el Ítem 2 reutilizará
    tal cual dentro de goal_callback() para el rechazo inmediato.

    Args:
        q_deg: 6 ángulos articulares en GRADOS.

    Returns:
        (ok, motivo): ok=True si todas las articulaciones están dentro de
        rango; si ok=False, `motivo` es un string legible que identifica
        EXACTAMENTE qué articulación violó qué límite (para que el broker
        pueda devolver el motivo exacto de rechazo, tal como exige la
        rúbrica).
    """
    _validar_q(q_deg)
    for i, (qi, (lo, hi)) in enumerate(zip(q_deg, JOINT_LIMITS_DEG), start=1):
        if not (lo <= qi <= hi):
            return False, (
                f"Articulación J{i} fuera de límites: {qi:.2f}° "
                f"no está en [{lo:.1f}°, {hi:.1f}°]."
            )
    return True, "OK: todas las articulaciones dentro de límites."


# ---------------------------------------------------------------------------
# 8. PRUEBA RÁPIDA DE HUMO (smoke test) AL EJECUTAR EL MÓDULO DIRECTAMENTE
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # Este bloque NO se ejecuta al hacer `import kinematics`, solo al correr
    # `python3 kinematics.py` directamente. Sirve como sanity-check rápido
    # sin necesidad de escribir un test aparte.
    q_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Pose "cero mecánico" (todas las articulaciones a 0 rad)
    x, y, z, roll, pitch, yaw = fk_pose(q_home)
    print("=== Smoke test: kinematics.py ===")
    print(f"q (rad) = {q_home}")
    print(f"Pose FK -> x={x:.2f} mm, y={y:.2f} mm, z={z:.2f} mm, "
          f"roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°")
    ok, motivo = dentro_de_limites_articulares([0, 0, 0, 0, 0, 0])
    print(f"Límites articulares en q=0: {motivo}")
