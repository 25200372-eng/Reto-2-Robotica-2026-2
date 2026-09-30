from __future__ import annotations
import math                         
from dataclasses import dataclass   
from typing import List, Sequence, Tuple
import numpy as np                  

@dataclass(frozen=True)
class DHParam:
        d: float       # mm
    a: float       # mm
    alpha: float   # rad
    offset: float  # rad

JETCOBOT_DH: List[DHParam] = [
    DHParam(d=134.75, a=0.0,    alpha=math.pi / 2,  offset=0.0),           # J1 (base -> hombro). Manual físico: 134.75 mm (GitBook: 131.22)
    DHParam(d=0.0,    a=-110.0, alpha=0.0,           offset=-math.pi / 2), # J2 (hombro -> codo, eslabón "húmero"). Manual físico: 110 mm (GitBook: 110.4)
    DHParam(d=0.0,    a=-96.0,  alpha=0.0,           offset=0.0),          # J3 (codo, eslabón "antebrazo")
    DHParam(d=63.4,   a=0.0,    alpha=math.pi / 2,   offset=-math.pi / 2), # J4 (muñeca 1, pitch)
    DHParam(d=75.05,  a=0.0,    alpha=-math.pi / 2,  offset=math.pi / 2),  # J5 (muñeca 2, roll)
    DHParam(d=45.6,   a=0.0,    alpha=0.0,           offset=0.0),          # J6 (muñeca 3 / brida-efector final)
]

JOINT_LIMITS_DEG: List[Tuple[float, float]] = [
    (-160.0, 160.0),  # J1 (manual físico: J1 -160° ~ +160°)
    (-165.0, 165.0),  # J2
    (-165.0, 165.0),  # J3
    (-165.0, 165.0),  # J4
    (-165.0, 165.0),  # J5
    (-175.0, 175.0),  # J6
]

NUM_JOINTS = len(JETCOBOT_DH)  # = 6, usado para validar longitud de q en todas las funciones

def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)  # Cachear cos/sin evita recomputarlos 2 veces cada uno
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ], dtype=np.float64)
    
def _validar_q(q: Sequence[float]) -> None:
    if len(q) != NUM_JOINTS:
        raise ValueError(
            f"Vector articular inválido: se esperaban {NUM_JOINTS} valores, "
            f"se recibieron {len(q)}."
        )
    for i, qi in enumerate(q):
        if not isinstance(qi, (int, float)) or math.isnan(qi) or math.isinf(qi):
            raise ValueError(f"El valor articular q[{i}]={qi!r} no es un número finito válido.")

def fk(q: Sequence[float]) -> np.ndarray:
    _validar_q(q)  
    T = np.eye(4, dtype=np.float64)  # T = identidad: partimos del sistema de la base
    for qi, dh in zip(q, JETCOBOT_DH):
        theta_i = qi + dh.offset               # theta real = variable articular + offset mecánico
        T_i = dh_transform(theta_i, dh.d, dh.a, dh.alpha)
        T = T @ T_i                            # Acumulamos la transformación (post-multiplicación)
    return T


def fk_all_joints(q: Sequence[float]) -> List[np.ndarray]:
    _validar_q(q)
    transformaciones: List[np.ndarray] = []
    T = np.eye(4, dtype=np.float64)
    for qi, dh in zip(q, JETCOBOT_DH):
        theta_i = qi + dh.offset
        T = T @ dh_transform(theta_i, dh.d, dh.a, dh.alpha)
        transformaciones.append(T.copy())  # .copy() evita que todas las entradas apunten al mismo objeto mutable
    return transformaciones

def pose_from_matrix(T: np.ndarray) -> Tuple[float, float, float, float, float, float]:
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    R = T[:3, :3]  # Submatriz de rotación 3x3
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
       return pose_from_matrix(fk(q))

def dentro_de_limites_articulares(q_deg: Sequence[float]) -> Tuple[bool, str]:
        _validar_q(q_deg)
    for i, (qi, (lo, hi)) in enumerate(zip(q_deg, JOINT_LIMITS_DEG), start=1):
        if not (lo <= qi <= hi):
            return False, (
                f"Articulación J{i} fuera de límites: {qi:.2f}° "
                f"no está en [{lo:.1f}°, {hi:.1f}°]."
            )
    return True, "OK: todas las articulaciones dentro de límites."

if __name__ == "__main__":
    q_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Pose "cero mecánico" (todas las articulaciones a 0 rad)
    x, y, z, roll, pitch, yaw = fk_pose(q_home)
    print("=== Smoke test: kinematics.py ===")
    print(f"q (rad) = {q_home}")
    print(f"Pose FK -> x={x:.2f} mm, y={y:.2f} mm, z={z:.2f} mm, "
          f"roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°")
    ok, motivo = dentro_de_limites_articulares([0, 0, 0, 0, 0, 0])
    print(f"Límites articulares en q=0: {motivo}")
