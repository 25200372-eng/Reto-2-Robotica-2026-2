#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verificar_fk.py
===============
RETO DEL BRAZO 2 (RB-2) — Ítem 1: script de EVIDENCIA para la verificación
física de la cinemática directa.

QUÉ RESUELVE ESTE SCRIPT
------------------------
La rúbrica exige un procedimiento muy específico y en este ORDEN:
  1. Se declara la predicción ANTES de medir (para que no se pueda "ajustar"
     la predicción después de ver el resultado real, lo que invalidaría la
     verificación como evidencia).
  2. Se lleva el brazo físico a la pose articular q elegida.
  3. Se mide la posición REAL del efector final (con cinta métrica, regla,
     calibre, o leyendo get_coords() del propio robot como referencia
     independiente si así lo define tu equipo).
  4. Se compara la predicción contra la medición y se calcula el error.
  5. Criterio de aceptación: error de posición ≤ 10 mm.

Este script NO depende de rclpy ni de pymycobot: es deliberadamente
standalone (solo numpy + kinematics.py) para poder ejecutarse en cualquier
laptop del equipo, incluso sin el robot conectado a esa máquina, siempre
que alguien mueva el brazo físicamente y reporte la medición por teclado.

CÓMO GARANTIZAMOS "PREDICCIÓN ANTES DE MEDIR"
----------------------------------------------
El script separa el flujo en DOS pasos con una pausa explícita entre ellos:
  Paso A: se calcula fk(q) y se ESCRIBE INMEDIATAMENTE en el archivo de
          evidencia (evidencia_fk.csv) con un timestamp, ANTES de pedir
          la medición real por teclado.
  Paso B: recién después de haber escrito la predicción en disco, el
          script pide la medición real y la agrega a la MISMA fila.
Esto deja un rastro verificable: el timestamp de la predicción es
necesariamente anterior al de la medición, lo cual sirve como evidencia
para la sustentación frente al docente.

USO
---
    python3 verificar_fk.py --label pose1 --q-deg 0 -30 30 0 45 0
    python3 verificar_fk.py --label pose2 --q-rad 0 -0.52 0.52 0 0.78 0
    python3 verificar_fk.py --resumen        # imprime las 3 poses registradas
"""

import argparse       # Parseo de argumentos de línea de comandos
import csv             # Lectura/escritura del archivo de evidencia
import math
import os
import sys
from datetime import datetime, timezone  # Timestamps con zona horaria para evidencia auditable

# Import del módulo de cinemática directa desarrollado para el Ítem 1.
# Se asume que kinematics.py vive en el mismo directorio que este script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kinematics import fk_pose, NUM_JOINTS  # noqa: E402  (import después de sys.path por diseño)

# Nombre del archivo CSV donde se acumula la evidencia. Se usa modo "append"
# para que las 3 poses de verificación queden todas en un solo archivo,
# tal como se entregará al docente.
ARCHIVO_EVIDENCIA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "evidencia_fk.csv")

# Criterio de aceptación definido por la rúbrica del RB-2.
UMBRAL_ACEPTACION_MM = 10.0


def _timestamp_iso() -> str:
    """Retorna el instante actual en formato ISO-8601 con UTC explícito.
    Usar UTC evita ambigüedades de huso horario entre integrantes del equipo
    que puedan estar registrando evidencia desde distintos países."""
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _asegurar_encabezado_csv() -> None:
    """
    Crea el archivo evidencia_fk.csv con su fila de encabezado SI todavía
    no existe. Si ya existe, no lo toca (para no perder evidencia previa
    de otras poses ya registradas por otros integrantes del equipo).
    """
    if not os.path.exists(ARCHIVO_EVIDENCIA):
        with open(ARCHIVO_EVIDENCIA, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "label", "timestamp_prediccion_utc", "q_rad",
                "x_pred_mm", "y_pred_mm", "z_pred_mm",
                "timestamp_medicion_utc",
                "x_medido_mm", "y_medido_mm", "z_medido_mm",
                "error_euclidiano_mm", "resultado",
            ])


def _pedir_float(mensaje: str) -> float:
    """
    Pide un número flotante por teclado, reintentando indefinidamente si el
    usuario ingresa texto no numérico. Aislar esta lógica evita repetir el
    mismo try/except tres veces (una por cada coordenada x, y, z).
    """
    while True:
        try:
            return float(input(mensaje).strip())
        except ValueError:
            print("  -> Entrada inválida, ingresa un número (ej: 152.3).")


def declarar_prediccion_y_verificar(label: str, q_rad) -> None:
    """
    Ejecuta el flujo completo de verificación para UNA pose:

      1. Calcula fk(q) -> (x, y, z, roll, pitch, yaw) predichos.
      2. Escribe la predicción en evidencia_fk.csv INMEDIATAMENTE (columnas
         de medición quedan vacías por ahora). Esto "sella" la predicción
         con un timestamp antes de que exista cualquier dato medido.
      3. Muestra en pantalla instrucciones para mover el brazo físico a
         ese mismo q y solicita la medición real (x, y, z) en mm.
      4. Calcula el error euclidiano 3D y lo compara contra el umbral de
         10 mm de la rúbrica, agregando el resultado a la misma fila.

    Args:
        label: identificador legible de la pose, ej. "pose1", "home", etc.
        q_rad: secuencia de 6 ángulos articulares en RADIANES.
    """
    if len(q_rad) != NUM_JOINTS:
        print(f"ERROR: se esperaban {NUM_JOINTS} ángulos, se recibieron {len(q_rad)}.")
        sys.exit(1)

    # --- PASO 1: calcular la predicción ---
    x_pred, y_pred, z_pred, roll, pitch, yaw = fk_pose(q_rad)
    ts_prediccion = _timestamp_iso()

    print("\n=== PREDICCIÓN (calculada ANTES de mover el brazo) ===")
    print(f"Pose:              {label}")
    print(f"q (rad):           {['%.4f' % v for v in q_rad]}")
    print(f"q (grados):        {['%.2f' % math.degrees(v) for v in q_rad]}")
    print(f"Posición predicha: x={x_pred:.2f} mm, y={y_pred:.2f} mm, z={z_pred:.2f} mm")
    print(f"Orientación:       roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°")
    print(f"Timestamp (UTC):   {ts_prediccion}")

    # --- PASO 2: sellar la predicción en el CSV ANTES de pedir la medición ---
    # Escribimos la fila ahora mismo, con las columnas de medición vacías,
    # y luego la volveremos a abrir para completar solo esas columnas.
    # Esto es deliberado: el archivo en disco es la prueba de que la
    # predicción existía antes de la medición.
    _asegurar_encabezado_csv()
    with open(ARCHIVO_EVIDENCIA, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            label, ts_prediccion, [round(v, 6) for v in q_rad],
            round(x_pred, 3), round(y_pred, 3), round(z_pred, 3),
            "", "", "", "", "", "PENDIENTE_DE_MEDICION",
        ])
    print(f"\n[OK] Predicción sellada en '{ARCHIVO_EVIDENCIA}'.")

    # --- PASO 3: instrucción y medición física ---
    input(
        "\n>>> Ahora mueve físicamente el JetCobot a este vector articular "
        "(con jog_angle/send_angles) y mide la posición real del efector "
        "final con tu instrumento (regla/calibre/tablero de referencia).\n"
        ">>> Presiona ENTER cuando el brazo esté quieto en la pose y listo para medir..."
    )
    x_med = _pedir_float("Posición medida X (mm): ")
    y_med = _pedir_float("Posición medida Y (mm): ")
    z_med = _pedir_float("Posición medida Z (mm): ")
    ts_medicion = _timestamp_iso()

    # --- PASO 4: cálculo de error y veredicto ---
    error_mm = math.sqrt((x_med - x_pred) ** 2 + (y_med - y_pred) ** 2 + (z_med - z_pred) ** 2)
    resultado = "APROBADO" if error_mm <= UMBRAL_ACEPTACION_MM else "RECHAZADO"

    print(f"\n=== RESULTADO ({label}) ===")
    print(f"Predicho: ({x_pred:.2f}, {y_pred:.2f}, {z_pred:.2f}) mm")
    print(f"Medido:   ({x_med:.2f}, {y_med:.2f}, {z_med:.2f}) mm")
    print(f"Error euclidiano: {error_mm:.2f} mm (umbral de aceptación: {UMBRAL_ACEPTACION_MM} mm)")
    print(f"Veredicto: {resultado}")

    # --- Actualizar la última fila del CSV con la medición y el veredicto ---
    # Leemos todas las filas, modificamos la última (la que acabamos de
    # sellar en el paso 2) y reescribimos el archivo completo. Es una
    # operación O(n) pero el CSV de evidencia tiene, como mucho, unas pocas
    # decenas de filas (3 poses x 4 integrantes), así que el costo es
    # irrelevante frente a la trazabilidad que ganamos.
    with open(ARCHIVO_EVIDENCIA, mode="r", newline="", encoding="utf-8") as f:
        filas = list(csv.reader(f))
    filas[-1][6] = ts_medicion
    filas[-1][7] = round(x_med, 3)
    filas[-1][8] = round(y_med, 3)
    filas[-1][9] = round(z_med, 3)
    filas[-1][10] = round(error_mm, 3)
    filas[-1][11] = resultado
    with open(ARCHIVO_EVIDENCIA, mode="w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows(filas)
    print(f"[OK] Evidencia completa guardada en '{ARCHIVO_EVIDENCIA}'.")


def mostrar_resumen() -> None:
    """
    Imprime en pantalla todas las filas registradas en evidencia_fk.csv,
    en formato de tabla legible. Útil para revisar rápidamente si ya se
    completaron las 3 poses de verificación exigidas y si todas pasaron
    el criterio de ≤ 10 mm antes de dar por cerrado el Ítem 1.
    """
    if not os.path.exists(ARCHIVO_EVIDENCIA):
        print(f"Aún no existe '{ARCHIVO_EVIDENCIA}'. Registra al menos una pose primero.")
        return

    with open(ARCHIVO_EVIDENCIA, mode="r", newline="", encoding="utf-8") as f:
        filas = list(csv.reader(f))

    encabezado, datos = filas[0], filas[1:]
    print(f"\n=== RESUMEN DE EVIDENCIA ({len(datos)} pose(s) registrada(s)) ===")
    for fila in datos:
        d = dict(zip(encabezado, fila))
        print(f"- {d['label']}: error={d['error_euclidiano_mm']} mm -> {d['resultado']}")

    aprobadas = sum(1 for f in datos if dict(zip(encabezado, f))["resultado"] == "APROBADO")
    print(f"\nTotal aprobadas: {aprobadas}/{len(datos)} (se requieren 3 poses aprobadas).")


def main() -> None:
    """Punto de entrada: parsea argumentos CLI y despacha a la función correspondiente."""
    parser = argparse.ArgumentParser(
        description="Verificación física de la cinemática directa del JetCobot (RB-2, Ítem 1)."
    )
    parser.add_argument("--label", type=str, help="Identificador de la pose (ej. pose1, home, pose_extendida).")
    grupo_q = parser.add_mutually_exclusive_group()
    grupo_q.add_argument("--q-rad", type=float, nargs=NUM_JOINTS, metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
                          help="Vector articular en RADIANES (6 valores).")
    grupo_q.add_argument("--q-deg", type=float, nargs=NUM_JOINTS, metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
                          help="Vector articular en GRADOS (6 valores). Se convierte internamente a radianes.")
    parser.add_argument("--resumen", action="store_true", help="Muestra el resumen de poses ya registradas y sale.")
    args = parser.parse_args()

    if args.resumen:
        mostrar_resumen()
        return

    if not args.label or (args.q_rad is None and args.q_deg is None):
        parser.error("Debes indicar --label y (--q-rad o --q-deg), o usar --resumen.")

    # Normalizamos siempre a radianes internamente, porque fk() de
    # kinematics.py trabaja en radianes (coherente con pymycobot get_radians()).
    q_rad = args.q_rad if args.q_rad is not None else [math.radians(v) for v in args.q_deg]

    declarar_prediccion_y_verificar(args.label, q_rad)


if __name__ == "__main__":
    main()
