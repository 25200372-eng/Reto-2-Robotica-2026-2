import argparse      
import csv           
import math
import os
import sys
from datetime import datetime, timezone 
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kinematics import fk_pose, NUM_JOINTS  # noqa: E402  (import después de sys.path por diseño)
ARCHIVO_EVIDENCIA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "evidencia_fk.csv")
UMBRAL_ACEPTACION_MM = 10.0

def _timestamp_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")

def _asegurar_encabezado_csv() -> None:
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
    while True:
        try:
            return float(input(mensaje).strip())
        except ValueError:
            print("  -> Entrada inválida, ingresa un número (ej: 152.3).")

def declarar_prediccion_y_verificar(label: str, q_rad) -> None:
    if len(q_rad) != NUM_JOINTS:
        print(f"ERROR: se esperaban {NUM_JOINTS} ángulos, se recibieron {len(q_rad)}.")
        sys.exit(1)
    x_pred, y_pred, z_pred, roll, pitch, yaw = fk_pose(q_rad)
    ts_prediccion = _timestamp_iso()
    print("\n=== PREDICCIÓN (calculada ANTES de mover el brazo) ===")
    print(f"Pose:              {label}")
    print(f"q (rad):           {['%.4f' % v for v in q_rad]}")
    print(f"q (grados):        {['%.2f' % math.degrees(v) for v in q_rad]}")
    print(f"Posición predicha: x={x_pred:.2f} mm, y={y_pred:.2f} mm, z={z_pred:.2f} mm")
    print(f"Orientación:       roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°")
    print(f"Timestamp (UTC):   {ts_prediccion}")
    _asegurar_encabezado_csv()
    with open(ARCHIVO_EVIDENCIA, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            label, ts_prediccion, [round(v, 6) for v in q_rad],
            round(x_pred, 3), round(y_pred, 3), round(z_pred, 3),
            "", "", "", "", "", "PENDIENTE_DE_MEDICION",
        ])
    print(f"\n[OK] Predicción sellada en '{ARCHIVO_EVIDENCIA}'.")
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
    error_mm = math.sqrt((x_med - x_pred) ** 2 + (y_med - y_pred) ** 2 + (z_med - z_pred) ** 2)
    resultado = "APROBADO" if error_mm <= UMBRAL_ACEPTACION_MM else "RECHAZADO"
    print(f"\n=== RESULTADO ({label}) ===")
    print(f"Predicho: ({x_pred:.2f}, {y_pred:.2f}, {z_pred:.2f}) mm")
    print(f"Medido:   ({x_med:.2f}, {y_med:.2f}, {z_med:.2f}) mm")
    print(f"Error euclidiano: {error_mm:.2f} mm (umbral de aceptación: {UMBRAL_ACEPTACION_MM} mm)")
    print(f"Veredicto: {resultado}")
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
    q_rad = args.q_rad if args.q_rad is not None else [math.radians(v) for v in args.q_deg]
    declarar_prediccion_y_verificar(args.label, q_rad)

if __name__ == "__main__":
    main()
