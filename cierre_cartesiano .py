#!/usr/bin/env python3
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 '..', 'src', 'arm_broker'))
from arm_broker import fk  # usa la tabla DH YA llenada por el equipo (ítem 1)  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True, help='mm, respecto a la base')
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--z', type=float, required=True)
    ap.add_argument('--puerto', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=1000000)
    ap.add_argument('--velocidad', type=int, default=30)
    ap.add_argument('--espera', type=float, default=5.0)
    ap.add_argument('--modo', type=int, default=1, help='0=angular, 1=lineal (send_coords)')
    args = ap.parse_args()

    try:
        from pymycobot.mycobot import MyCobot
    except ImportError:
        from pymycobot import MyCobot

    brazo = MyCobot(args.puerto, args.baud)

    actual = brazo.get_coords()
    if not actual or len(actual) < 6:
        sys.exit('El brazo no respondió get_coords(). ¿Puerto libre y brazo encendido?')
    rx, ry, rz = actual[3], actual[4], actual[5]
    print(f'Orientación actual (se mantiene): rx={rx:.1f}  ry={ry:.1f}  rz={rz:.1f}')

    objetivo = [args.x, args.y, args.z, rx, ry, rz]
    print(f'Enviando send_coords({objetivo}, vel={args.velocidad}, modo={args.modo}) ...')
    brazo.send_coords(objetivo, args.velocidad, args.modo)
    time.sleep(args.espera)

    angulos = brazo.get_angles()
    coords_firmware = brazo.get_coords()
    if not angulos or not coords_firmware:
        sys.exit('El brazo no respondió tras moverse; repite la medición.')

    q_real = [v * math.pi / 180.0 for v in angulos]
    fx, fy, fz = fk.fk(q_real)

    err_vs_pedido = math.sqrt((fx - args.x) ** 2 + (fy - args.y) ** 2 + (fz - args.z) ** 2)
    err_vs_firmware = math.sqrt(
        (fx - coords_firmware[0]) ** 2 +
        (fy - coords_firmware[1]) ** 2 +
        (fz - coords_firmware[2]) ** 2)

    print('\n--- Ítem 4: cierre cartesiano ---')
    print(f'Pedido               : ({args.x:7.1f}, {args.y:7.1f}, {args.z:7.1f}) mm')
    print(f'q ejecutado (grados) : {[round(v, 1) for v in angulos]}')
    print(f'FK propia(q)         : ({fx:7.1f}, {fy:7.1f}, {fz:7.1f}) mm')
    print(f'get_coords firmware  : ({coords_firmware[0]:7.1f}, {coords_firmware[1]:7.1f}, '
          f'{coords_firmware[2]:7.1f}) mm')

    marca = 'OK' if err_vs_pedido <= 10.0 else '>10mm'
    print(f'\nError FK propia vs pedido   : {err_vs_pedido:6.1f} mm  {marca}')
    print(f'Error FK propia vs firmware : {err_vs_firmware:6.1f} mm  (consistencia interna de tu FK)')

    # Pregunta abierta del reto: ¿por qué esa solución de codo y no la contraria?
    # Ajusta el signo/índice según cómo quedó tu tabla DH y tu convención de ejes.
    j3 = angulos[2]
    print(f"\nJoint 3 (codo) = {j3:.1f}°  → describe en tu informe si corresponde a 'codo arriba' o "
          f"'codo abajo' según tu convención, y por qué el firmware no tomó la solución contraria "
          f"(doble solución de la cinemática inversa en un brazo de 6 GDL).")


if __name__ == '__main__':
    main()
