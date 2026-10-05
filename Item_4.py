import math, time

# ============ CONEXIÓN: reutiliza `mc` si ya existe en el notebook ============
if 'mc' not in globals():
    from pymycobot.mycobot import MyCobot
    mc = MyCobot('/dev/ttyUSB0', 1000000)
    time.sleep(1.0)

# ============ ÍTEM 1: tabla DH + FK ============
PI = math.pi
DH = [(0, 0, 131.22, 0), (PI/2, 0, 0, -PI/2), (0, -110.4, 0, 0),
      (0, -96, 63.4, -PI/2), (PI/2, 0, 75.05, PI/2), (-PI/2, 0, 45.6, 0)]   # (alpha_{i-1}, a_{i-1}, d_i, offset_i)

def marcos(q):
    T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    pts = []
    for (al, a, d, off), th in zip(DH, q):
        ca, sa, ct, st = math.cos(al), math.sin(al), math.cos(th+off), math.sin(th+off)
        A = [[ct, -st, 0, a], [st*ca, ct*ca, -sa, -sa*d], [st*sa, ct*sa, ca, ca*d], [0, 0, 0, 1]]
        T = [[sum(T[i][k]*A[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        pts.append((T[0][3], T[1][3], T[2][3]))
    return pts

def fk(q):
    return marcos(q)[-1]

def codo(q):
    S, E, W = marcos(q)[1], marcos(q)[2], marcos(q)[3]
    az = math.atan2(W[1], W[0]); r = lambda v: v[0]*math.cos(az) + v[1]*math.sin(az)
    cruz = (r(W)-r(S))*(E[2]-S[2]) - (W[2]-S[2])*(r(E)-r(S))
    return 'ARRIBA' if cruz > 0 else 'ABAJO'

def leer(n=6, intentos=10):
    """Lee (ángulos, coords) con reintentos; None si el brazo no responde."""
    for _ in range(intentos):
        try:
            a = mc.get_angles(); time.sleep(0.3); c = mc.get_coords()
        except Exception:
            a = c = None
        if a and c and a != -1 and c != -1 and len(a) >= n and len(c) >= n:
            return list(a), list(c)
        time.sleep(0.5)
    return None, None

def esperar_fin():
    time.sleep(1.0)
    for _ in range(40):
        try:
            if mc.is_moving() == 0:
                break
        except Exception:
            time.sleep(5); break
        time.sleep(0.5)
    time.sleep(0.5)

# ============ ÍTEM 4: tres mediciones ============
VEL, MODO = 30, 0
MEDICIONES = [(40, 0, -60), (0, 40, -40), (-30, 0, -50)]    # (DX, DY, DZ) en mm, relativos a la pose inicial

a0, c0 = leer()
if c0 is None:
    raise RuntimeError("El brazo no responde. Revisa encendido, cable y que ningún otro programa use el puerto.")
print("Pose inicial:", [round(v, 1) for v in c0[:3]], "| orientación:", [round(v, 1) for v in c0[3:6]])
input("Área despejada y equipo avisado. Enter para MOVER el brazo (3 mediciones)... ")

resumen = []
for n, (dx, dy, dz) in enumerate(MEDICIONES, 1):
    objetivo = [c0[0]+dx, c0[1]+dy, c0[2]+dz] + c0[3:6]          # mantiene la orientación inicial
    pedido = objetivo[:3]
    if math.sqrt(sum(v*v for v in pedido)) > 480 or pedido[2] < 0:
        print(f"\n--- Medición {n}: objetivo {pedido} fuera de alcance, se omite ---"); continue

    mc.send_coords(objetivo, VEL, MODO)
    esperar_fin()
    ang, cf = leer()
    if ang is None:
        print(f"\n--- Medición {n}: el brazo no respondió ---"); continue

    q = [v*PI/180 for v in ang]
    x, y, z = fk(q)
    e_ped, e_fw = math.dist((x, y, z), pedido), math.dist((x, y, z), cf[:3])
    movio = max(abs(p-r) for p, r in zip(c0[:3], cf[:3])) >= 1.0

    print(f"\n=== MEDICIÓN {n}  (desplazamiento {dx}, {dy}, {dz} mm) ===")
    print("Pedido              :", [round(v, 1) for v in pedido])
    print("q ejecutado (°)     :", [round(v, 1) for v in ang])
    print("FK propia(q)        :", [round(v, 1) for v in (x, y, z)])
    print("get_coords firmware :", [round(v, 1) for v in cf[:3]])
    print(f"ERROR FK vs PEDIDO  : {e_ped:.1f} mm  {'OK' if e_ped <= 10 else '> 10 mm'}")
    print(f"Error FK vs firmware: {e_fw:.1f} mm")
    print(f"Joint 3 (codo)      : {ang[2]:.1f}°  -> CODO {codo(q)}")
    if not movio:
        print("AVISO: el brazo no se movió (objetivo inalcanzable con esa orientación).")
    resumen.append((n, e_ped, e_fw, ang[2], codo(q), movio))

    mc.send_coords(c0, VEL, MODO)          # vuelve a la pose inicial para la siguiente medición
    esperar_fin()

print("\n================ RESUMEN ÍTEM 4 ================")
print("N | err vs pedido | err vs firmware | q3 (°) | codo | se movió")
for n, e1, e2, q3, cd, mv in resumen:
    print(f"{n} | {e1:6.1f} mm    | {e2:6.1f} mm      | {q3:6.1f} | {cd:6} | {'sí' if mv else 'NO'}")
