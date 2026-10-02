import math
PI = math.pi

DH = [(0, 0, 131.22, 0), (PI/2, 0, 0, -PI/2), (0, -110.4, 0, 0),
      (0, -96, 63.4, -PI/2), (PI/2, 0, 75.05, PI/2), (-PI/2, 0, 45.6, 0)]

def fk(q):
    T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for (al, a, d, off), th in zip(DH, q):
        ca, sa, ct, st = math.cos(al), math.sin(al), math.cos(th+off), math.sin(th+off)
        A = [[ct, -st, 0, a], [st*ca, ct*ca, -sa, -sa*d], [st*sa, ct*sa, ca, ca*d], [0, 0, 0, 1]]
        T = [[sum(T[i][k]*A[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
    return T[0][3], T[1][3], T[2][3]

a = c = None
for _ in range(10):
    a = mc.get_angles()
    time.sleep(0.3)
    c = mc.get_coords()
    if a and c:
        break
    time.sleep(0.5)

print("angulos (°):", a)
print("coords     :", c)

if a and c:
    x, y, z = fk([v*PI/180 for v in a])
    err = math.sqrt((x-c[0])**2 + (y-c[1])**2 + (z-c[2])**2)
    print("FK propia  :", [round(v, 1) for v in (x, y, z)])
    print("ERROR: %.1f mm  %s" % (err, "OK" if err <= 10 else "> 10 mm"))
    print("Joint 3 (codo): %.1f°" % a[2])
else:
    print("El brazo no respondió.")
