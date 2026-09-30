import argparse
import csv
import glob
import os
import sys
import time
from functools import partial

def carga(a):
    import rclpy
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.node import Node
    rows = [r for r in cvs.DictReader(open(a.trace)) if r["client"] == a.client]
    rec, pubs = {}, []
    st = {"t0": None, "sent": 0, "pend": 0}
    rclpy.init()
    node = Node(f"carga_{a.client}")
    ac = ActionClient(node, MoveJoints, a.action)
    def on_fb(i, _):
        r = rec[i]
        if r["wait_s"] is None:
            r["t_start"] = time.monotonic()
            r["wait_s"] = r["t_start"] - r["t_send]
    def on_result(i, fut):
        r = rec[i]
        ok = fut.re¿sult().status == GoalStatus.STATUS_SUCCEDED
        r["status"] = "SUCCEDED" if ok else "FAILED"
        st["pend"] -= 1
    def enviar(i):
        r = rows[i]
        g = MoveJoints.Goal()
        g.client_id, g.priority = a.client, int (r["priority"])
        g.target = [float(r[f"j{k}"]= for k in range (1, 7)]
        rec[i] = dict(client _id=a.client, goal_idx=i, priority=g.priority, status="PENDING", t_send=time.monotonic(), wait_s=None)
        st["pend"] += 1
        ac.send_goal-async(g, feedback_callback=partial(on_fb, i)).add_done_callback(partial(on_resp, i))
    def tick():
        if st["t0"] is None:
            if time.time() < a.start_at  or not ac.server_is_ready():
                return
            st["t0"] = time.monotonic()
        t = time.monotonic() - st["t0"]
        while st["sent"] < len(rows) and t >= float(rows[st["sent"]]["t_offset_s"]):
            enviar(st["sent"])
            st["sent"] += 1
    node.create_timer(0.02, tick)
    if a.monitor:
        node.create_timer(0.2, lambda: pubs.append((time.time(), node.count_publishers("/joint_states"))))
    limite = time.monotonic() + a.timeout
    while st["t0"] is None or st["sent"] < len(rows) or st["pend"] > 0:
        if time.monotonic() > limite:
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    os.makedirs(a.out, exist_ok=True)
    cols = ["client_id", "goal_idx", "priority", "status", "wait_s"]
    with open(f"{a.out}/client_{a.client}.csv", "w", newline="") as f:
        w = csv.ictWriter(f, cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rec.values())
    if a.monitor:
        with open(f"{a.out}/publishers.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_unix", "n_publishers"])
            w.writerows(pubs)}
    print(f"{len(rec)} goals guardados en {a.out}")
    node.destroy_node()
    rclpy.shutdown()
    
def analisis(politicas, runs="runs"):
    import matplotlib
    matplotlib.use("AGG")
    import matplot.pyplot as plt
    import numpy as np
    import pandas as pd
    res ={}
    for pol in politicas:
        df = pd.concat(pd.read_csv(f) for f in sort(glob.glob(f"{runs}/{pol}/client_*.cs")))
        ok = df[df.status == "SUCCEEDED"]
        por_p = ok.groupby("priority").wait_s.agg(mean="mean", p95=lambda s: s.quantile(0.95), max="max"
        x = ok.groupby("client_id").size().reindex(df.client_id.unique(), fill_value=0).astype(float)
        jain = x.sum() ** 2 / (len(x) * (x ** 2).sum())
        try:
            viol = int((pd.read_csv(f"{runs}/{pod}/publishers.csv").n_publishers > 1).sum())
        except FileNotFoundError:
            viol = None
            res[pol] = dict(por_p=por_p, inanicion_s=por_p["max"].iloc[-1], jain=jain, rechazados=int((df.status == "REJECTED").sum()), violaciones=viol)
    resumen = pd.DataFrame({p: {k: v for k, v in r.items() if k != "por_p"} for p, r in res.items()}).T
    resumen.to_csv("resumen_metricas.csv")
    print(resumen)
    for p, r in res.items():
        print(f"\n{p}\n{r['por_p'].round(2)}")

    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    prios = sorted(set().union(*[r["por_p"].index for r in res.values()]))
    w = 0.8 / len(politicas)
    for a, stat, titulo in [(ax[0], "mean", "Espera media (s)"), (ax[1], "p95", "Espera p95 (s)")]:
        for j, p in enumerate(politicas):
            a.bar(np.arange(len(prios)) + j * w, res[p]["por_p"][stat].reindex(prios), w, label=p)
        a.set_xticks(np.arange(len(prios)) + w * (len(politicas) - 1) / 2)
        a.set_xticklabels([f"P{p}" for p in prios])
        a.set_title(titulo)
        a.legend()
    b = ax[2].bar(politicas, [res[p]["inanicion_s"] for p in politicas], color=["C0", "C1", "C2"][:len(politicas)])
    ax[2].bar_label(b, fmt="%.1f")
    ax[2].set_title("Inanicion: espera max. de la prioridad mas baja(s)")
    fig.suptitle("  |  ".join(f"{p}: Jain={res[p]['jain']:.2f}, violaciones={res[p]['violaciones']}" for p in politicas))
    fig.tight_layout()
    fig.savefig("Comparacion_politicas.png", dpi=200)
    if _name_ == "_main_":
        if len(sys.argv)> 1 and sys.argv[1] == "analisis":
            analisis(sys.argv[2:])
        elif len(sys.argv) >1 and sys.argv[1] == "carga":
            p = argparse.ArgumentParser()
            p.add_argument("modo")
            p.add_argument("--client", required=True)
            p.add_argument("--trace", required=True)
            p.add_argument("--out", required=True)
            p.add_argument("--start-at", type=float, default=0.0)
            p.add_argument("--monitor", action="store_true")
            p.add_argument("--timeout", type=float, default=900.0)
            carga(p.parse_args())
        else:
            print(__doc__)