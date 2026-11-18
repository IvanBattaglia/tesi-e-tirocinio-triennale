import sys
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Backend senza schermo (perfetto per Docker senza X11)
import matplotlib.pyplot as plt

CSV_FILE = sys.argv[1] if len(sys.argv) > 1 else "pid_test.csv"
OUT_IMG  = sys.argv[2] if len(sys.argv) > 2 else "pid_analisi.png"
DT = 0.020  # Periodo di campionamento: 50 Hz (20 ms)

# 1. Lettura e pulizia del CSV
try:
    df = pd.read_csv(CSV_FILE, header=None)
except pd.errors.EmptyDataError:
    print(f"ERRORE: Il file '{CSV_FILE}' è vuoto! Registra prima i dati con ros2 topic echo.")
    sys.exit(1)

# Rimuove eventuali righe incomplete o intestazioni testuali
df = df.apply(pd.to_numeric, errors='coerce').dropna()

if df.shape[1] < 6 or len(df) == 0:
    print(f"ERRORE: Formato CSV non valido (colonne trovate: {df.shape[1]}, righe: {len(df)}).")
    sys.exit(1)

# Conversione esplicita in array NumPy 1D per compatibilità con Matplotlib di sistema
data = df.iloc[:, :6].to_numpy(dtype=float)
target_sx = data[:, 0]
reale_sx  = data[:, 1]
pwm_sx    = data[:, 2]
target_dx = data[:, 3]
reale_dx  = data[:, 4]
pwm_dx    = data[:, 5]
tempo     = np.arange(len(data), dtype=float) * DT

# 2. Calcolo metriche diagnostiche sul primo gradino attivo
def analizza_motore(target, reale, pwm, t):
    attivo = np.where(np.abs(target) > 0.05)[0]
    if len(attivo) == 0:
        return "Nessun comando > 0.05 rad/s rilevato"

    i_start = attivo[0]
    i_end = attivo[-1]
    t_sub = t[i_start:i_end+1] - t[i_start]
    y_sub = reale[i_start:i_end+1]
    sp = target[i_start]

    # Overshoot %
    if sp > 0:
        picco = np.max(y_sub)
        overshoot = max(0.0, (picco - sp) / sp * 100.0)
    else:
        picco = np.min(y_sub)
        overshoot = max(0.0, (sp - picco) / abs(sp) * 100.0)

    # Tempo di salita (al 90% del target)
    raggiunto = np.where(np.abs(y_sub) >= 0.9 * abs(sp))[0]
    t_salita = f"{t_sub[raggiunto[0]]:.2f} s" if len(raggiunto) > 0 else "Non raggiunto"

    # Errore a regime (ultima metà dell'impulso)
    meta = len(y_sub) // 2
    err_regime = np.mean(np.abs(target[i_start+meta:i_end+1] - y_sub[meta:])) if meta > 2 else np.nan

    # Percentuale di saturazione PWM
    sat = np.mean(np.abs(pwm[i_start:i_end+1]) >= 250) * 100.0

    return (f"Target: {sp:.2f} rad/s | Salita (90%): {t_salita} | "
            f"Overshoot: {overshoot:.1f}% | Err. regime: {err_regime:.3f} rad/s | Sat. PWM: {sat:.0f}%")

stat_sx = analizza_motore(target_sx, reale_sx, pwm_sx, tempo)
stat_dx = analizza_motore(target_dx, reale_dx, pwm_dx, tempo)

# 3. Creazione dei grafici (2 colonne: Motore SX e Motore DX; 2 righe: Velocità e PWM)
fig, axes = plt.subplots(2, 2, figsize=(15, 9), sharex=True)
fig.suptitle("Analisi Risposta PID Controllo Velocità (50 Hz)", fontsize=14, fontweight='bold')

# --- MOTORE 1 (SINISTRO) ---
axes[0, 0].plot(tempo, target_sx, 'k--', linewidth=1.8, label='Setpoint SX [rad/s]')
axes[0, 0].plot(tempo, reale_sx, color='#1f77b4', linewidth=1.5, label='Velocità Reale SX [rad/s]')
axes[0, 0].set_title(f"Motore 1 (SX) - Velocità\n{stat_sx}", fontsize=9.5)
axes[0, 0].set_ylabel("Velocità angolare [rad/s]")
axes[0, 0].grid(True, linestyle=':', alpha=0.7)
axes[0, 0].legend(loc='upper right')

axes[1, 0].plot(tempo, pwm_sx, color='#d62728', linewidth=1.3, label='Uscita PID (PWM SX)')
axes[1, 0].axhline(255, color='gray', linestyle='--', linewidth=0.9, label='Limite PWM (±255)')
axes[1, 0].axhline(-255, color='gray', linestyle='--', linewidth=0.9)
axes[1, 0].set_title("Motore 1 (SX) - Sforzo di Controllo (PWM)", fontsize=10)
axes[1, 0].set_xlabel("Tempo [s]")
axes[1, 0].set_ylabel("PWM [-255 .. +255]")
axes[1, 0].set_ylim(-275, 275)
axes[1, 0].grid(True, linestyle=':', alpha=0.7)
axes[1, 0].legend(loc='upper right')

# --- MOTORE 2 (DESTRO) ---
axes[0, 1].plot(tempo, target_dx, 'k--', linewidth=1.8, label='Setpoint DX [rad/s]')
axes[0, 1].plot(tempo, reale_dx, color='#2ca02c', linewidth=1.5, label='Velocità Reale DX [rad/s]')
axes[0, 1].set_title(f"Motore 2 (DX) - Velocità\n{stat_dx}", fontsize=9.5)
axes[0, 1].set_ylabel("Velocità angolare [rad/s]")
axes[0, 1].grid(True, linestyle=':', alpha=0.7)
axes[0, 1].legend(loc='upper right')

axes[1, 1].plot(tempo, pwm_dx, color='#ff7f0e', linewidth=1.3, label='Uscita PID (PWM DX)')
axes[1, 1].axhline(255, color='gray', linestyle='--', linewidth=0.9, label='Limite PWM (±255)')
axes[1, 1].axhline(-255, color='gray', linestyle='--', linewidth=0.9)
axes[1, 1].set_title("Motore 2 (DX) - Sforzo di Controllo (PWM)", fontsize=10)
axes[1, 1].set_xlabel("Tempo [s]")
axes[1, 1].set_ylabel("PWM [-255 .. +255]")
axes[1, 1].set_ylim(-275, 275)
axes[1, 1].grid(True, linestyle=':', alpha=0.7)
axes[1, 1].legend(loc='upper right')

plt.tight_layout()
plt.savefig(OUT_IMG, dpi=180)
print(f"Grafico salvato con successo in: {OUT_IMG}")
print(f"SX -> {stat_sx}")
print(f"DX -> {stat_dx}")