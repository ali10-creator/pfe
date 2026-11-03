#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import time
import os
import signal
import sys

def main():
    log_file = '/tmp/pid_cmd_log.csv'

    plt.ion()
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 12))

    # Linear velocities (subplot 1)
    line_cmd_v, = ax1.plot([], [], label='Command v [m/s]', color='tab:blue')
    line_act_v, = ax1.plot([], [], label='Actual v [m/s]', linestyle='--', color='tab:orange')

    # Angular velocities (subplot 2)
    line_cmd_omega, = ax2.plot([], [], label='Command omega [rad/s]', color='tab:green')
    line_act_omega, = ax2.plot([], [], label='Actual omega [rad/s]', linestyle='--', color='tab:red')

    # Errors (subplot 3)
    line_v_err, = ax3.plot([], [], label='Linear Error (v - v_actual)', color='tab:purple')
    line_omega_err, = ax3.plot([], [], label='Angular Error (omega - omega_actual)', color='tab:brown')

    # Position error (subplot 4)
    line_pose_err, = ax4.plot([], [], label='Position Error (Euclidean)', color='tab:pink')

    ax1.set_ylabel('Linear Velocity (v)')
    ax2.set_ylabel('Angular Velocity (omega)')
    ax3.set_ylabel('Velocity Error')
    ax4.set_ylabel('Position Error [m]')
    ax4.set_xlabel('Time [s]')
    for ax in [ax1, ax2, ax3, ax4]:
        ax.grid()
        ax.legend()

    last_file_size = 0

    def reset_plot():
        for line in [line_cmd_v, line_act_v, line_cmd_omega, line_act_omega, line_v_err, line_omega_err, line_pose_err]:
            line.set_data([], [])
        for ax in [ax1, ax2, ax3, ax4]:
            ax.relim()
            ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()
        print("[INFO] Plot reset due to log file being cleared.")

    def update():
        nonlocal last_file_size

        if not os.path.exists(log_file):
            return

        current_size = os.path.getsize(log_file)
        if current_size < last_file_size:
            reset_plot()

        last_file_size = current_size

        try:
            df = pd.read_csv(log_file)
        except pd.errors.EmptyDataError:
            return

        required_cols = ['v', 'v_actual', 'omega', 'omega_actual', 'x', 'y', 'xd', 'yd']
        if df.shape[0] < 2 or not all(col in df.columns for col in required_cols):
            return

        # Calculate errors
        df['v_err'] = df['v'] - df['v_actual']
        df['omega_err'] = df['omega'] - df['omega_actual']
        df['pose_err'] = ((df['x'] - df['xd'])**2 + (df['y'] - df['yd'])**2).pow(0.5)

        # Update plot data, divide actual velocities by 2
        line_cmd_v.set_data(df['time'], df['v'])
        line_act_v.set_data(df['time'], df['v_actual'] / 2)  # Divide actual velocity by 2
        line_cmd_omega.set_data(df['time'], df['omega'])
        line_act_omega.set_data(df['time'], df['omega_actual'] / 2)  # Divide actual angular velocity by 2
        line_v_err.set_data(df['time'], df['v_err'])
        line_omega_err.set_data(df['time'], df['omega_err'])
        line_pose_err.set_data(df['time'], df['pose_err'])

        # Rescale axes
        ax1.set_xlim(0, df['time'].max() + 0.1)
        ax2.set_xlim(0, df['time'].max() + 0.1)
        ax3.set_xlim(0, df['time'].max() + 0.1)
        ax4.set_xlim(0, df['time'].max() + 0.1)

        ax1.set_ylim(min(df['v'].min(), df['v_actual'].min()) - 0.05,
                     max(df['v'].max(), df['v_actual'].max()) + 0.05)
        ax2.set_ylim(min(df['omega'].min(), df['omega_actual'].min()) - 0.05,
                     max(df['omega'].max(), df['omega_actual'].max()) + 0.05)
        ax3.set_ylim(min(df['v_err'].min(), df['omega_err'].min()) - 0.05,
                     max(df['v_err'].max(), df['omega_err'].max()) + 0.05)
        ax4.set_ylim(df['pose_err'].min() - 0.05, df['pose_err'].max() + 0.05)

        fig.suptitle("PID: Command vs Actual Velocities + Error + Position Error (Live Plot)")
        fig.canvas.draw()
        fig.canvas.flush_events()

    def signal_handler(sig, frame):
        print("\n[INFO] Plotting interrupted. Exiting.")
        plt.ioff()
        plt.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    while True:
        update()
        time.sleep(0.5)

if __name__ == '__main__':
    main()
