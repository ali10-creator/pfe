import pandas as pd
import matplotlib.pyplot as plt
import time
import os
import signal
import sys

def main():
    log_file = '/tmp/smc_cmd_log.csv'   # Change this path if needed

    plt.ion()
    fig, axs = plt.subplots(4, 1, figsize=(11, 12))

    # --- Velocity plots
    line1, = axs[0].plot([], [], label='v [cmd]', color='tab:blue')
    line1_actual, = axs[0].plot([], [], label='v_actual', color='tab:cyan')
    axs[0].set_ylabel('Linear velocity [m/s]')
    axs[0].legend()
    axs[0].grid()

    line2, = axs[1].plot([], [], label='omega [cmd]', color='tab:green')
    line2_actual, = axs[1].plot([], [], label='omega_actual', color='tab:olive')
    axs[1].set_ylabel('Angular velocity [rad/s]')
    axs[1].legend()
    axs[1].grid()

    # --- Error plots
    line_err_v, = axs[2].plot([], [], label='Linear velocity error', color='tab:red')
    line_err_omega, = axs[2].plot([], [], label='Angular velocity error', color='tab:orange')
    axs[2].set_ylabel('Velocity Errors')
    axs[2].legend()
    axs[2].grid()

    # --- Position error
    line_pos_err, = axs[3].plot([], [], label='Position error', color='tab:purple')
    axs[3].set_ylabel('Position Error [m]')
    axs[3].set_xlabel('Time [s]')
    axs[3].legend()
    axs[3].grid()

    fig.tight_layout()

    last_file_size = 0

    def reset_plot():
        for line in [line1, line1_actual, line2, line2_actual, line_err_v, line_err_omega, line_pos_err]:
            line.set_data([], [])
        for ax in axs:
            ax.relim(); ax.autoscale_view()
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

        # Accepts both position_error or pos_err as column
        pos_err_col = None
        for candidate in ['position_error', 'pos_err']:
            if candidate in df.columns:
                pos_err_col = candidate
                break

        # Check column existence
        required = ['time', 'v', 'v_actual', 'omega', 'omega_actual']
        if not all(col in df.columns for col in required):
            print("[WARNING] Missing columns in log. Columns found:", df.columns)
            return

        # Errors
        df['v_err'] = df['v'] - df['v_actual']
        df['omega_err'] = df['omega'] - df['omega_actual']

        # --- Plot data
        line1.set_data(df['time'], df['v'])
        line1_actual.set_data(df['time'], df['v_actual'])
        axs[0].set_xlim(0, df['time'].max() + 0.1)
        y1 = pd.concat([df['v'], df['v_actual']])
        axs[0].set_ylim(y1.min() - 0.05, y1.max() + 0.05)

        line2.set_data(df['time'], df['omega'])
        line2_actual.set_data(df['time'], df['omega_actual'])
        axs[1].set_xlim(0, df['time'].max() + 0.1)
        y2 = pd.concat([df['omega'], df['omega_actual']])
        axs[1].set_ylim(y2.min() - 0.05, y2.max() + 0.05)

        line_err_v.set_data(df['time'], df['v_err'])
        line_err_omega.set_data(df['time'], df['omega_err'])
        axs[2].set_xlim(0, df['time'].max() + 0.1)
        y3 = pd.concat([df['v_err'], df['omega_err']])
        axs[2].set_ylim(y3.min() - 0.05, y3.max() + 0.05)

        if pos_err_col is not None:
            line_pos_err.set_data(df['time'], df[pos_err_col])
            axs[3].set_xlim(0, df['time'].max() + 0.1)
            axs[3].set_ylim(df[pos_err_col].min() - 0.05, df[pos_err_col].max() + 0.05)
        else:
            line_pos_err.set_data([], [])
            axs[3].set_xlim(0, 1)
            axs[3].set_ylim(-0.1, 0.1)

        fig.suptitle("SMC: Commanded vs Actual Velocities, Errors, and Position Error")
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
