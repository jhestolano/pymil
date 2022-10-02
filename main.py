from ctypes import *
import matplotlib.pyplot as plt
import math

def run_sim(libpid):

    # Simulation variables.
    ti = 0. # Initial time.
    tf = 2. # Final time.
    ts = 0.001 # Step / sampling time.
    t = ti # Actual simulation time.

    # Motor parameters. The motor is modeled
    # as a first order low pass filter. Which
    # is a simplistic model but useful to quickly
    # test controller ideas.
    K = 145.47 # Motor DC gain.
    tau = 0.087 # Motor time constant.
    w = 0. # Motor actual speed.
    wr = 100.0 # Controller reference speed.

    # Simulation output variables.
    W = [w] # Speed vector.
    T = [t] # Time vector.
    U = [0.] # Input voltage vector.
    Wr = [wr] # Reference speed vector.

    # Controller gain tuning parameters.
    Ts = 0.2 # Settling time (desired).
    Pos = 0.05 # Max. Overshoot (desired).

    # Closed loop system becomes second order.
    # Damping ratio and natural frequency are used to get
    # some approximate gains.
    z = abs(math.log(Pos)) / math.sqrt((math.log(Pos))**2 + math.pi**2)
    wn = 4 / (z * Ts)
    kp = (2 * z * wn * tau - 1) / K
    ki = (tau * wn ** 2) / K
    kd = 0.

    # Initialize embedded C code.
    # This is the actual pid_init function defined
    # in pid.c file.
    libpid.pid_init(kp, ki, kd, ts)

    # Start simulation loop.
    while t < tf:

       # Calculate embedded C pid controller output.
       Vin = libpid.pid_run(wr, w)

       # Calculate motor dynamics.
       dw = (K * Vin - w) / tau

       # ODE-1 integration step.
       w += dw * ts
       t += ts

       W.append(w)
       U.append(Vin)
       Wr.append(wr)
       T.append(t)

    # Plot simulation results.
    fig, ax = plt.subplots()
    ax2 = ax.twinx()

    ax.plot(T, W, linewidth = 2., color = 'red')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (rad / s)')
    ax.tick_params(axis = 'y', labelcolor = 'red')

    ax2.plot(T, U, linewidth = 2., color = 'blue')
    ax2.set_ylabel('Voltage (V)')
    ax2.tick_params(axis = 'y', labelcolor = 'blue')

    plt.title('Motor Response.')
    plt.grid()
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    libpid_fname = './libpid.so'
    libpid = CDLL(libpid_fname)
    libpid.pid_init.argtypes = [c_float, c_float, c_float, c_float]
    libpid.pid_init.restype = None
    libpid.pid_run.argtypes = [c_float, c_float]
    libpid.pid_run.restype = c_float


    run_sim(libpid)
