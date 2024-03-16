import matplotlib.pyplot as plt
import numpy as np

def load_text_file(file_path, n, m, load_q=False, load_dq=False, load_qd=False, load_tau=False, load_x=False, load_dx=False, load_xd=False):
    time = []
    time_0 = None
    data = {'q': [], 'dq': [], 'qd': [], 'tau': [], 'x': [], 'dx': [], 'xd': []}

    with open(file_path, 'r') as file:
        for line in file:
            values = line.strip().split('\t')  # Split by tabs

            if time_0 is None:
                time_0 = float(values[0])

            time.append(float(values[0])-time_0)

            if load_q:
                data['q'].append([float(values[i]) for i in range(1, n + 1)])
            if load_dq:
                data['dq'].append([float(values[i]) for i in range(n + 1, 2 * n + 1)])
            if load_qd:
                data['qd'].append([float(values[i]) for i in range(2 * n + 1, 3 * n + 1)])
            if load_tau:
                data['tau'].append([float(values[i]) for i in range(3 * n + 1, 4 * n + 1)])
            if load_x:
                data['x'].append([float(values[i]) for i in range(4 * n + 1, 4 * n + 1 + m)])
            if load_dx:
                data['dx'].append([float(values[i]) for i in range(4 * n + 1 + m, 4 * n + 1 + 2 * m)])
            if load_xd:
                data['xd'].append([float(values[i]) for i in range(4 * n + 1 + 2 * m, 4 * n + 1 + 3 * m)])

    return time, data

def subplot_data(x, y, yd, ys, x_label, yd_label, y_label, ys_label, x_unit, y_unit, ys_unit, title):
    plt.figure()
    plt.subplot(2, 1, 1)

    for i in range(len(y[0])):
        plt.plot(x, [y[j][i] for j in range(len(y))], label=f'{y_label} {i+1}')

    for i in range(len(yd[0])):
        plt.plot(x, [yd[j][i] for j in range(len(yd))], label=f'{yd_label} {i+1}', linestyle='dotted', alpha=0.7)
    

    plt.xlabel(f'{x_label} in {x_unit}')
    plt.ylabel(f'{y_label} in {y_unit}')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    
    # Plot the additional data (ys) on the lower subplot
    for i in range(len(ys[0])):
        plt.plot(x, [ys[j][i] for j in range(len(ys))], label=f'{ys_label} {i+1}')
    
    plt.xlabel(f'{x_label} in {x_unit}')
    plt.ylabel(f'{ys_label} in {ys_unit}')
    plt.legend()
    plt.grid(True)

    plt.suptitle(title)
    plt.tight_layout()


# file_path = "D:\PROJECT\\robotics\A1\\njmove_400_200_10.txt"

# time, data = load_text_file(file_path, n=3, m=3, load_q=True, load_qd=True, load_tau=True)

# subplot_data(time, data['q'], data['qd'], data['tau'], 't', 'q', 'qd', 'tau', 's', 'rad', 'Nm', 'njmoveControl with kp=[400, 200, 10]')


# file_path = "D:\PROJECT\\robotics\A1\\njgoto_400_200_10.txt"

# time, data = load_text_file(file_path, n=3, m=3, load_q=True, load_qd=True, load_tau=True)

# subplot_data(time, data['q'], data['qd'], data['tau'], 't', 'q', 'qd', 'tau', 's', 'rad', 'Nm','njgotoControl with kp=[400, 200, 10]')


file_path = "D:\PROJECT\\robotics\A1\jgoto_(700,400,50)_(400,400,40).txt"

time, data = load_text_file(file_path, n=3, m=3, load_q=True, load_qd=True, load_tau=True)

subplot_data(time, data['q'], data['qd'], data['tau'], 't', 'q', 'qd', 'tau', 's', 'rad', 'Nm','jgotoControl with kp=[700, 400, 50], kv=[400, 400, 40]')


plt.show()