import shelve

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

sns.set_theme()
from constants import velocity_constraint


def plot_dq_constraints(state):
    ncols, nrows = 1, 3
    fig, matrix_axs = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3 * nrows), tight_layout=True)
    matrix_axs = np.array(matrix_axs).reshape((nrows, ncols))
    t = state[:, 0]
    for i in range(3):
        matrix_axs[i,0].hlines(np.array([-velocity_constraint[i], velocity_constraint[i]]), t[0], t[-1], label=f'dq{i+1} constraint', linestyle="--", color="r")
        matrix_axs[i,0].plot(t, state[:, 4+i], label=f'dq{i+1}')

    for ax in matrix_axs.flatten():
        ax.legend()
    plt.savefig("dq_constraints.png")


if __name__ == '__main__':
    global_db = shelve.open("cache")
    state = global_db['state']
    plot_dq_constraints(state)
    plt.show()