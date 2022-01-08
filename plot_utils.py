import os

import numpy as np


def open_log(work_dir):
    chunks = os.listdir(save_dir + "/buffer")
    chucks = sorted(chunks, key=lambda x: int(x.rex('_')[0]))
    file = f'RL/{work_dir}/eval_scores.npy'
    x = np.load(file, allow_pickle=True)


if __name__ == '__main__':
    open_log()