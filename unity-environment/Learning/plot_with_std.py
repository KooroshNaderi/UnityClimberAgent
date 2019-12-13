'''
Created on Oct 4, 2017

@author: amin
'''

from __future__ import division, print_function
from matplotlib import pyplot as plt
import numpy as np
import matplotlib
import os

# Set some parameters to apply to all plots. These can be overridden
# in each plot if desired
# Plot size to 14" x 7"
matplotlib.rc('figure', figsize=(14, 7))
# Font size to 14
matplotlib.rc('font', size=14)
# Do not display top and right frame lines
matplotlib.rc('axes.spines', top=False, right=False)
# Remove grid lines
matplotlib.rc('axes', grid=False)
# Set background color to white
matplotlib.rc('axes', facecolor='white')

dir_list = [
    'baseline_no_learning',
]
dir_size = 10
plot_whole_data = False
legends = []
for d in range(len(dir_list)):
    data = None
    for root, dirs, file_names in os.walk(os.path.join('results', dir_list[d])):
        for f in range(dir_size):
            new_data = np.genfromtxt(os.path.join(root, file_names[f]))
            if data is None:
                data = np.zeros((new_data.shape[0], dir_size))
            data[:, f] = new_data
    mean = np.mean(data, axis=1)
    std = np.std(data, axis=1)
    if plot_whole_data:
        p = plt.plot(list(range(data.shape[0])), mean)
        plt.fill_between(list(range(data.shape[0])), mean - std / 2, mean + std / 2, color=p[0].get_color(), alpha=0.3)
    else:
        step = 1000
        idx = np.arange(0, data.shape[0], step)
        mean_smooth = np.zeros(len(idx))
        std_smooth = np.zeros(len(idx))
        for i in range(len(idx)):
            mean_smooth[i] = np.mean(mean[i * step: (i + 1) * step])
            std_smooth[i] = np.mean(std[i * step: (i + 1) * step])
        p = plt.plot(idx, mean_smooth)
        # plt.fill_between(idx, mean_smooth-std_smooth/2, mean_smooth+std_smooth/2, color=p[0].get_color(), alpha=0.3)
    legends.append(dir_list[d][dir_list[d].rfind('/') + 1:])

plt.legend(legends)
plt.title('State cost + control cost in 10 minutes of simulation (bipedal walking task)')
plt.xlabel('Frame index')
plt.ylabel('State Cost + Control Cost')
# plt.yscale('log')
plt.show()
