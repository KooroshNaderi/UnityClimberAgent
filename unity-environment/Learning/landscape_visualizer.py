import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d
from sklearn.decomposition import PCA

mode = 1  # 1: Construct PCA Space, 2: Landscape
base_dir = os.path.join(os.getcwd(), 'data')
# print(base_dir)
size_grid = [300, 200]


def zero_mean(data):
    mean = np.mean(data, axis=0)
    # print(mean)
    return data - mean


def construct_pca_space():
    # file_address = os.path.join(base_dir, '2018.05.07-12.08.49-BestControlPoints.csv')
    file_address = os.path.join(base_dir, '2018.05.08-13.32.19-BestControlPoints.csv')
    data_header = np.genfromtxt(file_address, delimiter=',', max_rows=1, dtype=np.float32)
    point_size = int(data_header[0] * (data_header[1] + 1))
    data = np.genfromtxt(file_address, delimiter=',', skip_header=2, dtype=np.float32)
    data_mean = np.mean(data, axis=0)
    pca = PCA(n_components=point_size)
    pca_result = pca.fit_transform(X=data)
    print(pca_result.shape)

    reconstructed_data = np.dot(pca_result, pca.components_) + data_mean
    print(reconstructed_data.shape)
    diff = np.power(reconstructed_data - data, 2)
    diff = np.sum(diff, axis=1)
    diff = np.sqrt(diff)
    diff = np.sum(diff)
    print(diff)

    plt.plot(pca_result[:, 0], pca_result[:, 1], 'x')
    # print(pca.components_.shape)
    # new_data = np.dot(zero_mean(data), pca.components_.T)
    # plt.plot(new_data[:, 0], new_data[:, 1], '+')

    axis_data = np.concatenate((np.reshape(data_mean, (1, -1)), pca.components_), axis=0)
    np.savetxt(os.path.join(base_dir, 'AxisData.csv'), axis_data, delimiter=' '
               , header='First line is the data mean and other lines store PCA components')
    plt.show()


def visualize_landscape():
    file_address = os.path.join(base_dir, 'gridCosts.csv')
    data = np.genfromtxt(file_address, delimiter=',', dtype=np.float32)  # , max_rows=10)
    tasks = data[:, 0]
    data = data[:, 1:]
    x = np.arange(0, size_grid[0], 1)
    y = np.arange(0, size_grid[1], 1)
    x, y = np.meshgrid(y, x)
    task_strings = ['NONE', 'PUNCH_RIGHT', 'PUNCH_LEFT']
    for f in range(data.shape[0]):
        z = data[f:f+1, :]
        z = np.reshape(z, size_grid)
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_zlim(0, 150)
        ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=cm.inferno, linewidth=0, antialiased=False)
        # plt.title('Task: %s' % task_strings[int(tasks[f])])
        fig.savefig(os.path.join(os.path.join(base_dir, 'Screenshots'), 'frame%06d.png' % f))
        # plt.show()
        plt.close(fig)


if __name__ == '__main__':
    if mode == 1:
        construct_pca_space()
    elif mode == 2:
        visualize_landscape()
