# -*- coding: utf-8 -*-
"""
Created on Sun Jul 14 11:15:04 2024

@author: admin
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

random_state = np.random.RandomState(0)

data1 = pd.read_csv('./source/homework_data_1.txt', sep=' ')
data2 = pd.read_csv('./source/homework_data_2.txt', sep=' ')
data3 = pd.read_csv('./source/homework_data_3.txt', sep=' ')
data4 = pd.read_csv('./source/homework_data_4.txt', sep=' ')


from pykalman import KalmanFilter
from pykalman import UnscentedKalmanFilter

# 定义转移矩阵
def transition_function(state, noise):
    a = np.sin(state[0]) + state[1] * noise[0]
    b = state[1] + noise[1]
    return np.array([a, b])

# 定义观测矩阵
def observation_function(state, noise):
    C = np.array([[-1, 0.5], [0.2, 0.1]])
    return np.dot(C, state) + noise

    

# 定义过程噪声协方差
transition_covariance = 0.1 * np.eye(2)

# 定义观测噪声协方差
observation_covariance = np.eye(2) + random_state.randn(2, 2) * 0.1

# 定义初始状态
initial_state_mean = [0, 0]
initial_state_covariance = np.ones((2, 2))

# 实例化KalmanFilter
kf = UnscentedKalmanFilter(
    transition_function,
    observation_function,
    transition_covariance,
    observation_covariance,
    initial_state_mean,
    initial_state_covariance
)

# 使用KalmanFilter进行拟合

filtered_state_means1, _ = kf.filter(data1['y'])
filtered_state_means2, _ = kf.filter(data2['y'])
filtered_state_means3, _ = kf.filter(data3['y'])
filtered_state_means4, _ = kf.filter(data4['y'])

# 绘制拟合结果
plt.figure()
plt.plot(data1['x'], data1['y'], 'o', label='Data 1')
plt.plot(data1['x'], filtered_state_means1[:, 0], label='Filtered data 1')

plt.figure()
plt.plot(data4['x'], data4['y'], 'o', label='Data 4')
plt.plot(data4['x'], filtered_state_means4[:, 0], label='Filtered data 4')

plt.figure()
plt.plot(data2['x'], data2['y'], 'o', label='Data 2')
plt.plot(data2['x'], filtered_state_means2[:, 0], label='Filtered data 2')

plt.figure()
plt.plot(data3['x'], data3['y'], 'o', label='Data 3')
plt.plot(data3['x'], filtered_state_means3[:, 0], label='Filtered data 3')

plt.legend()
plt.show()