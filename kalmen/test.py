import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pykalman import KalmanFilter

# 加载数据
data1 = pd.read_csv('./source/stock_prices.csv', sep=',')

# 添加随机噪声，这里假设噪声遵循标准正态分布
noise = np.random.normal(0, 1, size=data1['Price'].shape)
data1_noisy = data1['Price'] + noise

# 定义Kalman滤波器参数
transition_matrix = np.array([[1, 0], [0, 1]])
observation_matrix = np.array([[1, 0]])
transition_covariance = 0.1 * np.eye(2)  # 可根据实际情况调整
observation_covariance = 1  # 噪声较大时可能需要增加
initial_state_mean = [100, 0]
initial_state_covariance = np.ones((2, 2))

# 实例化KalmanFilter
kf = KalmanFilter(
    transition_matrices=transition_matrix,
    observation_matrices=observation_matrix,
    transition_covariance=transition_covariance,
    observation_covariance=observation_covariance,
    initial_state_mean=initial_state_mean,
    initial_state_covariance=initial_state_covariance
)

# 对带噪声的数据进行滤波
filtered_state_means, _ = kf.filter(data1_noisy)

# 绘制原始数据、带噪声的数据和滤波后的数据
plt.figure()
plt.plot(data1['Day'], data1['Price'], 'g-', label='Original data')
plt.plot(data1['Day'], data1_noisy, 'r.', label='Noisy data')
plt.plot(data1['Day'], filtered_state_means[:, 0], 'b-', label='Filtered data')
plt.title('Data with Noise and Filtered Data')
plt.legend()
plt.show()

