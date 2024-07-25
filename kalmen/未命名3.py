# -*- coding: utf-8 -*-
"""
Created on Sun Jul 14 14:41:01 2024

@author: admin
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pykalman import KalmanFilter
from pykalman import UnscentedKalmanFilter

data = pd.read_csv('./source/stock_prices.csv', sep=',')

x = data["Day"]
y = data["Price"]


ukf = UnscentedKalmanFilter(lambda x, w: x + w, lambda x, v: x + v, 
                            transition_covariance=0.1, 
                            initial_state_mean = y.head().mean())

(filtered_state_means, filtered_state_covariances) = ukf.filter(y)
(smoothed_state_means, smoothed_state_covariances) = ukf.smooth(y)


plt.figure()
plt.plot(x, y, label='Data 1')
plt.plot(x, filtered_state_means, label='Filtered data 4')
#plt.plot(x, filtered_state_covariances.squeeze(), label='filtered_state_covariances')
plt.legend()

plt.figure()
plt.plot(x, y, label='Data 1')
plt.plot(x, smoothed_state_means, label='smoothed data 4')
#plt.plot(x, smoothed_state_covariances.squeeze(), label='smoothed_state_covariances')
plt.legend()
