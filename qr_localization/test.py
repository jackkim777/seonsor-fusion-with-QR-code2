import numpy as np
import matplotlib.pyplot as plt

# 예제 가속도 데이터 (m/s^2)
acceleration_data = np.array([0.0, 0.1, 0.2, 0.3, 0.2, 0.1, 0.0])
# 샘플링 시간 간격 (초)
delta_t = 1.0

# 속도 계산을 위한 초기 속도 설정 (예: 0 m/s)
initial_velocity = 0.0
# 위치 계산을 위한 초기 위치 설정 (예: 0 m)
initial_position = 0.0

# 속도 계산을 위한 배열 초기화
velocity_data = np.zeros(len(acceleration_data))
# 위치 계산을 위한 배열 초기화
position_data = np.zeros(len(acceleration_data))

# 속도 및 위치 계산
for i in range(1, len(acceleration_data)):
    velocity_data[i] = velocity_data[i-1] + acceleration_data[i] * delta_t
    position_data[i] = position_data[i-1] + velocity_data[i] * delta_t

# 초기 속도 및 초기 위치 추가
velocity_data += initial_velocity
position_data += initial_position

# 가속도, 속도, 위치 데이터를 시각화
time_data = np.arange(len(acceleration_data)) * delta_t

plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time_data, acceleration_data, label='Acceleration (m/s^2)')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_data, velocity_data, label='Velocity (m/s)')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_data, position_data, label='Position (m)')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()

plt.tight_layout()
plt.show()
