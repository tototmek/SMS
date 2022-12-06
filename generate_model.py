i1 = 0.09
i2 = 0.28
prev_dy = 0
y = [0 for i in range(70)]
u = [0 for i in range(5)] + [1 for i in range(69)]

for k in range(5, 70):
    dy = (u[k-3] - y[k-1]) * i1
    ddy = (dy - prev_dy) * i2
    real_dy = prev_dy + ddy
    y[k] = y[k-1] + real_dy
    prev_dy = real_dy

s = y[5:65]

print(s)

import matplotlib.pyplot as plt

plt.plot(s)
plt.show()
