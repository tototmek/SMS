import matplotlib.pyplot as plt
data = []
with open("y.out", "r") as file:
    for line in file:
        data.append(float(line))
print(data)
plt.plot(data)
plt.show()