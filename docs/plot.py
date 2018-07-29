import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import numpy as np

# Prepare data for plotting
count = 0
with open("data.txt", "r") as file:
	pos = []
	E = []
	Ep = []
	Ek = []
	for line in file:
		count += 1
		pos.append(float(line.split()[0]))
		E.append(float(line.split()[1]))
		Ep.append(float(line.split()[2]))
		Ek.append(float(line.split()[3]))

x = np.arange(0, count, 1)

fig, ax = plt.subplots()
ax.plot(x, pos, label="pos")
ax.plot(x, E, label="E")
ax.plot(x, Ep, label="Ep")
ax.plot(x, Ek, label="Ek")
ax.legend()

ax.set(xlabel = 'frame', ylabel = 'position / Energy', title = '1-d Spring')
ax.grid()

fig.savefig('data.png')
plt.show()