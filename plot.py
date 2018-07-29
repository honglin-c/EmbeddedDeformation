import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import numpy as np

# Prepare data for plotting
count = 0
with open("Fx_data.txt", "r") as file:
	Fx = []
	Fx_Ep = []
	Fx_rot = []
	Fx_reg = []
	Fx_con = []
	Fx_Ek = []
	Pos = []
	Velocity = []
	Fx_gv = []
	for line in file:
		count += 1
		Fx.append(float(line.split()[0]))
		Fx_Ep.append(float(line.split()[1]))
		Fx_rot.append(float(line.split()[2]))
		Fx_reg.append(float(line.split()[3]))
		Fx_con.append(float(line.split()[4]))
		Fx_Ek.append(float(line.split()[5]))
		Pos.append(float(line.split()[6]))
		Velocity.append(float(line.split()[7]))
		Fx_gv.append(float(line.split()[8]))
x = np.arange(0, count, 1)

fig1, ax = plt.subplots()
fig2, bx = plt.subplots()
fig3, cx = plt.subplots()
bx.plot(x, Fx, label="Fx")
ax.plot(x, Fx_Ep, label="Fx_Ep")
ax.plot(x, Fx_rot, label="Fx_rot")
ax.plot(x, Fx_reg, label="Fx_reg")
ax.plot(x, Fx_con, label="Fx_con")
cx.plot(x, Fx_Ek, label="Fx_Ek")
cx.plot(x, Pos, label="Position")
cx.plot(x, Velocity, label="Velocity")
bx.plot(x, Fx_gv, label="Fx_gv")
ax.legend()
bx.legend()
cx.legend()

ax.set(xlabel = 'frame', ylabel = 'Energy', title = 'Deformation')
ax.grid()

bx.set(xlabel = 'frame', ylabel = 'Energy', title = 'Deformation')
bx.grid()

cx.set(xlabel = 'frame', ylabel = 'Position', title = 'Deformation')
cx.grid()

fig1.savefig('data1.png')
fig2.savefig('data2.png')
fig3.savefig('data3.png')
plt.show()