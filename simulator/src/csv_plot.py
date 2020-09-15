import matplotlib.pyplot as plt
import csv
from mpl_toolkits import mplot3d

filename = 'straight_path_cilqr_le.csv'

error=[]
x=[]
y=[]
count=[]
max_error = 0
global entry_dict
entry_dict = {
"Count" : -1,
"Point" : -1,
"Cross Track" : -1,
"Max Error" : -1,
"x" : -1,
"y" : -1
}
with open(filename, mode = 'r') as csv_file:
	fieldnames=["Count","Point","Cross Track","Max Error","x","y"]
	# plots=csv.DictReader(csv_file,fieldnames=fieldnames,delimiter=',')
	plots = csv.DictReader(csv_file)
	line_count = 0
	lastpt = -1
	pt = -2
	avg_ct = 0
	for row in plots:
		if line_count == 0:
			line_count +=1
		else:
			pt = int(row["Point"])
			if pt == lastpt:
				avg_ct +=1
				x[pt] = (x[pt]*(avg_ct-1) + row["x"])/avg_ct
				y[pt] = (y[pt]*(avg_ct-1) + row["y"])/avg_ct
				error[pt] = (error[pt]*(avg_ct-1) + row["Cross Track"])/avg_ct
				lastpt = pt
			else:
				x.append(float(row["x"]))	
				y.append(float(row["y"]))
				error.append(float(row["Cross Track"]))
				count.append(pt)


			# x.append((row["x"]))
			# y.append(float(row["y"]))
			# error.append(float(row["Cross Track"]))
			# count.append(int(row["Count"]))
			# max_error = row["Max Error"]


fig3D = plt.figure(1)
ax = plt.axes(projection='3d')
ax.scatter3D(x,y,error,'green')
# px = plt.axes(projection='3d')
ax.plot3D(x,y,0,'blue')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('CT Error (m)')

fid2D = plt.figure(2)
plt.plot(count,error)
plt.title('Error vs Point number')
plt.xlabel('Point')
plt.ylabel('Error (m)')

plt.show()
