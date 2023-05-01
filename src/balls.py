import math
y = 0.0
z = -2.0
r = 1
numPoints = 26
sphere_r = r * math.sin(math.pi/numPoints)
points = []
for index in range(numPoints):
    points.append((r*math.cos((index*2*math.pi)/numPoints), y , r*math.sin((index*2*math.pi)/numPoints)-2.0))

for pt in points:
    print(f'{{Vector3{{ {pt[0]}, {pt[1]}, {pt[2]}}}, {sphere_r}, 2}},')
