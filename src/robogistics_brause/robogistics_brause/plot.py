import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def is_coordinate_inside_polygon(x, y):
    polygon = [(0.068, 0.341), (-0.324, 0.341), (-0.281, 0.172), (0.114, 0.124)]
    n = len(polygon)
    inside = False
    
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

# Define the coordinates to check
x = 0.0
y = 0.2

# Check if the coordinate is inside the polygon
result = is_coordinate_inside_polygon(x, y)

# Plot the polygon and the point
polygon = [(0.068, 0.341), (-0.324, 0.341), (-0.281, 0.172), (0.114, 0.124)]
polygon.append(polygon[0])  # Close the polygon
xs, ys = zip(*polygon)

fig, ax = plt.subplots()
ax.plot(xs, ys, 'r-')
ax.add_patch(Polygon(polygon, closed=True, alpha=0.2))

#place a huge circle at 0.0, 0.4 and name it Robot
ax.add_patch(plt.Circle((0.0, 0.4), 0.05, color='grey', alpha=0.2))
ax.annotate('Robot', xy=(0.0, 0.4), xytext=(0.0+0.05, 0.4+0.05),
                arrowprops=dict(facecolor='black', arrowstyle='->'))

#add lines along x values -0.2 and -0.05
ax.plot([-0.2, -0.2], [-0.2, 0.4], 'k-')
ax.plot([-0.05, -0.05], [-0.2, 0.4], 'k-')


if result:
    ax.plot(x, y, 'bo')
    ax.annotate('Inside', xy=(x, y), xytext=(x+0.05, y+0.05),
                arrowprops=dict(facecolor='black', arrowstyle='->'))
else:
    ax.plot(x, y, 'ro')
    ax.annotate('Outside', xy=(x, y), xytext=(x+0.05, y+0.05),
                arrowprops=dict(facecolor='black', arrowstyle='->'))

ax.set_xlim(min(xs) - 0.1, max(xs) + 0.1)
ax.set_ylim(min(ys) - 0.1, max(ys) + 0.1)
ax.set_aspect('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Pick Point Map')
#plt.show()
plt.savefig('src/robogistics_brause/robogistics_brause/validate_position.jpg')
