import math
import matplotlib.pyplot as plt


class OdometryVisualiser:
    def __init__(self):
        self.positions = []
        self.pose = (0.0, 0.0, 0.0)

    def update(self, pose):
        self.pose = pose
        x, y, theta = pose

        if not self.positions:
            self.positions.append((x, y))
            return

        last_x, last_y = self.positions[-1]

        distance = math.hypot(x - last_x, y - last_y)

        if distance >= 0.01:
            self.positions.append((x, y))

    def show_trajectory(self):
        x, y, theta = self.pose

        positions = self.positions + [(x, y)]

        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]

        plt.figure("Wheel Odometry")
        plt.clf()

        plt.plot(xs, ys, label="Robot trajectory")
        plt.scatter(xs[0], ys[0], c="green",
                    label="Starting position")
        plt.scatter(x, y, c="red",
                    label="Current position")

        plt.arrow(
            x, y,
            0.3 * math.cos(theta),
            0.3 * math.sin(theta),
            head_width=0.1,
            color="red",
            length_includes_head=True
        )

        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")
        plt.title("Wheel Odometry Trajectory")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()

        plt.show()
        plt.close()

        
