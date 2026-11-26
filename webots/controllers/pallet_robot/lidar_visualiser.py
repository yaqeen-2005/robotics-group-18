import matplotlib.pyplot as plt


class LidarVisualiser:
    def show_scan(self, points):
        x = [point[0] for point in points]
        y = [point[1] for point in points]

        plt.figure("LiDAR Scan")
        plt.clf()
        plt.scatter(y, x, s=5, c="blue")
        plt.scatter(0, 0, s=50, c="red", label="LiDAR")

        plt.xlabel("Sideways distance (m)")
        plt.ylabel("Forward distance (m)")
        plt.title("LiDAR Scan")

        plt.axis("equal")
        plt.gca().invert_xaxis()
        plt.grid(True)
        plt.legend()

        plt.show()