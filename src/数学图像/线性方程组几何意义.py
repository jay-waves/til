import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_planes_with_dense_grid():
    fig = plt.figure(figsize=(8, 8))

    # First plot (top-left): Three intersecting planes
    ax1 = fig.add_subplot(221, projection='3d')
    X, Y = np.meshgrid(np.linspace(-1, 1, 20), np.linspace(-1, 1, 20))  # Denser grid

    # Plane 1: z = 0
    Z1 = np.zeros_like(X)
    ax1.plot_wireframe(X, Y, Z1, color='black')

    # Plane 2: y = 0 (vertical plane)
    Z2 = np.zeros_like(X)
    ax1.plot_wireframe(X, np.zeros_like(Y), Z2, color='black')

    # Plane 3: x = 0 (vertical plane)
    ax1.plot_wireframe(np.zeros_like(X), Y, Z2, color='black')

    ax1.set_xlim([-1, 1])
    ax1.set_ylim([-1, 1])
    ax1.set_zlim([-1, 1])
    ax1.set_title('Three intersecting planes')
    ax1.xaxis.pane.fill = False  # Remove background shading
    ax1.yaxis.pane.fill = False
    ax1.zaxis.pane.fill = False

    # Second plot (top-right): Two planes intersecting at a line
    ax2 = fig.add_subplot(222, projection='3d')

    # Plane 1: z = x + 1
    Z3 = X + 1
    ax2.plot_wireframe(X, Y, Z3, color='black')

    # Plane 2: z = -x + 1
    Z4 = -X + 1
    ax2.plot_wireframe(X, Y, Z4, color='black')

    ax2.set_xlim([-1, 1])
    ax2.set_ylim([-1, 1])
    ax2.set_zlim([-1, 2])
    ax2.set_title('Two planes intersecting')
    ax2.xaxis.pane.fill = False  # Remove background shading
    ax2.yaxis.pane.fill = False
    ax2.zaxis.pane.fill = False

    # Third plot (bottom-left): Planes in an X configuration
    ax3 = fig.add_subplot(223, projection='3d')

    # Plane 1: z = x
    Z5 = X
    ax3.plot_wireframe(X, Y, Z5, color='black')

    # Plane 2: z = -x
    Z6 = -X
    ax3.plot_wireframe(X, Y, Z6, color='black')

    # Adding a third plane for completeness, z = 0
    Z7 = np.zeros_like(X)
    ax3.plot_wireframe(X, Y, Z7, color='black')

    ax3.set_xlim([-1, 1])
    ax3.set_ylim([-1, 1])
    ax3.set_zlim([-1, 1])
    ax3.set_title('Three planes in X shape')
    ax3.xaxis.pane.fill = False  # Remove background shading
    ax3.yaxis.pane.fill = False
    ax3.zaxis.pane.fill = False

    # Fourth plot (bottom-right): A simple tilted plane
    ax4 = fig.add_subplot(224, projection='3d')

    # Plane: z = 0.5 * x + 0.5 * y
    Z8 = 0.5 * X + 0.5 * Y
    ax4.plot_wireframe(X, Y, Z8, color='black')

    ax4.set_xlim([-1, 1])
    ax4.set_ylim([-1, 1])
    ax4.set_zlim([-1, 1])
    ax4.set_title('Tilted plane')
    ax4.xaxis.pane.fill = False  # Remove background shading
    ax4.yaxis.pane.fill = False
    ax4.zaxis.pane.fill = False

    plt.tight_layout()
    plt.show()

# Call the function to plot the planes with denser grid and no pane shading
plot_planes_with_dense_grid()


def plot_two_new_planes():
    fig = plt.figure(figsize=(8, 4))

    # First plot (left): Planes in an X shape (parallel and intersecting)
    ax1 = fig.add_subplot(121, projection='3d')
    X, Y = np.meshgrid(np.linspace(-1, 1, 20), np.linspace(-1, 1, 20))

    # Plane 1: tilted plane
    Z1 = X
    ax1.plot_wireframe(X, Y, Z1, color='black')

    # Plane 2: opposite tilted plane
    Z2 = -X
    ax1.plot_wireframe(X, Y, Z2, color='black')

    # Adding third intersecting plane, vertical
    Z3 = np.zeros_like(X)
    ax1.plot_wireframe(np.zeros_like(X), Y, Z3, color='black')

    ax1.set_xlim([-1, 1])
    ax1.set_ylim([-1, 1])
    ax1.set_zlim([-1, 1])
    ax1.set_title('Intersecting planes (X shape)')
    ax1.xaxis.pane.fill = False  # Remove background shading
    ax1.yaxis.pane.fill = False
    ax1.zaxis.pane.fill = False

    # Second plot (right): Three stacked planes with dashed lines
    ax2 = fig.add_subplot(122, projection='3d')

    # Three parallel planes stacked at different heights
    Z4 = np.zeros_like(X)
    ax2.plot_wireframe(X, Y, Z4, color='black')

    Z5 = Z4 + 0.5  # Stacked plane at height 0.5
    ax2.plot_wireframe(X, Y, Z5, color='black')

    Z6 = Z4 - 0.5  # Stacked plane at height -0.5
    ax2.plot_wireframe(X, Y, Z6, color='black')

    # Adding dashed lines (representing edges or intersections)
    ax2.plot([0, 0], [0, 0], [-0.5, 0.5], color='black', linestyle='dashed')

    ax2.set_xlim([-1, 1])
    ax2.set_ylim([-1, 1])
    ax2.set_zlim([-1, 1])
    ax2.set_title('Stacked parallel planes')
    ax2.xaxis.pane.fill = False  # Remove background shading
    ax2.yaxis.pane.fill = False
    ax2.zaxis.pane.fill = False

    plt.tight_layout()
    plt.show()

# Call the function to plot the two new requested figures
plot_two_new_planes()

