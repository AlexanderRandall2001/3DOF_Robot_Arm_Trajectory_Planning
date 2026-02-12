from Robot_Arm_3DOF_IK import ik_3dof
from Robot_Arm_3DOF_FK import fk_3dof
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Compute joint angles at a single timestep along a Cartesian trajectory for a 3DOF arm

def cartesian_3dof(start_thetas, target_xyz, lengths, T, dt):

    # Get starting joint positions from FK and take the end-effector coordinates

    joints, _ = fk_3dof(start_thetas, lengths)
    start_xyz = joints[-1]
    trajectory = []
    t = 0

    # Step through trajectory until total time is reached

    while t <= T + dt/2:

        # Compute quintic time-scaling function for smooth start and stop

        st = 10*(t/T)**3 - 15*(t/T)**4 + 6*(t/T)**5

        # Interpolate end-effector position in Cartesian space

        x = start_xyz[0] + st*(target_xyz[0] - start_xyz[0])
        y = start_xyz[1] + st*(target_xyz[1] - start_xyz[1])
        z = start_xyz[2] + st*(target_xyz[2] - start_xyz[2])

        # Convert interpolated position to joint angles using IK

        q = ik_3dof([x, y, z], lengths)[1]
        trajectory.append(q)

        # Increment time by timestep

        t += dt

    return trajectory

# Animate arm trajectory

def animate_3dof(traj, lengths, interval=50):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set axis limits based on total arm reach

    reach = sum(lengths)
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_zlim(0, reach)
    ax.set_box_aspect([1, 1, 1])

    # Lines representing the arm links

    line, = ax.plot([], [], [], "-", lw=4, color='#66FF66')

    # Update the arm configuration for each frame

    def update(frame):

        thetas = traj[frame]
        joints, _ = fk_3dof(thetas, lengths)
        xs, ys, zs = zip(*joints)
        line.set_data(xs, ys)
        line.set_3d_properties(zs)        
        return line,

    # Create the animation
    
    fig.ani = FuncAnimation(
        fig,
        update,
        frames=len(traj),
        interval=interval,
        blit=False
    )
    fig.ani.save("Robot_Arm_3DOF_Trajectory_Cartesian.gif", writer='pillow', fps=30)
    plt.show()

# Example usage

start_thetas = [0.0, 0.0, 0.0]
target_xyz = [-0.8, -0.6, 0.5]
lengths = [0.6, 0.6]
T  = 4.0 
dt = 0.05
traj = cartesian_3dof(start_thetas, target_xyz, lengths, T, dt)

animate_3dof(traj, lengths)