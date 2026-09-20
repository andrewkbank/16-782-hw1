import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
import sys

def parse_mapfile(filename):
    with open(filename, 'r') as file:
        assert file.readline().strip() == 'N', "Expected 'N' in the first line"
        x_size, y_size = map(int, file.readline().strip().split(','))
        
        assert file.readline().strip() == 'C', "Expected 'C' in the third line"
        collision_threshold = int(file.readline().strip())
        
        assert file.readline().strip() == 'R', "Expected 'R' in the fifth line"
        robotX, robotY = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'T', "Expected 'T' in the seventh line"
        target_trajectory = []
        line = file.readline().strip()
        while line != 'M':
            x, y = map(float, line.split(','))
            target_trajectory.append({'x': x, 'y': y})
            line = file.readline().strip()
        
        costmap = []
        for line in file:
            row = list(map(float, line.strip().split(',')))
            costmap.append(row)
        
        costmap = np.asarray(costmap).T
    
    return x_size, y_size, collision_threshold, robotX, robotY, target_trajectory, costmap

def parse_robot_trajectory_file(filename):
    robot_trajectory = []
    with open(filename, 'r') as file:
        for line in file:
            t, x, y = map(int, line.strip().split(','))
            robot_trajectory.append({'t': t, 'x': x, 'y': y})
    
    return robot_trajectory

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)
    
    x_size, y_size, collision_threshold, robotX, robotY, target_trajectory, costmap = parse_mapfile(sys.argv[1])
    robot_trajectory = parse_robot_trajectory_file('../output/robot_trajectory.txt')

    total_frames = len(robot_trajectory)
    if total_frames == 0:
        print("Error: robot_trajectory.txt is empty.")
        sys.exit(1)

    # Pre-convert to numpy arrays for fast rendering
    robot_x = np.array([p['x'] for p in robot_trajectory])
    robot_y = np.array([p['y'] for p in robot_trajectory])
    robot_t = np.array([p['t'] for p in robot_trajectory])

    target_x = np.array([p['x'] for p in target_trajectory])
    target_y = np.array([p['y'] for p in target_trajectory])
    total_target_steps = len(target_trajectory)

    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.22)
    
    ax.imshow(costmap)
    line1, = ax.plot([], [], lw=2, marker='o', markersize=3, color='b', label='robot')
    line2, = ax.plot([], [], lw=2, marker='o', markersize=3, color='r', label='target')
    title_text = ax.set_title(f"Step: 0/{total_frames - 1} | Time: 0s")
    ax.legend(loc='upper right')

    # Default speed scales sensibly with trajectory length
    default_speed = max(1, min(100, total_frames // 100))
    max_speed = max(50, min(500, total_frames // 10))

    # UI Controls: Sliders and Play/Pause Button
    ax_speed = plt.axes([0.20, 0.10, 0.55, 0.03])
    speed_slider = Slider(ax_speed, 'Speed (steps/frame)', 1, max_speed, valinit=default_speed, valstep=1)

    ax_time = plt.axes([0.20, 0.05, 0.55, 0.03])
    time_slider = Slider(ax_time, 'Timeline', 0, total_frames - 1, valinit=0, valstep=1)

    ax_button = plt.axes([0.80, 0.05, 0.12, 0.08])
    play_button = Button(ax_button, 'Pause')

    state = {
        'current_frame': 0,
        'is_playing': True,
        'updating_slider': False
    }

    def render_frame(f):
        f = max(0, min(total_frames - 1, f))
        line1.set_data(robot_x[:f + 1], robot_y[:f + 1])
        
        t = robot_t[f]
        t_bounded = min(t, total_target_steps)
        line2.set_data(target_x[:t_bounded], target_y[:t_bounded])
        
        title_text.set_text(f"Step: {f}/{total_frames - 1} | Sim Time: {t}s")

    def update(frame_dummy):
        if not state['is_playing']:
            return line1, line2

        speed = int(speed_slider.val)
        next_frame = state['current_frame'] + speed
        if next_frame >= total_frames - 1:
            next_frame = total_frames - 1
            state['is_playing'] = False
            play_button.label.set_text('Play')

        state['current_frame'] = next_frame
        render_frame(next_frame)

        # Sync timeline slider without triggering its callback loop
        state['updating_slider'] = True
        time_slider.set_val(next_frame)
        state['updating_slider'] = False

        return line1, line2

    def on_time_slider_changed(val):
        if state['updating_slider']:
            return
        frame = int(val)
        state['current_frame'] = frame
        render_frame(frame)
        fig.canvas.draw_idle()

    time_slider.on_changed(on_time_slider_changed)

    def toggle_play(event):
        state['is_playing'] = not state['is_playing']
        if state['is_playing']:
            play_button.label.set_text('Pause')
            if state['current_frame'] >= total_frames - 1:
                state['current_frame'] = 0
        else:
            play_button.label.set_text('Play')

    play_button.on_clicked(toggle_play)

    # 30 ms interval (~33 fps) - speed slider controls how many steps advance per tick
    ani = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)

    plt.show()
