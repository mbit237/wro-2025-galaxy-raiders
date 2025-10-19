import tkinter as tk
import math

SIZE = 900
SCALE = SIZE / 3000
ROBOT_R = 15
ARROW_LEN = 100

root = tk.Tk()
root.geometry("1600x1000")

history_data = [
    [[500, 1500, 90], [480,1490,95]],
    [[400, 1910, 85], [420,1900,70]],
    [[600, 2150, 100], [580,2190,105]]
    ]

# Draw functions
def circle(x, y, r=5, fill="cyan", outline="black", width=1):
    x = x * SCALE
    y = 900 - y * SCALE
    field.create_oval(x-r,y-r,x+r,y+r, fill=fill, outline=outline, width=width)

def line(x1, y1, x2, y2, fill="blue", arrow=tk.LAST): 
    x1 = x1 * SCALE
    y1 = 900 - y1 * SCALE
    x2 = x2 * SCALE
    y2 = 900 - y2 * SCALE
    field.create_line(x1, y1, x2, y2, arrow=arrow, fill=fill)

def draw_robot(idx):
    field.create_image((0, 0), image=field_img, anchor=tk.NW)
    pose, spike_pose = history_data[idx]
    x = pose[0]
    y = pose[1]
    heading = pose[2]
    circle(x, y, r=ROBOT_R, fill="yellow")
    a = heading / 180 * math.pi
    x2 = math.cos(a) * ARROW_LEN + x
    y2 = math.sin(a) * ARROW_LEN + y 
    line(x, y, x2, y2)

    x_spike = spike_pose[0]
    y_spike = spike_pose[1]
    heading_spike = spike_pose[2]
    a_spike = heading_spike / 180 * math.pi
    x2_spike = math.cos(a_spike) * ARROW_LEN + x_spike
    y2_spike = math.sin(a_spike) * ARROW_LEN + y_spike 
    line(x_spike, y_spike, x2_spike, y2_spike, fill="red")
    draw_trail(idx)
    draw_trail_spikes(idx)
    
def draw_trail(idx):
    end = 0
    for i in range(idx, end, -1):
        pose = history_data[i][0]
        x = pose[0]
        y = pose[1]
        pose = history_data[i-1][0]
        x2 = pose[0]
        y2 = pose[1]
        line(x, y, x2, y2, fill="magenta", arrow=None)

def draw_trail_spikes(idx):
    end = 0
    for i in range(idx, end, -1):
        pose = history_data[i][1]
        x = pose[0]
        y = pose[1]
        pose = history_data[i-1][1]
        x2 = pose[0]
        y2 = pose[1]
        line(x, y, x2, y2, fill="green", arrow=None)

def scale_change(v):
    draw_robot(int(v))

# widgets 
title = tk.Label(root, text='Galaxy Raiders Data Display')
title.pack()

v1 = tk.IntVar()

slider = tk.Scale(root, command=scale_change, variable=v1, from_=0, to=len(history_data)-1, orient=tk.HORIZONTAL)
slider.pack()

field = tk.Canvas(root, width=900, height=900)
field.pack()

field_img = tk.PhotoImage(file="FutureEngineers_Playfield_900.png")
draw_robot(int(v1.get()))

def foo():
    print('foo')
    root.after(1000, foo)

root.after(1000, foo)

root.mainloop() # run application once it is ready -- blocks