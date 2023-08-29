#! /usr/bin/env python
import tkinter as tk
import rospy

class TransformController:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("Transform Controller")
        self.window.geometry("300x400")
        self.window.resizable(False, False)
        self.window.configure(bg="white")
        
        self.initBar()

        # self.label_roll=tk.Label(self.window, text="Roll: 0")
        # self.label_roll.grid(row=0, column=1)
        # self.label_pitch=tk.Label(self.window, text="Pitch: 0")
        # self.label_pitch.grid(row=1, column=1)
        # self.label_yaw=tk.Label(self.window, text="Yaw: 0")
        # self.label_yaw.grid(row=2, column=1)

        # self.label_x=tk.Label(self.window, text="X: 0")
        # self.label_x.grid(row=3, column=1)
        # self.label_y=tk.Label(self.window, text="Y: 0")
        # self.label_y.grid(row=4, column=1)
        # self.label_z=tk.Label(self.window, text="Z: 0")
        # self.label_z.grid(row=5, column=1)
    def initBar(self):
        self.bar_roll=tk.Scale(self.window, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, label="Roll", command=self.update)
        self.bar_roll.grid(row=0, column=0)
        self.bar_pitch=tk.Scale(self.window, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, label="Pitch", command=self.update)
        self.bar_pitch.grid(row=1, column=0)
        self.bar_yaw=tk.Scale(self.window, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, label="Yaw", command=self.update)
        self.bar_yaw.grid(row=2, column=0)

        self.bar_x=tk.Scale(self.window, from_=-100, to=100, orient=tk.HORIZONTAL, length=300, label="X", command=self.update)
        self.bar_x.grid(row=3, column=0)
        self.bar_y=tk.Scale(self.window, from_=-100, to=100, orient=tk.HORIZONTAL, length=300, label="Y", command=self.update)
        self.bar_y.grid(row=4, column=0)
        self.bar_z=tk.Scale(self.window, from_=-100, to=100, orient=tk.HORIZONTAL, length=300, label="Z", command=self.update)
        self.bar_z.grid(row=5, column=0)

    def update(self,event):
        rospy.set_param("/roll",self.bar_roll.get())
        rospy.set_param("/pitch",self.bar_pitch.get())
        rospy.set_param("/yaw",self.bar_yaw.get())
        rospy.set_param("/x",self.bar_x.get())
        rospy.set_param("/y",self.bar_y.get())
        rospy.set_param("/z",self.bar_z.get())

        self.roll=self.bar_roll.get()
        self.pitch=self.bar_pitch.get()
        self.yaw=self.bar_yaw.get()
        self.x=self.bar_x.get()
        self.y=self.bar_y.get()
        self.z=self.bar_z.get()
        # self.label_roll.configure(text="Roll: "+str(self.roll))
        # self.label_pitch.configure(text="Pitch: "+str(self.pitch))
        # self.label_yaw.configure(text="Yaw: "+str(self.yaw))
        # self.label_x.configure(text="X: "+str(self.x))
        # self.label_y.configure(text="Y: "+str(self.y))
        # self.label_z.configure(text="Z: "+str(self.z))






if __name__ == '__main__':
    print("[ INFO ] Starting Controller...")
    rospy.init_node("tf_controller")
    print("[ INFO ] Node Established!")
    app = TransformController()
    app.window.mainloop()

    print("[ INFO ] App is terminated.")
