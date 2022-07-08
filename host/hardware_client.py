import time

import tkinter as tk
import tkinter.ttk as ttk

import serial

FPS = 10

display_mode = "GENERAL"


class ESCControl:
    MODE_IDLE           = b"0"
    MODE_CALIBRATION    = b"1"
    MODE_TORQUE         = b"2"
    MODE_VELOCITY       = b"3"
    MODE_POSITION       = b"4"
    
    def __init__(self):
        self._ser = serial.Serial(port="COM12", baudrate=115200, timeout=1)

        self.phase_current_measured = [0, 0, 0]
        self.position_measured = 0
        self.position_setpoint = 0
        self.bus_voltage_measured = 0
        self.torque_setpoint = 0

    def commandMode(self, mode):
        self._ser.write(mode)

    def update(self):
        self.updateCurrent()
        self.updateVoltage()
        self.updatePosition()
    
    def updateCurrent(self):
        self._ser.write(b"I")

        buffer = self._ser.readline()
            
        data = buffer.decode().split("\t")
        try:
            data = [float(elem) for elem in data]
        except:
            return
        if len(data) != 7:
            return
        self.phase_current_measured = data[0:3]
        self.i_alpha = data[3]
        self.i_beta = data[4]
        self.i_q = data[5]
        self.i_d = data[6]
        
    def updateVoltage(self):
        self._ser.write(b"V")
        
        buffer = b""

        buffer = self._ser.readline()
            
        data = buffer.decode().split("\t")
        try:
            data = [float(elem) for elem in data]
        except:
            return
        if len(data) != 8:
            return
        self.phase_voltage_setpoint = data[0:3]
        self.v_alpha = data[3]
        self.v_beta = data[4]
        self.v_q = data[5]
        self.v_d = data[6]
        self.bus_voltage_measured = data[7]
    
    def updatePosition(self):
        self._ser.write(b"p")
        
        buffer = b""

        buffer = self._ser.readline()
            
        data = buffer.decode().split("\t")
        try:
            data = [float(elem) for elem in data]
        except:
            return
        if len(data) != 2:
            return
        self.position_measured = data[0]
        self.position_setpoint = data[1]
        
    def updateVelocity(self):
        self._ser.write(b"v")
        
        buffer = b""

        buffer = self._ser.readline()
            
        data = buffer.decode().split("\t")
        try:
            data = [float(elem) for elem in data]
        except:
            return
        if len(data) != 2:
            return
        self.velocity_measured = data[0]
        self.velocity_setpoint = data[1]
        
    def updateGeneral(self):
        self._ser.write(b"G")
        
        buffer = b""

        buffer = self._ser.readline()
            
        data = buffer.decode().split("\t")
        try:
            data = [float(elem) for elem in data]
        except:
            return
        if len(data) != 6:
            return
        self.position_measured = data[0]
        self.position_setpoint = data[1]
        self.velocity_measured = data[2]
        self.velocity_setpoint = data[3]
        self.i_q = data[4]
        self.torque_setpoint = data[5]



class Plot(ttk.Frame):
    def __init__(self, parent, traces, name="plot", x_pkpk=[0, 100], y_pkpk=[-1, 1]):
        super().__init__(parent)
        self.parent = parent
        self.label = ttk.Label(self, text=name)
        self.label.grid(column=0, row=0)
        self.canvas = tk.Canvas(self, width=600, height=160, background='gray75')
        self.canvas.grid(column=0, row=1, sticky=(tk.N, tk.W, tk.E, tk.S))

        self.x_pkpk = x_pkpk
        self.y_pkpk = y_pkpk
        self.x_range = self.x_pkpk[1] - self.x_pkpk[0]
        self.y_range = self.y_pkpk[1] - self.y_pkpk[0]
        self.ys = {}
        self.y_colors = {}

        self.x_counter = 0

        for trace in traces:
            self.ys[trace["name"]] = [0.] * self.x_range
            self.y_colors[trace["name"]] = trace["color"] if trace["color"] else "#333333"
        
        
    def updateData(self, data):
        for key in self.ys:
            val = data.get(key)
            self.ys[key].append(val if val else 0)
            self.ys[key].pop(0)
        self.x_counter += 1

    def update(self):            
        self.h_scale = self.canvas.winfo_width() / self.x_range
        self.v_scale = self.canvas.winfo_height() / self.y_range

        self.x_counter = self.x_counter % self.x_range
        x_counter_half = (self.x_counter + .5 * self.x_range) % self.x_range

        self.canvas.delete("all")
        self.canvas.create_line(0, self.canvas.winfo_height() - (-self.y_pkpk[0]) * self.v_scale, self.canvas.winfo_width(), self.canvas.winfo_height()-(-self.y_pkpk[0]) * self.v_scale, fill="#666666")
        self.canvas.create_line(self.canvas.winfo_width() - self.x_counter * self.h_scale, 0, self.canvas.winfo_width() - self.x_counter * self.h_scale, self.canvas.winfo_height(), fill="#666666")
        self.canvas.create_line(self.canvas.winfo_width() - x_counter_half * self.h_scale, 0, self.canvas.winfo_width() - x_counter_half * self.h_scale, self.canvas.winfo_height(), fill="#999999")
        
        for i in range(self.x_range-1):
            for key in self.ys:
                self.canvas.create_line(
                    i*self.h_scale,
                    self.canvas.winfo_height()-(self.ys[key][i] - self.y_pkpk[0]) * self.v_scale,
                    (i+1)*self.h_scale,
                    self.canvas.winfo_height()-(self.ys[key][i+1] - self.y_pkpk[0]) * self.v_scale,
                    fill=self.y_colors[key],
                    width=1)
            
        self.parent.after(int(1000/FPS), self.update)

esc = ESCControl()


root = tk.Tk()
root.title(" MyServoGUI")
root.geometry("800x600")
root.minsize(800, 600)

section_plot = ttk.Frame(root)
section_plot.grid(column=0, row=0)

section_control = ttk.Frame(root)
section_control.grid(column=1, row=0, sticky=(tk.W, tk.N, tk.E))


button_idle_style = ttk.Style()
button_idle_style.configure("Idle.TButton", foreground="#000000")
button_calibrate_style = ttk.Style()
button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
button_torque_style = ttk.Style()
button_torque_style.configure("Torque.TButton", foreground="#666666")
button_velocity_style = ttk.Style()
button_velocity_style.configure("Velocity.TButton", foreground="#666666")
button_position_style = ttk.Style()
button_position_style.configure("Position.TButton", foreground="#666666")

def setMode(mode):
    if mode == ESCControl.MODE_IDLE:
        button_idle_style.configure("Idle.TButton", foreground="#000000")
        button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
        button_torque_style.configure("Torque.TButton", foreground="#666666")
        button_velocity_style.configure("Velocity.TButton", foreground="#666666")
        button_position_style.configure("Position.TButton", foreground="#666666")
        esc.commandMode(ESCControl.MODE_IDLE)
    elif mode == ESCControl.MODE_CALIBRATION:
        button_idle_style.configure("Idle.TButton", foreground="#666666")
        button_calibrate_style.configure("Calibrate.TButton", foreground="#000000")
        button_torque_style.configure("Torque.TButton", foreground="#666666")
        button_velocity_style.configure("Velocity.TButton", foreground="#666666")
        button_position_style.configure("Position.TButton", foreground="#666666")
        esc.commandMode(ESCControl.MODE_CALIBRATION)
    elif mode == ESCControl.MODE_TORQUE:
        button_idle_style.configure("Idle.TButton", foreground="#666666")
        button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
        button_torque_style.configure("Torque.TButton", foreground="#000000")
        button_velocity_style.configure("Velocity.TButton", foreground="#666666")
        button_position_style.configure("Position.TButton", foreground="#666666")
        esc.commandMode(ESCControl.MODE_TORQUE)
    elif mode == ESCControl.MODE_VELOCITY:
        button_idle_style.configure("Idle.TButton", foreground="#666666")
        button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
        button_torque_style.configure("Torque.TButton", foreground="#666666")
        button_velocity_style.configure("Velocity.TButton", foreground="#000000")
        button_position_style.configure("Position.TButton", foreground="#666666")
        esc.commandMode(ESCControl.MODE_VELOCITY)
    elif mode == ESCControl.MODE_POSITION:
        button_idle_style.configure("Idle.TButton", foreground="#666666")
        button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
        button_torque_style.configure("Torque.TButton", foreground="#666666")
        button_velocity_style.configure("Velocity.TButton", foreground="#666666")
        button_position_style.configure("Position.TButton", foreground="#000000")
        esc.commandMode(ESCControl.MODE_POSITION)

button_idle = ttk.Button(section_control, text="IDLE Mode", style="Idle.TButton", command=lambda: setMode(ESCControl.MODE_IDLE))
button_calibrate = ttk.Button(section_control, text="CALIBRATE Mode", style="Calibrate.TButton", command=lambda: setMode(ESCControl.MODE_CALIBRATION))
button_torque = ttk.Button(section_control, text="TORQUE Mode", style="Torque.TButton", command=lambda: setMode(ESCControl.MODE_TORQUE))
button_velocity = ttk.Button(section_control, text="VELOCITY Mode", style="Velocity.TButton", command=lambda: setMode(ESCControl.MODE_VELOCITY))
button_position = ttk.Button(section_control, text="POSITION Mode", style="Position.TButton", command=lambda: setMode(ESCControl.MODE_POSITION))
button_idle.grid(column=0, row=0, sticky=(tk.W, tk.E))
button_calibrate.grid(column=0, row=1, sticky=(tk.W, tk.E))
button_torque.grid(column=0, row=2, sticky=(tk.W, tk.E))
button_velocity.grid(column=0, row=3, sticky=(tk.W, tk.E))
button_position.grid(column=0, row=4, sticky=(tk.W, tk.E))

panel_selection = ttk.Frame(section_plot)
panel_selection.grid(column=0, row=0, sticky=(tk.W))
panel_general = ttk.Frame(section_plot)
panel_general.grid(column=0, row=1)
panel_current = ttk.Frame(section_plot)
panel_current.grid(column=0, row=1)
panel_voltage = ttk.Frame(section_plot)
panel_voltage.grid(column=0, row=1)

panel_current.grid_forget()
panel_voltage.grid_forget()


def showGeneralPlot():
    global display_mode
    
    panel_general.grid(column=0, row=1)
    panel_current.grid_forget()
    panel_voltage.grid_forget()
    display_mode = "GENERAL"
    
def showCurrentPlot():
    global display_mode
    
    panel_general.grid_forget()
    panel_current.grid(column=0, row=1)
    panel_voltage.grid_forget()
    display_mode = "CURRENT"
    
def showVoltagePlot():
    global display_mode
    
    panel_general.grid_forget()
    panel_current.grid_forget()
    panel_voltage.grid(column=0, row=1)
    display_mode = "VOLTAGE"
    
    
button_show_general = ttk.Button(panel_selection, text="General", command=showGeneralPlot)
button_show_general.grid(column=0, row=0)
button_show_current = ttk.Button(panel_selection, text="Current", command=showCurrentPlot)
button_show_current.grid(column=1, row=0)
button_show_voltage = ttk.Button(panel_selection, text="Voltage", command=showVoltagePlot)
button_show_voltage.grid(column=2, row=0)



plot_position = Plot(panel_general,
            traces=[
                {
                    "name": "position_measured",
                    "color": "#00FFFF",
                }, {
                    "name": "position_setpoint",
                    "color": "#FFFF00",
                }],
            name="Position",
            y_pkpk=[-20, 20])
plot_position.grid(column=0, row=0)
plot_velocity = Plot(panel_general,
            traces=[
                {
                    "name": "velocity_measured",
                    "color": "#00FFFF",
                }, {
                    "name": "velocity_setpoint",
                    "color": "#FFFF00",
                }],
            name="Velocity",
            y_pkpk=[-10, 10])
plot_velocity.grid(column=0, row=1)
plot_torque = Plot(panel_general,
            traces=[
                {
                    "name": "torque_measured",
                    "color": "#00FFFF",
                }, {
                    "name": "torque_setpoint",
                    "color": "#FFFF00",
                }],
            name="Torque",
            y_pkpk=[-5, 5])
plot_torque.grid(column=0, row=2)

plot_phase_current = Plot(panel_current,
            traces=[
                {
                    "name": "i_a",
                    "color": "#FF0000",
                }, {
                    "name": "i_b",
                    "color": "#00FF00",
                }, {
                    "name": "i_c",
                    "color": "#0000FF",
                }],
            name="Phase Current",
            y_pkpk=[-1, 1])
plot_phase_current.grid(column=0, row=0)
plot_alphabeta_current = Plot(panel_current,
            traces=[
                {
                    "name": "i_alpha",
                    "color": "#FFFF00",
                }, {
                    "name": "i_beta",
                    "color": "#00FFFF",
                }],
            name="α β Current",
            y_pkpk=[-1, 1])
plot_alphabeta_current.grid(column=0, row=1)
plot_qd_current = Plot(panel_current,
            traces=[
                {
                    "name": "i_q",
                    "color": "#FFFF00",
                }, {
                    "name": "i_d",
                    "color": "#00FFFF",
                }],
            name="Q D Current",
            y_pkpk=[-1, 1])
plot_qd_current.grid(column=0, row=2)

plot_bus_voltage = Plot(panel_voltage,
            traces=[
                {
                    "name": "v_bus",
                    "color": "#FF0000",
                }],
            name="Bus Voltage",
            y_pkpk=[-0.1, 15])
plot_bus_voltage.grid(column=0, row=0)
plot_phase_voltage = Plot(panel_voltage,
            traces=[
                {
                    "name": "v_a",
                    "color": "#FF0000",
                },
                {
                    "name": "v_b",
                    "color": "#00FF00",
                },
                {
                    "name": "v_c",
                    "color": "#0000FF",
                }],
            name="Phase Voltage",
            y_pkpk=[-10, 10])
plot_phase_voltage.grid(column=0, row=1)


def receivePacket():
    global display_mode
    
    if display_mode == "GENERAL":
        esc.updateGeneral()
        plot_position.updateData({
            "position_measured": esc.position_measured,
            "position_setpoint": esc.position_setpoint,
            })
        plot_velocity.updateData({
            "velocity_measured": esc.velocity_measured,
            "velocity_setpoint": esc.velocity_setpoint,
            })
        plot_torque.updateData({
            "torque_measured": esc.i_q,
            "torque_setpoint": esc.torque_setpoint,
            })
    
    elif display_mode == "CURRENT":
        esc.updateCurrent()
        plot_phase_current.updateData({
            "i_a": esc.phase_current_measured[0],
            "i_b": esc.phase_current_measured[1],
            "i_c": esc.phase_current_measured[2]
            })
        plot_alphabeta_current.updateData({
            "i_alpha": esc.i_alpha,
            "i_beta": esc.i_beta
            })
        plot_qd_current.updateData({
            "i_q": esc.i_q,
            "i_d": esc.i_d
            })
    
    elif display_mode == "VOLTAGE":
        esc.updateVoltage()
        plot_bus_voltage.updateData({
            "v_bus": esc.bus_voltage_measured
            })
        plot_phase_voltage.updateData({
            "v_a": esc.phase_voltage_setpoint[0],
            "v_b": esc.phase_voltage_setpoint[1],
            "v_c": esc.phase_voltage_setpoint[2]
            })
    
    root.after(1, receivePacket)

root.after(1, receivePacket)

plot_position.update()
plot_velocity.update()
plot_torque.update()
                     
plot_phase_current.update()
plot_alphabeta_current.update()
plot_qd_current.update()

plot_bus_voltage.update()
plot_phase_voltage.update()


root.mainloop()
