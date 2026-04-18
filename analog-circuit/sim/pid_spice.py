# PID dc motor speed controller

import PySpice
from PySpice.Spice.Netlist import Circuit
from PySpice.Unit import *
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import schemdraw
import schemdraw.elements as elm
        
class PIDConfig:
    def __init__(self):
        # global variables for transient analysis
        self.TSTEP      = 100@u_us
        self.TSTOP      = 1000@u_ms
        self.DEFAULT_SP = 1.0
        self.TEMP       = 25
        # power rails
        self.VCC        = 15@u_V
        self.VEE        = -15@u_V
        self.GND        = 0@u_V
        # opamp
        self.OPAMP      = "ua741"
        # motor model
        self.R_MOT      = 5.5@u_Ω
        self.L_MOT      = 1@u_mH
        self.KEMF_MOT   = 0.02
        self.J_MOT      = 10e-6
        self.RLOSS_MOT  = 200@u_Ω
        self.KTRQ_MOT   = 0.02


        # proportional circuit
        self.RIN_PROP   = 10@u_kΩ
        self.RFB_PROP   = 100@u_kΩ
        # integral circuit
        self.RIN_INT    = 10@u_kΩ
        self.RLIM_INT   = 10@u_kΩ
        self.C_INT      = 10@u_uF
        # differentiation circuit
        self.R1_DIFF    = 10@u_kΩ
        self.C_DIFF     = 100@u_nF
        self.RDAMP_DIFF = 100@u_Ω
        # summing circuit
        self.RIN1_SUM   = 100@u_kΩ
        self.RIN2_SUM   = 100@u_kΩ
        self.RIN3_SUM   = 100@u_kΩ
        self.RFB_SUM    = 100@u_kΩ
        # error amplifier
        self.R1_ERR     = 1@u_kΩ
        self.R2_ERR     = 1@u_kΩ
        self.R3_ERR     = 1@u_kΩ
        self.R4_ERR     = 1@u_kΩ
        
    def reconfig(self, cfg: dict):
        for k, v in cfg.items():
            if not hasattr(self, k):
                raise ValueError(f"Unknown config field: {k}")
            setattr(self, k, v)
        return self


class PID:
    def __init__(self, cfg=PIDConfig()):
        self.c = cfg
        self.circuit = Circuit("PID")

        # opamp
        self.circuit.include("ua741.sub")
        # power rails
        self.circuit.V("VCC", "VCC", self.circuit.gnd, self.c.VCC)
        self.circuit.V("VEE", "VEE", self.circuit.gnd, self.c.VEE)

    def build_circuit(self):
        self._system()
        return self.configure()

    def configure(self):
        if not hasattr(self, '_gains'):
            raise RuntimeError("Call _system() before configure().")
        k_err_sp, k_err_fb, kp, ki, kd, k_sum, tau_e, tau_m = self._gains

        return f"""
                    ----------------------------------------------
                    Config Report
                    ----------------------------------------------
                    1. Motor
                    tau_e               = {float(tau_e):.6e} s
                    tau_m               = {float(tau_m):.6e} s
                    2. PID
                    Kp                  = {float(kp):.4f}
                    Ki                  = {float(ki):.4f}
                    Kd                  = {float(kd):.6f}
                    3. Sum
                    Gain                = {float(k_sum):.4f}
                    4. Error
                    Gsp                 = {float(k_err_sp):.4f}
                    Gfb                 = {float(k_err_fb):.4f}
                    ----------------------------------------------
                """

    def export_circuit(self, path="out/pid.cir"):
        netlist = str(self.circuit)

        with open(path, "w") as f:
            f.write(netlist)
            f.write("\n.tran 100u 1\n")
            f.write(".end\n")

        return path

    def drive(self, kind="step", func=None, **kwargs):
        t = np.arange(0, float(self.c.TSTOP), float(self.c.TSTEP))

        if func is not None:
            y = func(t)
        elif kind == "step":
            A = kwargs.get("amplitude", self.c.DEFAULT_SP)
            t0 = kwargs.get("t0", 1e-3)
            y = np.where(t >= t0, A, 0.0)
        elif kind == "sin":
            A = kwargs.get("amplitude", self.c.DEFAULT_SP)
            f = kwargs.get("freq", 10)
            y = A * np.sin(2 * np.pi * f * t)
        elif kind == "triangle":
            A = kwargs.get("amplitude", self.c.DEFAULT_SP)
            f = kwargs.get("freq", 10)
            y = A * (2/np.pi) * np.arcsin(np.sin(2*np.pi*f*t))
        elif kind == "square":
            A = kwargs.get("amplitude", self.c.DEFAULT_SP)
            f = kwargs.get("freq", 10)
            y = A * np.sign(np.sin(2*np.pi*f*t))
        else:
            raise ValueError("Drive ERR.")

        pwl = [(ti, yi) for ti, yi in zip(t, y)]
        self.circuit.PieceWiseLinearVoltageSource(
            "Vdrive",
            "set_point",
            self.circuit.gnd,
            values=pwl
        )
        self.drive_t = t
        self.drive_y = y

        return t, y

    def _stats(self, analysis):
        t = np.array(analysis.time)
        y = np.array(analysis[self.sense_net])

        y_final = np.mean(y[-int(0.1 * len(y)):])

        y_peak = np.max(y)
        t_peak = t[np.argmax(y)]

        overshoot = ((y_peak - y_final) / y_final) * 100 if y_final != 0 else np.nan

        y10 = 0.1 * y_final
        y90 = 0.9 * y_final

        try:
            t10 = t[np.where(y >= y10)[0][0]]
            t90 = t[np.where(y >= y90)[0][0]]
            rise_time = t90 - t10
        except IndexError:
            rise_time = np.nan

        band = 0.02 * abs(y_final)

        settling_time = t[-1]
        for i in range(len(t)):
            if np.all(np.abs(y[i:] - y_final) <= band):
                settling_time = t[i]
                break

        y63 = 0.632 * y_final
        try:
            tau = t[np.where(y >= y63)[0][0]]
        except IndexError:
            tau = np.nan

        return {
            "final_value": float(y_final),
            "peak": float(y_peak),
            "overshoot_%": float(overshoot),
            "rise_time_s": float(rise_time),
            "settling_time_s": float(settling_time),
            "tau_s": float(tau),
            "peak_time_s": float(t_peak),
        }

    def plot(self):
        simulator = self.circuit.simulator(temperature=self.c.TEMP, nominal_temperature=self.c.TEMP)
        analysis = simulator.transient(step_time=self.c.TSTEP, end_time=self.c.TSTOP)

        print(self._stats(analysis))

        time = np.array(analysis.time)
        speed = np.array(analysis[self.sense_net])

        plt.figure()
        plt.plot(time * 1e3, speed, label="Speed (sense)")
        if hasattr(self, "drive_t"):
            plt.plot(self.drive_t * 1e3, self.drive_y, linestyle="--", label="Setpoint (input)")
        plt.xlabel("Time (ms)")
        plt.ylabel("Voltage")
        plt.title("PID Motor Response")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()





    def _motor(self, in_drive_voltage, out_speed):
        """
        Approximate electromechanical model of a dc motor.
        Reference:
        https://www.precisionmicrodrives.com/ab-025
        """
        # electrical
        self.circuit.L("Lmot", in_drive_voltage, "n_mot_L", self.c.L_MOT)
        self.circuit.V("senselmot", "n_mot_L", "n_mot1", 0)
        self.circuit.R("Rmot", "n_mot1", "n_mot2", self.c.R_MOT)
        self.circuit.BehavioralSource(
            "BEMF",
            "n_mot2",
            self.circuit.gnd,
            voltage_expression=f"{self.c.KEMF_MOT} * V({out_speed})"
        )
        # mechanical
        self.circuit.BehavioralSource(
            "TRQ",
            self.circuit.gnd,
            out_speed,
            current_expression=f"{self.c.KTRQ_MOT} * I(Vsenselmot)"
        )
        self.circuit.C("Jmot", out_speed, self.circuit.gnd, self.c.J_MOT)
        self.circuit.R("Rloss", out_speed, self.circuit.gnd, self.c.RLOSS_MOT)

        tau_e = self.c.L_MOT / self.c.R_MOT         # electrical time constant
        tau_m = self.c.J_MOT * self.c.RLOSS_MOT     # mechanical time constant

        return tau_e, tau_m
    
    def _proportional(self, in_net, out_net):
        # Rin
        self.circuit.R("Rin_prop", in_net, "inv_prop", self.c.RIN_PROP)
        # Rfb
        self.circuit.R("Rfb_prop", "inv_prop", out_net, self.c.RFB_PROP)
        # opamp
        self.circuit.X("U1", self.c.OPAMP, self.circuit.gnd, "inv_prop", out_net, "VCC", "VEE")

        kp = self.c.RFB_PROP / self.c.RIN_PROP

        return kp

    def _integrator(self, in_net, out_net):
        # RIN
        self.circuit.R("Rin_int", in_net, "inv_int", self.c.RIN_INT)
        # Rlim + C
        self.circuit.R("Rlim_int", "inv_int", "n_int", self.c.RLIM_INT)
        self.circuit.C("C_int", "n_int", out_net, self.c.C_INT)
        # opamp
        self.circuit.X("U2", self.c.OPAMP, self.circuit.gnd, "inv_int", out_net, "VCC", "VEE")

        ki = 1 / ((self.c.RIN_INT + self.c.RLIM_INT) * self.c.C_INT)

        return ki

    def _differentiator(self, in_net, out_net):
        # damping resistor
        self.circuit.R("Rdamp_diff", in_net, "n_diff_c", self.c.RDAMP_DIFF)
        # C
        self.circuit.C("C_diff", "n_diff_c", "inv_diff", self.c.C_DIFF)
        # R1
        self.circuit.R("Rin_diff", "inv_diff", out_net, self.c.R1_DIFF)
        # opamp
        self.circuit.X("U3", self.c.OPAMP, self.circuit.gnd, "inv_diff", out_net, "VCC", "VEE")

        kd = self.c.R1_DIFF * self.c.C_DIFF

        return kd

    def _summing(self, in_nets, out_net):
        # Rin1
        self.circuit.R("Rin1_sum", in_nets[0], "inv_sum", self.c.RIN1_SUM)
        # Rin2
        self.circuit.R("Rin2_sum", in_nets[1], "inv_sum", self.c.RIN2_SUM)
        # Rin3
        self.circuit.R("Rin3_sum", in_nets[2], "inv_sum", self.c.RIN3_SUM)
        # Rfb
        self.circuit.R("Rfb_sum", "inv_sum", out_net, self.c.RFB_SUM)
        # opamp
        self.circuit.X("U4", self.c.OPAMP, self.circuit.gnd, "inv_sum", out_net, "VCC", "VEE")

        gain = self.c.RFB_SUM * (1/self.c.RIN1_SUM + 1/self.c.RIN2_SUM + 1/self.c.RIN3_SUM)

        return gain

    def _error_amp(self, set_point_net, feedback_net, out_net):
        """
            ctrl_out = gsp * set_point - gfb * feedback        
        """
        # setpoint
        self.circuit.R("Rerr_sp1", set_point_net, "sub_np", self.c.R1_ERR)
        self.circuit.R("Rerr_sp2", "sub_np", self.circuit.gnd, self.c.R2_ERR)
        # feedback
        self.circuit.R("Rerr_fb1", feedback_net, "sub_inv", self.c.R3_ERR)
        self.circuit.R("Rerr_fbk", "sub_inv", out_net, self.c.R4_ERR)
        # opamp
        self.circuit.X("U_ERR", self.c.OPAMP, "sub_np", "sub_inv", out_net, "VCC", "VEE")

        Gfb = self.c.R4_ERR / self.c.R3_ERR
        Gsp = ((self.c.R3_ERR + self.c.R4_ERR) / self.c.R3_ERR) * (self.c.R2_ERR / (self.c.R1_ERR + self.c.R2_ERR))

        return Gsp, Gfb

    def _system(self):
        error_net     = "err_net"
        prop_out      = "prop_out"
        int_out       = "int_out"
        diff_out      = "diff_out"
        pid_out       = "pid_out"
        set_point_net = "set_point"
        speed_net     = "speed"

        k_err_sp, k_err_fb  = self._error_amp(set_point_net, speed_net, error_net)
        kp                  = self._proportional(error_net, prop_out)
        ki                  = self._integrator(error_net, int_out)
        kd                  = self._differentiator(error_net, diff_out)
        k_sum               = self._summing([prop_out, int_out, diff_out], pid_out)
        tau_e, tau_m        = self._motor(pid_out, speed_net)
        self.sense_net      = speed_net

        self._gains = (k_err_sp, k_err_fb, kp, ki, kd, k_sum, tau_e, tau_m)

        return self._gains
