# PID dc motor speed controller

import PySpice
from PySpice.Spice.Netlist import Circuit
from PySpice.Unit import *
import matplotlib.pyplot as plt
import numpy as np


# import os
# os.environ["NGSPICE_LIBRARY_PATH"] = "/opt/homebrew/lib/libngspice.dylib"


# global variables for transient analysis
TSTEP       = 10@u_us
TSTOP       = 10@u_ms
# power rails
VCC         = 15@u_V
VEE         = -15@u_V
GND         = 0@u_V
# opamp
OPAMP       = "ua741"
# proportional circuit
RIN_PROP    = 10@u_kΩ
RFB_PROP    = 10@u_kΩ
# integral circuit
RIN_INT     = 10@u_kΩ
RLIM_INT    = 1@u_kΩ
C_INT       = 1@u_uF
# differentiation circuit
R1_DIFF     = 10@u_kΩ
C_DIFF      = 1@u_uF
# summing circuit
RIN1_SUM    = 100@u_kΩ
RIN2_SUM    = 100@u_kΩ
RIN3_SUM    = 100@u_kΩ
RFB_SUM     = 100@u_kΩ
# motor model
R_MOT       = 5.5@u_Ω
L_MOT       = 50@u_mH
KEMF_MOT    = 900*10e-6
J_MOT       = 15*10e-9
RLOSS_MOT   = 300@u_Ω
KTRQ_MOT    = 900*10e-6
# system
RERR1       = 10@u_kΩ
RERR2       = 10@u_kΩ
SETPOINT    = 10@u_V



class PID:
    def __init__(self):
        self.circuit     = Circuit("PID")
        self.set_point   = SETPOINT
        self.circuit.include("ua741.sub")

        # power rails
        self.circuit.V("VCC", "VCC", self.circuit.gnd, VCC)
        self.circuit.V("VEE", "VEE", self.circuit.gnd, VEE)

        self._build()
    

    def _motor(self, in_drive_voltage, out_speed):
        """
        Approximate electromechanical model of a dc motor.
        Reference:
        https://www.precisionmicrodrives.com/ab-025
        """
        # electrical
        self.circuit.L("Lmot", in_drive_voltage, "n_mot_L", L_MOT)
        self.circuit.V("senselmot", "n_mot_L", "n_mot1", 0)
        self.circuit.R("Rmot", "n_mot1", "n_mot2", R_MOT)
        self.circuit.BehavioralSource(
            "BEMF",
            "n_mot2",
            self.circuit.gnd,
            voltage_expression=f"{KEMF_MOT} * V({out_speed})"
        )
        # mechanical
        self.circuit.BehavioralSource(
            "TRQ",
            out_speed,
            self.circuit.gnd,
            current_expression=f"{KTRQ_MOT} * I(Vsenselmot)"
        )
        self.circuit.C("Jmot", out_speed, self.circuit.gnd, J_MOT)
        self.circuit.R("Rloss", out_speed, self.circuit.gnd, RLOSS_MOT)

        drive_net = in_drive_voltage
        sense     = out_speed

        return drive_net, sense

    def _proportional(self, in_net, out_net):
        # Rin
        self.circuit.R("Rin_prop", in_net, "inv_prop", RIN_PROP)
        # Rfb
        self.circuit.R("Rfb_prop", "inv_prop", out_net, RFB_PROP)
        # opamp
        self.circuit.X("U1", OPAMP, self.circuit.gnd, "inv_prop", out_net, "VCC", "VEE")
        


        kp = RFB_PROP / RIN_PROP

        return kp

    def _integrator(self, in_net, out_net):
        # RIN
        self.circuit.R("Rin_int", in_net, "inv_int", RIN_INT)
        # Rlim + C
        self.circuit.R("Rlim_int", "inv_int", "n_int", RLIM_INT)
        self.circuit.C("C_int", "n_int", out_net, C_INT)
        # opamp
        self.circuit.X("U2", OPAMP, self.circuit.gnd, "inv_int", out_net, "VCC", "VEE")

        ki = 1 / ((RIN_INT + RLIM_INT) * C_INT)

        return ki

    def _differentiator(self, in_net, out_net):
        # C
        self.circuit.C("C_diff", in_net, "inv_diff", C_DIFF)
        # R1
        self.circuit.R("Rin_diff", "inv_diff", out_net, R1_DIFF)
        # opamp
        self.circuit.X("U3", OPAMP, self.circuit.gnd, "inv_diff", out_net, "VCC", "VEE")

        kd = R1_DIFF * C_DIFF

        return kd

    def _summing(self, in_nets, out_net):
        # Rin1
        self.circuit.R("Rin1_sum", in_nets[0], "inv_sum", RIN1_SUM)
        # Rin2
        self.circuit.R("Rin2_sum", in_nets[1], "inv_sum", RIN2_SUM)
        # Rin3
        self.circuit.R("Rin3_sum", in_nets[2], "inv_sum", RIN3_SUM)
        # Rfb
        self.circuit.R("Rfb_sum", "inv_sum", out_net, RFB_SUM)
        # opamp
        self.circuit.X("U4", OPAMP, self.circuit.gnd, "inv_sum", out_net, "VCC", "VEE")
        
        gain = -RFB_SUM * (1/RIN1_SUM + 1/RIN2_SUM + 1/RIN3_SUM)

        return gain

    def _system(self):
        error_net     = "err_net"
        prop_out      = "prop_out"
        int_out       = "int_out"
        diff_out      = "diff_out"
        pid_out       = "pid_out"
        set_point_net = "set_point"
        feedback_net  = "feedback"

        kp   = self._proportional(error_net, prop_out)
        ki   = self._integrator(error_net, int_out)
        kd   = self._differentiator(error_net, diff_out)
        gain = self._summing([prop_out, int_out, diff_out], pid_out)

        self.circuit.R("Rerr1", set_point_net, error_net, RERR1)
        self.circuit.R("Rerr2", feedback_net,  error_net, RERR2)

        drive_net, self.sense_net = self._motor(pid_out, feedback_net)

        return kp, ki, kd, gain

    def _build(self):
        self.circuit.V("Vsp", "set_point", self.circuit.gnd, self.set_point)
        self._system()

        return self.circuit

    def _plot(self):
        simulator = self.circuit.simulator(temperature=25, nominal_temperature=25)
        analysis  = simulator.transient(step_time=TSTEP, end_time=TSTOP)

        plt.plot(analysis.time, analysis.nodes[self.sense_net])
        plt.show()
        
    
    def _step_response_metrics(self, time, sense):
        sense      = np.array(sense)
        time       = np.array(time)
        final_val  = sense[-1]

        # rise time 10% to 90%
        t10        = time[np.where(sense >= 0.10 * final_val)[0][0]]
        t90        = time[np.where(sense >= 0.90 * final_val)[0][0]]
        rise_time  = t90 - t10
        # overshoot
        peak       = np.max(sense)
        overshoot  = ((peak - final_val) / final_val) * 100

        # settling time last time in ±2% band of final value
        band       = 0.02 * final_val
        outside    = np.where(np.abs(sense - final_val) > band)[0]
        settle_time = time[outside[-1]] if len(outside) else time[0]

        # time constant tau; time to reach 63.2% of final value
        tau_val    = 0.632 * final_val
        tau        = time[np.where(sense >= tau_val)[0][0]]

        return {
            "rise_time"    : rise_time,
            "overshoot_pct": overshoot,
            "settling_time": settle_time,
            "tau"          : tau,
        }

    def plot(self):
        simulator = self.circuit.simulator(temperature=25, nominal_temperature=25)
        analysis  = simulator.transient(step_time=TSTEP, end_time=TSTOP)

        # Convert PySpice UnitValues -> float arrays
        time  = np.array(analysis.time, dtype=float)
        sense = np.array(analysis.nodes[self.sense_net], dtype=float)

        metrics = self._step_response_metrics(time, sense)

        print(f"rise time     : {metrics['rise_time']*1e3:.3f} ms")
        print(f"overshoot     : {metrics['overshoot_pct']:.2f} %")
        print(f"settling time : {metrics['settling_time']*1e3:.3f} ms")
        print(f"tau           : {metrics['tau']*1e3:.3f} ms")

        final_val = sense[-1]

        plt.plot(time, sense)

        plt.axhline(final_val,
                    linestyle="--", color="gray", label="final")

        plt.axhline(final_val * 1.02,
                    linestyle=":", color="orange", label="+2%")

        plt.axhline(final_val * 0.98,
                    linestyle=":", color="orange", label="-2%")

        plt.xlabel("Time (s)")
        plt.ylabel("Speed (V)")
        plt.title("Motor Speed Response")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    pid = PID()
    print(pid.circuit)
    pid.__delattr__plot()