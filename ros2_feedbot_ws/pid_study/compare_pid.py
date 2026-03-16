#!/usr/bin/env python3

import rosbag2_py
import rclpy.serialization
from rosidl_runtime_py.utilities import get_message

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

plt.style.use("ggplot")

JOINTS = ["joint1","joint2","joint3","joint4"]

TUNED_BAG = "tuned/all_joints/tuned_alljoints"
UNTUNED_BAG = "untuned/all_joints/untuned_alljoints"


# ------------------------------------------------
# Step interpolation
# ------------------------------------------------

def step_interpolate(t_ref,v_ref,t_target):

    if len(t_ref) == 0:
        return np.zeros_like(t_target)

    result = np.zeros(len(t_target))
    idx = 0

    for i,t in enumerate(t_target):

        while idx+1 < len(t_ref) and t_ref[idx+1] <= t:
            idx += 1

        result[i] = v_ref[idx]

    return result


# ------------------------------------------------
# Performance metrics
# ------------------------------------------------

def compute_metrics(time,desired,actual):

    final = desired[-1]

    error = desired - actual

    rmse = np.sqrt(np.mean(error**2))

    iae = np.trapz(np.abs(error),time)

    sse = abs(final-actual[-1])

    overshoot = max(0,(np.max(actual)-final)/abs(final)*100)

    # Rise time
    rise_time = np.nan

    try:

        t10 = time[np.where(actual>=0.1*final)[0][0]]
        t90 = time[np.where(actual>=0.9*final)[0][0]]

        rise_time = t90 - t10

    except:
        pass


    # Settling time (5%)
    settling_time = np.nan
    tol = 0.05*abs(final)

    for i in range(len(actual)):

        if np.all(np.abs(actual[i:]-final)<=tol):

            settling_time = time[i]
            break


    return rise_time,overshoot,settling_time,rmse,iae,sse


# ------------------------------------------------
# Read ROS2 bag
# ------------------------------------------------

def read_bag(bag_path):

    storage = rosbag2_py.StorageOptions(uri=bag_path,storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage,converter)

    topics = reader.get_all_topics_and_types()
    topic_types = {t.name:t.type for t in topics}

    joint_state_type = get_message(topic_types["/joint_states"])

    ref_types = {}

    for j in JOINTS:

        topic=f"/{j}_controller/reference"

        if topic in topic_types:

            ref_types[topic]=get_message(topic_types[topic])

    data={j:{"ref_t":[],"ref_v":[],"act_t":[],"act_v":[]} for j in JOINTS}

    while reader.has_next():

        topic,data_raw,stamp = reader.read_next()

        t=stamp*1e-9

        if topic=="/joint_states":

            msg=rclpy.serialization.deserialize_message(data_raw,joint_state_type)

            for j in JOINTS:

                if j in msg.name:

                    idx=msg.name.index(j)

                    data[j]["act_t"].append(t)
                    data[j]["act_v"].append(msg.position[idx])


        if topic in ref_types:

            msg=rclpy.serialization.deserialize_message(data_raw,ref_types[topic])

            if len(msg.values)>0:

                j=topic.split("/")[1].replace("_controller","")

                if j in msg.dof_names:

                    idx=msg.dof_names.index(j)

                    data[j]["ref_t"].append(t)
                    data[j]["ref_v"].append(msg.values[idx])


    for j in JOINTS:

        data[j]["ref_t"]=np.array(data[j]["ref_t"])
        data[j]["ref_v"]=np.array(data[j]["ref_v"])

        data[j]["act_t"]=np.array(data[j]["act_t"])
        data[j]["act_v"]=np.array(data[j]["act_v"])

        if len(data[j]["ref_t"])<2:
            continue

        change=np.where(np.diff(data[j]["ref_v"])!=0)[0]

        if len(change)>0:
            step_time=data[j]["ref_t"][change[0]+1]
        else:
            step_time=data[j]["ref_t"][0]

        data[j]["ref_t"]-=step_time
        data[j]["act_t"]-=step_time


    return data


# ------------------------------------------------
# Plot analysis
# ------------------------------------------------

def analyze_joint(joint,data_u,data_t,pdf):

    ref_u_t=data_u[joint]["ref_t"]
    ref_u_v=data_u[joint]["ref_v"]
    act_u_t=data_u[joint]["act_t"]
    act_u_v=data_u[joint]["act_v"]

    ref_t_t=data_t[joint]["ref_t"]
    ref_t_v=data_t[joint]["ref_v"]
    act_t_t=data_t[joint]["act_t"]
    act_t_v=data_t[joint]["act_v"]


    desired_u=step_interpolate(ref_u_t,ref_u_v,act_u_t)
    desired_t=step_interpolate(ref_t_t,ref_t_v,act_t_t)


    # normalize starting point
    act_u_v=act_u_v-act_u_v[0]
    act_t_v=act_t_v-act_t_v[0]

    desired_u=desired_u-desired_u[0]
    desired_t=desired_t-desired_t[0]


    error_u=desired_u-act_u_v
    error_t=desired_t-act_t_v


    rise_u,os_u,set_u,rmse_u,iae_u,sse_u = compute_metrics(act_u_t,desired_u,act_u_v)
    rise_t,os_t,set_t,rmse_t,iae_t,sse_t = compute_metrics(act_t_t,desired_t,act_t_v)


    fig,axs=plt.subplots(2,1,figsize=(10,8),sharex=True)


    # Position
    axs[0].step(act_u_t,desired_u,"--",label="Desired")
    axs[0].plot(act_u_t,act_u_v,"r",label="Actual (Untuned)")
    axs[0].plot(act_t_t,act_t_v,"k",linewidth=2,label="Actual (Tuned)")

    axs[0].set_ylabel("Joint Position [rad]")
    axs[0].set_title(f"PID Tuning Comparison — {joint}")
    axs[0].legend(loc="upper right")


    # Error
    axs[1].plot(act_u_t,error_u,label="Untuned Error")
    axs[1].plot(act_t_t,error_t,label="Tuned Error")

    axs[1].set_xlabel("Time [s]")
    axs[1].set_ylabel("Tracking Error [rad]")
    axs[1].legend(loc="upper right")


    axs[0].set_xlim(-1,4)
    axs[1].set_xlim(-1,4)


    metrics_text=(

        f"Step Response Metrics\n"
        f"Untuned → Rise:{rise_u:.2f}s  OS:{os_u:.1f}%  RMSE:{rmse_u:.3f}  SSE:{sse_u:.4f}\n"
        f"Tuned → Rise:{rise_t:.2f}s  OS:{os_t:.1f}%  RMSE:{rmse_t:.3f}  SSE:{sse_t:.4f}"

    )


    axs[0].text(

        0.02,0.60,
        metrics_text,
        transform=axs[0].transAxes,
        fontsize=9,
        bbox=dict(boxstyle="round",facecolor="white",alpha=0.9)

    )


    plt.tight_layout()

    plt.savefig(f"pid_{joint}.png",dpi=300)
    pdf.savefig()

    plt.close()


    return rise_t,os_t,set_t,rmse_t,iae_t,sse_t


# ------------------------------------------------
# MAIN
# ------------------------------------------------

def main():

    print("Reading untuned bag...")
    data_u=read_bag(UNTUNED_BAG)

    print("Reading tuned bag...")
    data_t=read_bag(TUNED_BAG)

    results=[]

    with PdfPages("pid_results.pdf") as pdf:

        for j in JOINTS:

            print("Processing",j)

            metrics=analyze_joint(j,data_u,data_t,pdf)

            results.append((j,*metrics))


    print("\nPID Performance Summary\n")

    print("Joint | Rise | Overshoot | Settling | RMSE | IAE | SSE")

    for r in results:

        print(f"{r[0]} | {r[1]:.3f} | {r[2]:.2f} | {r[3]:.3f} | {r[4]:.4f} | {r[5]:.4f} | {r[6]:.4f}")


if __name__=="__main__":
    main()
