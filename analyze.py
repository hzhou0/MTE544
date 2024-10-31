import csv
from pathlib import Path

import matplotlib.pyplot as plt
from PIL.GimpGradientFile import linear


def csv_to_dict(file: Path):
    x = {}
    with open(file) as csvfile:
        reader = csv.DictReader(csvfile)
        for header in reader.fieldnames:
            if header.strip():
                x[header.strip()] = []

        for row in reader:
            for header, value in row.items():
                if header.strip():
                    x[header.strip()].append(float(value.strip()))
    return x


def relative_time(ns: list[int]):
    return [(t - ns[0]) * 1e-9 for t in ns]


def plot(root_data_dir: Path):
    def plot_point_controller(data_dir: Path, controller_type: str):
        angular_data = csv_to_dict(data_dir / "angular.csv")
        linear_data = csv_to_dict(data_dir / "linear.csv")
        robot_pose = csv_to_dict(data_dir / "robot_pose.csv")

        # Create a figure and define the layout
        fig, axs = plt.subplots(2, 2, figsize=(18, 10))  # 2 rows, 3 columns of subplots
        fig.suptitle(f"Point Controller ({controller_type})", fontsize=16)

        error_t = axs[0, 0]
        error_t.plot(
            relative_time(angular_data["stamp"]),
            angular_data["e"],
            label="Angular Error [rad]",
        )
        error_t.plot(
            relative_time(angular_data["stamp"]),
            angular_data["e_dot"],
            label="Angular Error Dot [rad/s]",
        )
        error_t.plot(
            relative_time(linear_data["stamp"]),
            linear_data["e"],
            label="Linear Error [m]",
        )
        error_t.plot(
            relative_time(linear_data["stamp"]),
            linear_data["e_dot"],
            label="Linear Error Dot [m/s]",
        )
        error_t.set_title("Error vs Time")
        error_t.set_xlabel("Time [s]")
        error_t.set_ylabel("Error")
        error_t.legend(loc="best")
        error_t.grid(True)

        state_t = axs[0, 1]
        state_t.plot(
            relative_time(robot_pose["stamp"]),
            robot_pose['x'],
            label="x [m]",
        )
        state_t.plot(
            relative_time(robot_pose["stamp"]),
            robot_pose["y"],
            label="y [m]",
        )
        state_t.plot(
            relative_time(robot_pose["stamp"]),
            robot_pose["theta"],
            label="theta [rad]",
        )
        state_t.set_title("State vs Time")
        state_t.set_xlabel("Time [s]")
        state_t.set_ylabel("State")
        state_t.legend(loc="best")
        state_t.grid(True)

        traj = axs[1, 0]
        traj.plot(
            robot_pose['x'],
            robot_pose['y'],
            label="Trajectory",
        )
        traj.set_title("x vs y")
        traj.set_xlabel("x [m]")
        traj.set_ylabel("y [m]")
        traj.legend(loc="best")
        traj.grid(True)

        e_rel = axs[1, 1]
        e_rel.plot(
            linear_data['e'],
            linear_data['e_dot'],
            label="Linear[m]",
        )
        e_rel.plot(
            angular_data['e'],
            angular_data['e_dot'],
            label="Angular[rad]",
        )
        e_rel.set_title("Error vs Error Dot")
        e_rel.set_xlabel("Error [m]/[rad]")
        e_rel.set_ylabel("Error Dot [m]/[rad]")
        e_rel.legend(loc="best")
        e_rel.grid(True)

        plt.show()


    def plot_traj_controller(data_dir: Path, traj_type: str):
        angular_data = csv_to_dict(data_dir / "angular.csv")
        linear_data = csv_to_dict(data_dir / "linear.csv")
        robot_pose = csv_to_dict(data_dir / "robot_pose.csv")

        fig=plt.figure()
        plt.plot(
            robot_pose['x'],
            robot_pose['y'],
            label="Trajectory",
        )
        plt.title(f"{traj_type} Trajectory Plot")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.legend(loc="best")
        plt.grid(True)

        plt.show()


    plot_point_controller(root_data_dir / "point_p", "P")
    plot_point_controller(root_data_dir / "point_pid", "PID")

    plot_traj_controller(root_data_dir/"sigmoid", "Sigmoid")
    plot_traj_controller(root_data_dir/"parabola", "Parabola")

def calc_metrics(data_dir: Path):
    angular_data = csv_to_dict(data_dir / "angular.csv")
    linear_data = csv_to_dict(data_dir / "linear.csv")
    robot_pose = csv_to_dict(data_dir / "robot_pose.csv")

    t_10percent=t_90percent=0
    start_e=linear_data['e'][0]
    for i,t in enumerate(linear_data["stamp"]):
        if t_10percent==0 and linear_data['e'][i]<start_e*0.9:
            t_10percent=t
        if t_90percent==0 and linear_data['e'][i]<start_e*0.1:
            t_90percent=t
            break
    rise_time=t_90percent-t_10percent

    overshoot=abs(angular_data['e'][-1])/abs(angular_data['e'][0])
    ss_error=abs(angular_data['e'][-1])
    print(rise_time*1e-9)
    print(overshoot)
    print(ss_error)



if __name__ == "__main__":
    script_dir = Path(__file__).parent
    #plot(script_dir / "data")
    calc_metrics(script_dir/'data'/'point_p')
    calc_metrics(script_dir/'data'/'point_pid')
