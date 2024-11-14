import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename):
    
    headers, values=FileReader(filename[0]).read_file()
    poseHeaders, poses=FileReader(filename[1]).read_file()
    print(filename[1])
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    pose_time_list = []
    pose_first_stamp = poses[0][-1]

    for pose in poses:
        pose_time_list.append(pose[-1] - pose_first_stamp)

    
    fig, axes = plt.subplots(2,1, figsize=(14,6))


    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values], label = "Estimates")
    axes[0].plot([lin[0] for lin in poses], [lin[1] for lin in poses], label = "Ground Truth")
    axes[0].set_title("state space")
    axes[0].legend()
    axes[0].grid()

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    axes[1].legend()
    axes[1].grid()

    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    plot_errors(filenames)
    # for filename in filenames:
    #     plot_errors(filename)


