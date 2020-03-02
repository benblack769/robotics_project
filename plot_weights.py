import pandas
import argparse
import numpy as np
import matplotlib.pyplot as plt

def plot_map(data):
    pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='plots output of game_calc csv file')
    parser.add_argument('csv_fname', type=str, help='time series csv file')
    args = parser.parse_args()

    data = pandas.read_csv(args.csv_fname)
    num_updates = 1250
    xvals = np.arange(len(data.guard_rew)) * num_updates
    data["x"] = xvals
    plt.plot('x', 'guard_rew',data=data , marker='o', markerfacecolor='blue', markersize=4, color='skyblue', linewidth=2)
    plt.plot('x', 'thief_rew',data=data, marker='o', markerfacecolor='red', markersize=4, color='#ff9999', linewidth=2)
    plt.plot('x', 'guard_report_reward',data=data, marker='o', markerfacecolor='blue', markersize=4, color='blue', linewidth=2)
    plt.plot('x', 'thief_report_reward',data=data, marker='o', markerfacecolor='red', markersize=4, color='red', linewidth=2)
    plt.legend()

    #plt.show()
    plt.savefig(args.csv_fname+".png")

#guard_rew,thief_rew,guard_report_reward,thief_report_reward
