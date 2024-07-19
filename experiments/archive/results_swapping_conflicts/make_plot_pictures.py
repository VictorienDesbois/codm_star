import pandas as pd
import matplotlib.pyplot as plt


def build_graph(graph_type):

    plt.rcParams.update({'font.size': 14})

    cols = []
    files = []

    if graph_type == "cost":
        cols = ['id ', ' cost']

    if graph_type == "time":
        cols = ['id ', ' time']

    if graph_type == "success":
        cols = [' nb_agents ', ' success_rate']

    files = ['office_{0}.csv'.format(graph_type), 
            'parallel_{0}.csv'.format(graph_type), 
            'pyramid_{0}.csv'.format(graph_type), 
            'smallobstacles_{0}.csv'.format(graph_type)]

    for i, f in enumerate(files):
        alg_2 = pd.read_csv('plots/CASTAR/' + f, sep=';', usecols=cols)
        alg_3 = pd.read_csv('plots/CODM/' + f, sep=';', usecols=cols)
        alg_4 = pd.read_csv('plots/CODMOPTI/' + f, sep=';', usecols=cols)

        if graph_type == "success":
            plt.plot(alg_2[cols[0]], alg_2[cols[1]], color="r", marker="x", linestyle="solid", label="WHCA*")
            plt.plot(alg_3[cols[0]], alg_3[cols[1]], color="c", marker="o", linestyle="dashed", label="CODM*")
            plt.plot(alg_4[cols[0]], alg_4[cols[1]], markerfacecolor='None', color="b", marker="D", linestyle="dotted", label="CODM*-opt")
        else:
            plt.plot(alg_2[cols[0]], alg_2[cols[1]], color="r", linestyle="solid", label="WHCA*")
            plt.plot(alg_3[cols[0]], alg_3[cols[1]], color="c", linestyle="dashed", label="CODM*")
            plt.plot(alg_4[cols[0]], alg_4[cols[1]], color="b", linestyle="dotted", label="CODM*-opt")

        picture_name = f.split(".")[0]

        if graph_type == "time":
            if i == 0:
                plt.legend(loc="upper left")
            plt.xlabel("Instances")
            plt.ylabel("Time (s)")
            plt.savefig("plots/pictures//{0}.pdf".format(picture_name))
            plt.show()

        if graph_type == "cost":
            if i == 0:
                plt.legend(loc="upper left")
            plt.xlabel("Instances")
            plt.ylabel("Cost (longest path size)")
            plt.savefig("plots/pictures//{0}.pdf".format(picture_name))
            plt.show()

        if graph_type == "success":
            if i == 0:
                plt.legend(loc="upper right")
            plt.xlabel("Nb agents")
            plt.ylabel("Success rate")
            plt.xlim((0, 300))
            plt.ylim((0, 1.1))
            plt.savefig("plots/pictures//{0}.pdf".format(picture_name))
            plt.show()

def main():
    build_graph("success")
    build_graph("time")
    build_graph("cost")

main()