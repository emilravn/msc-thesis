import numpy as np
import statistics
import matplotlib.pyplot as plt
from bag_decoder import BagFileParser


plot_kp_titles = ["0.005", "0.01", "0.02", "0.05", "0.07", "0.1", "0.2", "0.3", "0.15"]
plot_kd_titles = ["0.1", "0.05", "0.01", "0.2", "0.4", "0.8", "1.0", "2.0", "5.0"]
plot_ki_titles = ["0.4", "0.2", "0.1", "0.05", "0.01", "0.001"]
output_folder = "pid-plots/ki"


def individual_summary_boxplot(
    gain_constant, summary_data_list, output_destination_folder, pid_values,
    fig_width=None, fig_height=None
):
    if fig_width and fig_height:
        plt.figure(figsize=(fig_width, fig_height))
    else:
        plt.figure()

    # data[1] is ultrasonic distance
    plt.boxplot([data[1] for data in summary_data_list], labels=pid_values)
    plt.xlabel(f"{gain_constant} value")
    plt.ylabel("Distance to wall (cm)")
    plt.title(f"Individual summary box plot for {gain_constant} values")
    plt.tight_layout()

    plt.savefig(f"{output_destination_folder}/individual_box_plots.png")


def summary_boxplot(
        gain_constant, all_min_distances, output_destination_folder,
        fig_width=None, fig_height=None
):
    if fig_width and fig_height:
        plt.figure(figsize=(fig_width, fig_height))
    else:
        plt.figure()

    plt.boxplot(all_min_distances, showfliers=False)
    plt.xlabel("All cases")
    plt.ylabel("Distance to wall (cm)")
    plt.title(f"Summary Box Plot for {gain_constant} values")
    plt.tight_layout()

    plt.savefig(f"{output_destination_folder}/summary_box_plot.png")


def summary_plot(
        gain_constant, summary_data, plot_titles, output_folder, fig_width=None, fig_height=None
):
    if fig_width and fig_height:
        plt.figure(figsize=(fig_width, fig_height))
    else:
        plt.figure()

    for i, (encoder_filtered, min_distances_filtered) in enumerate(summary_data):
        plt.plot(encoder_filtered, min_distances_filtered, label=plot_titles[i])

    plt.xlabel("Distance driven (cm)")
    plt.ylabel("Distance to wall (cm)")
    plt.ylim(top=40)
    plt.title(f"PID experiment for {gain_constant} values")
    plt.legend(loc="lower right")

    plt.savefig(f"{output_folder}/summary_plot.png")


def write_dict_to_file(stats_dict, filename):
    with open(filename, 'w') as file:
        for key, value in stats_dict.items():
            file.write(f"{key}: {value}\n")


def summary_statistics(gain_constant, case_nr, data_list):
    # ensure list is made of floats
    data_list = list(map(float, data_list))
    data_list.sort()

    # calculate statistics
    stats_dict = {}
    stats_dict['gain_constant_case'] = f'{gain_constant}_{case_nr}'

    stats_dict['mean'] = statistics.mean(data_list)
    stats_dict['median'] = statistics.median(data_list)
    stats_dict['standard_deviation'] = statistics.stdev(data_list)
    stats_dict['minimum'] = min(data_list)
    stats_dict['maximum'] = max(data_list)
    stats_dict['range'] = stats_dict['maximum'] - stats_dict['minimum']
    stats_dict['midrange'] = (stats_dict['maximum'] + stats_dict['minimum']) / 2
    try:
        stats_dict['mode'] = statistics.mode(data_list)
    except statistics.StatisticsError:
        stats_dict['mode'] = None

    # calculate quartiles
    q1 = np.percentile(data_list, 25)
    q3 = np.percentile(data_list, 75)
    stats_dict['q1'] = q1
    stats_dict['q3'] = q3
    stats_dict['interquartile_range'] = q3 - q1

    # size
    stats_dict['size'] = len(data_list)

    return stats_dict


def plot_pid_experiments(gain_constant, plot_titles, output_folder):
    cases = [
        "1",
        "2",
        "3",
        "4",
        "5",
        "6",
        "7",
        "8",
        "9",
    ]

    # Store all data for summary plot
    summary_data = []
    all_min_distances = []

    for case, pid_value in zip(cases, plot_titles):
        plt.figure()

        bag_file = f"../experiments/pid-experiments/{gain_constant}/{case}/{case}_0.db3"

        parser = BagFileParser(bag_file)

        encoder = parser.get_messages("/total_encoder/distance")
        front = parser.get_messages("/ultrasonic/front/distance")
        middle = parser.get_messages("/ultrasonic/middle/distance")
        back = parser.get_messages("/ultrasonic/back/distance")

        encoder_timestamps = [encoder[i][0] for i in range(len(encoder))]
        encoder_distances = [encoder[i][1].data for i in range(len(encoder))]

        front_timestamps = [front[i][0] for i in range(len(front))]
        front_distances = [front[i][1].range for i in range(len(front))]

        middle_timestamps = [middle[i][0] for i in range(len(middle))]
        middle_distances = [middle[i][1].range for i in range(len(middle))]

        back_timestamps = [back[i][0] for i in range(len(back))]
        back_distances = [back[i][1].range for i in range(len(back))]

        front_distances_interp = np.interp(encoder_timestamps, front_timestamps, front_distances)
        middle_distances_interp = np.interp(
            encoder_timestamps, middle_timestamps, middle_distances
        )
        back_distances_interp = np.interp(encoder_timestamps, back_timestamps, back_distances)

        encoder_filtered = []
        min_distances_filtered = []
        for i in range(len(encoder)):
            encoder_value = encoder_distances[i]
            front_value = front_distances_interp[i]
            middle_value = middle_distances_interp[i]
            back_value = back_distances_interp[i]

            if encoder_value != 0:
                encoder_filtered.append(encoder_value)
                min_distance = min(front_value, middle_value, back_value)
                min_distances_filtered.append(min_distance)

        # plots for individual cases
        plt.plot(encoder_filtered, min_distances_filtered)
        plt.xlabel("Distance driven (cm)")
        plt.ylabel("Distance to wall (cm)")
        plt.title(f"PID experiment for {gain_constant} value: {pid_value}")

        plt.savefig(f"{output_folder}/{case}_plot.png")

        plt.close()

        # Store data for summary plot
        summary_data.append((encoder_filtered, min_distances_filtered))
        all_min_distances.extend(min_distances_filtered)

        # descriptive statistics for each case
        print(f"Computing statistics for {gain_constant} case {case} ...")
        distance_stats_dict = summary_statistics(gain_constant, case, min_distances_filtered)
        descriptive_stats_filename = output_folder+f"/{gain_constant}_{case}_stats.txt"
        write_dict_to_file(distance_stats_dict, descriptive_stats_filename)

    # Summary plot of all cases
    summary_plot(gain_constant, summary_data, plot_titles, output_folder)

    # Individual cases box plot
    individual_summary_boxplot(gain_constant, summary_data, output_folder, plot_titles)

    # Summarizing box plot of all cases
    summary_boxplot(gain_constant, all_min_distances, output_folder)


if __name__ == "__main__":
    print("Plotting kp ...")
    plot_pid_experiments("kp", plot_kp_titles, "pid-plots/kp")
    print("kp done!")

    print("Plotting kd ...")
    plot_pid_experiments("kd", plot_kd_titles, "pid-plots/kd")
    print("kd done!")

    print("Plotting ki ...")
    plot_pid_experiments("ki", plot_ki_titles, "pid-plots/ki")
    print("ki done!")
