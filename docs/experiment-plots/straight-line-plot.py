import numpy as np
import matplotlib.pyplot as plt
from bag_decoder import BagFileParser


def individual_summary_boxplot(summary_data_list, output_destination_folder, cases, fig_width=10, fig_height=6):
    plt.figure(figsize=(fig_width, fig_height))
    # data[1] is ultrasonic distance
    plt.boxplot([data[1] for data in summary_data_list], labels=cases)
    plt.xlabel("Case")
    plt.ylabel("Distance to wall")
    plt.title("Individual summary box plot")
    plt.tight_layout()

    plt.savefig(f"{output_destination_folder}/individual_box_plots.png")

def summary_boxplot(all_min_distances, output_destination_folder, fig_width=10, fig_height=6):
    plt.figure(figsize=(fig_width, fig_height))
    plt.boxplot(all_min_distances, showfliers=False)
    plt.xlabel('All cases')
    plt.ylabel('Distance to wall')
    plt.title('Summary box plot')
    plt.tight_layout()

    plt.savefig(f"{output_destination_folder}/summary_box_plot.png")

def main():
    cases = [
        "case1",
        "case2",
        "case3",
        "case4",
        "case5",
        "case6",
        "case7",
        "case8",
        "case9",
        "case10",
    ]
    output_folder = "1-straight-line"

    # Store all data for summary plot
    summary_data = []
    all_min_distances = []

    for case in cases:
        plt.figure()

        bag_file = "../experiments/1-straight-line/{}/{}_0.db3".format(case, case)

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

        plt.plot(encoder_filtered, min_distances_filtered)
        plt.xlabel("Distance driven (cm)")
        plt.ylabel("Distance to wall (cm)")
        plt.title(f"Straight line experiment for {case}")

        plt.savefig(f"{output_folder}/{case}_plot.png")

        plt.close()

        # Store data for summary plot
        summary_data.append((encoder_filtered, min_distances_filtered))
        all_min_distances.extend(min_distances_filtered)

    # Summary plot
    plt.figure(figsize=(14, 10))

    for i, (encoder_filtered, min_distances_filtered) in enumerate(summary_data):
        plt.plot(encoder_filtered, min_distances_filtered, label=cases[i])

    # Add acceptance range lines
    plt.axhline(
        y=18, color="black", linestyle="--", label="Lower/Upper acceptance limit (18/22 cm)"
    )
    plt.axhline(y=22, color="black", linestyle="--")

    plt.xlabel("Distance driven (cm)")
    plt.ylabel("Distance to wall (cm)")
    plt.title("Straight line experiment")
    plt.legend(loc="lower right")  # Legend moved to outside of plot

    plt.savefig(f"{output_folder}/summary_plot.png")

    # Individual cases box plot
    individual_summary_boxplot(summary_data, output_folder, cases)
    # Summarizing box plot
    summary_boxplot(all_min_distances, output_folder)


if __name__ == "__main__":
    main()
