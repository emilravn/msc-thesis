import numpy as np
import matplotlib.pyplot as plt
from bag_decoder import BagFileParser

if __name__ == "__main__":
    cases = ['case1', 'case2', 'case3', 'case4', 'case5', 'case6', 'case7', 'case8', 'case9', 'case10']
    output_folder = "3-square"

    # Store all data for summary plot
    summary_data = []

    for case in cases:

        plt.figure()

        bag_file = "../experiments/3-square/{}/{}_0.db3".format(case, case)

        parser = BagFileParser(bag_file)

        left_encoder = parser.get_messages("/left_encoder/distance")
        right_encoder = parser.get_messages("/right_encoder/distance")
        total_encoder = parser.get_messages("/total_encoder/distance")

        left_encoder_timestamps = [(left_encoder[i][0] - left_encoder[0][0]) / 1e9 for i in range(len(left_encoder))]
        left_encoder_distances = [left_encoder[i][1].data for i in range(len(left_encoder))]

        right_encoder_timestamps = [(right_encoder[i][0] - right_encoder[0][0]) / 1e9 for i in range(len(right_encoder))]
        right_encoder_distances = [right_encoder[i][1].data for i in range(len(right_encoder))]

        total_encoder_timestamps = [(total_encoder[i][0] - total_encoder[0][0]) / 1e9 for i in range(len(total_encoder))]
        total_encoder_distances = [total_encoder[i][1].data for i in range(len(total_encoder))]

        plt.plot(left_encoder_timestamps, left_encoder_distances, label="Left encoder")
        plt.plot(right_encoder_timestamps, right_encoder_distances, label="Right encoder")
        plt.plot(total_encoder_timestamps, total_encoder_distances, label="Total encoder")

        plt.xlabel('Time (s)')
        plt.ylabel('Distance driven (cm)')
        plt.title(f'Straight line with stops experiment for {case}')
        plt.xlim(0, 22)  # Adjust the y-axis to range from 0 to 50

        plt.legend(loc='upper left')  # Legend moved to outside of plot

        plt.savefig(f'{output_folder}/{case}_plot.png')

        plt.close()

        # Store data for summary plot
        summary_data.append((total_encoder_timestamps, total_encoder_distances, case))

    # Summary plot
    plt.figure(figsize=(14, 10))

    for timestamps, distances, case in summary_data:
        plt.plot(timestamps, distances, label=case)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Distance driven')
    plt.title('Summary of all cases')
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))  # Legend moved to outside of plot
    plt.savefig(f'{output_folder}/summary_plot.png')
