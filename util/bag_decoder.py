import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import matplotlib.pyplot as plt


class BagFileParser:
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        # create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {
            name_of: get_message(type_of) for id_of, name_of, type_of in topics_data
        }

    # def __del__(self):
    #     self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
        ).fetchall()
        # Deserialise all and timestamp them
        return [
            (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]))
            for timestamp, data in rows
        ]


if __name__ == "__main__":
    bag_file = "1_0.db3"

    parser = BagFileParser(bag_file)

    encoder = parser.get_messages("/total_encoder/distance")
    front = parser.get_messages("/ultrasonic/front/distance")
    middle = parser.get_messages("/ultrasonic/middle/distance")
    back = parser.get_messages("/ultrasonic/back/distance")

    # print(encoder[1000][1].data)
    # print(front[1000][1].range)
    # print(middle[1000][1].range)
    # print(back[1000][1].range)

    encoder_distances = [encoder[i][1].data for i in range(len(encoder))]
    min_distances = [min(front[i][1].range, middle[i][1].range, back[i][1].range) for i in range(len(encoder))]

    plt.plot(encoder_distances, min_distances)

    plt.show()