
class CropFollower():


def follow_crop(self, current_distance):
        dist_to_crop_cm = 20

        if current_distance > dist_to_crop_cm + 10:
            robot.set_speed(left_speed=0.6)
            print("turn left")
        elif current_distance > dist_to_crop_cm + 5:
            robot.set_speed(left_speed=0.5)
            print("turn slightly left")
        elif current_distance < dist_to_crop_cm - 10:
            robot.set_speed(right_speed=0.6)
            print("turn right")
        elif current_distance < dist_to_crop_cm - -5:
            robot.set_speed(right_speed=0.5)
            print("turn slightly right")
        else:
            robot.set_speed()
            print("i'm straight")

    def turn_left_around_crop_row(self, enc_dist_cm):
        global CROP_ROWS
        # Drive length of robot to make sure it is clear of the crop row
        if(enc_dist_cm < length_of_robot_cm + enc_dist_cm):
            robot.motors.forward()
        # Now driven length of robot and clear for turning left around crop row
        self.turn_90_degrees(enc_dist_cm)
        CROP_ROWS = CROP_ROWS - 1

    def turn_90_degrees(self, prev_enc_dist_cm):
        while True:
            current_enc_dist_cm = self.left_encoder_distance / 100
            if current_enc_dist_cm < prev_enc_dist_cm - 240:
                robot.motors.left()

    def crop_following_algorithm(self):
        if self.left_encoder_distance is not None and self.ultrasonic_distance is not None:
            while(CROP_ROWS > 0):
                if(self.left_encoder_distance) < length_of_crop_row_cm:
                    self.follow_crop(self.ultrasonic_distance)
                else:
                    self.turn_left_around_crop_row(self.left_encoder_distance)