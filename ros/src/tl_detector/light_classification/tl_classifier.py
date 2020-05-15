from styx_msgs.msg import TrafficLight
import cv2

SAVE_EVERY = 4


class TLClassifier(object):
    def __init__(self):
        # TODO load classifier

        self.save_idx = 40 * SAVE_EVERY
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # TODO implement light color prediction

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image, keep only the red pixels
        lower_red_hue_range = cv2.inRange(hsv_image, (0, 100, 100), (10, 255, 255))
        upper_red_hue_range = cv2.inRange(hsv_image, (160, 100, 100), (179, 255, 255))
        mask = cv2.bitwise_or(lower_red_hue_range, upper_red_hue_range)

        num_of_red_pixels = cv2.countNonZero(mask)

        if num_of_red_pixels >= 200:
            return TrafficLight.RED

        return TrafficLight.UNKNOWN

    def save_labeled(self, image, state, diff):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            :param state:
            :param image:
            :param diff:
        """
        if diff <= 60:
            if self.save_idx % SAVE_EVERY == 0:
                cv2.imwrite('./images/img_{0:01d}_{1:08d}_{2:08d}.png'.format(state, diff, self.save_idx / SAVE_EVERY),
                            image)
            self.save_idx += 1
        # TODO implement light color prediction
        return TrafficLight.UNKNOWN
