import cv2
import numpy as np
import os
import rosbag
from cv_bridge import CvBridge


FILENAME = 'congo'
ROOT_DIR = '/home/de/landmarks'
BAGFILE = ROOT_DIR + '/' + FILENAME + '.bag'

if __name__ == '__main__':
    bag = rosbag.Bag(BAGFILE)
    
    TOPIC = '/head_camera/rgb/image_raw'
    DESCRIPTION = 'camera_'
    image_topic = bag.read_messages(TOPIC)

    for k, b in enumerate(image_topic):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(b.message, b.message.encoding)
        cv_image.astype(np.uint8)
        descr = str(b.timestamp).zfill(11)
        cv2.imwrite(ROOT_DIR + '/camera/' + descr + '.png', cv_image)
        print('saved: ' + str(b.timestamp) + '.png')
    bag.close()

    image_folder = '/home/de/landmarks/camera'
    video_name = 'camera.avi'

    images = [img for img in sorted(os.listdir(image_folder)) if img.endswith(".png")]
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, 0, 10, (width,height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()

    print('PROCESS COMPLETE')