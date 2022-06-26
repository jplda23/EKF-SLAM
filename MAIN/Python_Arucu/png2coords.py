import cv2, PIL
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from cv2 import aruco
from scipy.io import savemat

ROOT_DIR = '/home/de/landmarks'

def pose_estimation(frame, k, d):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters =  aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, cameraMatrix=k, distCoeff=d)   

    # If markers are detected
    if len(corners) > 0:
        dist = np.zeros((len(ids), 3))
        arucoid = np.zeros((len(ids), 1))
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.2, k, d)
            dist[i] = tvec[0][0]
            arucoid[i] = ids[i]
            # Draw a square around the markers
            aruco.drawDetectedMarkers(frame, corners, ids) 
            # Draw Axis
            aruco.drawAxis(frame, k, d, rvec, tvec, 0.1)

        return frame, arucoid, corners, dist

    else:
        dist = np.matrix([float('NaN'), float('NaN'), float('NaN')])
        arucoid = np.matrix([float('NaN')])
        return gray, arucoid, corners, dist

if __name__ == '__main__':

    k = np.matrix([[554.254691191187, 0, 320.500000000000], [0, 554.254691191187, 240.500000000000], [0, 0, 1]])
    d = np.array([0.0,  0.0, 0.0,  0.0, 0.0])   

    image_folder = '/home/de/landmarks/camera'

    images = [img for img in sorted(os.listdir(image_folder)) if img.endswith(".png")]  
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    data = np.zeros([16*len(images), 4])
    j = 0

    for image in images:

        timestamp = image[:-4]
        frame = cv2.imread(os.path.join(image_folder, image))
        
        output, arucoid, corners, dist = pose_estimation(frame, k, d)

        cv2.imshow('Estimated Pose', output)
        cv2.imwrite(ROOT_DIR + '/aruco/' + timestamp + '.png', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        for s in range(0, len(arucoid)):

            data[j,0] = int(timestamp)*10**(-9)
            data[j,1] = arucoid[s][0]
            data[j,2] = dist[s, 0]
            data[j,3] = dist[s, -1]
            j += 1

    data = data[0:j]

    image_folder = '/home/de/landmarks/aruco'
    video_name = 'aruco.avi'

    images = [img for img in sorted(os.listdir(image_folder)) if img.endswith(".png")]
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, 0, 10, (width,height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()

    savemat("sensor_data11.mat", {"array": data})