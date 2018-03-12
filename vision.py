import math

import cv2
import numpy as np


def main():
    import cscore
    from networktables import NetworkTables
    from time import monotonic

    cs = cscore.CameraServer.getInstance()
    camera = cs.startAutomaticCapture()  # TODO: specify path if multiple cameras
    camera.setVideoMode(cscore.VideoMode.PixelFormat.kYUYV, 320, 240, 50)

    camera.getProperty('vertical_flip').set(True)
    camera.getProperty('horizontal_flip').set(True)
    camera.getProperty('gain_automatic').set(False)
    camera.getProperty('gain').set(30)
    camera.getProperty('saturation').set(32)

    sink = cs.getVideo(camera=camera)
    source = cs.putVideo('cv', 320, 240)

    nt = NetworkTables.getTable('/vision')
    entry = nt.getEntry('info')

    # Allocating memory is expensive. Preallocate arrays for the camera images.
    frame = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
    mask = np.zeros(shape=(240, 320), dtype=np.uint8)
    hsv = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    while True:
        time, frame = sink.grabFrame(frame)
        if time == 0:
            # error reading the frame, report to CV output stream
            source.notifyError(sink.getError())
        else:
            start_time = monotonic()
            info = process(frame, mask, hsv)
            info.append(monotonic() - start_time)

            entry.setNumberArray(info)

            source.putFrame(mask)


def process(frame: np.ndarray, mask: np.ndarray = None, hsv: np.ndarray = None, *,
            lower=(20, 125, 125), upper=(35, 255, 255),
            min_area_prop=1/64, focal_length=208.5):
    """Find cubes using our vision algorithm.
    Args
    ----
    frame: the image to process
    mask: optional array to store threshold binary image; must be same resolution
    hsv: optional array to store HSV-converted image; must be same resolution
    lower (Tuple[int, int, int]): HSV lower bound
    upper (Tuple[int, int, int]): HSV upper bound
    min_area_prop (float): minimum size in proportion to the image
    focal_length (float): focal length of camera
    Returns
    -------
    flat list of azimuths and zeniths (interleaved) of cubes detected
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV, dst=hsv)

    # construct a mask for the colour "yellow", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, lower, upper, dst=mask)
    cv2.erode(mask, None, dst=mask)
    cv2.dilate(mask, None, dst=mask)

    # find resolution of mask/image
    height, width = mask.shape

    # calculate the minimum contour size based on image size
    min_area = height * width * min_area_prop

    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[1]

    output = []
    # only proceed if at least one contour was founda
    if contours:
        # sort the contours in descending order of size
        contours.sort(key=cv2.contourArea, reverse=True)

        half_width = width / 2
        half_height = height / 2

        for contour in contours:
            contour_area = cv2.contourArea(contour)

            # only proceed if the contours are big enough
            # since the contours are sorted, we can stop once we find one too small
            if contour_area <= min_area:
                break

            # find the centre of the contour
            M = cv2.moments(contour)
            x, y = M["m10"] / M["m00"], M["m01"] / M["m00"]

            distance_x = x - half_width
            distance_y = y - half_height

            angle_x = math.atan2(-distance_x, focal_length)
            angle_y = math.atan2(-distance_y, focal_length)

            output.extend([angle_x, angle_y])

    return output
