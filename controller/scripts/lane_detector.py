import matplotlib.pyplot as plt
import numpy as np
import cv2
import math

def detect_edges(frame):
    '''
    Finds edges in a frame by:
    1. Converting image to grayscale
    2. Adjust gamma value to make lane lines stand out more
    3. Applies Gaussian filter to smooth out sharp edges
    4. Find edges using Canny Edge detection
    
    @ param frame: image from front camera
    @ returns: image with lane line edges found
    '''
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # convert to grayscale
    darker = adjust_gamma(gray) # make grayscale image darker
    blur = gaussian_smoothing(darker) # apply Gaussian filter to image
    edges = cv2.Canny(blur, 200, 400) # find edges using Canny Edge detector

    return edges

def region_of_interest(frame):
    '''
    Crops image to only show region of interest set by left_bottom, right_bottom, left_top and right_top.
    Called by detect_edges to crop front camera image.
    Modified from: https://github.com/dctian/DeepPiCar/blob/master/driver/code/hand_coded_lane_follower.py

    @param frame: image from front camera
    @ returns: cropped image
    '''
    height, width = frame.shape[:2] # get height and width of image
    mask = np.zeros_like(frame)
    left_bottom = [0, height] # define left bottom coordinate point
    right_bottom = [width, height] # define right bottom coordinate point
    left_top = [0, height * 0.875] # define left top coordinate point
    right_top = [width, height * 0.875] # define right top coordinate point
    vertices = np.array([[left_bottom, left_top, right_top, right_bottom]], dtype=np.int32)
    
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255, ) * mask.shape[2])

    return cv2.bitwise_and(frame, mask)

def gaussian_smoothing(frame, kernel_size = 5):
    """
    Apply Gaussian filter to the input image. Called by detect_edges.
    Modified from: https://github.com/dctian/DeepPiCar/blob/master/driver/code/hand_coded_lane_follower.py
    
    @param image: An np.array compatible with plt.imshow
    @param kernel_size (Default = 13): The size of the Gaussian kernel will affect the performance of the detector.
                                       It must be an odd number (3, 5, 7, ...)
    @returns: image with applied Gaussian filter. Needed for better edge detection. 
    """
    return cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)

def adjust_gamma(frame, gamma=0.30):
    '''
    Darkens grayscale image. Called by detect_edges.
    Modified from: https://github.com/dctian/DeepPiCar/blob/master/driver/code/hand_coded_lane_follower.py

    @param frame: grayscale image
    @param gamma: threshold parameter?
    @returns: darker grayscale image
    '''
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
    
    # apply gamma correction using the lookup table
    return cv2.LUT(frame, table)

def detect_line_segments(cropped_edges):
    '''
    Detects line segments in image using Hough Line Algorithm
    Modified from: https://github.com/dctian/DeepPiCar/blob/master/driver/code/hand_coded_lane_follower.py

    @param cropped_edges: cropped image with edges found by Canny Edge Detector
    @returns: coordinate points for each line found
    '''
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=15, maxLineGap=12)

    return line_segments

def average_slope_intercept(frame, line_segments):
    """
    Averages slopes found by detect_line_segments
    Modified from: https://github.com/dctian/DeepPiCar/blob/master/driver/code/hand_coded_lane_follower.py

    @param frame: original image from front camera
    @param line_segments: line segments found by detect_line_segments
    @returns: average of lines found 
    """
    lane_lines = []
    if line_segments is None:
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    print("left fit average", left_fit_average)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    print("right fit average", right_fit_average)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    '''

    '''
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1/2)  # make points from middle of the frame down
    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))

    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=5):
    '''
    Adds lane lines found to input frame
    Modified from: https://github.com/dctian/DeepPiCar/blob/master/driver/code/hand_coded_lane_follower.py

    @param frame: input frame from front camera
    @param lines: lines from average_slope_intercept
    @returns: input frame with lane lines plotted
    '''
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    return line_image

def show_image(title, frame):
    '''
    Shows image using CV2

    @param title: title of image
    @param frame: image to be displayed
    @returns: opens window displaying frame with title given
    '''
    cv2.imshow(title, frame) # show image
    cv2.waitKey(0) # exit window by pressing any of the arrow keys

def calc_steer_cmd(lane_lines, frame):
    """ 
    Find the steering angle based on lane line coordinate

    @param lane_lines: lane lines detected
    @param frame: image with lane lines
    @returns: steer command for vehicle based on geometry from lines found
    """
    if len(lane_lines) == 0:
        return 0
    
    height, width, _ = frame.shape
    if len(lane_lines) == 1: # if only one lane line is detected
        x1, y1, x2, y2 = lane_lines[0][0]
        # halfpoint = ((x1+x2)/2, (y1+y2)/2)
        halfpoint = ((x1+x2)/2, y2)
    elif len(lane_lines) == 2: # if two lane lines are detected
        # lx1, ly1, lx2, ly2 = lane_lines[0][0]
        # rx1, ry1, rx2, ry2 = lane_lines[1][0]
        # halfpoint = ((lx1 + rx1)/2, (ly1 + ry1)/2)
        # cv2.circle(frame, (lx1, ly1), 3, (255, 0, 0), 5)
        # cv2.circle(frame, (rx1, ry1), 3, (255, 0, 0), 5)
        x1, y1, x2, y2 = lane_lines[1][0]
        # halfpoint = ((x1+x2)/2, (y1+y2)/2)
        halfpoint = ((x1+x2)/2, y2)

    base_point = (width/2, height) # find bottom middle point of image where car is assumed to be
    x_offset = halfpoint[0] - base_point[0]
    y_offset = halfpoint[1] - base_point[1]

    # generate steer command for vehicle    
    steer_cmd = math.atan2(y_offset, x_offset) + np.pi/2

    return steer_cmd

def calc_steer_cmd2(lane_lines, frame):
    """ 
    Find the steering angle based on lane line coordinate

    FOR TESTING PURPOSES BUT SAME AS CALC_STEER_CMD function

    @param lane_lines: lane lines detected
    @param frame: image with lane lines
    @returns: steer command for vehicle based on geometry from lines found
    """
    if len(lane_lines) == 0:
        return 0
    
    height, width, _ = frame.shape
    if len(lane_lines) == 1: # if only one lane line is detected
        print("one line detected")
        x1, y1, x2, y2 = lane_lines[0][0]
        cv2.circle(frame, (x1, y1), 3, (255, 0, 0), 5)
        cv2.circle(frame, (x2, y2), 3, (255, 0, 0), 5)
        # halfpoint = ((x1+x2)/2, (y1+y2)/2)
        halfpoint = ((x1+x2)/2, y2)
    elif len(lane_lines) == 2: # if two lane lines are detected
        print("two lines detected")
        # lx1, ly1, lx2, ly2 = lane_lines[0][0]
        # rx1, ry1, rx2, ry2 = lane_lines[1][0]
        # halfpoint_left = (int((lx1 + lx2)/2), int((ly1 + ly2)/2))
        # print("halfpoint left", halfpoint_left)
        # halfpoint_right = (int((rx1 + rx2)/2), int((ry1 + ry2)/2))
        # cv2.circle(frame, (lx1, ly1), 3, (255, 0, 0), 5)
        # cv2.circle(frame, (rx1, ry1), 3, (255, 0, 0), 5)
        # cv2.circle(frame, halfpoint_left, (0, 255, 0), 3)
        # cv2.circle(frame, halfpoint_right, (0, 255, 0), 3)
        x1, y1, x2, y2 = lane_lines[1][0]
        # cv2.circle(frame, (x1, y1), 3, (255, 255, 0), 5)
        # cv2.circle(frame, (x2, y2), 3, (255, 255, 0), 5)
        # halfpoint = ((x1+x2)/2, (y1+y2)/2)
        halfpoint = ((x1+x2)/2, y2)

    base_point = (width/2, height) # find bottom middle point of image where car is assumed to be
    cv2.circle(frame, base_point, 3, (0, 0, 255), 5) # plot base point
    cv2.circle(frame, halfpoint, 3, (0, 0, 255), 5) # plot half point
    x_offset = halfpoint[0] - base_point[0]
    y_offset = halfpoint[1] - base_point[1]
     # plot steering cmd line in yellow. For development purposes
    cv2.line(frame, base_point, halfpoint, (30, 255, 255), 3) # plot steering cmd line in yellow
    
    # generate steer command for vehicle
    steer_cmd = math.atan2(y_offset, x_offset) + np.pi/2
    print("x_offset ", x_offset)
    print("y_offset ", y_offset)
    print("steer cmd ", steer_cmd)

    return steer_cmd, frame

def get_steer_cmd(frame_, curr_steer_angle):
    '''
    Function used by Webots simulator to generate steering command given image from front camera

    @param frame: image from Tesla Model 3 front camera
    @param curr_steer_angle: vehicle's current steering angle (for signal smoothing purposes)
    @returns: steering command for vehicle 
    '''
    frame = cv2.imread(frame_) # read in image
    edges = detect_edges(frame) # detect edges 

    cropped_edges = region_of_interest(edges) # crop image

    line_segments = detect_line_segments(cropped_edges) # find coordinates of lines found
    line_segment_image = display_lines(frame, line_segments) 

    lane_lines = average_slope_intercept(frame, line_segments) # average lane line slopes and intercepts
    if not lane_lines: # if no lane lines found then apply previous steering command
        return curr_steer_angle

    lane_lines_image = display_lines(frame, lane_lines)

    if lane_lines: # if lane lines were found then find a steering command for the vehicle
        steer_cmd = calc_steer_cmd(lane_lines, lane_lines_image)
    else:
        steer_cmd = curr_steer_angle # if no steering command could be generated then 

    # clip steering angle to vehicle limits
    max_angle = 1
    steer_cmd = np.clip(steer_cmd, -max_angle, max_angle)

    return steer_cmd

def main():
    '''
    For debugging purposes, same code as get_steer_cmd but this shows plots
    '''
    frame = cv2.imread('front_img3.jpg')
    edges = detect_edges(frame)
    show_image('edges', edges)

    cropped_edges = region_of_interest(edges)
    show_image('cropped edges', cropped_edges)

    line_segments = detect_line_segments(cropped_edges)
    line_segment_image = display_lines(frame, line_segments)
    show_image("line segments", line_segment_image)

    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    show_image("lane lines image", lane_lines_image)

    print("lane lines", lane_lines)
    if lane_lines:
        steer_cmd, tmp_frame = calc_steer_cmd2(lane_lines, lane_lines_image)
        show_image('calc steer cmd 2', tmp_frame)
        cv2.imwrite('path_and_cmd.jpg', tmp_frame)
    else:
        steer_cmd = 0
    
    max_angle = np.pi/6
    if steer_cmd > max_angle:
        steer_cmd = max_angle
    elif steer_cmd < -max_angle:
        steer_cmd = -max_angle
    
    print("steering command ", steer_cmd)
    print("steering command in degrees ", steer_cmd * 180/np.pi)


if __name__ == '__main__':
    main()    
