import cv2
import numpy as np
from PIL import ImageGrab
import win32gui

windows_list = []
toplist = []


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1*(3.5/5))
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    
    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)

    left_line = make_coordinates(image, left_fit_average)
    right_line = make_coordinates(image, right_fit_average)

    return np.array([left_line, right_line])

def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny

def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            cv2.line(line_image, (x1, y1), (x2, y2), [0,255,0], thickness=10) # Con cv2 dibujamos las lineas en la imagen
    return line_image

def region_of_interest(image):
    polygons = np.array([[(200, 700), (1120, 700), (630, 310)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image



def enum_win(hwnd, result):
    win_text = win32gui.GetWindowText(hwnd)
    windows_list.append((hwnd, win_text))

    
def get_window():
    # Game handle
    game_hwnd = 0
    for (hwnd, win_text) in windows_list:
        if "pygame window" in win_text:
            game_hwnd = hwnd
    return game_hwnd

def show_camera(game_hwnd):   
    while True:
        try:
            position = win32gui.GetWindowRect(game_hwnd)
            
            screen = ImageGrab.grab(position)
            screen = np.array(screen)
            screen = cv2.cvtColor(screen, cv2.COLOR_RGB2BGR)
            
            canny_image = canny(screen)

            cropped_image = region_of_interest(canny_image)

            lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
            averaged_lines = average_slope_intercept(screen, lines)
            line_image = display_lines(screen, averaged_lines)

            combo_image = cv2.addWeighted(screen, 0.8, line_image, 1, 1)

            cv2.imshow("Vamos a petar", combo_image)

            cv2.waitKey(100)
        except:
            continue    

def main():
    win32gui.EnumWindows(enum_win, toplist)
    game_hwnd = get_window()
    show_camera(game_hwnd)

main()    
    