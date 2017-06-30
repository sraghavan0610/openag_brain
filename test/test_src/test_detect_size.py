from openag_cv.detect_size import *
import pytest
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import imutils
import cv2
import os

TEST_DIR = os.path.join(os.path.dirname(__file__), '../data')
print(TEST_DIR)

@pytest.fixture
def frame():
    file_path = os.path.join(TEST_DIR, "plant_4.jpg")
    assert os.path.exists(file_path)
    img = cv2.imread(file_path)
    #img =  np.array(img, dtype=np.uint8)
    assert img.shape[0] > 10
    return img

def test_midpoint():
    tl = (1, 2)
    tr = (3, 4)
    assert midpoint(tl, tr) == (2.0, 3.0)

#@pytest.mark.usefixture("frame")
def test_creating_image_mask():
	pass	

#@pytest.mark.usefixture("frame")
def test_process_image(frame):
    print(TEST_DIR)
    print(frame.shape)
	#expected_image = cv2.imread('plant_4_result.jpg')
    detected_size_data = process_image(frame)
    actual_image = detected_size_data[0]
    cv2.imwrite(os.path.join(TEST_DIR, "plant_4_detected_size.jpg"), actual_image)
	#assert actual_image == output_detect_size_img
@pytest.fixture
def detect_size():
    file_path = os.path.join(TEST_DIR, "plant_4_detected_size.jpg")
    assert os.path.exists(file_path)
    img = cv2.imread(file_path)
    #img =  np.array(img, dtype=np.uint8)
    assert img.shape[0] > 10
    return img

def test_process_image(detect_size):
    print(TEST_DIR)
    print(detect_size.shape)
	#expected_image = cv2.imread('plant_4_result.jpg')
    actual_detected_size_data = process_image(detect_size)
    expected_image = actual_detected_size_data[0]
    cv2.imwrite(os.path.join(TEST_DIR, "actual_plant_4_detected_size.jpg"), expected_image)    


