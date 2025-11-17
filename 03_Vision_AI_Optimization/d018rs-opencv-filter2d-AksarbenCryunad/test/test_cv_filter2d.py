import cv2
import numpy as np
from src.cv_filter2d import identity_kernel, ones_kernel, original_kernel, doubling_kernel


image = np.array([[1, 2, 3, 4],
                  [5, 6, 7, 8],
                  [9, 10, 11, 12],
                  [13, 14, 15, 16]], dtype=np.float32)


def test_identity():
    k = identity_kernel()
    new_image = cv2.filter2D(image, -1, k, borderType=cv2.BORDER_CONSTANT)
    expected = np.array([[7, 9, 11, 4],
                         [15, 18, 21, 11],
                         [23, 30, 33, 19],
                         [13, 23, 25, 27]], dtype=np.float32)
    assert np.array_equal(expected, new_image)


def test_ones():
    k = ones_kernel()
    new_image = cv2.filter2D(image, -1, k, borderType=cv2.BORDER_CONSTANT)
    expected = np.array([[14, 24, 30, 22],
                         [33, 54, 63, 45],
                         [57, 90, 99, 69],
                         [46, 72, 78, 54]], dtype=np.float32)
    assert np.array_equal(expected, new_image)


def test_origianl():
    k = original_kernel()
    new_image = cv2.filter2D(image, -1, k, borderType=cv2.BORDER_CONSTANT)
    expected = np.array([[1, 2, 3, 4],
                         [5, 6, 7, 8],
                         [9, 10, 11, 12],
                         [13, 14, 15, 16]], dtype=np.float32)
    assert np.array_equal(expected, new_image)


def test_doubling():
    k = doubling_kernel()
    new_image = cv2.filter2D(image, -1, k, borderType=cv2.BORDER_CONSTANT)
    expected = np.array([[2, 4, 6, 8],
                         [10, 12, 14, 16],
                         [18, 20, 22, 24],
                         [26, 28, 30, 32]], dtype=np.float32)
    assert np.array_equal(expected, new_image)
