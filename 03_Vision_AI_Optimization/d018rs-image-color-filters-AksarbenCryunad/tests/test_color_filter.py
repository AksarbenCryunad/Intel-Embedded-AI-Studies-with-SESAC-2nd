import numpy as np
from src.color_filter import Filters


def test_kernels():
    f = Filters()

    # convert to lower case for string comp.
    available_filters = f.get_filter_names()
    available_filters = list(map(lambda x: x.lower(), available_filters))

    assert 8 <= len(available_filters), "모든 필터들이 정의되어야 합니다"

    # Case insenstive string comparison.
    assert "Original".lower() in available_filters, "Orinal filter"
    assert "Blur".lower() in available_filters, "Blur filter"
    assert "Gaussian blur".lower() in available_filters, "Gausian blur filter"
    assert "Sharpen".lower() in available_filters, "Sharpen filter"
    assert "Sobel (x)".lower() in available_filters, "Sobel (X) filter"
    assert "Sobel (y)".lower() in available_filters, "Sobel (y) filter"
    assert "Edge detection".lower() in available_filters, "Edge detection filter"


def get_filter(filters_dict: dict, filter_name: str):
    """
    A helper function to get data from given dictionary with case insensitive key
    """
    f = [v for k, v in filters_dict.items() if str(k).lower() ==
         filter_name.lower()]
    return f[0] if len(f) != 0 else None


def test_original():
    f = Filters()
    filter = get_filter(f.Kernels, "original")
    assert filter is not None
    assert np.array_equal(filter, np.array(
        [[0, 0, 0], [0, 1, 0], [0, 0, 0]], dtype=np.float32))


def test_blur():
    f = Filters()
    filter = get_filter(f.Kernels, "blur")
    assert filter is not None
    assert np.array_equal(filter, np.array(
        [[1, 1, 1], [1, 1, 1], [1, 1, 1]], dtype=np.float32) / 9)


def test_gaussianblur():
    f = Filters()
    filter = get_filter(f.Kernels, "gaussian blur")
    assert filter is not None
    assert np.array_equal(filter, np.array(
        [[1, 2, 1], [2, 4, 2], [1, 2, 1]], dtype=np.float32) / 16)


def test_sharpen():
    f = Filters()
    filter = get_filter(f.Kernels, "sharpen")
    assert filter is not None
    assert np.array_equal(filter, np.array(
        [[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32))


def test_sobelx():
    f = Filters()
    filter = get_filter(f.Kernels, "sobel (x)")
    assert filter is not None
    assert np.array_equal(filter, np.array(
        [[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.float32))


def test_sobely():
    f = Filters()
    filter = get_filter(f.Kernels, "sobel (y)")
    assert filter is not None
    assert np.array_equal(filter, np.array(
        [[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=np.float32))


def test_edgedetection():
    f = Filters()
    filter = get_filter(f.Kernels, "edge detection")
    assert filter is not None
    assert np.array_equal(filter, np.array(
        [[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]], dtype=np.float32))


def test_emboss():
    f = Filters()
    filter = get_filter(f.Kernels, "emboss")
    assert filter is not None
    assert np.array_equal(filter, np.array(
        [[-2, -1, 0], [-1, 1, 1], [0, 1, 2]], dtype=np.float32))
