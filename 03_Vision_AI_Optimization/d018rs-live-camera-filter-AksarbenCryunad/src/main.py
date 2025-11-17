import cv2
from filters import Filters


if __name__ == "__main__":
    APP_TITLE = "Live Camera Filter"

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    # Filters
    filters = Filters()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Current filter's name
        filter_name = filters.get_current_filter_name()

        # Apply the filter.
        filtered_frame = filters.apply_filter(frame, filter_name)

        # Overlay text
        cv2.putText(filtered_frame, f"Filter: {filter_name}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow(APP_TITLE, filtered_frame)

        # Key control
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        elif key == 82:  # UP arrow
            filters.switch_previous_filter()
        elif key == 84:  # DOWN arrow
            filters.switch_next_filter()

    cap.release()
    cv2.destroyAllWindows()
