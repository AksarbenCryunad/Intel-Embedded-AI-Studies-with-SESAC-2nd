# Color Filters

다음 image filter kernel들을 정의 하세요.
 * Original
 * Blur
 * Gaussian blur
 * Sharpen
 * Sobel (X)
 * Sobel (Y)
 * Edge detection

## Run Program
```
python3 -m venv .venv
source .venv/bin/activate
(.venv) pip install -r requirements.txt
(.venv) cd src
(.venv) python color_filter data/a.png
```

## Run Test
```
python3 -m venv .venv
source .venv/bin/activate
(.venv) pip install -r requirements.txt
(.venv) python -m pytest
```