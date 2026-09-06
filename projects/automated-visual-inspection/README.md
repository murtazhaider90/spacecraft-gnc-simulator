# Automated Visual Inspection System

A Raspberry Pi / Python / OpenCV inspection workflow for controlled image capture, repeatable preprocessing, feature checks, pass/fail decisions and timestamped result logging.

The public portfolio implementation uses generic image-processing primitives and synthetic test images. It does not publish any customer/employer images or imply production qualification.

Checks included in the demonstration implementation:

- image brightness bounds
- edge-density range
- expected feature count using connected components
- structured CSV logging with reason codes

```bash
pip install opencv-python numpy
python inspect.py --demo
```
