![Tests](https://github.com/utiasASRL/spinning-radar-doppler/actions/workflows/build_and_test.yml/badge.svg)

# spinning-radar-doppler
Code to extract and process Doppler measurements from a spinning radar sensor.

# Expected input format
The code currently expects that your radar data is contained as full radar scan images in a .png format.
Each image has a name corresponding to the unix timestamp at which it was collected at.
