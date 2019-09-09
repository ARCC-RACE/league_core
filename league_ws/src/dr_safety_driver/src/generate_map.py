#!/usr/bin python

from PIL import Image
import os

x = float(input("What is the horizontal dimensions of you track in meters: "))
y = float(input("What is the horizontal dimensions of you track in meters: "))
file_path = raw_input("Path to save generated map: ")
name = raw_input("Name of map: ")

res = 0.1  # 10cmx10cm per pixel
line_thickness = 2  # pixels

img = Image.new('L', (int(x / res + line_thickness), int(y / res + line_thickness)))
pixels = img.load()

for u in range(img.size[0]):
    for v in range(img.size[1]):
        if u <= line_thickness or v <= line_thickness or u >= img.size[0] - line_thickness or v >= img.size[1] - line_thickness:
            pixels[u, v] = 0
        else:
            pixels[u, v] = 255

img.show()
f = open(os.path.join(file_path, name) + ".yaml", "w+")
name = name + ".pgm"
f.write("image: " + name + "\nresolution: " + str(
    res) + "\norigin: [%d, %d, 0]\noccupied_thresh: 0.65\nfree_thresh: 0.196\nnegate: 0" %
        (line_thickness*res + res, line_thickness*res + res))
img.save(os.path.join(file_path, name))
