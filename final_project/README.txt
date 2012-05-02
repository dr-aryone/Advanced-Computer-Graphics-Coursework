HOMEWORK 3: RAY TRACING, RADIOSITY, & PHOTON MAPPING

NAME:  Rory Murphy


TOTAL TIME SPENT:  20+ (at least)
Please estimate the number of hours you spent on this assignment.


COLLABORATORS: 
You must do this assignment on your own, as described in the 
Academic Integrity Policy.  If you did discuss the problem or errors
messages, etc. with anyone, please list their names here.



RAYTRACING:
Raytracing, ray-sphere intersection, reflections, shadows, soft shadows and antialiasing all work. The one exception is spheres with radius less than one. These have all sorts of weird artifacts on them. All distributed ray tracing (soft shadows and antialiasing) was distributed randomly over the domain.



RADIOSITY:
Form Factors work correctly. Radiosity sort-of works. My scenes would always grow either too light or too dark. Now they stay a reasonable brightness, but the color doesn't seem to be reflecting properly. Occlusion is not properly implemented at all.



PHOTON MAPPING:
Visualization works properly, and caustic can be seen clearly. The homework showed running it with 500000 photons, which caused a segfault, but it can run with 100000 no problem. Diffuse photons are sent in completely random directions. Wrote some code to render caustics, but was never able to get it to work correctly.



OTHER NEW FEATURES OR EXTENSIONS FOR EXTRA CREDIT:
Include instructions for use and test cases and sample output as appropriate.
