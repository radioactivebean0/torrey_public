the scene I rendered is the bunny and buddha flying on the dragon. located at myscene.xml and the render is myscene.exr
I also tried to attempt speedup using both SIMD implementation of hitting boxes and adding a simple binned surface area heuristic similar to the one described in the PBRT book.
In addition I also tried used nth element instead of sorting and using SIMD for the some parts of the SAH calculations.
I saw greatest speedup with the Buddha figure over 3.5x when using surface area heuristic, the smaller figures like teapot and bunny got some improvement out of using surface area heuristic but mostly were better due to the SIMD implementations. 


For the question related to random vs longest splits
Rendering the images using random splits
  Min Time(3 runs)     BVH + render          Total
  teapot               0.013011 + 7.01001    7.023021
  bunny                0.380013 + 10.9714    11.351413
  buddha               1.72339 + 22.4365     24.15989
  dragon               1.39639 + 6.9649      8.34539
  party                4.18564 + 28.3746     32.56024


Rendering the images using longest axis
  Min Time(3 runs)     BVH + render          Total
  teapot               0.015618 + 4.82872    4.844338
  bunny                0.470136 + 5.98521    6.455346
  buddha               2.1069 + 8.21904      10.32594
  dragon               1.73286 + 4.19488     5.92774
  party                5.21912 + 10.0957     15.31482


Speedup and optimizations

Rendering time 2 threads, base case
  Min Time(3 runs)     BVH + render          Total
  teapot               0.012642 + 29.4498    29.462442
  bunny                0.389423 + 43.1008    43.489923
  buddha               1.71921 + 80.9757     82.69491
  dragon               1.38017 + 29.9505     31.33067
  party                4.18606 + 103.902     108.08806

add -march=native to compile args, split longest, simd box hit 2 at once (run with -hw 2_7 [filename])
  Min Time(3 runs)     BVH + render          Total
  teapot               0.00752 + 14.0429     14.05042
  bunny                0.490157 + 19.7728    20.262957
  buddha               2.26778 + 27.3646     29.63238
  dragon               1.76064 + 11.8784     13.63904
  party                5.5453 + 31.7266      37.2719

add -march=native to compile args, simd box hit 2 at once, some simd used to build sah and use nth element to sort (run with -hw 2_7 [filename] 2)
 Min Time(3 runs)     BVH + render          Total
  teapot              0.027177 + 11.6806    11.707777
  bunny               0.767311 + 16.1501    16.917411
  buddha              3.90347 + 18.6509     22.567464
  dragon              3.16564 + 8.29542     11.46106
  party               9.06497 + 26.7547     35.81967


Scene      Speedup
 teapot    2.52
 bunny     2.57
 buddha    3.66
 dragon    2.73
 party     3.02