the scene I rendered is the bunny and buddha flying on the dragon. located at myscene.xml and the render is myscene.exr


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




Rendering time 2 threads, random
  Min Time(3 runs)     BVH + render          Total
  teapot               0.012642 + 29.4498
  bunny                0.389423 + 43.1008
  buddha               1.71921 + 80.9757
  dragon               1.38017 + 29.9505
  party                4.18606 + 103.902

add -march=native to compile args, split longest, simd box hit 2 at once
  Min Time(3 runs)     BVH + render          Total
  teapot               
  bunny                
  buddha               
  dragon               
  party                5.5453 + 31.7266