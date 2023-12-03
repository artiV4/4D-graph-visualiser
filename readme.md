This program is designed to take in up to 8 command line arguments in the order
	int 	window width
	int 	window height
	float 	stepsize
	float 	min 			(used for xmin, ymin, zmin)
	float 	max 			(used for xmax, ymax, zmax)
	float	isoval
	int 	function		which function to use, 1 is sphere, 2 is sinusoidal waves, 3 is horizontal hyperboloid, defaults to 1 if invalid input
	bool	liveRender		should it render live, 0 for no, non-0 for yes
	
Within the window it will draw a box enclosing the drawing area (min, max) and the axes within it
If live rendering, it will render all values at an x-value each draw loop
If not live (dead?) rendering then it will first calculate everything and then draw it

How to Use:
To use this program, you need a working version of C++, a C++ compiler, OpenGL, and GLFW3.
Once all the requirements are met, to compile this program:
    On Windows: 
        1. In command prompt, in directory to the one where the program is
        2. Input "g++ assign3.cpp -lglfw3 -lopengl32 -lglew32"
        3. Run the executable with "a.exe (int windowWidth) (int windowHeight) (float stepsize) (float min) (float max) (float isoval) (int function) (int liveRender)"
			values are optioonal, but need to be in this order and earlier ones are needed to have subsequent ones
        4. The program is open now (hopefully)
    
    On Linux or Mac:
		idk google it


When the window is open, camera can be controlled with:
W to rotate model up (as if camera moves down)
S to rotate model down (as if camera moves up)
A to rotate model clockwise (left)
D to rotate camera counterclockwise (right)
Up arrow to zoom in
Down arrow to zoom out

Left mouse and dragging rotates the model in the direction the mouse is moved
mouse scrollwheel to zoom in or out

The Light source can be moved around the center using
I, J, K, L the same way as the camera can controlled with WASD
O, P to move the light source closer or further from the center

Z prints current camera Spherical coordinates and xyz coordinates

N, M advanced the slice forwads / backwards by one stepsize, need to release and press again to advance one more.
Holding B and pressing N or M lets you not have to keep releasing and pressing again to advance.

Note that with a very small step size or a very large (min, max) range will cause rendering to take very long and may cause the program to crash
Also note that resizing the window, whether by manually doing it or it automatically being resized based on system minimum or maximum sizes will mess up mouse tracking or cause the drawable area on the right side to be suddenly cut off, so don't do that either.
Also note that you can zoom in/out infinitely.
Conclusion:
This program is a fun and intuitive way to visualize 4d functions and lighting in 3d space

Have fun :)

P.S. if your graphics card dies during or after running this program, I take no responsibility.