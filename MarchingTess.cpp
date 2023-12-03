// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>

#include "TriTable.hpp"
#include "shader.hpp"


#define NEAR_TOP_LEFT     	128
#define NEAR_TOP_RIGHT    	64
#define FAR_TOP_RIGHT     	32
#define FAR_TOP_LEFT     	16
#define NEAR_BOTTOM_LEFT	8
#define NEAR_BOTTOM_RIGHT  	4
#define FAR_BOTTOM_RIGHT 	2
#define FAR_BOTTOM_LEFT  	1

#define EPS 				0.01f   // very small value for where needed

/* Constants, edit to fit your own device's speed */
const float cameraSpeedI = 0.98f; // speed of zooming in/out with arrow keys
const float cameraSpeedO = 1.02;
const float scrollCamSpeedI = 0.94; // speed of zooming in/out with scroll wheel
const float scrollCamSpeedO = 1.06;
const float rotateSpeed = glm::radians(.1f); // speed of angular rotation using wasd
const float MOUSE_SCALE = glm::radians(0.2); // scaling for angular rotation using mouse
const std::string filename = "test.ply"; // name for what file to write to, must be valid and end with .ply
//const bool liveRender = true; // whether the object should be drawn while being rendered

typedef float (*scalar_field_3d)(float, float, float);
typedef float (*scalar_field_4d)(float, float, float, float);

// empty functions which are defined later
std::vector<float> compute_normals(std::vector<float> vertices);
void writePLY(std::vector<float> vertices, std::vector<float> normals, std::string path);
// Axes class stores and draws the axes and box enclosing the drawable area
class Axes {

	glm::vec3 origin;
	glm::vec3 extentsPos;
	glm::vec3 extentsNeg;

	glm::vec3 xcol = glm::vec3(1.0f, 0.0f, 0.0f);
	glm::vec3 ycol = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 zcol = glm::vec3(0.0f, 0.0f, 1.0f);

public:

	Axes(glm::vec3 orig, glm::vec3 exPos, glm::vec3 exNeg) : origin(orig), extentsNeg(exNeg), extentsPos(exPos) {}

	void draw() {

		glMatrixMode( GL_MODELVIEW );
		glPushMatrix();


		glLineWidth(2.0f);
		glBegin(GL_LINES);
		glColor3f(xcol.x, xcol.y, xcol.z);
		glVertex3f(origin.x, origin.y, origin.z);
		glVertex3f(origin.x + extentsPos.x, origin.y, origin.z);
		glVertex3f(origin.x, origin.y, origin.z);
		glVertex3f(origin.x + extentsNeg.x, origin.y, origin.z);

		glColor3f(ycol.x, ycol.y, ycol.z);
		glVertex3f(origin.x, origin.y, origin.z);
		glVertex3f(origin.x, origin.y + extentsPos.y, origin.z);
		glVertex3f(origin.x, origin.y, origin.z);
		glVertex3f(origin.x, origin.y + extentsNeg.y, origin.z);

		
		glColor3f(zcol.x, zcol.y, zcol.z);
		glVertex3f(origin.x, origin.y, origin.z);
		glVertex3f(origin.x, origin.y, origin.z + extentsPos.z);
		glVertex3f(origin.x, origin.y, origin.z);
		glVertex3f(origin.x, origin.y, origin.z + extentsNeg.z);
		
		
		glEnd();

		glColor3f(0.5f,0.5f,0.0f);
		glLineWidth(2.0f);
		glBegin(GL_LINE_STRIP);
			glVertex3f(extentsNeg.x, extentsNeg.y, extentsNeg.z);
			glVertex3f(extentsPos.x, extentsNeg.y, extentsNeg.z); 
			glVertex3f(extentsPos.x, extentsNeg.y, extentsPos.z);
			glVertex3f(extentsNeg.x, extentsNeg.y, extentsPos.z);
			glVertex3f(extentsNeg.x, extentsNeg.y, extentsNeg.z);
			glVertex3f(extentsNeg.x, extentsPos.y, extentsNeg.z);
			glVertex3f(extentsPos.x, extentsPos.y, extentsNeg.z);
			glVertex3f(extentsPos.x, extentsNeg.y, extentsNeg.z);						
			glVertex3f(extentsPos.x, extentsPos.y, extentsNeg.z);
			glVertex3f(extentsPos.x, extentsPos.y, extentsPos.z);
			glVertex3f(extentsPos.x, extentsNeg.y, extentsPos.z);
			glVertex3f(extentsPos.x, extentsPos.y, extentsPos.z);
			glVertex3f(extentsNeg.x, extentsPos.y, extentsPos.z);
			glVertex3f(extentsNeg.x, extentsNeg.y, extentsPos.z);
			glVertex3f(extentsNeg.x, extentsPos.y, extentsPos.z);
			glVertex3f(extentsNeg.x, extentsPos.y, extentsNeg.z);
		glEnd();

		glPopMatrix();
	}

};

class continuousRender {
	scalar_field_3d f;
	float isoval, minx, maxx, miny, maxy, minz, maxz, stepsize, x, y, z;
	std::vector<float>& vertices;
	std::vector<float>& normals;
	float ftl, ftr, fbr, fbl, ntl, ntr, nbr, nbl;
	int which = 0;
	int* verts;
	
public:
	continuousRender(std::vector<float>& vertices, std::vector<float>& normals, scalar_field_3d f, float isoval, float minx, float maxx, float miny, float maxy, float minz, float maxz, float stepsize) : vertices(vertices), normals(normals), f(f), isoval(isoval), minx(minx), maxx(maxx), miny(miny), maxy(maxy), minz(minz), maxz(maxz), stepsize(stepsize), x(minx), y(miny), z(minz) {}

	bool nextIteration(){
		std::vector<float> tempVert;
		std::vector<float> tempNorm;
		
		// get all vertices at the plane normal to the x-axis at the current x value
		for (y = miny; y<maxy; y+=stepsize){
			for (z=minz; z<maxz; z+= stepsize){
				//test the cube
				ntl = (*f)(x, y+stepsize, z+stepsize);
				ntr = (*f)(x+stepsize, y+stepsize, z+stepsize);
				nbr = (*f)(x+stepsize, y, z+stepsize);
				nbl = (*f)(x, y, z+stepsize);
				ftl = (*f)(x, y+stepsize, z);
				ftr = (*f)(x+stepsize, y+stepsize, z);
				fbr = (*f)(x+stepsize, y, z);
				fbl = (*f)(x, y, z);

				which = 0;
				if (ntl < isoval) {
					which |= NEAR_TOP_LEFT;
				}
				if (ntr < isoval) {
					which |= NEAR_TOP_RIGHT;
				}
				if (nbr < isoval) {
					which |= NEAR_BOTTOM_RIGHT;
				}
				if (nbl < isoval) {
					which |= NEAR_BOTTOM_LEFT;
				}
				if (ftl < isoval) {
					which |= FAR_TOP_LEFT;
				}
				if (ftr < isoval) {
					which |= FAR_TOP_RIGHT;
				}
				if (fbr < isoval) {
					which |= FAR_BOTTOM_RIGHT;
				}
				if (fbl < isoval) {
					which |= FAR_BOTTOM_LEFT;
				}
				
				verts = marching_cubes_lut[which];
				
				if (verts[0] >= 0) {
					tempVert.push_back(x+stepsize*vertTable[verts[0]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[0]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[0]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[1]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[1]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[1]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[2]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[2]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[2]][2]);
				}
				if (verts[3] >= 0) {
					tempVert.push_back(x+stepsize*vertTable[verts[3]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[3]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[3]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[4]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[4]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[4]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[5]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[5]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[5]][2]);
				}
				if (verts[6] >= 0) {
					tempVert.push_back(x+stepsize*vertTable[verts[6]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[6]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[6]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[7]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[7]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[7]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[8]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[8]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[8]][2]);
				}
				if (verts[9] >= 0) {
					tempVert.push_back(x+stepsize*vertTable[verts[9]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[9]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[9]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[10]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[10]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[10]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[11]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[11]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[11]][2]);
				}
				if (verts[12] >= 0) {
					tempVert.push_back(x+stepsize*vertTable[verts[12]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[12]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[12]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[13]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[13]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[13]][2]);
					tempVert.push_back(x+stepsize*vertTable[verts[14]][0]);
					tempVert.push_back(y+stepsize*vertTable[verts[14]][1]);
					tempVert.push_back(z+stepsize*vertTable[verts[14]][2]);
				}
			}
		}
		//std::cerr << "rendering x: " << x << " vertices: " << tempVert.size() << std::endl;
		tempNorm = compute_normals(tempVert);
		vertices.insert(vertices.end(), tempVert.begin(), tempVert.end());
		normals.insert(normals.end(), tempNorm.begin(), tempNorm.end());
		x+=stepsize;
		if (x<maxx) return true;
		writePLY(vertices, normals, filename);
		std::cerr << "finished rendering" << std::endl;
		return false;
	}
};

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
glm::vec3 coords; // needs to be global to be accessed by the scrollwheel callback



float f1(float x, float y, float z) {
	return x*x+y*y+z*z;
}
float f2(float x, float y, float z) {
	return y-sin(x)*cos(z);
}
float f3(float x, float y, float z) {
	return x*x-y*y-z*z-z;
}


float fw1(float x, float y, float z, float w) {
	return x*y*z*w;
}

float fw2(float x, float y, float z, float w) {
	return x*x+y*y+z*z+w*w;
}

float fw3(float x, float y, float z, float w) {
	return sin(x)+y*y+z*z+w*w;
}
float fw4(float x, float y, float z, float w) {
	return sin(w)*y*y+sin(x)*z*z;
}


void writePLY(std::vector<float> vertices, std::vector<float> normals, std::string path){
	// Create and/or open the file
    std::ofstream file;
	file.open(path);
    if (!file){
        printf("%s could not be created or opened. Are you in the right directory?\n", path);
        // getchar();
        throw std::runtime_error("Error writing PLY, File could not be opened");
    }
	int numElem = vertices.size();
	int numVerts = numElem/3;
	int numIndex = numVerts/3;

	file << "ply\nformat ascii 1.0\ncomment Created by Blender 3.0.1 - www.blender.org\nelement vertex " << numVerts;
	file << "\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face ";	
	file << numIndex << "\nproperty list uchar uint vertex_indices\nend_header\n";
	for (int i = 0; i< numElem; i+=3){
		file << (float)vertices[i] << " " << (float)vertices[i+1] << " " << (float)vertices[i+2] << " " << (float)normals[i] << " " << (float)normals[i+1] << " " << (float)normals[i+2] << "\n";
	
	}
	for (int i = 0; i< numVerts; i+=3){
		file << 3 << " " << i << " " << i+1 << " " << i+2 << "\n";
		
	}
	
	file.close();
}

std::vector<float> marching_cubes(scalar_field_3d f, float isoval, float minx, float maxx, float miny, float maxy, float minz, float maxz, float stepsize) {
	std::vector<float> vertices;
	float x = minx;
	float y = miny;
	float z = minz;
	float ftl, ftr, fbr, fbl, ntl, ntr, nbr, nbl;
	int which = 0;
	int* verts;

	for ( ; x < maxx; x += stepsize) {
		for (y = miny ; y < maxy; y += stepsize) {
			for (z = minz ; z < maxz ; z += stepsize){
				//test the cube
				ntl = (*f)(x, y+stepsize, z+stepsize);
				ntr = (*f)(x+stepsize, y+stepsize, z+stepsize);
				nbr = (*f)(x+stepsize, y, z+stepsize);
				nbl = (*f)(x, y, z+stepsize);
				ftl = (*f)(x, y+stepsize, z);
				ftr = (*f)(x+stepsize, y+stepsize, z);
				fbr = (*f)(x+stepsize, y, z);
				fbl = (*f)(x, y, z);

				which = 0;
				if (ntl < isoval) {
					which |= NEAR_TOP_LEFT;
				}
				if (ntr < isoval) {
					which |= NEAR_TOP_RIGHT;
				}
				if (nbr < isoval) {
					which |= NEAR_BOTTOM_RIGHT;
				}
				if (nbl < isoval) {
					which |= NEAR_BOTTOM_LEFT;
				}
				if (ftl < isoval) {
					which |= FAR_TOP_LEFT;
				}
				if (ftr < isoval) {
					which |= FAR_TOP_RIGHT;
				}
				if (fbr < isoval) {
					which |= FAR_BOTTOM_RIGHT;
				}
				if (fbl < isoval) {
					which |= FAR_BOTTOM_LEFT;
				}

				verts = marching_cubes_lut[which];
				
				if (verts[0] >= 0) {
					vertices.push_back(x+stepsize*vertTable[verts[0]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[0]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[0]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[1]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[1]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[1]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[2]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[2]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[2]][2]);
				}
				if (verts[3] >= 0) {
					vertices.push_back(x+stepsize*vertTable[verts[3]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[3]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[3]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[4]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[4]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[4]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[5]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[5]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[5]][2]);
				}
				if (verts[6] >= 0) {
					vertices.push_back(x+stepsize*vertTable[verts[6]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[6]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[6]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[7]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[7]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[7]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[8]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[8]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[8]][2]);
				}
				if (verts[9] >= 0) {
					vertices.push_back(x+stepsize*vertTable[verts[9]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[9]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[9]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[10]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[10]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[10]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[11]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[11]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[11]][2]);
				}
				if (verts[12] >= 0) {
					vertices.push_back(x+stepsize*vertTable[verts[12]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[12]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[12]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[13]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[13]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[13]][2]);
					vertices.push_back(x+stepsize*vertTable[verts[14]][0]);
					vertices.push_back(y+stepsize*vertTable[verts[14]][1]);
					vertices.push_back(z+stepsize*vertTable[verts[14]][2]);
				} // last is always -1, probably there to keep in 16 int blocks?
			
			}
		}
	}

	return vertices;
}

std::vector<std::vector<float>> marching_tess (scalar_field_4d f, float isoval, float minx, float maxx, float miny, float maxy, float minz, float maxz, float minw, float maxw, float stepsize) {
	std::vector<std::vector<float>> returnVect;
	float x = minx;
	float y = miny;
	float z = minz;
	float w = minw;
	float ftl, ftr, fbr, fbl, ntl, ntr, nbr, nbl;
	int which = 0;
	int* verts;
	int index = 0;
	
	
	for( ; w<maxw; w+= stepsize){
		std::vector<float> vertices;
		for (x=minx ; x < maxx; x += stepsize) {
			for (y = miny ; y < maxy; y += stepsize) {
				for (z = minz ; z < maxz ; z += stepsize){
					//test the cube
					ntl = (*f)(x, y+stepsize, z+stepsize, w);
					ntr = (*f)(x+stepsize, y+stepsize, z+stepsize, w);
					nbr = (*f)(x+stepsize, y, z+stepsize, w);
					nbl = (*f)(x, y, z+stepsize,w);
					ftl = (*f)(x, y+stepsize, z,w);
					ftr = (*f)(x+stepsize, y+stepsize, z,w);
					fbr = (*f)(x+stepsize, y, z,w);
					fbl = (*f)(x, y, z,w);

					which = 0;
					if (ntl < isoval) {
						which |= NEAR_TOP_LEFT;
					}
					if (ntr < isoval) {
						which |= NEAR_TOP_RIGHT;
					}
					if (nbr < isoval) {
						which |= NEAR_BOTTOM_RIGHT;
					}
					if (nbl < isoval) {
						which |= NEAR_BOTTOM_LEFT;
					}
					if (ftl < isoval) {
						which |= FAR_TOP_LEFT;
					}
					if (ftr < isoval) {
						which |= FAR_TOP_RIGHT;
					}
					if (fbr < isoval) {
						which |= FAR_BOTTOM_RIGHT;
					}
					if (fbl < isoval) {
						which |= FAR_BOTTOM_LEFT;
					}

					verts = marching_cubes_lut[which];
					
					if (verts[0] >= 0) {
						vertices.push_back(x+stepsize*vertTable[verts[0]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[0]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[0]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[1]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[1]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[1]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[2]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[2]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[2]][2]);
					}
					if (verts[3] >= 0) {
						vertices.push_back(x+stepsize*vertTable[verts[3]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[3]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[3]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[4]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[4]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[4]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[5]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[5]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[5]][2]);
					}
					if (verts[6] >= 0) {
						vertices.push_back(x+stepsize*vertTable[verts[6]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[6]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[6]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[7]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[7]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[7]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[8]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[8]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[8]][2]);
					}
					if (verts[9] >= 0) {
						vertices.push_back(x+stepsize*vertTable[verts[9]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[9]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[9]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[10]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[10]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[10]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[11]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[11]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[11]][2]);
					}
					if (verts[12] >= 0) {
						vertices.push_back(x+stepsize*vertTable[verts[12]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[12]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[12]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[13]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[13]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[13]][2]);
						vertices.push_back(x+stepsize*vertTable[verts[14]][0]);
						vertices.push_back(y+stepsize*vertTable[verts[14]][1]);
						vertices.push_back(z+stepsize*vertTable[verts[14]][2]);
					}
				
				}
			}
		}
		index++;
		returnVect.push_back(vertices);
	}	

	return returnVect;
}

std::vector<float> compute_normals(std::vector<float> vertices){
	std::vector<float> normals;
	for (int i =0; i< vertices.size();i+=9){
		// due to winding order, the direction of the normal is set by the order of vertices
		glm::vec3 v1 = {vertices[i], vertices[i+1], vertices[i+2]};
		glm::vec3 v2 = {vertices[i+3], vertices[i+4], vertices[i+5]};
		glm::vec3 v3 = {vertices[i+6], vertices[i+7], vertices[i+8]};
		glm::vec3 diff1 = v1 - v2;
		glm::vec3 diff2 = v1-v3;
		glm::vec3 norm = glm::normalize(glm::cross(diff1, diff2));
		for (int j=0; j<3; j++){
			normals.push_back(norm.x);
			normals.push_back(norm.y);
			normals.push_back(norm.z);	
		}
	}
	return normals;
	
}	

std::vector<float> compute_normals_4d(std::vector<float> vertices){ // not needed? I can store just xyz
	std::vector<float> normals;
	for (int i =0; i< vertices.size();i+=12){
		// disregard w?
		glm::vec3 v1 = {vertices[i], vertices[i+1], vertices[i+2]};
		glm::vec3 v2 = {vertices[i+3], vertices[i+4], vertices[i+5]};
		glm::vec3 v3 = {vertices[i+6], vertices[i+7], vertices[i+8]};
		glm::vec3 diff1 = v1 - v2;
		glm::vec3 diff2 = v1-v3;
		glm::vec3 norm = glm::normalize(glm::cross(diff1, diff2));
		for (int j=0; j<3; j++){
			normals.push_back(norm.x);
			normals.push_back(norm.y);
			normals.push_back(norm.z);	
		}
	}
	return normals;
	
}


int main( int argc, char* argv[])
{
	
	///////////////////////////////////////////////////////
	float screenW = 1400;
	float screenH = 900;
	float stepsize = 0.1f;

	float xmin = -5;
	float xmax = 5;
	float isoval = 1;
	scalar_field_4d selectedF = fw1;
	bool liveRender = false;

	if (argc > 1 ) {
		screenW = atoi(argv[1]);
	}
	if (argc > 2) {
		screenH = atoi(argv[2]);
	}
	if (argc > 3) {
		stepsize = atof(argv[3]);
	}
	if (argc > 4) {
		xmin = atof(argv[4]);
	}
	if (argc > 5) {
		xmax = atof(argv[5]);
	}
	if (argc > 6) {
		isoval = atof(argv[6]);
	}
	if (argc > 7) {
		int func = atoi(argv[7]);
		if (func==1) selectedF=fw1;
		else if (func==2) selectedF=fw2;
		else if (func==3) selectedF=fw3;
		else if (func==4) selectedF=fw4;
		// if not an integer [1,3] then defaults to f1
	}
	/* if (argc > 8) {
		liveRender = atoi(argv[8]);
		
	} */
	float ymin = xmin;
	float ymax = xmax;
	float zmin = xmin;
	float zmax = xmax;
	float wmin = xmin;
	float wmax = xmax;
	
	///////////////////////////////////////////////////////

	// Initialise GLFW
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	// glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	// glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	// glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	// glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( screenW, screenH, "Marching   : [Redacted]  ", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}


	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// set callback for when scroll wheel is used, couldn't find a different way to implement scroll wheel
	glfwSetScrollCallback(window, scroll_callback);

	// Dark blue background
	glClearColor(0.2f, 0.2f, 0.3f, 0.0f);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);


	GLuint ProgramID = LoadShaders( "PhongTexture.vertexshader", "PhongTexture.fragmentshader" );


	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glm::mat4 Projection = glm::perspective(glm::radians(45.0f), screenW/screenH, 0.001f, 1000.0f);
	glLoadMatrixf(glm::value_ptr(Projection));

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	
	glm::vec3 up = {0.0f, 1.0f, 0.0f};
	glm::vec3 center = {0.0f, 0.0f, 0.0f};
	// radius = glm::length(start point)
	// theta = arctan(1) = 45 degrees
	// phi = arctan(sqrt(50)/5) = approx 54.735 degrees
	
	coords = {sqrt(3*(5*5)), M_PI/4., atan(sqrt(50.)/5)}; // radius, rotation about y-axis, angle from y-axis
	
	glm::vec3 eye = {coords[0]*sin(coords[2])*cos(coords[1]), 
				coords[0]*cos(coords[2]),
				coords[0]*sin(coords[1])*sin(coords[2])}; // X, Y, Z = 5,5,5 based on the given coords

	glm::vec3 lightCoords = {sqrt(3*(5*5)), M_PI/4., atan(sqrt(50.)/5)}; // have light start at the same place as camera
	glm::vec3 lightPos = {coords[0]*sin(coords[2])*cos(coords[1]), 
				coords[0]*cos(coords[2]),
				coords[0]*sin(coords[1])*sin(coords[2])}; // X, Y, Z = 5,5,5 based on the given coords
	glm::vec4 color(0.f, 0.8f, 0.8f, 1.0f); // colour of the rendered triangles
	float alpha = 64;

	GLuint MVPID, MID, VID, LightPosID, colorID, alphaID;
	
	glm::mat4 V = glm::lookAt(eye, center, up);
	glm::mat4 M = glm::mat4(1.0f);
	glm::mat4 MV = V * M;
	glLoadMatrixf(glm::value_ptr(V));
	glm::mat4 MVP = Projection * V * M;
	
	MVPID = glGetUniformLocation(ProgramID, "MVP");
	MID = glGetUniformLocation(ProgramID, "M");
	VID = glGetUniformLocation(ProgramID, "V");
	LightPosID = glGetUniformLocation(ProgramID, "LightPosition_worldspace");
	colorID = glGetUniformLocation(ProgramID, "modelcolor");
	alphaID = glGetUniformLocation(ProgramID, "alpha");
	
	glUseProgram(ProgramID);
	glUniformMatrix4fv(MID, 1, GL_FALSE, &M[0][0]); //model matrix always identity.
	glUniformMatrix4fv(MVPID, 1, GL_FALSE, &MVP[0][0]);
	
	
	
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	
	int numBuffers = (int) ((wmax-wmin)/stepsize);
	std::cout << "numBuffers: " << numBuffers <<std::endl;
	
	std::vector<std::vector<float>> marchingVerts;
	std::vector<std::vector<float>> normals;
	
	GLuint vaoID[numBuffers];
	GLuint VertexID[numBuffers];
	GLuint NormalID[numBuffers];
	
	glGenVertexArrays(numBuffers, &vaoID[0]);
	glGenBuffers(numBuffers, &VertexID[0]);
	glGenBuffers(numBuffers, &NormalID[0]);
	
	
	
	// Render then draw
	if (!liveRender){
		marchingVerts = marching_tess(selectedF, isoval, xmin, xmax, ymin, ymax, zmin, zmax, wmin, wmax, stepsize);
		for(int i = 0; i<numBuffers; i++){
			
			normals.push_back(compute_normals(marchingVerts[i]));
		
			glBindVertexArray(vaoID[i]);
			
			glBindBuffer(GL_ARRAY_BUFFER, VertexID[i]);
			glBufferData(GL_ARRAY_BUFFER, marchingVerts[i].size()*sizeof(float), &marchingVerts[i][0], GL_STATIC_DRAW);
			
			// 1st attribute buffer : vertices
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(
				0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
				3,                  // size
				GL_FLOAT,           // type
				GL_FALSE,           // normalized?
				0,                  // stride
				(void*) 0           // buffer offset
			);
			
			//glBufferSubdata
			
			glBindBuffer(GL_ARRAY_BUFFER, NormalID[i]);
			glBufferData(GL_ARRAY_BUFFER, normals[i].size()*sizeof(float), &normals[i][0], GL_STATIC_DRAW);
			
			// 2nd attribute buffer : normals
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(
				1,                  // attribute. No particular reason for 0, but must match the layout in the shader.
				3,                  // size
				GL_FLOAT,           // type
				GL_FALSE,           // normalized?
				0,                  // stride
				(void*) 0           // buffer offset
			);
			
			
			glBindVertexArray(0); // unbind vao
		}
		//writePLY(marchingVerts, normals, filename);
	} 
	
	// Need to make the continuousRender object regardles of liveRendering, if created in an Else{} then i bugs out
	//continuousRender contRend(marchingVerts, normals, selectedF, isoval, xmin, xmax, ymin, ymax, zmin, zmax, stepsize);
	
	// Axes and box
	Axes ax(center, glm::vec3(xmin, ymin, zmin), glm::vec3(xmax, ymax, zmax));
	
	
	bool mouseHold = false;
	double currMouseX, currMouseY;
	double prevMouseX, prevMouseY;
	
	
	bool zheld = false;
	bool nheld = false;
	bool mheld = false;
	bool rendering = true;
	int Windex = 0;
	
	do{
		
		double currTime = glfwGetTime();
		static double lastTime = glfwGetTime();
		float deltaTime = (currTime - lastTime)/2; // too fast otherwise

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
		glUseProgram(ProgramID);
		
		// Recalculate projection
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glm::mat4 Projection = glm::perspective((float) M_PI/4, screenW/screenH, 0.001f, 1000.0f);
		glLoadMatrixf(glm::value_ptr(Projection));

		glMatrixMode( GL_MODELVIEW );
		glPushMatrix();
	
		if(!nheld && glfwGetKey(window, GLFW_KEY_N ) == GLFW_PRESS){
			Windex = max(0, Windex-1);
			if(glfwGetKey(window, GLFW_KEY_B ) == GLFW_RELEASE) nheld=true;
			
		}
		if(nheld && glfwGetKey(window, GLFW_KEY_N ) != GLFW_PRESS){
			nheld=false;
		}
		
		if(!mheld && glfwGetKey(window, GLFW_KEY_M ) == GLFW_PRESS){
			Windex = min(numBuffers, Windex+1);
			if(glfwGetKey(window, GLFW_KEY_B ) == GLFW_RELEASE) mheld=true;
			
		}
		if(mheld && glfwGetKey(window, GLFW_KEY_M ) != GLFW_PRESS){
			mheld=false;
		}
	
		// pressing z prints the current camera coordinates and position to std::cerr
		if(!zheld && glfwGetKey(window, GLFW_KEY_Z ) == GLFW_PRESS){
			std::cerr << glm::to_string(eye) << std::endl;
			std::cerr << glm::to_string(coords) << std::endl;
			zheld=true;
		}
		if(zheld && glfwGetKey(window, GLFW_KEY_Z ) != GLFW_PRESS){
			zheld=false;
		}
		
		// LIGHT POSITION CONTROL
		
		// IJKL control of rotation of Light source and OP for radius
		if(glfwGetKey(window, GLFW_KEY_L ) == GLFW_PRESS){
			lightCoords[1] += rotateSpeed;
			if (lightCoords[1] >= 2*M_PI) lightCoords[1] = 0.f;
			
		}
		if(glfwGetKey(window, GLFW_KEY_J ) == GLFW_PRESS){
			lightCoords[1] -= rotateSpeed;
			if (lightCoords[1] < 0.f) lightCoords[1] = deltaTime*2*M_PI;
			
		}
		if(glfwGetKey(window, GLFW_KEY_K ) == GLFW_PRESS){
			lightCoords[2] = glm::max(lightCoords[2]-deltaTime*rotateSpeed, EPS);
			
		}
		if(glfwGetKey(window, GLFW_KEY_I ) == GLFW_PRESS){
			lightCoords[2] = glm::min(lightCoords[2]+deltaTime*rotateSpeed, (float)M_PI-EPS);
			
		}
		if(glfwGetKey(window, GLFW_KEY_P ) == GLFW_PRESS){
			lightCoords[0] *= cameraSpeedI;
			
		}
		
		if(glfwGetKey(window, GLFW_KEY_O ) == GLFW_PRESS){
			lightCoords[0] *= cameraSpeedO;
		}
		// Calculate current light position based on the coordinates
		lightPos = {lightCoords[0]*sin(lightCoords[2])*cos(lightCoords[1]), 
				lightCoords[0]*cos(lightCoords[2]),
				lightCoords[0]*sin(lightCoords[1])*sin(lightCoords[2])};
		
		
		// CAMERA CONTROL
		
		// Arrow key to control radius of camera
		if(glfwGetKey(window, GLFW_KEY_UP ) == GLFW_PRESS){
			coords[0] *= cameraSpeedI;		
		}	
		if(glfwGetKey(window, GLFW_KEY_DOWN ) == GLFW_PRESS){
			coords[0] *= cameraSpeedO;
		}
		
		
		// WASD camera rotation control
		if(glfwGetKey(window, GLFW_KEY_D ) == GLFW_PRESS){
			coords[1] += deltaTime*rotateSpeed;
			if (coords[1] >= 2*M_PI) coords[1] = 0.f;
		}
		if(glfwGetKey(window, GLFW_KEY_A ) == GLFW_PRESS){
			coords[1] -= deltaTime*rotateSpeed;
			if (coords[1] < 0.f) coords[1] = 2*M_PI;			
		}
		if(glfwGetKey(window, GLFW_KEY_S ) == GLFW_PRESS){
			coords[2] = glm::max(coords[2]-deltaTime*rotateSpeed, EPS);			
		}
		if(glfwGetKey(window, GLFW_KEY_W ) == GLFW_PRESS){
			coords[2] = glm::min(coords[2]+deltaTime*rotateSpeed, (float)M_PI-EPS);			
		}
		
		
		// Mouse based control of camera rotation
		// mouse doesn't need deltatime because it tracks difference in mouse position (deltaPos?)
		if(!mouseHold && GLFW_PRESS==glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)){ // first time mouse pressed
			glfwGetCursorPos(window, &prevMouseX, &prevMouseY);
            prevMouseY = screenH-prevMouseY;
            if(prevMouseX<0) prevMouseX=0;
            else if(prevMouseX>screenW) prevMouseX=screenW;
            if(prevMouseY<0) prevMouseY=0;
            else if(prevMouseY>screenH) prevMouseY=screenH;
			mouseHold = true;
		}
		else if(mouseHold && GLFW_PRESS==glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)){ //mouse still held
			glfwGetCursorPos(window, &currMouseX, &currMouseY);
			currMouseY = screenH-currMouseY;
            if(currMouseX<0) currMouseX=0;
            else if(currMouseX>screenW) currMouseX=screenW;
            if(currMouseY<0) currMouseY=0;
            else if(currMouseY>screenH) currMouseY=screenH;
			coords[1] -= MOUSE_SCALE*(prevMouseX-currMouseX);
			coords[2] -= MOUSE_SCALE*(prevMouseY-currMouseY);
			coords[2]=glm::clamp(coords[2], EPS, (float)M_PI-EPS);
			prevMouseX=currMouseX;
			prevMouseY=currMouseY;
		}
		else if(mouseHold && GLFW_PRESS!=glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)){ //mouse no longer held
		mouseHold = false;
		}
		
		// Calculate current camera position based on the coordinates
		eye = {coords[0]*sin(coords[2])*cos(coords[1]), 
				coords[0]*cos(coords[2]),
				coords[0]*sin(coords[1])*sin(coords[2])};

		
		// recalculate ModelViewProjection matrix
		V = glm::lookAt(eye, center, up);
		M = glm::mat4(1.0f);
		MV = V * M;
		glLoadMatrixf(glm::value_ptr(V));
		MVP = Projection * V * M;
		
		
		glUniformMatrix4fv(MVPID, 1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(VID, 1, GL_FALSE, &V[0][0]);
		glUniform3f(LightPosID, lightPos.x, lightPos.y, lightPos.z);
		glUniform4fv(colorID, 1, &color[0]);
		glUniform1f(alphaID, alpha);
	
		// Draw from server side array
		glBindVertexArray(vaoID[Windex]);
		glDrawArrays(GL_TRIANGLES, 0, marchingVerts[Windex].size()); 	
		glBindVertexArray(0);
		
		//Turn off shader for axes and box
		glUseProgram(0);
		ax.draw();

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();


	} // Check if the ESC key was pressed or the window was closed
	while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		   glfwWindowShouldClose(window) == 0 );

	// Close OpenGL window and terminate GLFW
	glfwTerminate();
	return 0;
}

// callback for mouse scroll to change "zoom in" or radius
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    if(yoffset>0) coords[0] *= scrollCamSpeedO;
	else if(yoffset<0) coords[0] *= scrollCamSpeedI;
}