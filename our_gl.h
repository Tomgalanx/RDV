#ifndef __OUR_GL_H__
#define __OUR_GL_H__
#include "tgaimage.h"
#include "geometry.h"

extern Matrix ModelView;
extern Matrix Projection;
extern Matrix Viewport;

Matrix viewport(int x, int y, int w, int h);
void projection(float coeff=0.f); // coeff = -1/c
Matrix lookat(Vec3f eye, Vec3f center, Vec3f up);
Matrix1 v2m(Vec3f v);
Vec3f m2v(Matrix1 m);
Matrix translate(float x, float y , float z);

struct IShader {
    virtual ~IShader();
    virtual Vec4f vertex(int iface, int nthvert) = 0;
    virtual bool fragment(Vec3f bar, TGAColor &color) = 0;
};

//void triangle(Vec4f *pts, IShader &shader, TGAImage &image, float *zbuffer);
void triangle(mat<4,3,float> &pts, IShader &shader, TGAImage &image, float *zbuffer,TGAColor color);
#endif //__OUR_GL_H__

