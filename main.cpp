#include <vector>
#include <limits>
#include <iostream>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

Model *model        = NULL;

const int width  = 800;
const int height = 800;


Vec3f light_dir(0,0,-3);
Vec3f       eye(2,0,3);
Vec3f    center(0,0,0);
Vec3f        up(0,1,0);


struct Shader : public IShader {
    mat<2,3,float> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
    mat<4,3,float> varying_tri; // triangle coordinates (clip coordinates), written by VS, read by FS
    mat<3,3,float> varying_nrm; // normal per vertex to be interpolated by FS
    mat<3,3,float> ndc_tri;     // triangle in normalized device coordinates

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        varying_nrm.set_col(nthvert, proj<3>((Projection*ModelView).invert_transpose()*embed<4>(model->normal(iface, nthvert), 0.f)));
        Vec4f gl_Vertex = Projection*ModelView*embed<4>(model->vert(iface, nthvert));
        varying_tri.set_col(nthvert, gl_Vertex);
        ndc_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f bn = (varying_nrm*bar).normalize();
        Vec2f uv = varying_uv*bar;

        mat<3,3,float> A;
        A[0] = ndc_tri.col(1) - ndc_tri.col(0);
        A[1] = ndc_tri.col(2) - ndc_tri.col(0);
        A[2] = bn;

        mat<3,3,float> AI = A.invert();

        Vec3f i = AI * Vec3f(varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0);
        Vec3f j = AI * Vec3f(varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0);

        mat<3,3,float> B;
        B.set_col(0, i.normalize());
        B.set_col(1, j.normalize());
        B.set_col(2, bn);

        Vec3f n = (B*model->normal(uv)).normalize();

        float diff = std::max(0.f, n*light_dir);
        color = model->diffuse(uv)*diff;

        return false;
    }
};


/*
 * Méthode pour afficher un rectangle a partir de trois points
 * En paramètre le ZBuffer et la couleur du triangle
 * (Méthode récupert sur le git tinyrender)
*/
void triangleTest(Vec3f t0, Vec3f t1, Vec3f t2, TGAImage &image, TGAColor color, float *zbuffer) {
    if (t0.y==t1.y && t0.y==t2.y) return; // i dont care about degenerate triangles
    if (t0.y>t1.y) std::swap(t0, t1);
    if (t0.y>t2.y) std::swap(t0, t2);
    if (t1.y>t2.y) std::swap(t1, t2);
    int total_height = t2.y-t0.y;
    for (int i=0; i<total_height; i++) {
        bool second_half = i>t1.y-t0.y || t1.y==t0.y;
        int segment_height = second_half ? t2.y-t1.y : t1.y-t0.y;
        float alpha = (float)i/total_height;
        float beta  = (float)(i-(second_half ? t1.y-t0.y : 0))/segment_height; // be careful: with above conditions no division by zero here
        Vec3i A =               t0 + (t2-t0)*alpha;
        Vec3i B = second_half ? t1 + (t2-t1)*beta : t0 + (t1-t0)*beta;
        if (A.x>B.x) std::swap(A, B);
        for (int j=A.x; j<=B.x; j++) {
            float phi = B.x==A.x ? 1. : (float)(j-A.x)/(float)(B.x-A.x);
            Vec3i P = A + (B-A)*phi;
            P.x = j; P.y = t0.y+i; // a hack to fill holes (due to int cast precision problems)
            int idx = j+(t0.y+i)*width;
            if (zbuffer[idx]<P.z) {
                zbuffer[idx] = P.z;
                image.set(P.x, P.y, color); // attention, due to int casts t0.y+i != A.y
            }
        }
    }
}


int main(int argc, char** argv) {
    if (2>argc) {
        std::cerr << "Usage: " << argv[0] << " obj/model.obj" << std::endl;
        return 1;
    }

    // Tous les ZBuffers pour toutes les images :

    // Image en 3D sans texture, sans anaglyph
    float *zbuffer = new float[width*height];

    // Image en anaglyph, droite(bleu)
    float *zbuffer_droit = new float[width*height];

    // Image en anaglyph, gauche(rouge)
    float *zbuffer_gauche = new float[width*height];


    // Image en 3D avec texture
    float *zbuffer_texture = new float[width*height];


    // Initialisation des ZBuffers
    for (int i=width*height; i--;){

        zbuffer[i] = -std::numeric_limits<float>::max();
        zbuffer_droit[i] = -std::numeric_limits<float>::max();
        zbuffer_gauche[i] = -std::numeric_limits<float>::max();
        zbuffer_texture[i] = -std::numeric_limits<float>::max();
    }


    // Création des TGAImage pour chaque image


    // Image utilise pour la 3D et l'anaglyph
    TGAImage frame(width, height, TGAImage::RGB);


    // Les deux images de l'anaglyph
    TGAImage gauche(width, height, TGAImage::RGB);
    TGAImage droite(width, height, TGAImage::RGB);

    // Image avec texture
    TGAImage texture(width, height, TGAImage::RGB);



    // Mise en place de la caméra
    lookat(eye, center, up);
    viewport(width/8, height/8, width*3/4, height*3/4);
    projection(-1.f/(eye-center).norm());
    light_dir = proj<3>((Projection*ModelView*embed<4>(light_dir, 0.f))).normalize();


    for (int m=1; m<argc; m++) {

        // On charge le model
        model = new Model(argv[m]);
        Shader shader;
        for (int i=0; i<model->nfaces(); i++) {

            std::vector<int> face = model->face(i);

            // Attrituts pour les coordonnées des triangles
            Vec3f world_coords[3];
            Vec3i screen_coords[3];

            Vec3f gauche_coords[3];
            Vec3f droite_coords[3];


            for (int j=0; j<3; j++) {
                shader.vertex(i, j);

                Vec3f v = model->vert(face[j]);

                // Coordonné simple utillisé pour la 3D sans anaglyph
                screen_coords[j] = Vec3i((v.x+1.)*width/2., (v.y+1.)*height/2., (v.z+1.)*255/2.);

                // Création de deux caméras pour la vision anaglyph

                // Décalage vers la gauche
                gauche_coords[j]=Vec3i(  ((v.x+1.)*width/2.)-8 , (v.y+1.)*height/2., (v.z+1.)*255/2.);
                // Décalage vers la droite
                droite_coords[j]=Vec3i(  ((v.x+1.)*width/2.)+8 , (v.y+1.)*height/2., (v.z+1.)*255/2.);

                world_coords[j]  = v;

            }

            // Création des triangles pour la textures
            triangle(shader.varying_tri, shader, texture, zbuffer_texture);


            // Calcul de l'intensité de la lumière
            Vec3f n = cross((world_coords[2] - world_coords[0]),(world_coords[1] - world_coords[0]));
            n.normalize();
            float intensity = n*light_dir;

            //printf("Intensity %f",intensity);
            if (intensity>0) {


                // Création des triangles pour l'anaglyph
                triangleTest(droite_coords[0], droite_coords[1], droite_coords[2], droite, TGAColor(intensity*255,0,0), zbuffer_droit);
                triangleTest(gauche_coords[0], gauche_coords[1], gauche_coords[2], gauche, TGAColor(0, 0, intensity*255), zbuffer_gauche);


                // Création des triangles pour la 3D sans anaglyph
                triangleTest(screen_coords[0],screen_coords[1],screen_coords[2],frame,TGAColor(intensity*255, intensity*255, intensity*255, 255), zbuffer);

            }


        }
        delete model;
    }

    //frame = gauche.fusionner(droite);


    // affichage de la 3D sans anaglyph
    frame.flip_vertically(); // to place the origin in the bottom left corner of the image
    frame.write_tga_file("3D.tga");

    // Fusion des deux images (droite et gauche)
    frame = gauche.unification(droite);

    // Affichage de l'anaglyph
    frame.flip_vertically(); // to place the origin in the bottom left corner of the image
    frame.write_tga_file("anaglyph.tga");


    // Affichage de la texture
    texture.flip_vertically(); // to place the origin in the bottom left corner of the image
    texture.write_tga_file("texture.tga");

    delete [] zbuffer;
    return 0;
}

