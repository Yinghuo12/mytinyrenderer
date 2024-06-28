#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.h"
#include "tgaimage.h"

//模型类
class Model {
private:
	std::vector<Vec3f> verts_;//顶点集，每个顶点都是三维向量
	std::vector<std::vector<Vec3i> > faces_;//面片集

	//纹理内容
	std::vector<Vec3f> norms_;
	std::vector<Vec2f> uv_;
	TGAImage diffusemap_;
	TGAImage normalmap_;
	TGAImage specularmap_;

	void loadTexture(std::string filename, const char* suffix, TGAImage& image);


public:
	Model(const char *filename);//根据.obj文件路径导入模型
	~Model();
	int nverts();//返回模型顶点数量
	int nfaces();//返回模型面片数量
	Vec3f normal(int iface, int nthvert);
	Vec3f normal(Vec2f uv);
	Vec3f vert(int i);//返回第i个顶点
	Vec3f vert(int iface, int nthvert);
    Vec2f uv(int iface, int nthvert);
    TGAColor diffuse(Vec2f uv);
    float specular(Vec2f uv);
	std::vector<int> face(int idx);//返回第idx个面

};

#endif //__MODEL_H__