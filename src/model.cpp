#include "model.h"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

//构造函数，输入参数是.obj文件路径
Model::Model(const char *filename) : verts_(), faces_(), norms_(), uv_() {
    std::ifstream in;
    in.open (filename, std::ifstream::in);//打开.obj文件
    if (in.fail()) return;
    std::string line;
    while (!in.eof()) {//没有到文件末尾的话
        std::getline(in, line);//读入一行
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {//如果这一行的前两个字符是“v ”的话，代表是顶点数据
            iss >> trash;
            Vec3f v;//读入顶点坐标
            for (int i=0;i<3;i++) iss >> v[i];
            verts_.push_back(v);//加入顶点集
        } 
        //纹理内容
        else if(!line.compare(0, 3, "vt ")) {
            iss >> trash >> trash;
            Vec2f uv;
            for (int i = 0; i < 2; i++) iss >> uv[i];
            uv_.push_back(uv);
        }
        //纹理内容
        else if (!line.compare(0, 3, "vn ")) {
            iss >> trash >> trash;
            Vec3f normal;
            for (int i = 0; i < 3; i++) iss >> normal[i];
            norms_.push_back(normal);
        }
        else if (!line.compare(0, 2, "f ")) {//如果这一行的前两个字符是“f ”的话，代表是面片数据
            std::vector<Vec3i> f;
            Vec3i tmp;    //顶点索引，纹理坐标，法线向量
            iss >> trash;
            while (iss >> tmp[0] >> trash >> tmp[1] >> trash >> tmp[2]) {//读取顶点索引，
                for (int i = 0; i < 3; i++) tmp[i]--;
                f.push_back(tmp);
            }
            faces_.push_back(f);//把该面片加入模型的面片集
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << " vt# " << uv_.size() << " vn# " << norms_.size() << std::endl;  //输出顶点、面片、纹理坐标、法线向量数量
    loadTexture(filename, "_diffuse.tga", diffusemap_);     //纹理内容
    loadTexture(filename, "_nm.tga",      normalmap_);
    loadTexture(filename, "_spec.tga",    specularmap_);
}


Model::~Model() {
}

int Model::nverts() {
    return (int)verts_.size();
}

int Model::nfaces() {
    return (int)faces_.size();
}

std::vector<int> Model::face(int idx) {
    std::vector<int> face;
    std::vector<Vec3i> tmp = faces_[idx];
    for (int i = 0; i < tmp.size(); i++)
        face.push_back(tmp[i][0]);
    return face;
}

Vec3f Model::vert(int i) {
    return verts_[i];
}

Vec3f Model::vert(int iface, int nthvert) {
    return verts_[faces_[iface][nthvert][0]];
}

void Model::loadTexture(std::string filename, const char* suffix, TGAImage& image)
{
    std::string texfile(filename);
    size_t dot = texfile.find_last_of(".");
    if (dot != std::string::npos) {
        texfile = texfile.substr(0, dot) + std::string(suffix);
        std::cerr << "texture file " << texfile << " loading " << (image.read_tga_file(texfile.c_str()) ? "ok" : "failed") << std::endl;
        image.flip_vertically();
    }
}

TGAColor Model::diffuse(Vec2f uvf) {
    Vec2i uv(uvf[0]*diffusemap_.get_width(), uvf[1]*diffusemap_.get_height());
    return diffusemap_.get(uv[0], uv[1]);
}

Vec3f Model::normal(Vec2f uvf) {
    Vec2i uv(uvf[0]*normalmap_.get_width(), uvf[1]*normalmap_.get_height());
    TGAColor c = normalmap_.get(uv[0], uv[1]);
    Vec3f res;
    for (int i=0; i<3; i++)
        res[2-i] = (float)c[i]/255.f*2.f - 1.f;
    return res;
}

Vec2f Model::uv(int iface, int nthvert) {
    return uv_[faces_[iface][nthvert][1]];
}

float Model::specular(Vec2f uvf) {
    Vec2i uv(uvf[0]*specularmap_.get_width(), uvf[1]*specularmap_.get_height());
    return specularmap_.get(uv[0], uv[1])[0]/1.f;
}

Vec3f Model::normal(int iface, int nthvert) {
    int idx = faces_[iface][nthvert][2];
    return norms_[idx].normalize();
}

