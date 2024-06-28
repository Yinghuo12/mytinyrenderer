#include <cmath>
#include <limits>       //用于定义无穷
#include <iostream>
#include <vector>
#include <algorithm>

#include "tgaimage.h"   //tga画图库
#include "model.h"      //模型类，主要实现模型的读取
#include "geometry.h"   //几何库，主要定义了Vec2和Vec3类型


//定义颜色
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(0,   255, 0,   255);

//定义宽度高度深度
const int width  = 800;
const int height = 800;
const int depth  = 255;


//初始化模型
//Model * model = new Model("../obj/diablo3_pose/diablo3_pose.obj");
Model * model = new Model("../obj/african_head/african_head.obj");

//创建深度缓冲矩阵
float *zbuffer = new float[width*height];
void clearzbuffer(){
    for (int i = width*height; i--; zbuffer[i] = -std::numeric_limits<float>::max());  //(-∞)
}


//位置信息
Vec3f light_dir = Vec3f(0, 0, -1).normalize();      //光源位置  光照负方向 即光源相对于物体的位置
Vec3f cameraPos(1, 0.5, 1.5);         //相机位置
Vec3f centerPos(0, 0, 0);             //中心点位置
Vec3f        up(0, 1, 0);             //指向上方向的向量



//四阶列向量
Matrix local2homo(Vec3f v) {
    Matrix m(4, 1);
    m[0][0] = v.x;
    m[1][0] = v.y;
    m[2][0] = v.z;
    m[3][0] = 1.0f;
    return m;
}

//降维
Vec3f homo2vertices(Matrix m) {
    return Vec3f(m[0][0], m[1][0], m[2][0]);
}

//模型变换矩阵
Matrix modelMatrix() {
    return Matrix::identity(4);   //模型坐标已经是NDC坐标([-1, 1]范围内),因此无需变换，用单位矩阵代替
}

//视图变换矩阵
Matrix viewMatrix() {
    return Matrix::identity(4);
}

//透视投影变换矩阵
Matrix projectionMatrix() {
    Matrix projection = Matrix::identity(4);
    //projection[3][2] = -1.0f / (cameraPos - centerPos).norm()；
    projection[3][2] = -1.0f / cameraPos.z;
    return projection;
}

//透视除法（前三个分量都除以第四个分量 即第四维归一）
Matrix projectionDivision(Matrix m) {
    m[0][0] = m[0][0] / m[3][0];
    m[1][0] = m[1][0] / m[3][0];
    m[2][0] = m[2][0] / m[3][0];
    m[3][0] = 1.0f;
    return m;
}

//视口变换矩阵     //将[-1,1]^2中的点变换到以(x,y)为原点，w,h为宽与高的屏幕区域内
Matrix viewportMatrix(int x, int y, int w, int h) {
    Matrix m = Matrix::identity(4);
    m[0][3] = x + w / 2.f;
    m[1][3] = y + h / 2.f;
    m[2][3] = depth / 2.f;

    m[0][0] = w / 2.f;
    m[1][1] = h / 2.f;
    m[2][2] = depth / 2.f;
    return m;
}


//摄像机变换矩阵    
//https://zhuanlan.zhihu.com/p/400791821   
//https://www.zhihu.com/question/447781866/answer/1859618164 
//https://blog.csdn.net/qq960885333/article/details/8448036
//更改摄像机视角=更改物体位置和角度，操作为互逆矩阵
//摄像机变换是先旋转再平移，所以物体需要先平移后旋转，且都是逆矩阵
Matrix cameraMatrix(Vec3f camera, Vec3f center, Vec3f up) {
    //计算出z，根据z和up算出x，再算出y
    Vec3f z = (camera - center).normalize();
    Vec3f x = (up ^ z).normalize();
    Vec3f y = (z ^ x).normalize();
    Matrix rotation = Matrix::identity(4);
    Matrix translation = Matrix::identity(4);
    //***矩阵的第四列是用于平移的,需要将物体平移-camera***
    for (int i = 0; i < 3; i++) {
        translation[i][3] = -camera[i];
    }
    //正交矩阵的逆 = 正交矩阵的转置
    for (int i = 0; i < 3; i++) {
        rotation[0][i] = x[i];
        rotation[1][i] = y[i];
        rotation[2][i] = z[i];
    }
    //这样乘法的效果是先平移物体，再旋转
    Matrix res = rotation * translation;
    return res;
}

//mvp变换和视口变换
Matrix model_ = modelMatrix();
Matrix view_ = viewMatrix();
Matrix projection_ = projectionMatrix();
//Matrix viewport_ = viewportMatrix(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
Matrix viewport_ = viewportMatrix(0, 0, width, height);
Matrix camera_ = cameraMatrix(cameraPos, centerPos, up);



//画线算法
void line(int x0, int y0, int x1, int y1, TGAImage &image, const TGAColor& color){
    bool steep = false;
    //如果陡线，则化为缓线  加绝对值是因为要考虑斜率小于-1的情况 
    if(std::abs(x0 - x1) < std::abs(y0 - y1)){
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    //保持从左往右画
    if(x0 > x1){
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = y1 - y0;

    //计算斜率
    //float derror = std::abs(dy/static_cast<float>(dx));
    // float error = 0.0f;

    int derror = std::abs(dy) * 2;   //为了优化浮点数除法运算的消耗时间，而采用整数除法
    int error = 0;
    
    //从x0开始画
    int y = y0;
    for(int x = x0; x <= x1; x++){
        //若斜率大于1，真实坐标为(y,x)；否则为(x,y)
        if(steep){
            image.set(y, x, color);
        }else{
            image.set(x, y, color);
        }
        error += derror;
        //误差矫正
        // if(error > 0.5f){
        if(error > dx) {
            y += (y1 > y0 ? 1 : -1);
            // error -= 1.0f;
            error -= dx * 2;
        }
    }
}


//扫描线算法着色(坐标1，坐标2，坐标3，tga指针，颜色)
void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
    //三角形面积为0的情况(三点共y则跳过该三角形)
    if (t0.y == t1.y && t0.y == t2.y) return;

    //根据y的大小对坐标进行排序，从上往下依次为t2,t1,t0
    if (t0.y > t1.y) std::swap(t0, t1);
    if (t0.y > t2.y) std::swap(t0, t2);
    if (t1.y > t2.y) std::swap(t1, t2);

    int total_height = t2.y - t0.y;

    //以高度差作为循环控制变量，此时不需要考虑斜率，因为着色完后每行都会被填充
    for (int i = 0; i < total_height; i++) {
        //根据t1将三角形分割为上下两半
        bool top_triangle = (i > t1.y - t0.y || t1.y == t0.y);       //判断是否是上三角
        int segment_height = top_triangle ? t2.y-t1.y : t1.y-t0.y;   //上三角的高和下三角的高

        //类似放缩比
        float alpha = static_cast<float>(i)/total_height;
        float beta  = top_triangle ? static_cast<float>(i-(t1.y-t0.y))/segment_height : static_cast<float>(i)/segment_height;    
        //float beta  = (float)(i-(top_triangle ? t1.y-t0.y : 0))/segment_height;   //更加简洁的写法
        //beta: 该位置在上/下三角形中所占比例

        /****************
            注意：这里的除法要先取float再除法，而不是先除法再转换为float
            以下则是错误的写法：
            float beta  = top_triangle ? (float)((i-(t1.y-t0.y))/segment_height) : (float)(i/segment_height);
        *****************/
       
        //计算A,B两点的坐标，利用直线的参数方程
        Vec2i A =                                  t0 +(t2-t0)*alpha;      //从t0指向t2的向量乘上放缩比（斜边即最长边）
        Vec2i B = top_triangle ? t1+(t2-t1)*beta : t0+(t1-t0)*beta;       //上三角则t1指向t2的向量乘占比，下三角则t0指向t1的向量乘占比

        if (A.x > B.x) std::swap(A, B);

        //根据A,B和当前高度对tga着色  每轮循环时的高度为t0.y+i
        for (int j = A.x; j <= B.x; j++) {
            image.set(j, t0.y+i, color);
        }
    }
}



//计算重心坐标函数  
//(利用叉乘判断是否在三角形内部)
Vec3f barycentric(Vec3f *pts, Vec3f P) {
   //计算向量[AB,AC,PA]
    Vec3f AB(pts[1].x - pts[0].x, pts[1].y - pts[0].y, pts[1].z - pts[0].z);
    Vec3f AC(pts[2].x - pts[0].x, pts[2].y - pts[0].y, pts[2].z - pts[0].z);
    Vec3f PA(pts[0].x - P.x, pts[0].y - P.y, pts[0].z - P.z);

    //法向量n:[u,v,1]分别与[ABx,ACx,PAx],[ABy,ACy,PAy]垂直，则后两个叉乘值为k[u,v,1]=[ku,kv,k]  ①k不为0时,同除k可得[u,v,1]  ②对于现在的应用场景，只要检测到k为0，则三点共线
    Vec3f X(AB.x, AC.x, PA.x);
    Vec3f Y(AB.y, AC.y, PA.y);
    Vec3f n = X ^ Y;
    //三点共线时，叉乘结果为0向量,此时返回(-1,1,1)
    if (abs(n.z) > 1e-2)
        //若1-u-v，u，v全为大于0的数，表示点在三角形内部
        return Vec3f(1.f-(n.x+n.y)/n.z, n.x/n.z, n.y/n.z);    //AP=uAB+vAC等价于P=(1-u-v)A+uB+vC  注意这里写法，先加再除比先除再加精度要高，否则会出现很多黑点
    return Vec3f(-1,1,1);
}




//包围盒平面着色
void Rasterization(Vec3f* pts, TGAImage& image, const TGAColor& color)
{
    //包围盒
    Vec2f bboxMin(image.get_width() - 1, image.get_height() - 1);   //图片的右下角(像素的范围从0开始，而宽度从1开始)
    Vec2f bboxMax(0, 0);  //左上角
    //计算三角形的包围盒
    bboxMin.x = std::min({ bboxMin.x, pts[0].x, pts[1].x, pts[2].x });
    bboxMin.y = std::min({ bboxMin.y, pts[0].y, pts[1].y, pts[2].y });
    bboxMax.x = std::max({ bboxMax.x, pts[0].x, pts[1].x, pts[2].x });
    bboxMax.y = std::max({ bboxMax.y, pts[0].y, pts[1].y, pts[2].y });

    Vec3f P;
    //遍历包围盒内的所有像素，根据重心坐标判断是否在三角形内部，如果在，就绘制这个像素，否则就忽略它
    for (P.x = bboxMin.x;  P.x <= bboxMax.x; P.x++)
    {
        for (P.y = bboxMin.y; P.y <= bboxMax.y; P.y++)
        {
            Vec3f baryCoord = barycentric(pts, P);
            if (baryCoord.x < 0 || baryCoord.y < 0 || baryCoord.z < 0)
                continue;
            image.set(P.x, P.y, color);
        }
    }
   
}

//世界坐标转屏幕坐标函数（视口变换）
Vec3f World2Screen(Vec3f v) {
    return Vec3f(static_cast<int>((v.x+1.0)*width/2.0), static_cast<int>((v.y+1.0)*height/2.0), v.z); //屏幕坐标一定是int类型 否则会出现破面情况
}


//绘制zbuffer三角形(坐标数组，zbuffer指针，tga指针，颜色)
void zbuffer_triangle(Vec3f *pts, float *zbuffer, TGAImage &image, TGAColor color) {

   //包围盒
    Vec2f bboxMin(image.get_width() - 1, image.get_height() - 1);   //图片的右下角(像素的范围从0开始，而宽度从1开始)
    Vec2f bboxMax(0, 0);  //左上角

    //计算三角形的包围盒
    bboxMin.x = std::min({ bboxMin.x, pts[0].x, pts[1].x, pts[2].x });
    bboxMin.y = std::min({ bboxMin.y, pts[0].y, pts[1].y, pts[2].y });
    bboxMax.x = std::max({ bboxMax.x, pts[0].x, pts[1].x, pts[2].x });
    bboxMax.y = std::max({ bboxMax.y, pts[0].y, pts[1].y, pts[2].y });


    Vec3f P;
    //遍历包围盒内的所有像素，根据重心坐标判断是否在三角形内部，如果在，就绘制这个像素，否则就忽略它
    for (P.x = bboxMin.x;  P.x <= bboxMax.x; P.x++)
    {
        for (P.y = bboxMin.y; P.y <= bboxMax.y; P.y++)
        {
            Vec3f baryCoord = barycentric(pts, P);
            if (baryCoord.x < 0 || baryCoord.y < 0 || baryCoord.z < 0)
                continue;
            //计算zbuffer，并且每个顶点的z值乘上对应的质心坐标分量
            P.z = pts[0].z * baryCoord.x + pts[1].z * baryCoord.y + pts[2].z * baryCoord.z;
                
            if (zbuffer[static_cast<int>(P.x+P.y*width)] < P.z) {   //将像素点的坐标转换为整数，以便在深度缓冲中进行索引。
                zbuffer[static_cast<int>(P.x+P.y*width)] = P.z;
                image.set(P.x, P.y, color);
            }
        }
    }
}




//绘制zbuffer三角形+纹理贴图(漫反射纹理)(坐标数组，纹理数组，zbuffer指针，tga指针，颜色)
void zbuffer_texture_triangle(Vec3f *pts, Vec2f* uvs, float *zbuffer, TGAImage &image, float intensity) {

    // 包围盒
    Vec2f bboxMin(image.get_width() - 1, image.get_height() - 1);   //图片的右下角(像素的范围从0开始，而宽度从1开始)
    Vec2f bboxMax(0, 0);  //左上角

    //计算三角形的包围盒
    bboxMin.x = std::min({ bboxMin.x, pts[0].x, pts[1].x, pts[2].x });
    bboxMin.y = std::min({ bboxMin.y, pts[0].y, pts[1].y, pts[2].y });
    bboxMax.x = std::max({ bboxMax.x, pts[0].x, pts[1].x, pts[2].x });
    bboxMax.y = std::max({ bboxMax.y, pts[0].y, pts[1].y, pts[2].y });

    Vec3f P;
    //遍历包围盒内的所有像素，根据重心坐标判断是否在三角形内部，如果在，就绘制这个像素，否则就忽略它    
    for (P.x = bboxMin.x; P.x <= bboxMax.x; P.x++)
    {
        for (P.y = bboxMin.y; P.y <= bboxMax.y; P.y++)
        {
            Vec2f uvP;
            Vec3f baryCoord = barycentric(pts, P);
            if (baryCoord.x < 0 || baryCoord.y < 0 || baryCoord.z < 0)
                continue;
            //计算zbuffer，每个顶点的z值乘上对应的质心坐标分量  
            P.z = pts[0].z*baryCoord.x + pts[1].z*baryCoord.y + pts[2].z*baryCoord.z;
                
            //计算纹理坐标
            uvP = uvs[0]*baryCoord.x + uvs[1]*baryCoord.y + uvs[2]*baryCoord.z;

            if (zbuffer[static_cast<int>(P.x+P.y*width)] < P.z) {   //将像素点的坐标转换为整数，以便在深度缓冲中进行索引。
                zbuffer[static_cast<int>(P.x+P.y*width)] = P.z;
                TGAColor color = model->diffuse(uvP) * intensity;
                image.set(P.x, P.y, color);
            }
        }
    }
}


/***********************************以下为测试代码**************************************************/


//测试画线函数
void test_line(){
    //构造tga(宽，高，指定颜色空间)
    TGAImage image(100, 100, TGAImage::RGB);
    line(13, 20, 80, 40, image, white);    //线段A
    line(20, 13, 40, 80, image, red);      //线段B
    line(80, 40, 13, 20, image, red);      //线段C

    image.flip_vertically();
    image.write_tga_file("line.tga");
}


//测试模型画线
void test_line_model(){

    TGAImage  image(width, height, TGAImage::RGB);

    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i); //创建face数组用于保存一个face的三个顶点坐标
        for (int j = 0; j < 3; j++) { //每次取出face数组中的两个点画线
            Vec3f v0 = model->vert(face[j]);
            Vec3f v1 = model->vert(face[(j + 1) % 3]);
            //根据顶点v0和v1画线
            //先要进行模型坐标到屏幕坐标的转换。  (-1,-1)对应(0,0)：左下角   (1,1)对应(width,height)：右上角
            int x0 = (v0.x + 1.0) * width / 2.0;
            int y0 = (v0.y + 1.0) * height / 2.0;
            int x1 = (v1.x + 1.0) * width / 2.0;
            int y1 = (v1.y + 1.0) * height / 2.0;

            //画线
            line(x0, y0, x1, y1, image, white);
        }
    }

    image.flip_vertically();
    image.write_tga_file("line_model.tga");


}



//测试三角形平面着色
void test_triangle(){
    //构造tga(宽，高，指定颜色空间)
    TGAImage image(200, 200, TGAImage::RGB);
    Vec2i t0[3] = { Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80) };
	Vec2i t1[3] = { Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180) };
	Vec2i t2[3] = { Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180) };
	triangle(t0[0], t0[1], t0[2], image, red);
	triangle(t1[0], t1[1], t1[2], image, white);
	triangle(t2[0], t2[1], t2[2], image, green);

    image.flip_vertically();
    image.write_tga_file("triangle.tga");

    
}



//测试模型平面着色（光栅化）
void test_triangle_model(){
  
    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < model->nfaces(); i++) {    //对于每个三角形
        std::vector<int> face = model->face(i);    //face存储一个面的三个顶点
        Vec3f screen_coords[3];  //屏幕坐标
        Vec3f world_coords[3];   //空间坐标
        for (int j = 0; j < 3; j++) {    //对于三角形的每个顶点
            world_coords[j] = model->vert(face[j]);    //空间坐标即模型坐标
            screen_coords[j] = World2Screen(world_coords[j]);    //屏幕坐标    (-1,-1)映射为(0,0)  （1,1）映射为(width,height)    
        }

        //用空间坐标计算法向量
        Vec3f n = ((world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]));     //向量叉乘运算
        n.normalize();         //归一化处理

        float intensity = n * light_dir;   //光照强度=法向量*光照方向   即法向量和光照方向重合时，亮度最高
        //强度小于0，说明平面朝向为内  即背面裁剪
        if (intensity > 0) {
            Rasterization(screen_coords, image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
           // Rasterization(screen_coords, image, TGAColor(rand()%255, rand()%255, rand()%255, 255));   
        }
    }

    image.flip_vertically();
    image.write_tga_file("triangle_model.tga");



}




//测试模型Z-buffer平面着色(光栅化)
void test_zbuffer_model(){
    clearzbuffer();
    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < model->nfaces(); i++) {    //对于每个三角形
        std::vector<int> face = model->face(i);    //face存储一个面的三个顶点
        Vec3f screen_coords[3];  //屏幕坐标
        Vec3f world_coords[3];   //空间坐标
        for (int j = 0; j < 3; j++) {    //对于三角形的每个顶点
            world_coords[j]  = model->vert(face[j]);;       //空间坐标即模型坐标
            //世界坐标转换屏幕坐标
            screen_coords[j] = World2Screen(world_coords[j]);    //屏幕坐标    (-1,-1)映射为(0,0)  （1,1）映射为(width,height)
        }

        //用空间坐标计算法向量
        Vec3f n = ((world_coords[2] - world_coords[0])^(world_coords[1] - world_coords[0]));     //向量叉乘运算
        n.normalize();
        float intensity = n * light_dir;   //光照强度=法向量*光照方向   即法向量和光照方向重合时，亮度最高
        //强度小于0，说明平面朝向为内  即背面裁剪
        //渲染屏幕坐标
        if (intensity > 0) {
            zbuffer_triangle(screen_coords, zbuffer, image, TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));
        }
    }

    image.flip_vertically();
    image.write_tga_file("Z-buffer_model.tga");


}


//测试模型Z-buffer平面着色(光栅化+纹理贴图)
void test_zbuffer_texture_model(){
    clearzbuffer();

    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < model->nfaces(); i++) {    //对于每个三角形
        std::vector<int> face = model->face(i);    //face存储一个面的三个顶点
        Vec3f screen_coords[3];  //屏幕坐标
        Vec3f world_coords[3];   //空间坐标
        for (int j = 0; j < 3; j++) {    //对于三角形的每个顶点
            world_coords[j]  = model->vert(face[j]);;       //空间坐标即模型坐标
            //世界坐标转换屏幕坐标
            screen_coords[j] = World2Screen(world_coords[j]);    //屏幕坐标    (-1,-1)映射为(0,0)  （1,1）映射为(width,height)
        }

        //用空间坐标计算法向量
        Vec3f n = ((world_coords[2] - world_coords[0])^(world_coords[1] - world_coords[0]));     //向量叉乘运算
        n.normalize();
        float intensity = n * light_dir;   //光照强度=法向量*光照方向   即法向量和光照方向重合时，亮度最高
        //强度小于0，说明平面朝向为内  即背面裁剪
        //渲染屏幕坐标
        if (intensity > 0) {
            Vec2f uv[3];
            for (int j = 0; j < 3; j++) uv[j] = model->uv(i, j);
            zbuffer_texture_triangle(screen_coords, uv, zbuffer, image, intensity);
        }
    }

    image.flip_vertically();
    image.write_tga_file("Z-buffer_texture_model.tga");

}




//Perspective projection/Moving the camera 透视投影与相机移动
void test_perspective_projection(){

    clearzbuffer();

    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < model->nfaces(); i++)
    {
        std::vector<int> face = model->face(i);   //获取模型的第i个面片
        Vec3f screen_coords[3];    //存贮第i个面片三个顶点的屏幕坐标
        Vec3f world_coords[3];     //存储第i个面片三个顶点的世界坐标
        for (int j = 0; j < 3; j++)
        {
            world_coords[j] = model->vert(face[j]);
            //Vec3f final_matrix = homo2vertices(viewport_ * projectionDivision(projection_ * view_ * model_ * local2homo(world_coords[j])));
            Vec3f final_matrix= homo2vertices(viewport_  * projectionDivision(projection_ * view_ *  model_ * camera_ * local2homo(world_coords[j])));
            screen_coords[j] = {static_cast<int>(final_matrix.x), static_cast<int>(final_matrix.y), static_cast<int>(final_matrix.z)};
            
        }

        Vec3f normal = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
        normal.normalize();
        float intensity = normal * light_dir;
        if (intensity > 0)
        {
            Vec2f uv[3];
            for (int j = 0; j < 3; j++) uv[j] = model->uv(i, j);
            zbuffer_texture_triangle(screen_coords, uv, zbuffer, image, intensity);
        }
    }


    image.flip_vertically();
    image.write_tga_file("perspective_projection.tga");

}




// //Lesson 6: Shader
class IShader {

public:
    virtual Vec3f vertex(int iface, int nthvert) = 0;        //面片和顶点
    virtual bool fragment(Vec3f barycoord, TGAColor &color) = 0;   //片元和颜色
    void Shader(Vec3f *pts, IShader &shader, TGAImage &image, TGAImage &zbufferImage);

};



void IShader::Shader(Vec3f *pts, IShader &shader, TGAImage &image, TGAImage & zbuffer_image) {
    // 包围盒
    Vec2f bboxMin(image.get_width() - 1, image.get_height() - 1);   //图片的右下角(像素的范围从0开始，而宽度从1开始)
    Vec2f bboxMax(0, 0);  //左上角

    //计算三角形的包围盒
    bboxMin.x = std::min({ bboxMin.x, pts[0].x, pts[1].x, pts[2].x });
    bboxMin.y = std::min({ bboxMin.y, pts[0].y, pts[1].y, pts[2].y });
    bboxMax.x = std::max({ bboxMax.x, pts[0].x, pts[1].x, pts[2].x });
    bboxMax.y = std::max({ bboxMax.y, pts[0].y, pts[1].y, pts[2].y });

    Vec3f P;
    TGAColor color;
    //遍历包围盒内的所有像素，根据重心坐标判断是否在三角形内部，如果在，就绘制这个像素，否则就忽略它
    for (P.x = bboxMin.x; P.x <= bboxMax.x; P.x++) {
        for (P.y = bboxMin.y; P.y <= bboxMax.y; P.y++) {
            Vec3f baryCoord = barycentric(pts, P);
            
            float z_P = pts[0].z*baryCoord.x + pts[1].z*baryCoord.y + pts[2].z*baryCoord.z;   //计算当前像素的深度值
            int frag_depth = std::max(0, std::min(255, static_cast<int>(z_P+.5)));  //将深度值转换为 0-255之间的整数
           
            //如果当前像素的深度值小于zbuffer中该像素的深度值，则更新该像素的深度值和颜色值，否则跳过
            if (baryCoord.x < 0 || baryCoord.y < 0 || baryCoord.z < 0 || zbuffer_image.get(P.x, P.y)[0] > frag_depth) 
                continue;

            //调用片元着色器计算当前像素颜色
            bool discard = shader.fragment(baryCoord, color);
            if (!discard) {
                //zbufferImage
                zbuffer_image.set(P.x, P.y, TGAColor(frag_depth));
                image.set(P.x, P.y, color);
            }
        }
    }
}




//高洛德着色器
class GouraudShader : public IShader {
public:
   
    //根据传入的质心坐标，颜色，以及varying_intensity计算出当前像素的颜色
    virtual bool fragment(Vec3f barycoord, TGAColor &color) {
        float intensity = varying_intensity * barycoord;

        if (intensity>.85) intensity = 1;
        else if (intensity>.60) intensity = .80;
        else if (intensity>.45) intensity = .60;
        else if (intensity>.30) intensity = .45;
        else if (intensity>.15) intensity = .30;
        else intensity = 0;


        color = TGAColor(255, 255, 255)*intensity;
        return false;                              
    }

    
    //接受两个变量，(面序号，顶点序号)
    virtual Vec3f vertex(int iface, int nthvert) {     //重写虚函数
        //根据面序号和顶点序号读取模型对应顶点，并扩展为4维 
        Vec3f gl_Vertex = model->vert(iface, nthvert);   //模型顶点
        //变换顶点坐标到屏幕坐标（视角矩阵*投影矩阵*变换矩阵*v）
        Matrix vertex = viewport_  * projectionDivision(projection_ * view_ *  model_ * local2homo(gl_Vertex));
        Vec3f result = homo2vertices(vertex);

        //计算光照强度（顶点法向量*光照方向）
        // Vec3f normal = proj<3>(embed<4>(model->normal(iface, nthvert))).normalize();
        //varying_intensity[nthvert] = std::max(0.f, model->normal(iface, nthvert) *light_dir); // get diffuse lighting intensity
        
        return result;
    }


public:
    //顶点着色器会将数据写入varying_intensity
    //片元着色器从varying_intensity中读取数据
    Vec3f varying_intensity; 

};




void test_shader() {
    clearzbuffer();
    TGAImage         image(width, height, TGAImage::RGB);
    TGAImage zbuffer_image(width, height, TGAImage::GRAYSCALE);

    //实例化高洛德着色
    GouraudShader gouraud_shader;

    for (int i=0; i<model->nfaces(); i++) {     //对于每个三角形
        Vec3f screen_coords[3];
        for (int j=0; j<3; j++) {
            //通过顶点着色器读取模型顶点
            //变换顶点坐标到屏幕坐标（视角矩阵*投影矩阵*变换矩阵*v） ***其实并不是真正的屏幕坐标，因为没有除以最后一个分量
            //计算光照强度
            screen_coords[j].x = static_cast<int>(gouraud_shader.vertex(i, j).x);
            screen_coords[j].y = static_cast<int>(gouraud_shader.vertex(i, j).y);
            screen_coords[j].z = gouraud_shader.vertex(i, j).z;
        }
        //遍历完3个顶点，一个三角形光栅化完成
        //绘制三角形，triangle内部通过片元着色器对三角形着色
        gouraud_shader.Shader(screen_coords, gouraud_shader, image, zbuffer_image);
    }

    image.flip_vertically();
    image.write_tga_file("shader.tga");
    zbuffer_image.flip_vertically();
    zbuffer_image.write_tga_file("shader_zbuffer.tga");

}





/**************************************以上为测试代码****************************************/



int main(int argc, char** argv){

    test_line();
    test_line_model();
    test_triangle();
    test_triangle_model();
    test_zbuffer_model();
    test_zbuffer_texture_model();
    test_perspective_projection();  
    test_shader();

    delete[] zbuffer;   
    delete model;

    return 0;

}