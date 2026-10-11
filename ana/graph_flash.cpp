// inherited ~/tools/graph.cpp

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cmath>
#include <simple_svg_1.0.0.hpp>
#include <algorithm>

#define T_LIMIT 2500
#define X_LIMIT 40
#define MAIN_LINE_WIDTH 0.5
#define AXIS_LINE_WIDTH1 0.1
#define AXIS_LINE_WIDTH2 0.3
#define AXIS_LINE_WIDTH3 0.5
#define AXIS_COLOR 100
#define M_MAX 300
#define PI 3.1415
#define CONTROL_HZ 30 // honban.cppのCONTROL_HZ (mode4の周期)
//#define INPUT_FILE_PATH "../record/tugounoiide-ta/free_fall.csv"
//#define INPUT_FILE_PATH "../record/tugounoiide-ta/launch.csv"
#define INPUT_FILE_PATH "../record/tugounoiide-ta/drop1.csv"
//#define INPUT_FILE_PATH "../record/tugounoiide-ta/tsurisage_seigyo2.csv"
//#define INPUT_FILE_PATH "../record/tugounoiide-ta/tsurisage_nashi2.csv"

//index,flag1,flag2,ori,avel,acce,mp,temp ( Big-Endian )

using namespace std;
using namespace svg;


class Vec;

class Quat{
    public:
        float w;
        float x;
        float y;
        float z;
        constexpr Quat() : w(1),x(0),y(0),z(0){}
        Quat(float _w,float _x,float _y,float _z) : w(_w),x(_x),y(_y),z(_z){}
        friend Quat operator*(const Quat &q1 , const Quat &q2){
            return Quat(
                    -q1.x*q2.x - q1.y*q2.y - q1.z*q2.z + q1.w*q2.w,
                     q1.w*q2.x - q1.z*q2.y + q1.y*q2.z + q1.x*q2.w,
                     q1.z*q2.x + q1.w*q2.y - q1.x*q2.z + q1.y*q2.w,
                    -q1.y*q2.x + q1.x*q2.y + q1.w*q2.z + q1.z*q2.w
                    );
        }
        // copy constructor ( so that Quat can use as Vector<Quat> )
        Quat(const Quat &rhs) : w(rhs.w),x(rhs.x),y(rhs.y),z(rhs.z){}
        Vec toV();
        void print(){
            cout << w << " "  << x << " " << y << " " << z << endl;
        }
        Quat invert(){
            Quat q;
            q.w =  w;
            q.x = -x;
            q.y = -y;
            q.z = -z;
            return q;
        }

};
class Vec{
    public:
        float x;
        float y;
        float z;

        Vec() : x(0),y(0),z(0){}
        Vec(float _x,float _y,float _z) : x(_x),y(_y),z(_z){}
        // copy constructor 
        Vec(const Vec &rhs) : x(rhs.x),y(rhs.y),z(rhs.z){}
        // multiply
        friend Vec operator+(const Vec v1 , const Vec v2){
            return Vec(v1.x+v2.x, v1.y+v2.y, v1.z+v2.z);
        }
        //無駄なcopyを防ぐために
        Vec& operator+=(const Vec v2){
            x += v2.x;
            y += v2.y;
            z += v2.z;
            return *this;
        }
        friend Vec operator-(const Vec v1 , const Vec v2){
            return Vec(v1.x-v2.x, v1.y-v2.y, v1.z-v2.z);
        }
        Vec& operator-=(const Vec v2){
            x -= v2.x;
            y -= v2.y;
            z -= v2.z;
            return *this;
        }
        friend Vec operator*(const Vec v , const float f){
            return Vec(v.x*f,v.y*f,v.z*f);
        }
        Vec& operator*=(const float f){
            x *= f;
            y *= f;
            z *= f;
            return *this;
        }
        friend Vec operator/(const Vec v , const float f){
            return Vec(v.x/f,v.y/f,v.z/f);
        }
        // cross
        friend Vec operator%(const Vec v1 , const Vec v2){
            return Vec( 
                v1.y*v2.z - v1.z*v2.y,
                v1.z*v2.x - v1.x*v2.z,
                v1.x*v2.y - v1.y*v2.x
            );
        }
        // dot
        friend float operator*(const Vec v1 , const Vec v2){
            return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
        }
        void set(float _x,float _y,float _z){
            x = _x;
            y = _y;
            z = _z;
        }

        void print(){
            cout << x << " " << y << " " << z << endl;
        }
        float clip(Vec minv, Vec maxv){
            float fac[3] = {1,1,1};
            if(0<x) fac[0] = maxv.x/x;
            else if(x<0) fac[0] = minv.x/x;

            if(0<y) fac[1] = maxv.y/y;
            else if(y<0) fac[1] = minv.y/y;

            if(0<z) fac[2] = maxv.z/z;
            else if(z<0) fac[2] = minv.z/z;

            float k = std::min({1.0F, fac[0], fac[1], fac[2]});
            if(!std::isfinite(k)){
                k = 0;
            }

            //cout << k << endl;
            x *= k;
            y *= k;
            z *= k;
            return k;
        }
        float clip(float minv, float maxv){
            float fac[3] = {1,1,1};
            if(0<x) fac[0] = maxv/x;
            else if(x<0) fac[0] = minv/x;

            if(0<y) fac[1] = maxv/y;
            else if(y<0) fac[1] = minv/y;

            if(0<z) fac[2] = maxv/z;
            else if(z<0) fac[2] = minv/z;

            //float k = std::min(1.0F, std::min(fac[0], std::min(fac[1], fac[2])));
            float k = std::min({1.0F, fac[0], fac[1], fac[2]});
            if(!std::isfinite(k)){
                k = 0;
            }

            x *= k;
            y *= k;
            z *= k;
            return k;
        }
        Vec rotate(Quat q){
            Quat q1(x,y,z,0);
            Quat q2 = q * q1 * q.invert();
            return Vec(q2.x,q2.y,q2.z);
        }
        float abssum(){
            return abs(x)+abs(y)+abs(z);
        }
        void plot(Path& r,Path& g,Path& b,const int index,const float t, const float max, bool do_reverse){
            float fac = 1;
            if(do_reverse){
                fac = -1;
            }

            if(max < abs(x)){
                r << Point(t, index*2*X_LIMIT + 0);
            }else{
                r << Point(t, index*2*X_LIMIT + fac*x*X_LIMIT/max);
            }
            if(max < abs(y)){
                g << Point(t, index*2*X_LIMIT + 0);
            }else{
                g << Point(t, index*2*X_LIMIT + fac*y*X_LIMIT/max);
            }
            if(max < abs(z)){
                b << Point(t, index*2*X_LIMIT + 0);
            }else{
                b << Point(t, index*2*X_LIMIT + fac*z*X_LIMIT/max);
            }
        }

};

inline Vec Quat::toV(){
    Vec v(0,0,0);
    if( w < 0){
        w *= -1;
        x *= -1;
        y *= -1;
        z *= -1;
    }
    // avoid non-stable divide
    if( w < -0.999 || 0.999 < w ){

    }else{
        float fac = 2 * acos(w) / sqrt( 1 - pow(w , 2));
        v.x = x * fac;
        v.y = y * fac;
        v.z = z * fac;
    }
    return v;
}

void split(string str, char separator, string out[]){
    int j=0;
    int size = str.size();
    for(int i=0; i<size; i++){
        if(str[i]==separator){
            j++;
        }else{
            out[j].push_back(str[i]);
        }
    }
    return;
}

unsigned char str2bin(string str, bool is_bigendian){
    unsigned char uc = 0x00;
    int size = str.size();
    for(int i=0; i<size; i++){
        if(is_bigendian){
            if(str[i]=='1'){
                uc |= (1<<(7-i));
            }
        }else{
            if(str[i]=='1'){
                uc |= (1<<i);
            }
        }
    }
    return uc;
}

int main(){
    Document doc("fg.svg", Layout(Dimensions(T_LIMIT,X_LIMIT), Layout::TopLeft));

    Color grayColor = Color(AXIS_COLOR,AXIS_COLOR,AXIS_COLOR);

    doc << Rectangle("back1",Point(0, -X_LIMIT), T_LIMIT, 2*X_LIMIT, Fill(Color::White),Stroke());
    doc << Rectangle("back2",Point(0, 2*X_LIMIT-X_LIMIT), T_LIMIT, 2*X_LIMIT, Fill(Color::White),Stroke());
    Path p_w = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Black));
    Path p_k = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Black));
    Path p_b0 = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Blue));
    Path p_g0 = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Green));
    Path p_r0 = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Red));
    Path p_b1 = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Blue));
    Path p_g1 = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Green));
    Path p_r1 = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Red));
    //Path p_c = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Cyan));
    //Path p_m = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Magenta));
    //Path p_y = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Yellow));

    // t axis
    for(float i=0; i<=T_LIMIT; i+=10){
        float width;
        if((int)i%50 == 0){
            if((int)i%100 == 0){
                width = AXIS_LINE_WIDTH3;
            }else{
                width = AXIS_LINE_WIDTH2;
            }
        }else{
            width = AXIS_LINE_WIDTH1;
        }
        doc << Line("l",Point(i,-X_LIMIT),Point(i,3*X_LIMIT),Stroke(width,grayColor));
    }
    /*
    // x axis
    for(float i=-X_LIMIT; i<=X_LIMIT; i+=10){
        float width;
        if((int)i%50 == 0){
            if((int)i%100 == 0){
                width = AXIS_LINE_WIDTH3;
            }else{
                width = AXIS_LINE_WIDTH2;
            }
        }else{
            width = AXIS_LINE_WIDTH1;
        }
        doc << Line("l",Point(0,i),Point(T_LIMIT,i),Stroke(width,grayColor));
    }

    // x axis
    for(float i=X_LIMIT*2-X_LIMIT; i<=X_LIMIT*2+X_LIMIT; i+=10){
        float width;
        if((int)i%50 == 0){
            if((int)i%100 == 0){
                width = AXIS_LINE_WIDTH3;
            }else{
                width = AXIS_LINE_WIDTH2;
            }
        }else{
            width = AXIS_LINE_WIDTH1;
        }
        doc << Line("l",Point(0,i),Point(T_LIMIT,i),Stroke(width,grayColor));
    }
    */

    string tmp;
    vector<Quat> ori;
    vector<Vec> avel;
    vector<Vec> acce;
    vector<Vec> mp;
    vector<int> index;
    vector<unsigned char> flag1;
    vector<unsigned char> flag2;
    vector<float> temp;

    int dat_index = 0;

    // substitue data to vector
    fstream csv(INPUT_FILE_PATH, ios::in);
    //dump  (flashのcsvのheaderは1行)
    getline(csv, tmp);
    while(getline(csv, tmp)){
        string dat_string[8];
        split(tmp, ',', dat_string);
        
        index.push_back(stoi(dat_string[0]));
        flag1.push_back(str2bin(dat_string[1], 1));
        flag2.push_back(str2bin(dat_string[2], 1));

        cout << dat_index << endl;
        cout << tmp << endl;


        string dat_string_tmp[4];
        split(dat_string[3], ' ', dat_string_tmp);
        //for(int i=0; i<4; i++){
        //    cout << dat_string_tmp[i] << endl;
        //    ori[i].push_back(stof(dat_string_tmp[i]));
        //}
        Quat ori_n(stof(dat_string_tmp[0]),stof(dat_string_tmp[1]),stof(dat_string_tmp[2]),stof(dat_string_tmp[3]));
        ori.push_back(ori_n);

        dat_string_tmp[0] = "";
        dat_string_tmp[1] = "";
        dat_string_tmp[2] = "";
        dat_string_tmp[3] = "";
        split(dat_string[4], ' ', dat_string_tmp);
        Vec tmpv(stof(dat_string_tmp[0]),stof(dat_string_tmp[1]),stof(dat_string_tmp[2]));
        avel.push_back(tmpv);

        dat_string_tmp[0] = "";
        dat_string_tmp[1] = "";
        dat_string_tmp[2] = "";
        dat_string_tmp[3] = "";
        split(dat_string[5], ' ', dat_string_tmp);
        tmpv.set(stof(dat_string_tmp[0]),stof(dat_string_tmp[1]),stof(dat_string_tmp[2]));
        acce.push_back(tmpv);

        dat_string_tmp[0] = "";
        dat_string_tmp[1] = "";
        dat_string_tmp[2] = "";
        dat_string_tmp[3] = "";
        split(dat_string[6], ' ', dat_string_tmp);
        tmpv.set(stof(dat_string_tmp[0]),stof(dat_string_tmp[1]),stof(dat_string_tmp[2]));
        mp.push_back(tmpv);

        temp.push_back(stof(dat_string[7]));


        dat_index++;
    }
    csv.close();


    // plot & cal
    int size = index.size();
    float t = 0;
    for(int i=0; i<size; i++){

        Vec oriv = ori[i].toV();
        int x,mode;
        bool doPlot = true;
        mode = ((int)flag1[i]&(0b00000011)) + 1;
        cout << mode << endl;
        switch(mode){
            case 2:
                t += 100.0f;
                break;
            case 3:
                t += 30.0;
                break;
            case 4:
                t += 1000.0f/CONTROL_HZ;
                break;
        }

        if(38200 < t && t < 58902){
            doPlot = true;
        }else{
            doPlot = false;

        }

        if(doPlot){
            //oriv.plot(p_r0,p_g0,p_b0,0,t/10,PI, true);
            acce[i].plot(p_r0,p_g0,p_b0,0,t/10,6.28, true);
            mp[i].plot(p_r1,p_g1,p_b1,1,t/10,M_MAX, true);
            p_k << Point((float)t/10, (float)mode * X_LIMIT);
        }

        /*
        x = mp[i].x*X_LIMIT/M_MAX;
            p_r2 << Point((float)t, 2*X_LIMIT+(float)x);
        x = oriv.x*X_LIMIT/PI;
            p_r1 << Point((float)t, (float)x);
        x = mp[i].y*X_LIMIT/M_MAX;
            p_g2 << Point((float)t, 2*X_LIMIT+(float)x);
        x = oriv.y*X_LIMIT/PI;
            p_g1 << Point((float)t, (float)x);
        x = mp[i].z*X_LIMIT/M_MAX;
            p_b2 << Point((float)t, 2*X_LIMIT+(float)x);
        x = oriv.z*X_LIMIT/PI;
            p_b1 << Point((float)t, (float)x);
        x = index[i]*X_LIMIT/255;
        */
            //p_k << Point((float)t, (float)x);


    }
    
    
    doc << p_w;
    doc << p_k;
    doc << p_b0;
    doc << p_g0;
    doc << p_r0;
    doc << p_b1;
    doc << p_g1;
    doc << p_r1;


    doc.save();
    return 0;
}
/*
NaD:213

*/
