// inherited ~/tools/graph.cpp

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cmath>
#include <simple_svg_1.0.0.hpp>

#define T_LIMIT 1000
#define X_LIMIT 100
#define MAIN_LINE_WIDTH 0.3
#define AXIS_LINE_WIDTH1 0.1
#define AXIS_LINE_WIDTH2 0.3
#define AXIS_LINE_WIDTH3 0.5
#define AXIS_COLOR 150
#define CONTROL_HZ 30 // honban.cppのCONTROL_HZ (mode4の周期)
#define INPUT_FILE_PATH "../record/tugounoiide-ta/launch_ground.csv"

//index,flag1,flag2,orimp ( Big-Endian )

using namespace std;
using namespace svg;

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
    Document doc("fg.svg", Layout(Dimensions(T_LIMIT,X_LIMIT), Layout::BottomLeft));

    Color grayColor = Color(AXIS_COLOR,AXIS_COLOR,AXIS_COLOR);

    doc << Rectangle("back",Point(0, 0), T_LIMIT, X_LIMIT, Fill(Color::White),Stroke());
    Path p_w = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Black));
    Path p_k = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Black));
    Path p_b = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Blue));
    Path p_g = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Green));
    Path p_r = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Red));

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
        doc << Line("l",Point(i,0),Point(i,X_LIMIT),Stroke(width,grayColor));
    }
    // x axis
    //for(float i=0; i<=X_LIMIT; i+=10){
    //    float width;
    //    if((int)i%50 == 0){
    //        if((int)i%100 == 0){
    //            width = AXIS_LINE_WIDTH3;
    //        }else{
    //            width = AXIS_LINE_WIDTH2;
    //        }
    //    }else{
    //        width = AXIS_LINE_WIDTH1;
    //    }
    //    doc << Line("l",Point(0,i),Point(T_LIMIT,i),Stroke(width,grayColor));
    //}

    vector<float> time;
    string tmp;
    vector<float> ori[4];
    vector<float> avel[3];
    vector<float> acce[3];
    vector<float> mp[3];
    vector<int> index;
    vector<unsigned char> flag1;
    vector<unsigned char> flag2;
    vector<float> temp;

    int dat_index = 0;

    // substitue data to vector
    fstream csv(INPUT_FILE_PATH, ios::in);
    //dump
    getline(csv, tmp);
    getline(csv, tmp);
    while(getline(csv, tmp)){
        string dat_string[8];
        split(tmp, ',', dat_string);
        
        time.push_back(stof(dat_string[0]));
        index.push_back(stoi(dat_string[1]));
        flag1.push_back(str2bin(dat_string[2], 1));
        flag2.push_back(str2bin(dat_string[3], 1));

        string dat_string_tmp[4];
        split(dat_string[4], ' ', dat_string_tmp);
        for(int i=0; i<4; i++){
            ori[i].push_back(stof(dat_string_tmp[i]));
        }
        dat_string_tmp[0] = "";
        dat_string_tmp[1] = "";
        dat_string_tmp[2] = "";
        dat_string_tmp[3] = "";
        split(dat_string[5], ' ', dat_string_tmp);
        for(int i=0; i<3; i++){
            mp[i].push_back(stof(dat_string_tmp[i]));
        }

        dat_index++;
    }
    csv.close();


    // plot & cal
    int size = index.size();
    float t = 0;
    for(int i=1; i<size; i++){
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

        if(doPlot){
            x = index[i-1]*X_LIMIT/255;
            p_k << Point((float)t/10, (float)x);
            x = index[i]*X_LIMIT/255;
            p_k << Point((float)t/10, (float)x);
        }


        //x = mp[0][i]*X_LIMIT/200;
        //p_r << Point((float)t, (float)x);
        //x = mp[1][i]*X_LIMIT/200;
        //p_g << Point((float)t, (float)x);
        //x = mp[2][i]*X_LIMIT/200;
        //p_b << Point((float)t, (float)x);

    }
    
    
    doc << p_w;
    doc << p_k;
    doc << p_b;
    doc << p_g;
    doc << p_r;


    doc.save();
    return 0;
}
/*
NaD:213

*/
