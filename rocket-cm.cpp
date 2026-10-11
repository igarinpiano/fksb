#include <iostream>
#include <fstream>
#include <cmath>
#include <numbers>
#include <simple_svg_1.0.0.hpp>

#define LLIMIT 0.0
#define HLIMIT 100.0
#define HEIGHT 250.0
#define MAIN_LINE_WIDTH 0.1
#define AXIS_LINE_WIDTH1 0.1
#define AXIS_LINE_WIDTH2 0.3
#define AXIS_LINE_WIDTH3 0.5
#define AXIS_COLOR 150

using namespace std;
using namespace svg;

int main(){
    Document doc("graph.svg", Layout(Dimensions(HLIMIT-LLIMIT,HEIGHT), Layout::BottomLeft));
    doc << Rectangle("white",Point(LLIMIT, 0), HLIMIT-LLIMIT, HEIGHT, Fill(Color::White),Stroke());
    Path p0 = Path(Fill(), Stroke(0.1,Color::Black));
    Path p1 = Path(Fill(), Stroke(0.1,Color::Black));
    Path pb = Path(Fill(), Stroke(0.2,Color::Blue));
    Path pg = Path(Fill(), Stroke(0.2,Color::Green));
    Path pr = Path(Fill(), Stroke(0.2,Color::Red));

    Color grayColor = Color(AXIS_COLOR,AXIS_COLOR,AXIS_COLOR);
    // t axis
    for(float i=LLIMIT; i<=HLIMIT; i+=10){
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
        doc << Line("l",Point(i,0),Point(i,HEIGHT),Stroke(width,grayColor));
    }
    // x axis
    for(float i=0; i<=HEIGHT; i+=10){
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
        doc << Line("l",Point(LLIMIT,i),Point(HLIMIT,i),Stroke(width,grayColor));
    }


    const double nose_ofs = 78.4;
    const double fksb_cm = 47.0 + 20.0; //com + parachute
    const double fksb_mass = 275.0;
    const double rocket_cm = 325.0;
    const double rocket_cp = 328.0;
    const double rocket_diameter = 78.0;
    const double rocket_mass = 288.0;


    const double fksb_com = 47.0;
    const double weight_density = 7.8; // [g/cm/cm/cm]

    double x = 0.0;
    while(x<200){
        //double weight_volume = pow(74.0/2 , 2) * 3.1415926535 * fksb_pos/2; //mm mm mm
        //double weight_mass = weight_density * weight_volume /1000; //g

        //double fksb_pos = nose_ofs + x + fksb_cm;
        double fksb_pos = nose_ofs + 50.0 + fksb_cm;
        double weight_pos = 34.5;
        double weight_mass = x; //g
        //double weight_mass = 100; //g

        double c = (rocket_mass * rocket_cm + fksb_mass * fksb_pos + weight_mass * weight_pos) / (fksb_mass+rocket_mass+weight_mass);
        double calib = (rocket_cp - c) / rocket_diameter;

        cout << "ofs : " << fksb_pos << endl;
        cout << "weight_mass : " << weight_mass << endl;
        cout << "com : " << c << " caliber : " << calib << endl;
        cout << endl;

        p0 << Point(x, c);
        p1 << Point(x, calib*100);
        x += 10;
    }


    doc << p0;
    doc << p1;
    doc << pb;
    doc << pg;
    doc << pr;


    doc.save();
    return 0;
}
