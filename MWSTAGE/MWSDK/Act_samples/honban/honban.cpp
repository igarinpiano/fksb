#include <TWELITE>
#include <NWK_SIMPLE>
#include <BRD_APPTWELITE>
#include <string.h>
#include <cmath>
#include <algorithm>

#define BNO_DEV 0x28
#define BNO_REG_A 0x08 //acceleration register adress(6)
#define BNO_REG_M 0x0E //magnetron register adress(6)
#define BNO_REG_G 0x14 //gyro(differential of pos) register adress(6)
#define BNO_REG_Q 0x20 //quaternion register adress(8byte)
#define BNO_REG_T 0x34 //templature register adress(1)
#define BNO_CALIB_STAT 0x35
#define BNO_OPR_MODE 0x3D
#define BNO_UNIT_SEL 0x3B
#define BNO_RST_PIN 10
#define ESC_PIN0 5
#define ESC_PIN1 19
#define ESC_PIN2 4
#define HOLD 4
#define BTN 8
#define IN 0 //A1Pinは番号0

#define APP_ID 0x1234abcd
#define CHANNEL 13
#define REC_LID 0x01
#define LID 0x00
#define FLASH_MAX 0x07FFFF
#define FLASH_PAGE 256 // page program 1回で書ける範囲
#define DAT_LEN 30
#define PKT_LEN 17
#define ISMAIN false

// constant
constexpr uint16_t M_MIN =  0;
constexpr uint16_t M_NOR =  150;
constexpr uint16_t M_MAX =  300; //3200
constexpr float M_RES =  0.03f; // restoring factor
//constexpr uint16_t SLEEP_DELAY =1000; //ms
//constexpr uint16_t STANBY_DELAY = 100; 
constexpr uint16_t CONTROL_HZ = 30; // controlling frequency Timer0
constexpr float CONTROL_DELAY = 1.0f/CONTROL_HZ; // controlling frequency Timer0
constexpr float GAIN =  0.4f; // gain
constexpr float K_P_GAIN = 0.15f;
constexpr float K_D_GAIN = 0.04f;
//constexpr float K_I_GAIN = 0.0f;
constexpr float K_P = GAIN * -K_P_GAIN / CONTROL_DELAY;
constexpr float K_D = GAIN * -K_D_GAIN / CONTROL_DELAY;
//constexpr float K_I = GAIN * -K_I_GAIN / CONTROL_DELAY;
constexpr float LPF_FAC = 1.0f;
constexpr float LPF_FAC2 = 1.0f - LPF_FAC;
constexpr float MUSHI = 0.00f; //目標値付近で「無」になるため 7°ぐらい
constexpr float MUSHI_AVEL = 0;
// 注意: BNO055の最大加速度は40m/s/s
constexpr float LAUNCH_ACCE = 25; //3gぐらい
constexpr float FREE_ACCE = 6; 
constexpr float EXPAND_ACCE = 25; //parachute展開時 もしかしたら分離の時点でこれぐらいかかるかもな
constexpr float LANDING_ACCE = 30; 

//constexpr float K_I  = GAIN * -0.0f / CONTROL_DELAY;

// paste from ~/fksb/godot_ps


constexpr float Ixx = 747626.041145661;
constexpr float Ixy = 9988.67491157437;
constexpr float Ixz = 9796.290439624201;
constexpr float Iyx = 9988.67491157437;
constexpr float Iyy = 733198.4770532331;
constexpr float Iyz = -8747.89672499152;
constexpr float Izx = 9796.290439624201;
constexpr float Izy = -8747.89672499152;
constexpr float Izz = 133509.29448540072;
constexpr float Wzz = 4713.268;
constexpr float TOR_MAX = 12448130.0f;

constexpr float MMF = -Wzz/CONTROL_DELAY;  // min maxを求めるときにつかう
constexpr float WFAC = CONTROL_DELAY/Wzz * 1.0f; //角加速度出すときtorqueにかけざんする数
// torqueを正にしたときのmpの増減の向き(モーターの取りつけ向き)
constexpr float MDIR_X  = ISMAIN ? -1.0f :  1.0f;
constexpr float MDIR_YZ = ISMAIN ?  1.0f : -1.0f;

constexpr int16_t A_ZERO =   819; // 16384 * 0.05 5% 1000microsecondsに相当
constexpr int16_t A_MIN  =   861; // モーターが実際に回転を始めるduty(1単位で実測)
constexpr int16_t A_MAX  =   1638; // 10%のduty
constexpr int16_t A_RANGE =  777; // A_MAX - A_MIN
constexpr uint16_t DUTY_MAX = 16384; //ref https://mwx.twelite.info/latest1/api-reference/predefined_objs/timers#change_hz


/*
       pwmのピンの対応(LEDつけてしらべた) シルクnumberやえ
timer0 : 10
timer1 : 11
timer2 : 12
timer3 : 13
timer4 : 17
       */
void send(const uint8_t adr, const byte dat[PKT_LEN], uint8_t len);
void printByte(const byte buff[], const uint8_t len);
void printByte(const byte buff);
void printByteBin(const byte buff);
void majiKeisandekiru3();
void torqueRange(const float m, const float dir, float &lo, float &hi);
class Vec;

class I2C{
    private:
        byte adr = 0x00;

    public:
        void setAdr(const byte _adr){
            adr = _adr;
        }
        bool write(byte REG, byte VAL){
            if(auto&& wrt = Wire.get_writer(adr)){
                wrt << REG;
                wrt << VAL;
            }else{
                return false;
            }
            return true;
        }
        bool set(byte REG){
            if(auto&& wrt = Wire.get_writer(adr)){
                wrt << REG;
            }else{
                return false;
            }
            return true;
        }
        bool read(byte val[], uint8_t len){
            if(auto&& rdr = Wire.get_reader(adr, len)){
                for(uint8_t i=0; i<len; i++){
                    rdr >> val[i];
                }
            }else{
                return false;
            }
            return true;
        }
        //bool setread(byte REG, byte val[], uint8_t len){
        //    if(set(REG))
        //        return read(val, len);
        //    return false;
        //}
        bool readByte(byte REG, byte &val){
            if(set(REG)){
                if(auto&& rdr = Wire.get_reader(adr, 1)){
                    rdr >> val;
                }else{
                    return false;
                }
                return true;
            }
            return false;
        }
};
class Flash{
    public:
        uint32_t cAdr = 0x00000000;
        void read(uint32_t adr, byte dat[], uint8_t len){
            if(auto&& trs = SPI.get_rwer()){
                trs << 0x03;
                trs << (byte)(adr >> 16);
                trs << (byte)(adr >> 8) ;
                trs << (byte)adr       ;
                 delay(1);
                for(uint8_t i=0; i<4; i++){
                    trs << 0x00;
                    trs >> dat[0];
                }
                for(uint8_t i=0; i<len; i++){
                    trs << 0x00;
                    trs >> dat[i];
                }
            }
            //cAdr += len;
        }
        bool isWriteEnabled(){
            //return true;
            if(auto&& trs = SPI.get_rwer()){
                byte stat;
                trs << 0x05;
                 delay(1);
                trs << 0x00;
                trs >> stat;
                trs << 0x00;
                trs >> stat;
                if(!(stat&(1<<0))){
                    return true;
                }else{
                    return false;
                }
            }
            // SPIが使えないときに呼び出し側のwhileで永久に止まらないようにする
            return true;
        }
        // 残り容量が足りるか (FLASH_MAXをこえて書くとaddressが0に戻って先頭のデータを壊す)
        bool hasRoom(uint32_t len){
            return cAdr + len <= (uint32_t)FLASH_MAX + 1;
        }
        //WEL  !!  max 256 byte !!
        // PPはpageの境界(256byte)をこえると同じpageの先頭に戻って書いてしまうので，境界で分割する
        void write(uint32_t adr, byte dat[], uint8_t len, bool doWrite){
            if(doWrite){
                uint8_t done = 0;
                while(done < len){
                    uint32_t a = adr + done;
                    uint16_t room = FLASH_PAGE - (uint16_t)(a % FLASH_PAGE);
                    uint8_t n = (len - done < room) ? (len - done) : (uint8_t)room;
                    writePage(a, &dat[done], n);
                    done += n;
                }
                cAdr += len;
            }
            return;
        }
        void writePage(uint32_t adr, byte dat[], uint8_t len){
            while(!isWriteEnabled()){
                delay(1);
            }
            //WREN ( allow twelite to write)
            if(auto&& trs = SPI.get_rwer()){
                trs << 0x06;
            }
            // wrenからすぐにprogramしないとダメ ここでstatus checkするな でもたぶんifは別々にすべき
            //write(PP)
            if(auto&& trs = SPI.get_rwer()){
                trs << 0x02;
                trs << (byte)(adr >> 16);
                trs << (byte)(adr >> 8) ;
                trs << (byte)adr       ;
                for(uint8_t i=0; i<len; i++){
                    trs << dat[i];
                }
            }
        }
        void eraseAll(){
            //RDSR status check
            while(!isWriteEnabled()){
                //これするときはよゆうあるから10ms
                delay(10);
            }
            //WREN ( allow twelite to write)
            if(auto&& trs = SPI.get_rwer()){
                trs << 0x06;
            }
            //CER 
            if(auto&& trs = SPI.get_rwer()){
                trs << 0xC7;
            }
            //delay(2000);
            while(!isWriteEnabled()){
                delay(200);
            }
            cAdr = 0;
            return;
        }
        void print(uint32_t adr, uint16_t len){
            if(auto&& trs = SPI.get_rwer()){
                byte dump;
                trs << 0x03;
                trs << (byte)(adr >> 16);
                trs << (byte)(adr >> 8) ;
                trs << (byte)adr       ;
                delay(1);
                for(uint8_t i=0; i<4; i++){
                    trs << 0x00;
                    trs >> dump;
                }
                for(uint16_t i=0; i<len+4; i++){
                    trs << 0x00; //dummy byte( to generate clock)
                    trs >> dump;
                    Serial.print(dump, HEX);
                    Serial.print(" ");
                }
                Serial.println("");
            }
        }
};
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
        void setToByte(byte buffer[]){
            const float scale = (1 << 14);
            int16_t x1, y1, z1, w1;
            w1 = (int16_t)(w * scale);
            x1 = (int16_t)(x * scale);
            y1 = (int16_t)(y * scale);
            z1 = (int16_t)(z * scale);
            buffer[0] = (uint8_t)((w1 >> 0) & 0xFF);
            buffer[1] = (uint8_t)((w1 >> 8) & 0xFF);
            buffer[2] = (uint8_t)((x1 >> 0) & 0xFF);
            buffer[3] = (uint8_t)((x1 >> 8) & 0xFF);
            buffer[4] = (uint8_t)((y1 >> 0) & 0xFF);
            buffer[5] = (uint8_t)((y1 >> 8) & 0xFF);
            buffer[6] = (uint8_t)((z1 >> 0) & 0xFF);
            buffer[7] = (uint8_t)((z1 >> 8) & 0xFF);
        }
        void setFromByte(const byte buffer[]){
            const float scale = (1.0 / (1 << 14));
            int16_t x1, y1, z1, w1;
            x1 = y1 = z1 = w1 = 0;
            w1 = (int16_t)((((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]));
            x1 = (int16_t)((((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]));
            y1 = (int16_t)((((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]));
            z1 = (int16_t)((((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]));
            w = w1 *  scale;
            x = x1 *  scale;
            y = y1 *  scale;
            z = z1 *  scale;
        }
        Vec toV();
        void print(){
            Serial << w << " "  << x << " " << y << " " << z << mwx::crlf;
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

        void setToByte(byte buffer[], const uint8_t k){
            float scale = 1.0;
            switch(k){
                case 0:
                    scale = 900.0; // rad/s
                    break;
                case 1:
                    scale = 16.0; // uT
                    break;
                case 2:
                    scale = 100.0; // m/s^2
                    break;
                default:
                    break;
            }
            int16_t x1, y1, z1;
            x1 = (int16_t)(x * scale);
            y1 = (int16_t)(y * scale);
            z1 = (int16_t)(z * scale);
            buffer[0] = (uint8_t)((x1 >> 0) & 0xFF);
            buffer[1] = (uint8_t)((x1 >> 8) & 0xFF);
            buffer[2] = (uint8_t)((y1 >> 0) & 0xFF);
            buffer[3] = (uint8_t)((y1 >> 8) & 0xFF);
            buffer[4] = (uint8_t)((z1 >> 0) & 0xFF);
            buffer[5] = (uint8_t)((z1 >> 8) & 0xFF);
        }
        void setFromByte(const byte buffer[], const uint8_t k){ //0: gyro, 1:mag, 2:acc
            float scale = 1.0;;
            switch(k){
                case 0:
                    scale = 900.0; // rad/s
                    break;
                case 1:
                    scale = 16.0; // uT
                    break;
                case 2:
                    scale = 100.0; // m/s^2
                    break;
                default:
                    break;
            }

            int16_t x1, y1, z1;
            x1 = y1 = z1 = 0;
            x1 = (int16_t)((((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]));
            y1 = (int16_t)((((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]));
            z1 = (int16_t)((((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]));
            x = x1 / scale;
            y = y1 / scale;
            z = z1 / scale;
        }
        void print(){
            Serial << x << " " << y << " " << z << mwx::crlf;
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

            //Serial << k << crlf;
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
            return std::fabs(x)+std::fabs(y)+std::fabs(z);
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
    // variable
I2C bno;
Flash f;
Quat ori(1,0,0,0); // orientation
Quat tar(1,0,0,0); // target orientation 
Vec avel; // angular velocity = gyro
Vec prev; // angular velocity = gyro
Vec acce;
Vec error_i(0,0,0);

byte flag1;
byte flag2;
byte buff[8];
int16_t mp[3];
float mpf[3] = {0,0,0}; // mpの小数部まで持っておく(int16_tに直接足すと切り捨てで小さい変化が消える)
byte dat[DAT_LEN];
uint8_t mode = 1;
uint8_t index = 0;
uint8_t temp; // temperature

uint32_t tick = 0;


// set rotation speed
void setrs(const uint8_t num, const int16_t val){ //num0,1,2 |-> Timer4,1,3   0 < val < 1024(1<<10)  0xFF00で切れない最小のpwm，0xFFFFで0電圧
    // 0.05 ~ 0.10 ( 50Hz )
    //cppのcompalierは生のbyte列をintと解釈するらしいからちゃんとcastする
    if(val == (int16_t)0xFF00 || val == (int16_t)0xFFFF || 0 <= val && val <= (1<<10)){
        float value;
        if(val == (int16_t)0xFF00){
            value = A_ZERO;
        }else if(val==(int16_t)0xFFFF){
            value = 0;
        }else{
            value = (float)val*(float)(A_RANGE) / (float)(1<<10)  + (float)A_MIN;
        }
        switch(num){
            case 0:
                Timer4.change_duty((uint16_t)value, DUTY_MAX);
                break;
            case 1:
                Timer1.change_duty((uint16_t)value, DUTY_MAX);
                break;
            case 2:
                Timer3.change_duty((uint16_t)value, DUTY_MAX);
                break;
        }
    }
    return;
}
void setmode(const uint8_t modei,const bool is_force=false){ //モード切りかえ時にcalll modeについては一番下をみろ
    if(modei == 0){
        setrs(0, 0xFF00);
        setrs(1, 0xFF00);
        setrs(2, 0xFF00);
        //送信の必要性がないのでflagはなし
       mode = 0;
       f.cAdr = 0;
       memset(dat, 0, DAT_LEN);
       Serial << flush;
    }
    if(modei == 1){
        setrs(0, 0xFF00);
        setrs(1, 0xFF00);
        setrs(2, 0xFF00);

        mode = 1;
        flag1 &= ~(1<<0);
        flag1 &= ~(1<<1);
        flag1 &= ~(1<<2);
        flag1 &= ~(1<<3);
        flag1 &= ~(1<<4);
        flag1 &= ~(1<<7);
        flag2 &= ~(1<<0);
        flag2 &= ~(1<<1);
        if(is_force) flag1 |= (1<<4); // force stopped
        printByteBin(flag1);
    }
    if(modei == 2){
        f.eraseAll();
        index = 0;
        setrs(0, 0xFF00);
        setrs(1, 0xFF00);
        setrs(2, 0xFF00);

        mode = 2;
        flag1 |=  (1<<0);
        flag1 &= ~(1<<1);
    }
    if(modei == 3){
        Serial << "hoge" << crlf;

        mode = 3;
        flag1 &= ~(1<<0);
        flag1 |=  (1<<1);
    }
    if(modei == 4){

        mode = 4;
        flag1 |=  (1<<0);
        flag1 |=  (1<<1);
        if(is_force){
            flag1 |=  (1<<3); // force started
            flag1 &= ~(1<<4);
        }

        Timer0.begin(CONTROL_HZ,true,false);
        Timer0.change_hz(CONTROL_HZ,0);
    }else{
        Timer0.end();
    }
}
void get_data(){ //get and write data

    // -------------------- get data --

    // calibration status
    bno.set(BNO_CALIB_STAT);
    bno.read(buff, 1);
    //if(buff[0]&(1<<6)){
    //    Serial << "6" << crlf;
    //}
    //if(buff[0]&(1<<7)){
    //    Serial << "7" << crlf;
    //}
    //Serial.println(buff[0], BIN);

    // memory full
    // 消去していない領域には上書きできない(先頭に戻って書くと既存のデータが壊れる)ので，いっぱいになったら書き込みをやめる
    bool doWrite = f.hasRoom(DAT_LEN);
    if(!doWrite && (flag1&(1<<6))){
        flag1 &= ~(1<<6);
        Serial << "------------------ memory full -----------------------" << crlf;
    }

        // write inedex flags
    buff[0] = index;
    f.write(f.cAdr, buff, 1, doWrite);
    buff[0] = flag1;
    f.write(f.cAdr, buff, 1, doWrite);
    buff[0] = flag2;
    f.write(f.cAdr, buff, 1, doWrite);


        // acceralation
    bno.set(BNO_REG_A);
    bno.read(buff, 6);
    acce.setFromByte(buff,2);
    acce.x = -acce.x;
    acce.z = -acce.z;
    acce.setToByte(buff,2);
    f.write(f.cAdr, buff, 6, doWrite);

        // write mps
    memcpy(&buff[0], &mp[0], 2);
    memcpy(&buff[2], &mp[1], 2);
    memcpy(&buff[4], &mp[2], 2);
    f.write(f.cAdr, buff, 6, doWrite);

    //    // magnetic IMU mode だと取得不可
    //bno.set(BNO_REG_M);
    //bno.read(buff, 6);
    //acce.setFromByte(buff,2);
    //f.write(f.cAdr, buff, 6, 1);

        // temperature
    bno.set(BNO_REG_T);
    bno.read(buff, 1);
    temp = buff[0];
    f.write(f.cAdr, buff, 1, doWrite);

        // gyro
    bno.set(BNO_REG_G);
    bno.read(buff, 6);
    avel.setFromByte(buff,0);
    avel.x = -avel.x;
    avel.z = -avel.z;
    avel.setToByte(buff,0);
    f.write(f.cAdr, buff, 6, doWrite);

        // quaternion  データ送信にbuffを使うから移動しないでほしいじょ
    bno.set(BNO_REG_Q);
    bno.read(buff, 8);
    ori.setFromByte(buff);
    if(flag2&(1<<0)){
        tar = ori;
        flag2 &= ~(1<<0);
        flag1 |=  (1<<7); // target ori ok
    }
    ori = tar.invert() * ori;
    ori.x = -ori.x;
    ori.z = -ori.z;
    ori.setToByte(buff);
    f.write(f.cAdr, buff, 8, doWrite);

}
void makePacket(byte dat[], uint8_t &index, byte q[8], int16_t mp[3], byte &flag1, byte &flag2){
    dat[0] = index;
    dat[1] = flag1;
    dat[2] = flag2;
    for(uint8_t j=3; j<3+8; j++){
        dat[j] = q[j-3];
    }//3 ~ 10

    //11 12
    //13 14
    //15 16
    memcpy(&dat[11], &mp[0], 2);
    memcpy(&dat[13], &mp[1], 2);
    memcpy(&dat[15], &mp[2], 2);
}

// benri
void printByte(const byte buff[], const uint8_t len){
    for(uint8_t i=0; i<len; i++){
        Serial.print(buff[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
}
void printByte(const byte buff){
    Serial.println(buff, HEX);
}
void printByteBin(const byte buff){
    for(uint8_t i=0; i<8; i++){
        if(buff&(1<<i)){
            Serial << "1";
        }else{
            Serial << "0";
        }
    }
    Serial << crlf;
}

/*** the setup procedure (called on boot) */
void setup() {
    // for controlling time interruption
    Timer0.setup();

    Timer1.setup();
    Timer1.begin(50,false,true);
    Timer1.change_duty(1);
    Timer1.change_hz(50,0);
    Timer3.setup();
    Timer3.begin(50,false,true);
    Timer3.change_duty(1);
    Timer3.change_hz(50,0);
    Timer4.setup();
    Timer4.begin(50,false,true);
    Timer4.change_duty(1);
    Timer4.change_hz(50,0);
    pinMode(BNO_RST_PIN, OUTPUT);
    digitalWrite(BNO_RST_PIN, LOW);
     delay(10);
    digitalWrite(BNO_RST_PIN, HIGH);

    Serial << "--- program start ---" << crlf;

    index = 0;
    flag1 = 0b00000000;
    flag2 = 0b00000000;
    // mp[0] = ----
    

        /****** setup spi   ******/


    SPI.begin(0 // DIO19をチップセレクトとして使用
              , { 5000000UL // クロック周波数 
            , SPI_CONF::MSBFIRST
            , SPI_CONF::SPI_MODE0
    });
     delay(1000);

    if(auto&& trs = SPI.get_rwer()){

            // このへん要検討 -----------------------------------------------------------||

        byte deb;
        trs << 0x9F;
         delay(1);
        trs << 0x00;
        trs >> deb;
        trs << 0x00;
        trs >> deb;
        if(deb==0x9D){
            Serial << "flash memory : ok" << mwx::crlf;
            flag1 |= (1<<6);
        }else{
            Serial << "flash memory : failed!" << mwx::crlf;
        }
    }

        /****** setup i2c   ******/

    Wire.begin();  
     delay(500);

    // bno setup ( https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
    bno.setAdr(BNO_DEV);
    // set units
    // UNIT_SELはCONFIGMODE(リセット直後)でしか書きかえられないので，OPR_MODEを変える前に1回でまとめて書く
    // bit0 acc:m/s^2  bit1 gyro:rad/s  bit4 temp:℃  bit7 Android(default)
    bno.write(BNO_UNIT_SEL, (1<<7) | (0<<0) | (1<<1) | (0<<4));
     delay(10);
    if(bno.write(BNO_OPR_MODE, 0x08)){
         delay(20);

        Serial << "bno : ok" << mwx::crlf;
        flag1 |= (1<<5);
    }else{
        Serial << "bno : failed!" << mwx::crlf;
    }

        /******  twelite setup ******/

    the_twelite
        << TWENET::appid(APP_ID)    // set application ID (identify network group)
        << TWENET::channel(CHANNEL) // set channel (pysical channel)
        << TWENET::rx_when_idle();  // open receive circuit (if not set, it can't listen packts from others)
    auto&& nwksmpl = the_twelite.network.use<NWK_SIMPLE>();
    nwksmpl << NWK_SIMPLE::logical_id(LID) // set Logical ID. (0xFE means a child device with no ID)
            << NWK_SIMPLE::repeat_max(3);   // can repeat a packet up to this times. (being kind of a router)
    the_twelite.begin(); // start twelite!
     delay(100);


    setrs(0, 0xFF00);
    setrs(1, 0xFF00);
    setrs(2, 0xFF00);
    tar.w = 1.0f;
    tar.x = 0.0f;
    tar.y = 0.0f;
    tar.z = 0.0f;
    //////////////////////////    これより下debug    //////////////////////////

    printByteBin(flag1);

    tar.print();
    // loop stop
    //flag2 |= (1<<7); // debug stop
    //flag1 |= (1<<0); // writing
    //flag1 |= (1<<1); // launched
    // test of varoius fucntions


    /*
     delay(500);

        f.cAdr = 0x000000;
        for(uint8_t i=0; i<8; i++){
            buff[i] = i;
            printByte(buff[i]);
        }
        f.write(f.cAdr, buff, 8, true);

     delay(500);

    f.eraseAll();    
        f.cAdr = 0x000000;
        buff[0] = 0x55;
        buff[1] = 0x56;
        buff[2] = 0x57;
        f.write(f.cAdr, buff, 3, true);
     delay(500);


        f.cAdr = 0x000000;
        f.read(f.cAdr, dat, 10);
        printByte(dat, 10);
        */
}

/*** the loop procedure (called every event) */
void loop() {
    // do run? 
    if(flag2&(1<<7)){


    }else{
        switch(mode){
            case 0:
                while(Serial.available())  {
                    int c = Serial.read();
                    switch(c) {
                        case 'r': //read 1section
                            f.read(f.cAdr, dat, DAT_LEN);
                            Serial << dat;
                            //printByte(dat, DAT_LEN);
                            f.cAdr += DAT_LEN;
                            break;
                        case 's': //prepare
                            f.cAdr = 0;
                            memset(dat, 0, DAT_LEN);
                            Serial << flush;
                            break;
                    }
                }
                break;
            case 1:
                 delay(500);

                // bno status
                bno.set(BNO_CALIB_STAT);
                bno.read(buff, 1);
                if(buff[0]&(1<<6)){
                    Serial << "6" << crlf;
                }
                if(buff[0]&(1<<7)){
                    Serial << "7" << crlf;
                }

                // acceralation
                bno.set(BNO_REG_A);
                bno.read(buff, 6);
                //acce.setFromByte(buff,2);

                if(index%2 == 0){
                    dat[0] = index;
                    dat[1] = flag1;
                    dat[2] = flag2;
                    memcpy(&dat[3], &buff, 6);

                    send(REC_LID, dat, 9);
                }

                while(Serial.available())  {
                    int c = Serial.read();

                    switch(c) {
                        case 's': //prepare for extracting data
                            setmode(0);
                            break;
                    }
                }
                index++;
                break;
            case 2:
                //printByteBin(flag1);
                // is launched?
                delay(100);

                get_data(); 
                //launching judgment
                if(LAUNCH_ACCE < acce.z ){
                    //Serial << acce.z << crlf;
                    setmode(3);
                }
                //10回に1回
                if(index%10 == 0){
                    ori.setToByte(buff);
                    makePacket(dat, index, buff, mp, flag1, flag2);
                    send(REC_LID, dat, PKT_LEN);
                }

                index++;
                break;
            case 3:
                 delay(30);

                get_data(); 

                // M_NOR まで上げる
                for(uint8_t i=0; i<3; i++){
                    mpf[i] += (M_NOR-mpf[i]) *M_RES;
                    mp[i] = (int16_t)(mpf[i] + 0.5f);
                }
                for(uint8_t i=0; i<3; i++){
                    if(M_MIN <= mp[i] && mp[i] <= M_MAX){
                        setrs(i, mp[i]);
                    }else{
                        setrs(i, 0xFF00);
                    }
                }

                // free fall
                if(!(flag1&(1<<2)) && acce.z < FREE_ACCE && prev.z < FREE_ACCE){
                    tick = millis();
                    flag1 |= (1<<2);
                }

                if(flag1&(1<<2) && !(flag2&(1<<1)) && EXPAND_ACCE < std::fabs(acce.z)){
                    if(millis() - tick >= 500){
                        tick = millis();
                        flag2 |= (1<<1);
                    }
                }
                if(flag1&(1<<2) && flag2&(1<<1)){
                    if(millis() - tick >= 1000){
                        setmode(4);
                    }
                }


                //10回に1回
                if(index%10 == 0){
                    ori.setToByte(buff);
                    makePacket(dat, index, buff, mp, flag1, flag2);
                    send(REC_LID, dat, PKT_LEN);
                }
                // これ注意!!!!!
                prev = acce;
                index++;
                break;
            case 4:
                if(Timer0.available()){

                    get_data(); 

                    majiKeisandekiru3();

                    for(uint8_t i=0; i<3; i++){
                        if(M_MIN <= mp[i] && mp[i] <= M_MAX){
                            setrs(i, mp[i]);
                        }else{
                            setrs(i, 0xFF00);
                        }
                    }
                    //if(LANDING_ACCE < abs(acce.z)){
                    //    setmode(1);
                    //}
                    //10回に1回
                    if(index%10 == 0){
                        ori.setToByte(buff);
                        makePacket(dat, index, buff, mp, flag1, flag2);
                        send(REC_LID, dat, PKT_LEN);
                    }
                    index++;
                }
                break;
            default:
                delay(100);
                break;
        }
    }
}

// mpがm のとき，mpを0 ~ M_MAXの範囲におさめたまま加えられるtorqueの範囲[lo, hi]
// dir : torqueを正にしたときにmpが増えるなら1，減るなら-1
void torqueRange(const float m, const float dir, float &lo, float &hi){
    float down = m * MMF;           // mpを0まで下げるtorque(dir=1のとき負)
    float up   = (m - M_MAX) * MMF; // mpをM_MAXまで上げるtorque(dir=1のとき正)
    if(0 < dir){
        lo = down;
        hi = up;
    }else{
        lo = -up;
        hi = -down;
    }
}
void majiKeisandekiru3(){

    //loopの中でinstantiateしてもほぼ時間食わないらしいから安心しで作ってくれ

    // error
    Vec oriv = ori.toV();
    Vec avel_ap = avel * LPF_FAC + prev * LPF_FAC2;
    //Vec avel_ap = avel;

    if(std::fabs(oriv.x) < MUSHI){ 
        oriv.x = 0;
    }
    if(std::fabs(oriv.y) < MUSHI){ 
        oriv.y = 0;
    }
    if(std::fabs(oriv.z) < MUSHI){ 
        oriv.z = 0;
    }
    if(std::fabs(avel_ap.x) < MUSHI_AVEL){ 
        avel_ap.x = 0;
    }
    if(std::fabs(avel_ap.y) < MUSHI_AVEL){ 
        avel_ap.y = 0;
    }
    if(std::fabs(avel_ap.z) < MUSHI_AVEL){ 
        avel_ap.z = 0;
    }
    error_i += oriv;
    //float tmp = 2.0f * -oriv.x * M_MAX / 3.14f;
    //if(M_MIN<= tmp && tmp < M_MAX){
    //    mp[0] = tmp;
    //}else{
    //    mp[0] = M_MIN;
    //}

    
    Vec deltaV = avel_ap * K_D;
        deltaV += oriv * K_P;
        //deltaV += error_i * K_I;

    Serial << "deltaV" << crlf;
    deltaV.print();

    Vec torque;
    //torque.x = B_PMOI[0]*deltaV.x + (B_PMOI[2] - B_PMOI[1])*avel_ap.z*avel_ap.y * K_D;
    //torque.y = B_PMOI[1]*deltaV.y + (B_PMOI[0] - B_PMOI[2])*avel_ap.x*avel_ap.z * K_D;
    //torque.z = B_PMOI[2]*deltaV.z + (B_PMOI[1] - B_PMOI[0])*avel_ap.y*avel_ap.x * K_D;

    //torque.x = B_PMOI[0]*deltaV.x;
    //torque.y = B_PMOI[1]*deltaV.y;
    //torque.z = B_PMOI[2]*deltaV.z;


    //deltaV.x = 0;
    //deltaV.y = 0;
    //deltaV.z = 0;


    torque.x = Ixx * deltaV.x + Ixy * deltaV.y + Ixz * deltaV.z;
    torque.y = Iyx * deltaV.x + Iyy * deltaV.y + Iyz * deltaV.z;
    torque.z = Izx * deltaV.x + Izy * deltaV.y + Izz * deltaV.z;

    Serial << "torque" << crlf;
    torque.print();


    // torqueの向きとmpの増減の向きは軸ごとに違う(MDIR)ので，それに合わせて上限下限を決める
    Vec min; 
    Vec max; 
    torqueRange(mpf[0], MDIR_X,  min.x, max.x);
    torqueRange(mpf[1], MDIR_YZ, min.y, max.y);
    torqueRange(mpf[2], MDIR_YZ, min.z, max.z);

    torque.clip(min,max);
    torque.clip(-TOR_MAX, TOR_MAX);


    mpf[0] += MDIR_X  * torque.x*WFAC + (M_NOR-mpf[0]) *M_RES;
    mpf[1] += MDIR_YZ * torque.y*WFAC + (M_NOR-mpf[1]) *M_RES;
    mpf[2] += MDIR_YZ * torque.z*WFAC + (M_NOR-mpf[2]) *M_RES;

    for(uint8_t i=0; i<3; i++){
        if(mpf[i] <= 1){
            mpf[i] = 1;
        }
        if(mpf[i] >= M_MAX){
            mpf[i] = M_MAX;
        }
        mp[i] = (int16_t)(mpf[i] + 0.5f);
    }
    prev = avel;
}
void send(const uint8_t adr, const byte dat[], uint8_t len) {
    if (auto&& pkt = the_twelite.network.use<NWK_SIMPLE>().prepare_tx_packet()) {
        pkt << tx_addr(adr)  // 0..0xFF (LID 0:parent, FE:child w/ no id, FF:LID broad cast), 0x8XXXXXXX (long address)
            << tx_retry(0x2) // set retry (0x3 send four times in total)
            << tx_packet_delay(0,5,20); // send packet w/ delay (send first packet with randomized delay from 100 to 200ms, repeat every 20ms)
        pack_bytes(pkt.get_payload() // set payload data objects.
            , make_pair(dat, (int)len) // string should be paired with length explicitly.
        );
        pkt.transmit();
    }
}
void on_rx_packet(packet_rx& rx, bool_t &handled) {
    //expand_bytes(rx.get_payload().begin(), rx.get_payload().end()
    //            , com       // 4bytes of msg
    //);
    auto dat = rx.get_payload();
    if(dat.size() == 0){
        handled = true;
        return;
    }
    char com = dat[0];

    Serial << com << mwx::crlf;

    // correspond to keyboad
    // ここだけ書きかえればよい
    switch(com){
        case '1':
            setmode(1,true);
            break;
        case '2':
            setmode(2,true);
            break;
        case '3':
            setmode(3,true);
            break;
        case '4':
            setmode(4,true);
            break;
        case '5':
            flag2 |= (1<<0);
            break;

            /*
        case 'i':
            K_P_GAIN += 0.1;
            K_P  = GAIN * -K_P_GAIN / CONTROL_DELAY;
            Serial << "K_P_GAIN : " << K_P_GAIN << crlf;
            break;
        case 'k':
            K_P_GAIN -= 0.1;
            K_P  = GAIN * -K_P_GAIN / CONTROL_DELAY;
            Serial << "K_P_GAIN : " << K_P_GAIN << crlf;
            break;
        case 'l':
            K_D_GAIN += 0.05;
            K_D  = GAIN * -K_D_GAIN / CONTROL_DELAY;
            Serial << "K_D_GAIN : " << K_D_GAIN << crlf;
            break;
        case 'j':
            K_D_GAIN -= 0.05;
            K_D  = GAIN * -K_D_GAIN / CONTROL_DELAY;
            Serial  << "K_D_GAIN : "<< K_D_GAIN << crlf;
            break;
        case 't':
            GAIN += 0.1;
            K_P  = GAIN * -K_P_GAIN / CONTROL_DELAY;
            K_D  = GAIN * -K_D_GAIN / CONTROL_DELAY;
            Serial << "GAIN : " << GAIN << crlf;
            break;
        case 'g':
            
            GAIN -= 0.1;
            K_P  = GAIN * -K_P_GAIN / CONTROL_DELAY;
            K_D  = GAIN * -K_D_GAIN / CONTROL_DELAY;
            Serial  << "GAIN : "<< GAIN << crlf;
            break;
        case 'y':
            K_I_GAIN += 0.05;
            K_I  = GAIN * -K_I_GAIN / CONTROL_DELAY;
            Serial << "K_I_GAIN : " << K_I_GAIN << crlf;
            break;
        case 'h':
            K_I_GAIN -= 0.05;
            K_I  = GAIN * -K_I_GAIN / CONTROL_DELAY;
            Serial << "K_I_GAIN : " << K_I_GAIN << crlf;
            break;
            */

        // debug command
        //case 'z':
        //    flag2 |=  (1<<7);
        //    break;
        //case 'x':
        //    flag2 &= ~(1<<7);
        //    break;
    }
    handled = true; //処理完了とする
}



/* bit flags
    
    bit flags
    flag1(new)
        0 : mode
        1 : mode
        2 : state 1=expanded
        3 : force started 1=started
        4 : force stopped 1=stopped
        5 : is bno ok?
        6 : is flash ok?
        7 : is target ori ok?
    flag2(new)
        0 : used for target setting
        1 : used for separated judgement
        7 : debug

    flag1
        0 : do write? 1=write
        1 : is launched?
        2 : do control?
        3 : 
        4 : 
        5 : 
        6 : 
        7 : is stopping running? 1=stopping
    flag2   
        0 : is bno connection ok?
        1 : is bno calibrated?
        2 : is flash connection ok?
        3 : is separated?
        4 : is landing?
        5 : 
        6 : 
        7 : 

        operating modes ( 0x3D ) 0000<3:0>

          | accel | mag | gyro | relativeOri | absOri
    ------------------------------------------------------------
        0 |       |     |      |             |      
    ------------------------------------------------------------
        1 |   X   |     |      |             |      
    ------------------------------------------------------------
        2 |       |  X  |      |             |      
    ------------------------------------------------------------
        3 |       |     |  X   |             |      
    ------------------------------------------------------------
        4 |   X   |  X  |      |             |      
    ------------------------------------------------------------
        5 |   X   |     |  X   |             |      
    ------------------------------------------------------------
        6 |       |  X  |  X   |             |      
    ------------------------------------------------------------
        7 |   X   |  X  |  X   |             |      
    ------------------------------------------------------------
 >>     8 |   X   |     |  X   |      X      |                    <<
    ------------------------------------------------------------
        9 |   X   |  X  |      |             |   X  
    ------------------------------------------------------------
        a |   X   |  X  |      |      X      |      
    ------------------------------------------------------------
        b |   X   |  X  |  X   |             |   X  
    ------------------------------------------------------------
        c |   X   |  X  |  X   |             |   X  

*/
/* flash drive byte costruction



4*1024*1024/8bit/(30fps*5min*60sec) = 58.3 byte/frame

     0    index
     1    flag1
     2    flag2
     3-8  acce
     9-14 mp
    15    temp
    16-21 avel
    22-29 ori

     
     
     
     
     

*/
/* about controlling

   制御概論

    0.機体の慣性モメント(MOI)(これはFreeCAD+Pythonで，huygens steinerの定理から，回転カンまわりのMOIを求めておく)から主慣性モーメント(primary MOI)を計算(固有値分解だけど，ちょっと詳しいことは理解していない．MathNet.Numerics.LinearAlgebra.Factorizationでもつかってくれ)
      同じようにFreeCAD+Pythonで，motor+hweel(RW)の慣性モーメントを求めておく(こっちは非対角成分ゼロになるからeigen decompositionしなくていい)
      モーターにKtもしくはKvという値が記載されているはず．されていて助かった．Kt ∝ Kv^-1 で，Kvをrad/s/VになおせばそのままKtが出る
      このKtに想定最大電流をかけ算すると最大トルクが出る．このトルクがけっこう大事
      モーターの最大回転数なども実測でメモしておく

      1 – 10をループ
     1.姿勢q・角速度aを取得
     2.目標姿勢との差q'を取得 絶対値が回転量となるベクトルに変換q''
     3.それぞれ(q'', a)に定数Kp, Kd(PID Controllerにおいてはgain と呼ばれる)をかけ算 それをぜんぶ足したベクトルΔv
     4.Δvをeular's equation of motion に代入し，必要トルクτを計算  Eular'sにωがある項は，モーターが回ってると効いてくる(gyro effectと呼ばれる)からそれも打ちけす(このために，RWの回転軸と直交方向のMOIが必要) この計算はたぶん紙あったほうがいい
     5.各次元(x,y,z)で，最大トルクが無限大としたときに，加えられる最大のトルク(モーターの回転数が0< <maxで制限されることに起因するもの)を正と負の側でそれぞれ求めておく
     6.↑で求めた最大トルクを"ひとつたりとも"越えることがないようにτベクトルをスケーリング(定数倍)する
     7.モーターの方の最大トルクも越えないようにスケーリング
     8.τ = Ida/dt の式をつかって，da/dtを求めるここにΔtをかければどれくらい回転数を変化させればいいかわかる
     9.normalの回転数に収束するように，normal回転数との差に0.1ぐらいをかけたやつを回転数に足しつづける
    10.モーターに求めた信号を印加



*/
/* about mode
number        name        mode bit    clock   packet  doWrite?  doControl? description
0      : 読み出しモード   ----          ---   ---     false     false      PCからデータを読み出すときのモード
1      : スリープモード   0b00         1fps   1pps    false     false      動作の正常性を確認するためのmode
2      : フライトモード   0b01        10fps   1pps    true      false      打ち上げ待機モード 地上局からやる この状態でZ軸向き加速度を検知するとmode4に移行
3      : 飛行中           0b10        1000/30fps   3pps    true      false      飛行中のフラグ
4      : 制御中           0b11        30fps   3pps    true      true       制御中 (CONTROL_HZ)

*/
/* other memo

   座標系は，fksB本体基板に描画されているものを信頼して，z-axisは重力と逆方向
   Eularの運動方程式のgyro項ないほうがいいかも…

    とりあえずうまくいくgain
constexpr float GAIN =  0.10f; // gain
constexpr float K_P_GAIN = 0.60f;
constexpr float K_D_GAIN = 0.15f;

    こっちのほうがうまくいきゾ
constexpr float GAIN =  0.25f; // gain
constexpr float K_P_GAIN = 0.50f;
constexpr float K_D_GAIN = 0.15f;

    10秒ほどで安定
constexpr float GAIN =  0.4f; // gain
constexpr float K_P_GAIN = 0.40f;
constexpr float K_D_GAIN = 0.40f;

    mode3で，prev = acce;してるから注意

*/
/* To-Do




   やること
   //- gyroとかの座標系の角煮おねがいます
   - flagとコマンドを充実させでくだしあ
   //- Timer0でtimer interruptを実装
   - flag2 is no longer usedなので，ちかい打ちに削除尾根ギアします
   //- clip()と，godotからcopyする予定の定数ががまだ未実装
   //- gain調整まだ完了してない
   - あとはlaunch/separate判定とか
   - 解析framework制作もー


*/

