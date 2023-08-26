#include <chipmunk/chipmunk.h>

#ifndef A_TYPES_CPP
#define A_TYPES_CPP


struct a_vec3{
    double x;
    double y;
    int z;

};

class a_vec2{

    public:
        double x;
        double y;

        a_vec2(double xin, double yin) { x = xin; y = yin;}
        a_vec2(){x=0;y=0;}
        a_vec2(cpVect in){x=in.x;y=in.y;}

        operator cpVect() const {return {x,y};}

       a_vec2 operator-(const a_vec2 &right){
            return a_vec2(x-right.x,y-right.y);
       }
       a_vec2 operator+(const a_vec2 &right){
            return a_vec2(x+right.x,y+right.y);
       }
       a_vec2 operator/(const double &right){

            return a_vec2(x/right,y/right);
       }

       double angle(const a_vec2 &in){
        return std::atan2(in.y-y,in.x-x);

       }

       double dist(const a_vec2 &in){

            return std::sqrt((in.x-x)*(in.x-x) + (in.y-y)*(in.y-y));
       }
};
#endif
