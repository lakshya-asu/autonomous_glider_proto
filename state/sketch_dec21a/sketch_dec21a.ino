#include <Geometry.h>
#include <Vector.h>
#include <PID_v1.h>
#define pi 3.14
enum state{live, idle, glide, helix, land, kill}

void setup() {
  // put your setup code here, to run once:

state STATE=idle;
}

void loop() {
  // put your main code here, to run repeatedly:
 
 //idle
if(idle)
select_state()

}


float get2dcorrection(vec1,vec2)
{
  theta1=(atan2(current.y, - current.x));
  theta2=(atan2(ideal.y, - ideal.x));
  theta3=(theta2-theta1);
  
  if (abs(diff) > math.pi)
        {
          if (diff < 0)
            theta3 += 2 * pi;
        else
            theta3 -= 2 * pi;
        }
  return theta3;
}


void select_state()
{
 if(live)
  STATE=idle;
 else if(killswitch)
  STATE=kill;
 else if(theta<=60 && h>10)
  STATE=GLIDE;
 else if(theta>60 && h>10)
  STATE=HELIX;
 else if(h<10)
  STATE=LAND;
 else STATE=idle;
}

void controller(state STATE)
{
  switch(STATE):
    case glide: glide();
      break;
    case 
}
void glide(){
//compute PID for theta
  videal=8.5;
//PID
PID(vcurr, theta, videal)
  leftOutput=rightOutput;
}

void lander(){
  thetaland=thetastall(vcurr);
  PID(thetacurr, theta, thetaland)
}
