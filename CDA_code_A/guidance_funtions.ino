/*----------------------------------------------------------------
------------------------guiding funtions--------------------------
----------------------------------------------------------------*/

void distancecalc(){
 float flat1=flat;     // flat1 = our current latitude. flat is from the gps data. 
 float flon1=flon;  // flon1 = our current longitude. flon is from the fps data.
float dist_calc=0;
float dist_calc2=0;
float diflat=0;
float diflon=0;
x2lat=      ;  //enter a latitude point here   this is going to be your waypoint
x2lon=      ;  // enter a longitude point here  this is going to be your waypoint
//------------------------------------------ distance formula below. Calculates distance from current location to waypoint
diflat=radians(x2lat-flat1);  //notice it must be done in radians
flat1=radians(flat1);    //convert current latitude to radians
x2lat=radians(x2lat);  //convert waypoint latitude to radians
diflon=radians((x2lon)-(flon1));   //subtract and convert longitudes to radians
dist_calc = (sin(diflat/2.0)sin(diflat/2.0));
dist_calc2= cos(flat1);
dist_calc2=cos(x2lat);
dist_calc2*=sin(diflon/2.0);                                       
dist_calc2*=sin(diflon/2.0);
dist_calc +=dist_calc2;
dist_calc=(2atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
dist_calc=6371000.0; //converting to meters
Serial.println(“distance from target”);
Serial.println(dist_calc);    //print the distance in meters
}

void headingcalc(){
 flon1 = radians(flon1);  //also must be done in radians
 x2lon = radians(x2lon);  //radians duh.
 heading = atan2(sin(x2lon-flon1)*cos(x2lat),cos(flat1)*sin(x2lat)-sin(flat1)cos(x2lat)cos(x2lon-flon1)),23.1415926535;
  heading = heading180/3.1415926535;  // convert from radians to degrees
  int head =heading; //make it a integer now
  if(head<0){
  heading+=360;   //if the heading is negative then add 360 to make it positive
}
  Serial.println(“heading:”);
  Serial.println(heading);   // print the heading.
  Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
  Wire.send(“A”);              
  Wire.endTransmission();
  delay(10);                  
  Wire.requestFrom(slaveAddress, 2);       
  i = 0;
  while(Wire.available() && i < 2)
  { 
    headingData[i] = Wire.receive();
    i++;
  }
  headingValue = headingData[0]*256 + headingData[1];
 int pracheading = headingValue / 10;      // this is the heading of the compass
 if(pracheading>0){
   headingcompass=pracheading;
 }
  Serial.println(“current heading:”);
  Serial.println(headingcompass);
  x4=headingcompass-heading;   //getting the difference of our current heading to our needed bearing
  Serial.println(“turning angle:”,x4);
}

 int guidingfunction(){
  int turn;
//--------------------------------------below tells us which way we need to turn
if(x4>=-180){
  if(x4<=0){
    turn=8;    //set turn =8 which means “right”         
  }
}
if(x4<-180){
  turn=5;      //set turn = 5 which means “left”
}
if(x4>=0){
  if(x4<180){
    turn=5;   //set turn = 5 which means “left”
  }
}
if(x4>=180){     //set turn =8 which means “right”
  turn=8;
}
int hd = headingcompass;
if(hd==heading){
    turn=3;   //then set turn = 3 meaning go “straight”
}

//executing turn
if(turn==3){
  Serial.println(“straight”);
}
//-------------------------------------------------------------------------------------turn right
if(turn==8){
rightturn();
}
//------------------------------------------------------------------------------------------turn left
if(turn==5){
leftturn();
}
void rightturn(){
}
void leftturn(){
}
