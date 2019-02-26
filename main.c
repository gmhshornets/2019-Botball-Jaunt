#include <kipr/botball.h>

double bias; //variable to hold the calibration value
int right_motor=0, left_motor=2;//ports

int main(){
    //cg(); 
    bot_claw_open();
    top_claw_open();
    msleep(500);
    while(digital(0) == 0){
        mav(0, 920);
    	mav(2, 800);
    	msleep(10);
    }
    msleep(500);
    bot_claw_closed();
    top_claw_closed();
    msleep(500);
    mav(0, -920);
    mav(2, -800);
    msleep(3000);
    //drive_until_blue();
    //drive_with_gyro(800, 4000.0);
    //turn_with_gyro(-700, 700, 580000.0);
    //cg();
    //msleep(1000);
    //turn_with_gyro(700, -700, 580000.0); //?????
    //cg();
    //drive_until_black();
    
    //follow_tape_until_tape();
    
    /*bot_claw_open();
    msleep(1000);
    top_claw_open();
    msleep(1000);
    bot_claw_closed();
    msleep(1000);
    top_claw_closed();
    msleep(1000);
    ao();*/
}

//initializes the ports for the motors.
void declare_motors(int lmotor, int rmotor)
{
    right_motor = rmotor;
    left_motor = lmotor;
}

void cg() //calibrate gyro
{
   	//takes the average of 50 readings
    int i = 0;
    double avg = 0;
    while(i < 50)
    {
        avg += gyro_z();
        msleep(1);
        i++;
    }
    bias = avg / 50.0;
}

//turns the robot to reach a desired theta. 
//~580000 KIPR degrees = 90 degrees
void turn_with_gyro(int left_wheel_speed, int right_wheel_speed, double targetTheta)
{
    double theta = 0;//declares the variable that stores the current degrees
    mav(right_motor, right_wheel_speed);//starts the motors
    mav(left_motor, left_wheel_speed);
    //keeps the motors running until the robot reaches the desired angle
    while(theta < targetTheta)
    {
        msleep(10);//turns for .01 seconds
        theta += abs(gyro_z() - bias) * 10;//theta = omega(angular velocity, the value returned by gyroscopes) * time
    }
    //stops the motors after reaching the turn
    mav(right_motor, 0);
    mav(left_motor, 0);
    //ao();//maybe?
}

//drives straight forward or backwards. The closer speed is to 0 the faster it will correct itself and the more consistent it will be but just do not go at max speed. Time is in ms. 
void drive_with_gyro(int speed, double time)
{ 
    double startTime = seconds();
    double theta = 0;
    while(seconds() - startTime < (time / 1000.0))
    {
        if(speed > 0)
        {
            mav(right_motor, (double)(speed - speed * (1.920137e-16 + 0.000004470956*theta - 7.399285e-28*pow(theta, 2) - 2.054177e-18*pow(theta, 3) + 1.3145e-40*pow(theta, 4))));  //here at NAR, we are VERY precise
            mav(left_motor, (double)(speed + speed * (1.920137e-16 + 0.000004470956*theta - 7.399285e-28*pow(theta, 2) - 2.054177e-18*pow(theta, 3) + 1.3145e-40*pow(theta, 4))));
        }
        else//reverses corrections if it is going backwards
        {
            mav(right_motor, (double)(speed + speed * (1.920137e-16 + 0.000004470956*theta - 7.399285e-28*pow(theta, 2) - 2.054177e-18*pow(theta, 3) + 1.3145e-40*pow(theta, 4)))); 
            mav(left_motor, (double)(speed - speed * (1.920137e-16 + 0.000004470956*theta - 7.399285e-28*pow(theta, 2) - 2.054177e-18*pow(theta, 3) + 1.3145e-40*pow(theta, 4))));
        }
        //updates theta
        msleep(10);
        theta += (gyro_z() - bias) * 10;
    }
}

void drive_until_black()
{
    while(analog(0) < 3500) //adjust sensor number and sensor value
    {
        drive_with_gyro(800, 10.0);
    }
}

void drive_until_blue()
{
    while(analog(0) < 2800) //adjust sensor number and sensor value
    {
        drive_with_gyro(700,10.0);
    }
}

void follow_tape(double cm){
    cmpc(0);
    cmpc(2);
    while(((gmpc(0)+gmpc(2))/2) < 77.5*cm){
        if (analog(0) < 2000){
            mav(2, 1000);
            mav(0, 100);
            msleep(1);
            off(2);
            off(0);
        }
        else{
            mav(0, 1024);
            mav(2, 100);
            msleep(1);
            off(0);
            off(2);
        }
    }
}

void follow_tape_until_tape(){
    cmpc(0);
    cmpc(2);
    if (analog(0) < 2000){
            mav(2, 1000);
            mav(0, 100);
            msleep(1);
            off(2);
            off(0);
        }
        else{
            mav(0, 1024);
            mav(2, 100);
            msleep(1);
            off(0);
            off(2);
        }
}

/*void drive_on_blue(double distance) //change the motor numbers depending on ports
{
    void followTape(double cm){
    cmpc(0);
    cmpc(2);
    while(((gmpc(0)+gmpc(2))/2) < 77.5*cm){
        if (analog(0) < 3000){
            mav(2, 1000);
            mav(0, 100);
            msleep(1);
            off(2);
            off(0);
        }
        else{
            mav(0, 1024);
            mav(2, 100);
            msleep(1);
            off(0);
            off(2);
        }
    }
}*/

/*void turn_until_blue()
{
    while(analog(2) < 2000) //change blue value
    {
        turn_with_gyro(-300, 0, 10);
    }
}*/
    
/*void collecting_people()
{
    turn_until_blue();
    drive_with_gyro(800, 50);*/
    
void bot_claw_open()
{
    enable_servos();
    set_servo_position(1,1350);
    msleep(500);
    disable_servos();
}

void bot_claw_closed()
{
    enable_servos();
    set_servo_position(1,650);
    msleep(500);
    disable_servos();
}
    
void top_claw_open()
{
    enable_servos();
    set_servo_position(0,1400);
    msleep(500);
    disable_servos();
}
    
void top_claw_closed()
{
    enable_servos();
    set_servo_position(0,650);
    msleep(500);
    disable_servos();
}
    
    

    
    
