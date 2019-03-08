#include <kipr/botball.h>

void bot_claw_open(); //testing github editing
void bot_claw_closed(); //more github editing test (for pull request)
int rmotor=0, lmotor=2; //ports

int main()
{
    bot_claw_open();
    msleep(500);
    while(digital(0) == 0)
    {
        mav(rmotor, 920);
    	mav(lmotor, 800);
    	msleep(10);
    }
    bot_claw_closed();
    mav(rmotor, -920);
    mav(lmotor, -800);
    msleep(3000);
}
    
void bot_claw_open()
{
    enable_servos();
    set_servo_position(1,1600);
    msleep(100);
    disable_servos();
}

void bot_claw_closed()
{
    enable_servos();
    set_servo_position(1,600);
    msleep(100);
    disable_servos();
}
